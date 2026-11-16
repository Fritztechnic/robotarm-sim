//! Kinematik: Vorwärtskinematik (Denavit-Hartenberg, Standard-Konvention), analytische
//! Jacobi-Matrix und numerische inverse Kinematik (gedämpfte Pseudoinverse / Levenberg-Marquardt).
//! Längen in mm, Winkel intern in rad.
//!
//! Aufbau: 1. Vektoren und Matrizen · 2. Roll/Pitch/Yaw · 3. Arm und DH · 4. Jacobi und Fehler ·
//! 5. lineare Algebra · 6. IK · 7. Nullraum · 8. Arbeitsraum.

use serde::{Deserialize, Serialize};
use std::f64::consts::{PI, TAU};

/// Vektor im Raum: x, y, z in mm.
pub type V3 = [f64; 3];
/// Homogene 4×4-Transformation (Drehung + Verschiebung).
pub type M4 = [[f64; 4]; 4];

pub const IDENT: M4 = [
    [1.0, 0.0, 0.0, 0.0],
    [0.0, 1.0, 0.0, 0.0],
    [0.0, 0.0, 1.0, 0.0],
    [0.0, 0.0, 0.0, 1.0],
];

// ---------- 1. Vektoren und Matrizen ----------

fn dot(a: V3, b: V3) -> f64 {
    a[0] * b[0] + a[1] * b[1] + a[2] * b[2]
}

fn cross(a: V3, b: V3) -> V3 {
    [a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]]
}

fn sub(a: V3, b: V3) -> V3 {
    [a[0] - b[0], a[1] - b[1], a[2] - b[2]]
}

fn norm(a: V3) -> f64 {
    dot(a, a).sqrt()
}

/// Matrix · Matrix (die Reihenfolge zählt).
pub fn mul(a: &M4, b: &M4) -> M4 {
    std::array::from_fn(|i| std::array::from_fn(|j| (0..4).map(|k| a[i][k] * b[k][j]).sum()))
}

/// Position (letzte Spalte).
pub fn pos(m: &M4) -> V3 {
    [m[0][3], m[1][3], m[2][3]]
}

/// Spalte c der Matrix: 0 = x-Achse, 1 = y-Achse, 2 = z-Achse des Koordinatensystems.
pub fn col(m: &M4, c: usize) -> V3 {
    [m[0][c], m[1][c], m[2][c]]
}

/// Inverse einer starren Transformation: Rᵀ und −Rᵀ·p.
pub fn inv(m: &M4) -> M4 {
    let mut r = IDENT;
    for i in 0..3 {
        for j in 0..3 {
            r[i][j] = m[j][i];
        }
        r[i][3] = -(0..3).map(|k| m[k][i] * m[k][3]).sum::<f64>();
    }
    r
}

/// Standard-DH: Rz(θ) · Tz(d) · Tx(a) · Rx(α)
pub fn dh(theta: f64, d: f64, a: f64, alpha: f64) -> M4 {
    let (st, ct) = theta.sin_cos();
    let (sa, ca) = alpha.sin_cos();
    [
        [ct, -st * ca, st * sa, a * ct],
        [st, ct * ca, -ct * sa, a * st],
        [0.0, sa, ca, d],
        [0.0, 0.0, 0.0, 1.0],
    ]
}

// ---------- 2. Roll / Pitch / Yaw ----------

/// Rotation aus Roll/Pitch/Yaw (rad): R = Rz(yaw) · Ry(pitch) · Rx(roll), plus Position.
pub fn from_xyz_rpy(p: V3, rpy: V3) -> M4 {
    let (sr, cr) = rpy[0].sin_cos();
    let (sp, cp) = rpy[1].sin_cos();
    let (sy, cy) = rpy[2].sin_cos();
    [
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr, p[0]],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr, p[1]],
        [-sp, cp * sr, cp * cr, p[2]],
        [0.0, 0.0, 0.0, 1.0],
    ]
}

/// Inverse zu `from_xyz_rpy` (rad). Im Gimbal-Lock (Pitch = ±90°) ist nur roll ∓ yaw bestimmt: roll = 0.
pub fn to_rpy(m: &M4) -> V3 {
    let sp = -m[2][0];
    let pitch = sp.clamp(-1.0, 1.0).asin();
    if sp.abs() > 0.999_999 {
        [0.0, pitch, (-m[0][1]).atan2(m[1][1])]
    } else {
        [m[2][1].atan2(m[2][2]), pitch, m[1][0].atan2(m[0][0])]
    }
}

/// Rotationsvektor (Achse · Winkel, Winkel in 0..π) der Restdrehung R_t · R_cᵀ.
/// Stabil auch nahe 0° und 180°: dort verschwindet der antisymmetrische Anteil, die Achse wird dann
/// aus der Diagonale (R = 2 n nᵀ − I) bestimmt und mit dem Vorzeichen des antisymmetrischen Anteils ausgerichtet.
pub fn rot_error(t: &M4, c: &M4) -> V3 {
    let re: [[f64; 3]; 3] = std::array::from_fn(|i| std::array::from_fn(|j| (0..3).map(|k| t[i][k] * c[j][k]).sum()));
    let v = [(re[2][1] - re[1][2]) / 2.0, (re[0][2] - re[2][0]) / 2.0, (re[1][0] - re[0][1]) / 2.0];
    let s = norm(v);
    let cos = (re[0][0] + re[1][1] + re[2][2] - 1.0) / 2.0;
    let angle = s.atan2(cos);
    if s > 1e-3 || (s > 1e-9 && cos > 0.0) {
        return v.map(|x| x * angle / s);
    }
    if cos > 0.0 {
        return [0.0; 3];
    }
    let i = (0..3).max_by(|&a, &b| re[a][a].total_cmp(&re[b][b])).unwrap();
    let mut ax = [0.0; 3];
    ax[i] = ((re[i][i] + 1.0) / 2.0).max(0.0).sqrt();
    for j in (0..3).filter(|&j| j != i) {
        ax[j] = re[i][j] / (2.0 * ax[i]);
    }
    if dot(ax, v) < 0.0 {
        ax = ax.map(|x| -x);
    }
    ax.map(|x| x * angle)
}

// ---------- 3. Arm und DH ----------

#[derive(Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct Joint {
    pub a: f64,
    pub alpha_deg: f64,
    pub d: f64,
    pub offset_deg: f64,
    pub min_deg: f64,
    pub max_deg: f64,
}

impl Joint {
    pub fn min(&self) -> f64 {
        self.min_deg.to_radians()
    }
    pub fn max(&self) -> f64 {
        self.max_deg.to_radians()
    }
    /// Gelenke mit mindestens einer Umdrehung Spielraum (J1, J4, J6): q und q ± 2π sind dieselbe Pose.
    pub fn full_turn(&self) -> bool {
        self.max() - self.min() >= TAU
    }
}

pub type Arm = [Joint; 6];

/// Alle Gelenk-Frames: Index 0 = Basis, Index i = Frame nach Gelenk i.
pub fn fk_frames(arm: &Arm, q: &[f64; 6]) -> [M4; 7] {
    let mut f = [IDENT; 7];
    for i in 0..6 {
        let j = &arm[i];
        f[i + 1] = mul(&f[i], &dh(q[i] + j.offset_deg.to_radians(), j.d, j.a, j.alpha_deg.to_radians()));
    }
    f
}

/// Pose der Werkzeugspitze (letztes Frame).
pub fn fk(arm: &Arm, q: &[f64; 6]) -> M4 {
    fk_frames(arm, q)[6]
}

/// Elfin-Cobot (aus dem importierten Modell abgeleitet): DH-Werte und Anschläge.
pub fn cobot_arm() -> Arm {
    // J1, J4 und J6: intern ±720° Spielraum, damit der kürzeste Weg nie an einer Grenze scheitert
    // (340° → 10° = +30°). Angezeigt wird nach Rückstellung bei ±360° auf 0 (App::advance).
    let j = |a, al, d, off, lim: f64| Joint { a, alpha_deg: al, d, offset_deg: off, min_deg: -lim, max_deg: lim };
    [
        j(0.278, -90.0, 131.673, 0.0, 720.0),
        j(240.342, 0.0, 0.0, -90.0, 120.0),
        j(0.757, 89.157, -1.113, 90.0, 150.0),
        j(3.826, -93.323, 261.045, 180.0, 720.0),
        j(0.133, 91.621, 3.769, 0.0, 120.0),
        j(0.0, 0.0, 83.335, 0.0, 720.0),
    ]
}

/// Gesamt-θ der Modell-Ruhepose: So stand der Arm im importierten Modell; daraus wird das 3D-Bild geeicht.
pub const COBOT_REST_THETA_DEG: [f64; 6] = [-57.797, -83.942, -156.059, -175.644, -73.088, 0.0];

/// Kürzester Weg: Vollkreis-Gelenke um ganze Umdrehungen verschieben, bis `q` dem Bezug `r`
/// am nächsten liegt (359° → 0° heißt +1°). Die Gelenkgrenzen bleiben gewahrt.
pub fn wrap_near(arm: &Arm, mut q: [f64; 6], r: &[f64; 6]) -> [f64; 6] {
    for i in 0..6 {
        let j = &arm[i];
        if j.full_turn() {
            let mut v = q[i] - ((q[i] - r[i]) / TAU).round() * TAU;
            while v > j.max() {
                v -= TAU;
            }
            while v < j.min() {
                v += TAU;
            }
            q[i] = v;
        }
    }
    q
}

fn clamp_q(arm: &Arm, q: &mut [f64; 6]) {
    for i in 0..6 {
        q[i] = q[i].clamp(arm[i].min(), arm[i].max());
    }
}

// ---------- 4. Jacobi und Fehler ----------

/// Geometrische Jacobi-Matrix aus fertigen Frames (exakt, analytisch): Spalte i = [zᵢ × (pₑ − oᵢ); zᵢ],
/// zᵢ = Drehachse, oᵢ = Ursprung des Gelenks i. Zeilen 0–2: Spitzengeschwindigkeit (mm/rad),
/// Zeilen 3–5: Winkelgeschwindigkeit (rad/rad).
fn jacobian_of(f: &[M4; 7]) -> [[f64; 6]; 6] {
    let pe = pos(&f[6]);
    let mut j = [[0.0; 6]; 6];
    for i in 0..6 {
        let z = col(&f[i], 2);
        let v = cross(z, sub(pe, pos(&f[i])));
        for k in 0..3 {
            j[k][i] = v[k];
            j[3 + k][i] = z[k];
        }
    }
    j
}

pub fn jacobian(arm: &Arm, q: &[f64; 6]) -> [[f64; 6]; 6] {
    jacobian_of(&fk_frames(arm, q))
}

/// Rotationsfehler (rad) wird mit dieser Länge (mm) gewichtet, damit beide Fehleranteile vergleichbar sind.
const L: f64 = 100.0;

/// Fehler einer Stellung gegenüber dem Ziel; die Frames werden für die Jacobi-Matrix wiederverwendet.
struct Eval {
    frames: [M4; 7],
    /// [Δp (mm); Rotationsvektor (rad)] – ohne Winkelvorgabe sind die letzten drei 0
    e: [f64; 6],
    pe: f64,
    re: f64,
}

impl Eval {
    fn new(arm: &Arm, q: &[f64; 6], target: &M4, orient: bool) -> Self {
        let frames = fk_frames(arm, q);
        let dp = sub(pos(target), pos(&frames[6]));
        let dr = if orient { rot_error(target, &frames[6]) } else { [0.0; 3] };
        Eval { frames, e: [dp[0], dp[1], dp[2], dr[0], dr[1], dr[2]], pe: norm(dp), re: norm(dr) }
    }
    fn cost(&self) -> f64 {
        self.pe + L * self.re
    }
}

// ---------- 5. Lineare Algebra ----------

/// Löst A·x = b für symmetrisch positiv definite A per Cholesky (A = L·Lᵀ). None, wenn A nicht positiv definit ist.
fn spd_solve<const N: usize>(a: [[f64; N]; N], b: [f64; N]) -> Option<[f64; N]> {
    let mut l = [[0.0; N]; N];
    for i in 0..N {
        for j in 0..=i {
            let s = a[i][j] - (0..j).map(|k| l[i][k] * l[j][k]).sum::<f64>();
            if i == j {
                if s <= 1e-14 {
                    return None;
                }
                l[i][i] = s.sqrt();
            } else {
                l[i][j] = s / l[j][j];
            }
        }
    }
    let mut y = [0.0; N];
    for i in 0..N {
        y[i] = (b[i] - (0..i).map(|k| l[i][k] * y[k]).sum::<f64>()) / l[i][i];
    }
    let mut x = [0.0; N];
    for i in (0..N).rev() {
        x[i] = (y[i] - (i + 1..N).map(|k| l[k][i] * x[k]).sum::<f64>()) / l[i][i];
    }
    Some(x)
}

/// Gedämpfte Pseudoinverse: dq = Jᵀ (J Jᵀ + λ² I)⁻¹ e für die ersten N Zeilen von J.
fn damped_step<const N: usize>(jac: &[[f64; 6]; 6], e: &[f64; 6], lambda: f64) -> Option<[f64; 6]> {
    let mut a = [[0.0; N]; N];
    for i in 0..N {
        for j in 0..N {
            a[i][j] = (0..6).map(|k| jac[i][k] * jac[j][k]).sum::<f64>() + if i == j { lambda * lambda } else { 0.0 };
        }
    }
    let y = spd_solve::<N>(a, std::array::from_fn(|i| e[i]))?;
    Some(std::array::from_fn(|j| (0..N).map(|i| jac[i][j] * y[i]).sum()))
}

// ---------- 6. Inverse Kinematik ----------

/// Ergebnis der IK: Winkel plus Güte der Lösung.
pub struct IkResult {
    pub q: [f64; 6],
    pub pos_err: f64,
    pub rot_err: f64,
    pub converged: bool,
}

/// Einstellungen: mit/ohne Winkelvorgabe, Toleranzen (mm, rad), Anzahl deterministischer Neustarts.
pub struct IkOptions {
    pub use_orientation: bool,
    pub pos_tol: f64,
    pub rot_tol: f64,
    pub restarts: usize,
}

/// Maximale Schrittweite eines Gelenks pro Iteration (rad): hält die Linearisierung gültig.
const MAX_STEP: f64 = 0.5;

/// Levenberg-Marquardt ab dem Startwert `q0`. Die Dämpfung λ sinkt nach jedem akzeptierten und
/// steigt nach jedem abgelehnten Schritt (Faktor 4); angenommen wird nur, was den Gesamtfehler senkt.
/// Gelenkgrenzen werden durch Projektion (clamp) eingehalten.
fn iterate(arm: &Arm, target: &M4, q0: [f64; 6], o: &IkOptions) -> IkResult {
    let ori = o.use_orientation;
    let mut q = q0;
    clamp_q(arm, &mut q);
    let mut lambda = 1.0;
    let mut cur = Eval::new(arm, &q, target, ori);
    for _ in 0..300 {
        if cur.pe < o.pos_tol && cur.re < o.rot_tol {
            break;
        }
        let jac = {
            let mut j = jacobian_of(&cur.frames);
            for row in &mut j[3..] {
                for v in row.iter_mut() {
                    *v *= L;
                }
            }
            j
        };
        let ew = [cur.e[0], cur.e[1], cur.e[2], cur.e[3] * L, cur.e[4] * L, cur.e[5] * L];
        let mut improved = false;
        for _ in 0..12 {
            let dq = if ori { damped_step::<6>(&jac, &ew, lambda) } else { damped_step::<3>(&jac, &ew, lambda) };
            if let Some(mut dq) = dq {
                let m = dq.iter().fold(0.0f64, |m, v| m.max(v.abs()));
                if m > MAX_STEP {
                    dq = dq.map(|v| v * MAX_STEP / m);
                }
                let mut qn: [f64; 6] = std::array::from_fn(|i| q[i] + dq[i]);
                clamp_q(arm, &mut qn);
                let next = Eval::new(arm, &qn, target, ori);
                if next.cost() < cur.cost() {
                    q = qn;
                    cur = next;
                    lambda = (lambda * 0.5).max(1e-6);
                    improved = true;
                    break;
                }
            }
            lambda *= 4.0;
        }
        if !improved {
            break;
        }
    }
    IkResult { q, pos_err: cur.pe, rot_err: cur.re, converged: cur.pe < o.pos_tol && cur.re < o.rot_tol }
}

/// Xorshift64: kleiner deterministischer Zufallsgenerator (Gleichverteilung in [0, 1)).
struct Rng(u64);

impl Rng {
    fn next(&mut self) -> f64 {
        self.0 ^= self.0 << 13;
        self.0 ^= self.0 >> 7;
        self.0 ^= self.0 << 17;
        (self.0 >> 11) as f64 / (1u64 << 53) as f64
    }
}

/// Löst die IK. Erster Versuch startet am aktuellen Winkelsatz (kleinste Bewegung), danach folgen
/// deterministische Zufalls-Neustarts innerhalb der Gelenkgrenzen. Bei reiner Positionsvorgabe wird
/// die Lösung zusätzlich im Nullraum zur Komfortstellung geschoben (`polish`).
pub fn solve_ik(arm: &Arm, target: &M4, q_start: [f64; 6], o: &IkOptions) -> IkResult {
    let r = search(arm, target, q_start, o);
    if r.converged && !o.use_orientation && o.restarts > 0 {
        polish(arm, target, r, o, 25)
    } else {
        r
    }
}

fn search(arm: &Arm, target: &M4, q_start: [f64; 6], o: &IkOptions) -> IkResult {
    let mut best = iterate(arm, target, q_start, o);
    let mut rng = Rng(0x9E37_79B9_7F4A_7C15);
    for _ in 0..o.restarts {
        if best.converged {
            break;
        }
        // Vollkreis-Gelenke: eine Umdrehung genügt für die Startwerte
        let q: [f64; 6] = std::array::from_fn(|i| {
            let (lo, hi) = (arm[i].min().max(-PI), arm[i].max().min(PI));
            lo + rng.next() * (hi - lo)
        });
        let r = iterate(arm, target, q, o);
        if r.converged || r.pos_err + L * r.rot_err < best.pos_err + L * best.rot_err {
            best = r;
        }
    }
    best
}

// ---------- 7. Nullraum ----------

/// Nur Position vorgegeben: der Arm hat 3 überzählige Freiheitsgrade. Im Nullraum der Positions-Jacobi-Matrix,
///   Δq = (I − J⁺J) g,   g = −W q,   J⁺ = Jᵀ (J Jᵀ + ε I)⁻¹,
/// werden die Handgelenke weg von Anschlägen und Singularitäten zur Komfortstellung gezogen,
/// ohne die Zielposition zu verlassen (jeder Schritt wird per IK auf das Ziel zurückprojiziert).
pub fn polish(arm: &Arm, target: &M4, mut r: IkResult, o: &IkOptions, iters: usize) -> IkResult {
    const W: [f64; 6] = [0.0, 0.1, 0.1, 1.0, 0.3, 1.0];
    for _ in 0..iters {
        let jf = jacobian(arm, &r.q);
        let g: [f64; 6] = std::array::from_fn(|i| -W[i] * r.q[i]);
        let jg: [f64; 6] = std::array::from_fn(|i| (0..6).map(|k| jf[i][k] * g[k]).sum());
        let Some(dq_pos) = damped_step::<3>(&jf, &jg, 0.0316) else { break };
        let mut dq: [f64; 6] = std::array::from_fn(|j| g[j] - dq_pos[j]);
        let m = dq.iter().fold(0.0f64, |m, v| m.max(v.abs()));
        if m < 1e-5 {
            break;
        }
        dq = dq.map(|v| v * 0.5 * 0.03 / m.max(0.03));
        let n = iterate(arm, target, std::array::from_fn(|i| r.q[i] + dq[i]), o);
        if !n.converged {
            break;
        }
        r = n;
    }
    r
}

// ---------- 8. Arbeitsraum ----------

/// Auflösung der Hüllflächen: Azimut × Polarwinkel.
pub const RA: usize = 48;
pub const RP: usize = 24;

/// Arbeitsraum der Werkzeugspitze (nur Position) als Hüllfläche um die Schulterachse:
/// für jede Richtung der kleinste und größte erreichbare Abstand.
#[derive(Clone)]
pub struct Reach {
    pub c: V3,
    pub lo: Vec<f64>,
    pub hi: Vec<f64>,
}

impl Reach {
    /// Außenradius: weiteste Stelle des Arbeitsraums.
    pub fn outer(&self) -> f64 {
        self.hi.iter().cloned().fold(0.0, f64::max)
    }
    /// Totzone: kleinster Abstand, den die Spitze irgendwo erreicht.
    pub fn dead(&self) -> f64 {
        self.lo.iter().cloned().fold(f64::MAX, f64::min)
    }
    /// Bis zu diesem Radius ist JEDE Richtung erreichbar (Kugel ohne Lücken).
    pub fn full(&self) -> f64 {
        self.hi.iter().cloned().fold(f64::MAX, f64::min)
    }
}

/// Zufallsstichprobe im Gelenkraum (deterministisch, 900 000 Stück). Gelenkgrenzen werden eingehalten.
pub fn reach(arm: &Arm) -> Reach {
    let c = pos(&fk_frames(arm, &[0.0; 6])[1]);
    let mut rng = Rng(0x2545_F491_4F6C_DD1D);
    let (mut lo, mut hi) = (vec![f64::MAX; RA * RP], vec![0.0f64; RA * RP]);
    for _ in 0..900_000 {
        let q: [f64; 6] = std::array::from_fn(|i| {
            let j = &arm[i];
            // 20 % der Proben sitzen direkt an den Anschlägen: dort liegen die Extreme
            let t = if rng.next() < 0.2 { (rng.next() < 0.5) as u8 as f64 } else { rng.next() };
            let (a, b) = (j.min().max(-PI), j.max().min(PI));
            a + (b - a) * t
        });
        let d = sub(pos(&fk(arm, &q)), c);
        let r = norm(d);
        if r < 1e-9 {
            continue;
        }
        let az = ((d[1].atan2(d[0]) / TAU + 0.5) * RA as f64) as usize % RA;
        let pol = (((d[2] / r).clamp(-1.0, 1.0).acos() / PI) * RP as f64) as usize;
        let k = pol.min(RP - 1) * RA + az;
        lo[k] = lo[k].min(r);
        hi[k] = hi[k].max(r);
    }
    for v in lo.iter_mut().filter(|v| **v == f64::MAX) {
        *v = 0.0;
    }
    Reach { c, lo, hi }
}

#[cfg(test)]
mod tests {
    use super::*;

    pub fn default_arm() -> Arm {
        let j = |a, al, d, off| Joint { a, alpha_deg: al, d, offset_deg: off, min_deg: -170.0, max_deg: 170.0 };
        [j(0.0, 90.0, 100.0, 0.0), j(120.0, 0.0, 0.0, 90.0), j(0.0, 90.0, 0.0, 90.0),
         j(0.0, -90.0, 120.0, 0.0), j(0.0, 90.0, 0.0, 0.0), j(0.0, 0.0, 60.0, 0.0)]
    }

    #[test]
    fn fk_zero_pose_points_up() {
        let p = pos(&fk(&default_arm(), &[0.0; 6]));
        let want = [0.0, 0.0, 400.0];
        for i in 0..3 {
            assert!((p[i] - want[i]).abs() < 1e-9, "{p:?}");
        }
    }

    #[test]
    fn analytic_jacobian_matches_finite_differences() {
        let arm = cobot_arm();
        let q = [0.4, -0.9, 1.1, 0.7, -0.5, 0.3];
        let jac = jacobian(&arm, &q);
        let h = 1e-6;
        for i in 0..6 {
            let (mut a, mut b) = (q, q);
            a[i] += h;
            b[i] -= h;
            let (fa, fb) = (fk(&arm, &a), fk(&arm, &b));
            let r = rot_error(&fa, &fb);
            for k in 0..3 {
                let dp = (pos(&fa)[k] - pos(&fb)[k]) / (2.0 * h);
                assert!((dp - jac[k][i]).abs() < 1e-4, "pos {k},{i}: {dp} vs {}", jac[k][i]);
                assert!((r[k] / (2.0 * h) - jac[3 + k][i]).abs() < 1e-6, "rot {k},{i}");
            }
        }
    }

    #[test]
    fn rot_error_is_exact_near_180_degrees() {
        for ang in [3.0f64, 3.1, 3.14, PI - 1e-5] {
            let (s, c) = ang.sin_cos();
            // Drehung um die Achse (1,2,2)/3 mit dem Winkel ang (Rodrigues)
            let a = [1.0 / 3.0, 2.0 / 3.0, 2.0 / 3.0];
            let mut t = IDENT;
            for i in 0..3 {
                for j in 0..3 {
                    let k = [[0.0, -a[2], a[1]], [a[2], 0.0, -a[0]], [-a[1], a[0], 0.0]][i][j];
                    t[i][j] = (i == j) as u8 as f64 * c + (1.0 - c) * a[i] * a[j] + s * k;
                }
            }
            let e = rot_error(&t, &IDENT);
            assert!((norm(e) - ang).abs() < 1e-6, "{ang}: {}", norm(e));
            assert!(dot(e, a) > 0.0, "{ang}: Achse falsch herum");
        }
    }

    #[test]
    fn cholesky_solves_spd_systems() {
        let a = [[4.0, 1.0, 0.5], [1.0, 3.0, 0.2], [0.5, 0.2, 2.0]];
        let x = [1.0, -2.0, 3.0];
        let b: [f64; 3] = std::array::from_fn(|i| (0..3).map(|k| a[i][k] * x[k]).sum());
        let s = spd_solve(a, b).unwrap();
        for i in 0..3 {
            assert!((s[i] - x[i]).abs() < 1e-12);
        }
        assert!(spd_solve([[1.0, 2.0], [2.0, 1.0]], [1.0, 1.0]).is_none());
    }

    #[test]
    fn wrap_takes_the_short_way() {
        let arm = cobot_arm();
        let d = |v: f64| v.to_radians();
        let q = wrap_near(&arm, [d(0.0), 0.0, 0.0, d(-450.0), 0.0, 0.0], &[d(359.0), 0.0, 0.0, 0.0, 0.0, 0.0]);
        assert!((q[0].to_degrees() - 360.0).abs() < 1e-9, "{}", q[0].to_degrees());
        assert!((q[3].to_degrees() + 90.0).abs() < 1e-9, "{}", q[3].to_degrees());
        // J2 hat Grenzen < 360°: bleibt unverändert
        assert_eq!(wrap_near(&arm, [0.0, d(100.0), 0.0, 0.0, 0.0, 0.0], &[0.0; 6])[1], d(100.0));
    }

    #[test]
    fn rpy_roundtrip() {
        let rpy = [0.3, -0.7, 2.1];
        let back = to_rpy(&from_xyz_rpy([0.0; 3], rpy));
        for i in 0..3 {
            assert!((rpy[i] - back[i]).abs() < 1e-12);
        }
    }

    #[test]
    fn ik_roundtrip_random_poses() {
        let arm = default_arm();
        let mut s = 12345u64;
        let mut r = || {
            s = s.wrapping_mul(6364136223846793005).wrapping_add(1442695040888963407);
            ((s >> 33) as f64 / (1u64 << 31) as f64) * 2.0 - 1.0
        };
        let opt = IkOptions { use_orientation: true, pos_tol: 1e-4, rot_tol: 1e-6, restarts: 60 };
        let mut ok = 0;
        for _ in 0..50 {
            let mut q = [0.0; 6];
            for v in q.iter_mut() {
                *v = r() * 120f64.to_radians();
            }
            let t = fk(&arm, &q);
            let res = solve_ik(&arm, &t, [0.1; 6], &opt);
            let t2 = fk(&arm, &res.q);
            let ep = (0..3).map(|i| (t[i][3] - t2[i][3]).powi(2)).sum::<f64>().sqrt();
            if res.converged && ep < 1e-3 {
                ok += 1;
            }
        }
        assert!(ok >= 48, "nur {ok}/50 gelöst");
    }

    #[test]
    fn ik_unreachable_reports_failure() {
        let arm = default_arm();
        let t = from_xyz_rpy([2000.0, 0.0, 0.0], [0.0; 3]);
        let opt = IkOptions { use_orientation: false, pos_tol: 1e-3, rot_tol: 1e-3, restarts: 5 };
        assert!(!solve_ik(&arm, &t, [0.0; 6], &opt).converged);
    }

    #[test]
    fn polish_keeps_position_and_reduces_wrist_cost() {
        let arm = cobot_arm();
        let t = from_xyz_rpy([300.0, 150.0, 450.0], [0.0; 3]);
        let opt = IkOptions { use_orientation: false, pos_tol: 1e-3, rot_tol: 1e-5, restarts: 20 };
        let raw = search(&arm, &t, [0.0; 6], &opt);
        let pol = solve_ik(&arm, &t, [0.0; 6], &opt);
        assert!(raw.converged && pol.converged);
        let cost = |q: &[f64; 6]| q[3] * q[3] + q[5] * q[5];
        assert!(cost(&pol.q) <= cost(&raw.q) + 1e-9);
    }
}
