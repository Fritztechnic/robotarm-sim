//! Kinematik des 6-Achs-Arms. Längen in mm, Winkel intern in rad.
//!
//! Grundidee: Der Elfin ist ein klassischer Knickarm mit Kugelhandgelenk:
//!   * J1 und J2 schneiden sich im Schulterpunkt S,
//!   * J2 und J3 sind parallel (ebener Ober-/Unterarm),
//!   * J4, J5 und J6 schneiden sich im Handgelenkpunkt W.
//! Damit ist die inverse Kinematik EXAKT lösbar (geschlossene Form, Paden-Kahan-Teilprobleme) und liefert
//! alle Haltungen auf einmal: Schulter vorn/hinten × Ellbogen oben/unten × Handgelenk normal/gekippt (≤ 8).
//! Es gibt keine Zufallsstartwerte und keine Abhängigkeit von der vorherigen Stellung mehr; die Wahl
//! der Haltung trifft der Aufrufer (nächstliegende Lösung bzw. Planer).
//!
//! Ohne Werkzeugwinkel (nur Position) legt eine feste Regel die Orientierung fest (`auto_tool`): Das Werkzeug
//! zeigt vom Schulterpunkt weg, J4 = J6 = 0. Die Gelenkstellung ist dadurch eine Funktion der Position
//! (je Haltung): Hin- und Rückweg sind identisch, nichts driftet, keine versteckten Umorientierungen.
//!
//! Aufbau: 1. Vektoren · 2. Drehungen · 3. Arm/DH · 4. Schraubachsen · 5. Teilprobleme ·
//! 6. Analytische IK · 7. Jacobi + Newton (Nachpolitur, Rückfall) · 8. Öffentliche IK · 9. Arbeitsraum.

use serde::{Deserialize, Serialize};
use std::f64::consts::{PI, TAU};

/// Vektor im Raum: x, y, z in mm.
pub type V3 = [f64; 3];
/// Homogene 4×4-Transformation (Drehung + Verschiebung).
pub type M4 = [[f64; 4]; 4];
type M3 = [[f64; 3]; 3];

pub const IDENT: M4 = [
    [1.0, 0.0, 0.0, 0.0],
    [0.0, 1.0, 0.0, 0.0],
    [0.0, 0.0, 1.0, 0.0],
    [0.0, 0.0, 0.0, 1.0],
];

// ---------- 1. Vektoren und Matrizen ----------

fn add(a: V3, b: V3) -> V3 {
    [a[0] + b[0], a[1] + b[1], a[2] + b[2]]
}

fn sub(a: V3, b: V3) -> V3 {
    [a[0] - b[0], a[1] - b[1], a[2] - b[2]]
}

fn scale(a: V3, s: f64) -> V3 {
    [a[0] * s, a[1] * s, a[2] * s]
}

fn dot(a: V3, b: V3) -> f64 {
    a[0] * b[0] + a[1] * b[1] + a[2] * b[2]
}

fn cross(a: V3, b: V3) -> V3 {
    [a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]]
}

fn norm(a: V3) -> f64 {
    dot(a, a).sqrt()
}

fn unit(a: V3) -> V3 {
    let n = norm(a);
    if n < 1e-12 { [0.0; 3] } else { scale(a, 1.0 / n) }
}

/// Anteil von x senkrecht zur Einheitsrichtung w.
fn perp(w: V3, x: V3) -> V3 {
    sub(x, scale(w, dot(w, x)))
}

/// Matrix · Matrix (die Reihenfolge zählt).
pub fn mul(a: &M4, b: &M4) -> M4 {
    std::array::from_fn(|i| std::array::from_fn(|j| (0..4).map(|k| a[i][k] * b[k][j]).sum()))
}

/// Punkt transformieren: R·p + t.
fn apply(m: &M4, p: V3) -> V3 {
    std::array::from_fn(|i| m[i][0] * p[0] + m[i][1] * p[1] + m[i][2] * p[2] + m[i][3])
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

// ---------- 2. Drehungen ----------

/// Rodrigues: Drehung um die Einheitsachse w mit Winkel θ: R = I + sinθ·[w]× + (1 − cosθ)·[w]×².
fn rodrigues(w: V3, th: f64) -> M3 {
    let (s, c) = th.sin_cos();
    let k = [[0.0, -w[2], w[1]], [w[2], 0.0, -w[0]], [-w[1], w[0], 0.0]];
    std::array::from_fn(|i| std::array::from_fn(|j| if i == j { c } else { 0.0 } + (1.0 - c) * w[i] * w[j] + s * k[i][j]))
}

fn m3_apply(r: &M3, v: V3) -> V3 {
    std::array::from_fn(|i| r[i][0] * v[0] + r[i][1] * v[1] + r[i][2] * v[2])
}

/// Drehung aus Rotationsvektor v (Achse · Winkel), als 4×4 ohne Verschiebung: exp([v]×).
pub fn rot_exp(v: V3) -> M4 {
    let r = rodrigues(unit(v), norm(v));
    let mut m = IDENT;
    for i in 0..3 {
        m[i][..3].copy_from_slice(&r[i]);
    }
    m
}

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

/// Rotationsvektor (Achse · Winkel, Winkel in 0..π) der Restdrehung R_t · R_cᵀ (Log-Abbildung).
/// Stabil auch nahe 0° und 180°: dort verschwindet der antisymmetrische Anteil, die Achse wird dann
/// aus der Diagonale (R = 2 n nᵀ − I) bestimmt und mit dem Vorzeichen des antisymmetrischen Anteils ausgerichtet.
pub fn rot_error(t: &M4, c: &M4) -> V3 {
    let re: M3 = std::array::from_fn(|i| std::array::from_fn(|j| (0..3).map(|k| t[i][k] * c[j][k]).sum()));
    let v = [(re[2][1] - re[1][2]) / 2.0, (re[0][2] - re[2][0]) / 2.0, (re[1][0] - re[0][1]) / 2.0];
    let s = norm(v);
    let cos = (re[0][0] + re[1][1] + re[2][2] - 1.0) / 2.0;
    let angle = s.atan2(cos);
    if s > 1e-3 || (s > 1e-12 && cos > 0.0) {
        return scale(v, angle / s);
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
        ax = scale(ax, -1.0);
    }
    scale(ax, angle)
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
        self.max() - self.min() >= TAU - 1e-9
    }
}

pub type Arm = [Joint; 6];

/// Alle Gelenk-Frames: Index 0 = Basis, Index i = Frame nach Gelenk i. Gelenk i dreht um z von Frame i−1.
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

/// (a, α, d, Offset) je Gelenk.
type Dh = [(f64, f64, f64, f64); 6];

/// Elfin-Cobot, ideale Geometrie: Kugelhandgelenk, J1/J2 schneiden sich, J2 ∥ J3, alle α exakt ±90°/0°.
const COBOT_DH: Dh = [
    (0.0, -90.0, 131.673, 0.0),
    (240.342, 0.0, 0.0, -90.0),
    (0.0, 90.0, 0.0, 90.0),
    (0.0, -90.0, 261.045, 180.0),
    (0.0, 90.0, 0.0, 0.0),
    (0.0, 0.0, 83.335, 0.0),
];

/// Aus dem 3D-Modell gemessene Achsen (Ausgleichsrechnung über die Gelenkflächen). Die kleinen Abweichungen
/// (a₁, a₃, a₄, d₃, d₅ ≈ 0, α um bis zu 3,3° schief) sind Messrauschen, sie verdrehten den Kopf um ~2,5°.
/// Sie dienen nur noch als Anker, um die Modellteile auf die idealen Achsen zu setzen.
const MODEL_DH: Dh = [
    (0.278, -90.0, 131.673, 0.0),
    (240.342, 0.0, 0.0, -90.0),
    (0.757, 89.157, -1.113, 90.0),
    (3.826, -93.323, 261.045, 180.0),
    (0.133, 91.621, 3.769, 0.0),
    (0.0, 0.0, 83.335, 0.0),
];

/// Gesamt-θ der Modell-Ruhepose: So stand der Arm im importierten Modell.
const MODEL_REST_THETA_DEG: [f64; 6] = [-57.797, -83.942, -156.059, -175.644, -73.088, 0.0];

fn arm_from(dh: &Dh, lim: [f64; 6]) -> Arm {
    std::array::from_fn(|i| {
        let (a, alpha_deg, d, offset_deg) = dh[i];
        Joint { a, alpha_deg, d, offset_deg, min_deg: -lim[i], max_deg: lim[i] }
    })
}

/// Elfin-Cobot mit Anschlägen. J1, J4 und J6 haben intern ±720° Spielraum, damit der kürzeste Weg nie an
/// einer Grenze scheitert (340° → 10° = +30°); angezeigt wird nach Rückstellung bei ±360° auf 0.
pub fn cobot_arm() -> Arm {
    arm_from(&COBOT_DH, [720.0, 120.0, 150.0, 720.0, 120.0, 720.0])
}

/// Ist das (noch) die gemessene Modellgeometrie? Dann wird sie durch die ideale ersetzt (alte Configs).
pub fn is_model_geometry(arm: &Arm) -> bool {
    arm.iter().zip(MODEL_DH).all(|(j, m)| (j.a - m.0).abs() < 1e-6 && (j.alpha_deg - m.1).abs() < 1e-6 && (j.d - m.2).abs() < 1e-6 && (j.offset_deg - m.3).abs() < 1e-6)
}

/// Ideale DH-Werte übernehmen, Anschläge behalten.
pub fn idealize(arm: &mut Arm) {
    for (j, c) in arm.iter_mut().zip(COBOT_DH) {
        (j.a, j.alpha_deg, j.d, j.offset_deg) = c;
    }
}

/// Frames der Modell-Ruhepose (gemessene Achsen): Teil k wird mit F_k(q) · F_k(Ruhe)⁻¹ bewegt.
pub fn model_rest_frames() -> [M4; 7] {
    let arm = arm_from(&MODEL_DH, [720.0; 6]);
    let q: [f64; 6] = std::array::from_fn(|i| (MODEL_REST_THETA_DEG[i] - MODEL_DH[i].3).to_radians());
    fk_frames(&arm, &q)
}

/// Winkel auf (−π, π].
fn wrap_pi(a: f64) -> f64 {
    a - ((a - PI) / TAU).ceil() * TAU
}

/// Kürzester Weg: Vollkreis-Gelenke um ganze Umdrehungen verschieben, bis `q` dem Bezug `r`
/// am nächsten liegt (359° → 0° heißt +1°). Die Gelenkgrenzen bleiben gewahrt.
pub fn wrap_near(arm: &Arm, mut q: [f64; 6], r: &[f64; 6]) -> [f64; 6] {
    for i in 0..6 {
        let j = &arm[i];
        if j.full_turn() {
            let mut v = r[i] + wrap_pi(q[i] - r[i]);
            while v > j.max() + 1e-12 {
                v -= TAU;
            }
            while v < j.min() - 1e-12 {
                v += TAU;
            }
            q[i] = v;
        }
    }
    q
}

/// Lösung in die Gelenkgrenzen legen: je Gelenk die Darstellung q + k·2π, die erlaubt ist und 0 am
/// nächsten liegt. None, wenn ein Gelenk in keiner Darstellung erlaubt ist.
fn fit_limits(arm: &Arm, q: [f64; 6]) -> Option<[f64; 6]> {
    let mut out = [0.0; 6];
    for i in 0..6 {
        let (lo, hi) = (arm[i].min() - 1e-9, arm[i].max() + 1e-9);
        let base = wrap_pi(q[i]);
        out[i] = [0.0, TAU, -TAU, 2.0 * TAU, -2.0 * TAU].iter().map(|k| base + k).find(|v| (lo..=hi).contains(v))?;
    }
    Some(out)
}

/// Größter Gelenkabstand (rad), Vollkreis-Gelenke über den kürzeren Weg.
pub fn joint_dist(arm: &Arm, a: &[f64; 6], b: &[f64; 6]) -> f64 {
    let b = wrap_near(arm, *b, a);
    (0..6).map(|i| (a[i] - b[i]).abs()).fold(0.0, f64::max)
}

fn clamp_q(arm: &Arm, q: &mut [f64; 6]) {
    for i in 0..6 {
        q[i] = q[i].clamp(arm[i].min(), arm[i].max());
    }
}

// ---------- 4. Schraubachsen (Produkt der Exponentiale) ----------
//
// Jedes Gelenk i ist eine Drehachse (Richtung ωᵢ, Punkt rᵢ) in der Nullstellung. Dann gilt
//   T(q) = e^{ξ₁q₁} · … · e^{ξ₆q₆} · M,   M = T(0),
// identisch zur DH-Kette, aber jede Drehung wirkt direkt im Weltsystem: Das macht die Geometrie sichtbar.

#[derive(Clone, Copy)]
struct Axis {
    w: V3,
    r: V3,
}

/// Drehung um eine Achse durch r: R·x + (r − R·r).
fn screw(a: &Axis, th: f64) -> M4 {
    let r = rodrigues(a.w, th);
    let t = sub(a.r, m3_apply(&r, a.r));
    let mut m = IDENT;
    for i in 0..3 {
        m[i][..3].copy_from_slice(&r[i]);
        m[i][3] = t[i];
    }
    m
}

/// Mittelpunkt der kürzesten Verbindung zweier Achsen (schneiden sie sich: der Schnittpunkt).
fn meet(a: &Axis, b: &Axis) -> V3 {
    let d = sub(b.r, a.r);
    let c = dot(a.w, b.w);
    let den = 1.0 - c * c;
    if den < 1e-12 {
        return a.r;
    }
    let t = (dot(d, a.w) - c * dot(d, b.w)) / den;
    let s = (c * dot(d, a.w) - dot(d, b.w)) / den;
    scale(add(add(a.r, scale(a.w, t)), add(b.r, scale(b.w, s))), 0.5)
}

/// Abstand eines Punktes von einer Achse.
fn axis_dist(a: &Axis, p: V3) -> f64 {
    norm(perp(a.w, sub(p, a.r)))
}

/// Geometrie des Arms in der Nullstellung.
struct Chain {
    ax: [Axis; 6],
    /// Werkzeugpose bei q = 0
    home: M4,
    /// Schulterpunkt: J1 ∩ J2
    s: V3,
    /// Handgelenkpunkt: J4 ∩ J5 (∩ J6)
    wc: V3,
    /// Erfüllt der Arm die Struktur exakt? Sonst liefert die geschlossene Form nur Startwerte.
    exact: bool,
}

fn chain(arm: &Arm) -> Chain {
    let f = fk_frames(arm, &[0.0; 6]);
    let ax: [Axis; 6] = std::array::from_fn(|i| Axis { w: col(&f[i], 2), r: pos(&f[i]) });
    let (s, wc) = (meet(&ax[0], &ax[1]), meet(&ax[3], &ax[4]));
    let exact = axis_dist(&ax[0], s) < 1e-6
        && axis_dist(&ax[1], s) < 1e-6
        && norm(cross(ax[1].w, ax[2].w)) < 1e-9
        && [3, 4, 5].iter().all(|&i| axis_dist(&ax[i], wc) < 1e-6);
    Chain { ax, home: f[6], s, wc, exact }
}

// ---------- 5. Paden-Kahan-Teilprobleme ----------

/// Teilproblem 1: Winkel θ, der p um die Achse auf q dreht (e^{ξθ}·p = q).
fn sp1(a: &Axis, p: V3, q: V3) -> f64 {
    let (u, v) = (perp(a.w, sub(p, a.r)), perp(a.w, sub(q, a.r)));
    dot(a.w, cross(u, v)).atan2(dot(u, v))
}

/// Teilproblem 2: e^{ξ₁θ₁}·e^{ξ₂θ₂}·p = q für zwei Achsen, die sich in c schneiden.
/// Der Zwischenpunkt z = e^{ξ₂θ₂}p = e^{−ξ₁θ₁}q hat bekannte Anteile entlang ω₁ und ω₂:
///   z − c = α·ω₁ + β·ω₂ ± γ·(ω₁ × ω₂).
/// Bis zu zwei Lösungen (θ₁, θ₂); das bool sagt, ob exakt (sonst die nächstgelegene Näherung).
fn sp2(a1: &Axis, a2: &Axis, c: V3, p: V3, q: V3) -> (Vec<(f64, f64)>, bool) {
    let (u, v) = (sub(p, c), sub(q, c));
    let k = dot(a1.w, a2.w);
    let den = k * k - 1.0;
    if den.abs() < 1e-12 {
        return (vec![], false);
    }
    let alpha = (k * dot(a2.w, u) - dot(a1.w, v)) / den;
    let beta = (k * dot(a1.w, v) - dot(a2.w, u)) / den;
    let x = cross(a1.w, a2.w);
    let g2 = (dot(u, u) - alpha * alpha - beta * beta - 2.0 * alpha * beta * k) / dot(x, x);
    let exact = g2 > -1e-6 * dot(u, u).max(1.0) && (norm(u) - norm(v)).abs() < 1e-6 * norm(u).max(1.0);
    let gs: &[f64] = if g2 > 1e-18 { &[1.0, -1.0] } else { &[0.0] };
    let g = g2.max(0.0).sqrt();
    let out = gs
        .iter()
        .map(|sg| {
            let z = add(c, add(add(scale(a1.w, alpha), scale(a2.w, beta)), scale(x, sg * g)));
            (sp1(a1, z, q), sp1(a2, p, z))
        })
        .collect();
    (out, exact)
}

/// Teilproblem 3: θ mit |e^{ξθ}·p − q| = δ. Mit u′, v′ = Anteile senkrecht zur Achse:
///   |u′|² + |v′|² − 2|u′||v′|·cos(θ₀ − θ) = δ² − (ω·(p − q))²,  θ₀ = Winkel von u′ nach v′.
/// Bis zu zwei Lösungen; ist δ nicht erreichbar, die nächstgelegene (bool = false).
fn sp3(a: &Axis, p: V3, q: V3, d: f64) -> (Vec<f64>, bool) {
    let (up, vp) = (perp(a.w, sub(p, a.r)), perp(a.w, sub(q, a.r)));
    let h = dot(a.w, sub(p, q));
    let d2 = d * d - h * h;
    let (nu, nv) = (norm(up), norm(vp));
    if nu * nv < 1e-12 {
        return (vec![0.0], (nu * nu + nv * nv - d2).abs() < 1e-6);
    }
    let th0 = dot(a.w, cross(up, vp)).atan2(dot(up, vp));
    let c = (nu * nu + nv * nv - d2) / (2.0 * nu * nv);
    let b = c.clamp(-1.0, 1.0).acos();
    let sols = if b < 1e-12 { vec![th0] } else { vec![th0 - b, th0 + b] };
    (sols, c.abs() <= 1.0 + 1e-9)
}

// ---------- 6. Analytische IK ----------

/// Kandidaten aus der geschlossenen Form. Für den idealen Arm exakt, sonst Startwerte für Newton.
struct Seed {
    q: [f64; 6],
    exact: bool,
}

/// Volle Pose (6D): Handgelenkpunkt → Ellbogen (TP3) → Schulter (TP2) → Handgelenk (TP2 + TP1).
fn seeds_pose(c: &Chain, t: &M4) -> Vec<Seed> {
    let g = mul(t, &inv(&c.home));
    let w = apply(&g, c.wc);
    let mut out = vec![];
    let (q3s, e3) = sp3(&c.ax[2], c.wc, c.s, norm(sub(w, c.s)));
    for q3 in q3s {
        let p = apply(&screw(&c.ax[2], q3), c.wc);
        let (q12, e12) = sp2(&c.ax[0], &c.ax[1], c.s, p, w);
        for (q1, q2) in q12 {
            let g3 = mul(&mul(&screw(&c.ax[0], q1), &screw(&c.ax[1], q2)), &screw(&c.ax[2], q3));
            // Handgelenk: e^{ξ₄q₄}·e^{ξ₅q₅}·e^{ξ₆q₆} = g3⁻¹ · T · M⁻¹
            let gw = mul(&inv(&g3), &g);
            let p6 = add(c.wc, c.ax[5].w);
            let (q45, e45) = sp2(&c.ax[3], &c.ax[4], c.wc, p6, apply(&gw, p6));
            for (q4, q5) in q45 {
                let g45 = mul(&screw(&c.ax[3], q4), &screw(&c.ax[4], q5));
                let pp = add(c.wc, any_perp(c.ax[5].w));
                let q6 = sp1(&c.ax[5], pp, apply(&mul(&inv(&g45), &gw), pp));
                out.push(Seed { q: [q1, q2, q3, q4, q5, q6], exact: c.exact && e3 && e12 && e45 });
            }
        }
    }
    out
}

fn any_perp(w: V3) -> V3 {
    let a = if w[0].abs() < 0.9 { [1.0, 0.0, 0.0] } else { [0.0, 1.0, 0.0] };
    unit(perp(w, a))
}

/// Abstand, den der Ellbogen zwischen Schulter und Handgelenk mindestens einstellen kann (Anschläge J3).
fn elbow_min(c: &Chain, arm: &Arm) -> f64 {
    let (lo, hi) = (arm[2].min(), arm[2].max());
    let d = |th: f64| norm(sub(apply(&screw(&c.ax[2], th), c.wc), c.s));
    let th0 = sp1(&c.ax[2], c.wc, c.s);
    let mut m = d(lo).min(d(hi));
    for k in -2..=2 {
        let th = th0 + k as f64 * TAU;
        if (lo..=hi).contains(&th) {
            m = m.min(d(th));
        }
    }
    m
}

/// Sicherheitsabstand zum Ellbogenanschlag (mm) und Breite des weichen Übergangs (mm).
const ELBOW_MARGIN: f64 = 5.0;
const BLEND: f64 = 15.0;

/// Weiches max(0, y): gleich y für y ≥ 2m, 0 für y ≤ −2m, dazwischen die Parabel (y + 2m)²/(8m).
/// Wert und Steigung sind überall stetig (C¹).
fn soft_ramp(y: f64, m: f64) -> f64 {
    if y >= 2.0 * m {
        y
    } else if y <= -2.0 * m {
        0.0
    } else {
        (y + 2.0 * m).powi(2) / (8.0 * m)
    }
}

/// Regel für den automatischen Werkzeugwinkel (ohne Vorgabe): Die Werkzeugachse zeigt vom Schulterpunkt S
/// zum Ziel P (gestreckte Haltung, maximale Reichweite), der Handgelenkpunkt liegt dann bei |W − S| = r − L.
/// Nah an der Schulter schafft der Ellbogen das nicht mehr; der Wunschabstand wird weich nach unten begrenzt:
///   d(r) = ρ + soft_ramp(r − L − ρ, m)   (ρ = kleinster Ellbogenabstand + Reserve)
/// und das Werkzeug knickt in der Armebene um ψ ab, sodass |W − S| = d:
///   d² = r² + L² − 2·r·L·cos ψ   ⇒   cos ψ = (r² + L² − d²) / (2·r·L).
/// Weil d glatt in r − L übergeht, setzt ψ mit endlicher Steigung ein (keine √-Spitze wie bei einer harten Grenze).
/// Gedreht wird um die J2-Achse der jeweiligen Schulterseite, so bleibt die Regel auch über der J1-Achse stetig.
/// Rückgabe: Werkzeugrichtung u.
fn auto_tool(e: V3, r: f64, l: f64, rho: f64, w2: V3) -> V3 {
    let d = rho + soft_ramp(r - l - rho, BLEND);
    let psi = if r < 1e-9 || l < 1e-9 || d <= r - l { 0.0 } else { ((r * r + l * l - d * d) / (2.0 * r * l)).clamp(-1.0, 1.0).acos() };
    m3_apply(&rodrigues(w2, psi), e)
}

/// Nur Position (3D): Werkzeugrichtung nach `auto_tool`, J6 = 0, J4 so nah an 0 wie möglich (Armebene).
/// Je Schulterseite (vorn/hinten) und Ellbogen (oben/unten) eine Lösung, also bis zu 4.
fn seeds_point(c: &Chain, arm: &Arm, p: V3) -> Vec<Seed> {
    let lv = sub(pos(&c.home), c.wc);
    let (l, lh) = (norm(lv), unit(lv));
    let ps = sub(p, c.s);
    let r = norm(ps);
    let (w1, w2h) = (c.ax[0].w, c.ax[1].w);
    // J1-Winkel, bei dem die Armebene P enthält: J2-Achse senkrecht zur Waagerechten von S nach P
    let horiz = perp(w1, ps);
    let q1f = if norm(horiz) < 1e-9 {
        0.0
    } else {
        let n = unit(cross(w1, horiz));
        dot(w1, cross(w2h, n)).atan2(dot(w2h, n))
    };
    let rho = elbow_min(c, arm) + ELBOW_MARGIN;
    let mut out = vec![];
    for q1c in [q1f, q1f + PI] {
        let w2 = m3_apply(&rodrigues(w1, q1c), w2h);
        let u = auto_tool(unit(ps), r, l, rho, w2);
        let w = sub(p, scale(u, l));
        let (q3s, e3) = sp3(&c.ax[2], c.wc, c.s, norm(sub(w, c.s)));
        for q3 in q3s {
            let pe = apply(&screw(&c.ax[2], q3), c.wc);
            let (q12, e12) = sp2(&c.ax[0], &c.ax[1], c.s, pe, w);
            let Some(&(q1, q2)) = q12.iter().min_by(|a, b| wrap_pi(a.0 - q1c).abs().total_cmp(&wrap_pi(b.0 - q1c).abs())) else { continue };
            let g3 = mul(&mul(&screw(&c.ax[0], q1), &screw(&c.ax[1], q2)), &screw(&c.ax[2], q3));
            let (mut q4, mut q5, mut e45) = (0.0, 0.0, true);
            if l > 1e-9 {
                // e^{ξ₄q₄}·e^{ξ₅q₅}·(W₀ + L̂) = g3⁻¹·(W + u)
                let (s45, e) = sp2(&c.ax[3], &c.ax[4], c.wc, add(c.wc, lh), apply(&inv(&g3), add(w, u)));
                if let Some(&(a, b)) = s45.iter().min_by(|a, b| wrap_pi(a.0).abs().total_cmp(&wrap_pi(b.0).abs())) {
                    (q4, q5, e45) = (a, b, e);
                }
            }
            out.push(Seed { q: [q1, q2, q3, q4, q5, 0.0], exact: c.exact && e3 && e12 && e45 });
        }
    }
    out
}

// ---------- 7. Jacobi-Matrix und gedämpftes Newton-Verfahren ----------

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

/// Ergebnis der IK: Winkel plus Güte der Lösung.
pub struct IkResult {
    pub q: [f64; 6],
    pub pos_err: f64,
    pub rot_err: f64,
    pub converged: bool,
}

/// Einstellungen: mit/ohne Winkelvorgabe, Toleranzen (mm, rad), Anzahl Neustarts für den Rückfall
/// (nur nötig, wenn die geschlossene Form keine Lösung hat, z. B. bei selbst geänderter Geometrie).
pub struct IkOptions {
    pub use_orientation: bool,
    pub pos_tol: f64,
    pub rot_tol: f64,
    pub restarts: usize,
}

/// Maximale Schrittweite eines Gelenks pro Iteration (rad): hält die Linearisierung gültig.
const MAX_STEP: f64 = 0.5;

/// Levenberg-Marquardt ab dem Startwert `q0`. Die Dämpfung λ sinkt nach jedem akzeptierten und
/// steigt nach jedem abgelehnten Schritt; angenommen wird nur, was den Gesamtfehler senkt.
/// Gelenkgrenzen werden durch Projektion (clamp) eingehalten. Dient zur Nachpolitur der geschlossenen
/// Form (bei idealer Geometrie 0 Iterationen) und als Rückfall.
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
        let mut jac = jacobian_of(&cur.frames);
        for row in &mut jac[3..] {
            for v in row.iter_mut() {
                *v *= L;
            }
        }
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

// ---------- 8. Öffentliche IK ----------

/// Interne Genauigkeit der Lösungen (mm, rad): weit unter jeder sichtbaren oder messbaren Größe.
const TIGHT_P: f64 = 1e-6;
const TIGHT_R: f64 = 1e-9;

fn seeds(arm: &Arm, t: &M4, orient: bool) -> Vec<Seed> {
    let c = chain(arm);
    if orient { seeds_pose(&c, t) } else { seeds_point(&c, arm, pos(t)) }
}

/// Alle Lösungen (Haltungen) für das Ziel, exakt und innerhalb der Gelenkgrenzen, in fester Reihenfolge.
/// Mit Winkel: Pose `t` vollständig (≤ 8). Ohne Winkel: nur die Position von `t`, Orientierung nach der
/// Auto-Regel (≤ 4). Vollkreis-Gelenke liegen in (−180°, 180°]; Kopien ±360° ergeben sich über `wrap_near`.
pub fn ik_all(arm: &Arm, t: &M4, orient: bool) -> Vec<[f64; 6]> {
    let o = IkOptions { use_orientation: orient, pos_tol: TIGHT_P, rot_tol: TIGHT_R, restarts: 0 };
    let mut out: Vec<[f64; 6]> = vec![];
    let exact_chain = chain(arm).exact;
    for s in seeds(arm, t, orient) {
        // Beim idealen Arm ist ein nicht exakter Zweig geometrisch unerreichbar (z. B. Ellbogen über gestreckt)
        if exact_chain && !s.exact {
            continue;
        }
        let Some(q) = fit_limits(arm, s.q) else { continue };
        let r = iterate(arm, t, q, &o);
        let Some(q) = fit_limits(arm, r.q) else { continue };
        if r.converged && out.iter().all(|x| joint_dist(arm, x, &q) > 1e-6) {
            out.push(q);
        }
    }
    out
}

/// Die Lösung aus `sols`, die `q_ref` am nächsten liegt (Vollkreis-Gelenke über den kürzeren Weg).
pub fn nearest(arm: &Arm, sols: &[[f64; 6]], q_ref: &[f64; 6]) -> Option<[f64; 6]> {
    sols.iter().map(|s| wrap_near(arm, *s, q_ref)).min_by(|a, b| joint_dist(arm, a, q_ref).total_cmp(&joint_dist(arm, b, q_ref)))
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

/// Löst die IK: alle exakten Haltungen bestimmen, die zur aktuellen Stellung `q_start` nächstgelegene nehmen
/// (kleinste Bewegung). Gibt es keine exakte Lösung (außer Reichweite, Anschläge), liefert das gedämpfte
/// Newton-Verfahren die bestmögliche Näherung: der Arm streckt sich so weit wie möglich zum Ziel.
pub fn solve_ik(arm: &Arm, target: &M4, q_start: [f64; 6], o: &IkOptions) -> IkResult {
    let ori = o.use_orientation;
    if let Some(q) = nearest(arm, &ik_all(arm, target, ori), &q_start) {
        let e = Eval::new(arm, &q, target, ori);
        return IkResult { q, pos_err: e.pe, rot_err: e.re, converged: e.pe < o.pos_tol && e.re < o.rot_tol };
    }
    // Rückfall: Newton ab der aktuellen Stellung, ab den genäherten geschlossenen Lösungen und ab Zufallswerten
    let mut starts = vec![q_start];
    starts.extend(seeds(arm, target, ori).into_iter().map(|s| wrap_near(arm, s.q, &q_start)));
    let mut rng = Rng(0x9E37_79B9_7F4A_7C15);
    starts.extend((0..o.restarts).map(|_| std::array::from_fn(|i| {
        let (lo, hi) = (arm[i].min().max(-PI), arm[i].max().min(PI));
        lo + rng.next() * (hi - lo)
    })));
    let mut best: Option<IkResult> = None;
    for s in starts {
        let r = iterate(arm, target, s, o);
        let better = match &best {
            None => true,
            Some(b) if r.converged != b.converged => r.converged,
            Some(b) if r.converged => joint_dist(arm, &r.q, &q_start) < joint_dist(arm, &b.q, &q_start),
            Some(b) => r.pos_err + L * r.rot_err < b.pos_err + L * b.rot_err,
        };
        if better {
            best = Some(r);
        }
    }
    let mut r = best.unwrap();
    r.q = wrap_near(arm, r.q, &q_start);
    r
}

// ---------- 9. Arbeitsraum ----------

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

    fn default_arm() -> Arm {
        let j = |a, al, d, off| Joint { a, alpha_deg: al, d, offset_deg: off, min_deg: -170.0, max_deg: 170.0 };
        [j(0.0, 90.0, 100.0, 0.0), j(120.0, 0.0, 0.0, 90.0), j(0.0, 90.0, 0.0, 90.0),
         j(0.0, -90.0, 120.0, 0.0), j(0.0, 90.0, 0.0, 0.0), j(0.0, 0.0, 60.0, 0.0)]
    }

    struct Lcg(u64);
    impl Lcg {
        /// gleichverteilt in [−1, 1)
        fn next(&mut self) -> f64 {
            self.0 = self.0.wrapping_mul(6364136223846793005).wrapping_add(1442695040888963407);
            ((self.0 >> 33) as f64 / (1u64 << 31) as f64) * 2.0 - 1.0
        }
    }

    fn pose_err(arm: &Arm, q: &[f64; 6], t: &M4) -> (f64, f64) {
        let f = fk(arm, q);
        (norm(sub(pos(&f), pos(t))), norm(rot_error(t, &f)))
    }

    #[test]
    fn fk_zero_pose_points_up() {
        let p = pos(&fk(&default_arm(), &[0.0; 6]));
        for i in 0..3 {
            assert!((p[i] - [0.0, 0.0, 400.0][i]).abs() < 1e-9, "{p:?}");
        }
        // Idealer Elfin: gerade nach oben, Werkzeug exakt senkrecht (keine 2,5°-Verdrehung mehr)
        let t = fk(&cobot_arm(), &[0.0; 6]);
        assert!(norm(sub(pos(&t), [0.0, 0.0, 716.395])) < 1e-9, "{:?}", pos(&t));
        assert!(norm(sub(col(&t, 2), [0.0, 0.0, 1.0])) < 1e-12, "Werkzeugachse nicht senkrecht");
    }

    #[test]
    fn cobot_has_exact_spherical_wrist_and_shoulder() {
        assert!(chain(&cobot_arm()).exact);
        assert!(chain(&default_arm()).exact);
        assert!(!chain(&arm_from(&MODEL_DH, [720.0; 6])).exact);
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
        let a = [1.0 / 3.0, 2.0 / 3.0, 2.0 / 3.0];
        for ang in [3.0f64, 3.1, 3.14, PI - 1e-5] {
            let e = rot_error(&rot_exp(scale(a, ang)), &IDENT);
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
        assert!((joint_dist(&arm, &[d(359.0), 0.0, 0.0, 0.0, 0.0, 0.0], &[d(1.0), 0.0, 0.0, 0.0, 0.0, 0.0]) - d(2.0)).abs() < 1e-12);
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
    fn analytic_ik_finds_every_posture_exactly() {
        // Zufällige Stellungen: die geschlossene Form muss genau diese Stellung unter ihren Lösungen haben,
        // und jede gefundene Lösung muss die Pose auf 1e-6 mm / 1e-9 rad treffen.
        for arm in [cobot_arm(), default_arm()] {
            let mut rnd = Lcg(12345);
            let mut counts = [0usize; 9];
            for _ in 0..300 {
                let q: [f64; 6] = std::array::from_fn(|i| rnd.next() * arm[i].max().min(PI) * 0.95);
                let t = fk(&arm, &q);
                let sols = ik_all(&arm, &t, true);
                counts[sols.len()] += 1;
                for s in &sols {
                    let (ep, er) = pose_err(&arm, s, &t);
                    assert!(ep < 1e-6 && er < 1e-9, "Lösung ungenau: {ep} mm, {er} rad");
                }
                assert!(sols.iter().any(|s| joint_dist(&arm, s, &q) < 1e-6), "Stellung {q:?} fehlt unter {} Lösungen", sols.len());
            }
            println!("Anzahl Haltungen: {counts:?}");
        }
    }

    #[test]
    fn free_mode_depends_only_on_position() {
        // Ohne Winkel: dieselbe Position ergibt je Haltung exakt dieselbe Stellung, egal wo der Arm vorher war.
        let arm = cobot_arm();
        let opt = IkOptions { use_orientation: false, pos_tol: 1e-3, rot_tol: 1e-5, restarts: 0 };
        let mut rnd = Lcg(77);
        let mut n = 0;
        for _ in 0..200 {
            let p = [rnd.next() * 500.0, rnd.next() * 500.0, 300.0 + rnd.next() * 300.0];
            let t = from_xyz_rpy(p, [0.0; 3]);
            let sols = ik_all(&arm, &t, false);
            for s in &sols {
                assert!(norm(sub(pos(&fk(&arm, s)), p)) < 1e-6);
                assert!(s[3].abs() < 1e-9 && s[5].abs() < 1e-9, "J4/J6 nicht 0: {s:?}");
                let near: [f64; 6] = std::array::from_fn(|i| s[i] + rnd.next() * 0.1);
                let r = solve_ik(&arm, &t, near, &opt);
                assert!(r.converged && joint_dist(&arm, &r.q, s) < 1e-9);
                n += 1;
            }
        }
        assert!(n > 200, "zu wenige Lösungen: {n}");
    }

    /// Fährt die Gerade a→b in 1-mm-Schritten mit der jeweils nächstgelegenen Lösung; größter Gelenkschritt (rad).
    fn worst_step(arm: &Arm, a: V3, b: V3) -> f64 {
        let opt = IkOptions { use_orientation: false, pos_tol: 1e-3, rot_tol: 1e-5, restarts: 0 };
        let n = norm(sub(b, a)).ceil() as usize;
        let at = |k: usize| from_xyz_rpy(add(a, scale(sub(b, a), k as f64 / n as f64)), [0.0; 3]);
        let mut q = nearest(arm, &ik_all(arm, &at(0), false), &[0.0; 6]).unwrap();
        let mut worst = 0.0f64;
        for k in 1..=n {
            let r = solve_ik(arm, &at(k), q, &opt);
            assert!(r.converged, "nicht lösbar bei {:?}", pos(&at(k)));
            worst = worst.max(joint_dist(arm, &r.q, &q));
            q = r.q;
        }
        worst
    }

    #[test]
    fn auto_tool_is_continuous_close_to_the_shoulder() {
        let arm = cobot_arm();
        // Einsatz des Werkzeugknicks ψ nah an der Schulter: weich, endliche Steigung (kein √-Sprung)
        let w = worst_step(&arm, [300.0, 0.0, 250.0], [60.0, 0.0, 250.0]);
        assert!(w < 2f64.to_radians(), "Knick-Einsatz: {:.2}°/mm", w.to_degrees());
        // Quer über die J1-Achse: die Haltung geht stetig auf die andere Seite über, J1 bleibt stehen
        let w = worst_step(&arm, [250.0, 30.0, 500.0], [-250.0, 30.0, 500.0]);
        assert!(w < 2f64.to_radians(), "über die J1-Achse: {:.2}°/mm", w.to_degrees());
    }

    #[test]
    fn ik_unreachable_reports_failure() {
        let arm = default_arm();
        let t = from_xyz_rpy([2000.0, 0.0, 0.0], [0.0; 3]);
        let opt = IkOptions { use_orientation: false, pos_tol: 1e-3, rot_tol: 1e-3, restarts: 5 };
        let r = solve_ik(&arm, &t, [0.0; 6], &opt);
        assert!(!r.converged);
        // bestmögliche Näherung: gestreckt Richtung Ziel
        let gap = (2000f64.powi(2) + 100f64.powi(2)).sqrt() - 300.0;
        assert!((r.pos_err - gap).abs() < 1.0, "{} statt {gap}", r.pos_err);
    }

    #[test]
    fn edited_geometry_still_solves_via_newton() {
        // Gemessene (schiefe) Geometrie: geschlossene Form nur als Startwert, Newton macht es exakt.
        let arm = arm_from(&MODEL_DH, [720.0, 120.0, 150.0, 720.0, 120.0, 720.0]);
        let mut rnd = Lcg(5);
        for _ in 0..50 {
            let q: [f64; 6] = std::array::from_fn(|i| rnd.next() * arm[i].max().min(PI) * 0.8);
            let t = fk(&arm, &q);
            let sols = ik_all(&arm, &t, true);
            assert!(!sols.is_empty());
            for s in &sols {
                let (ep, er) = pose_err(&arm, s, &t);
                assert!(ep < 1e-6 && er < 1e-9);
            }
        }
    }
}
