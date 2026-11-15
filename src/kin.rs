//! Kinematik: Vorwärtskinematik (Denavit-Hartenberg, Standard-Konvention) und
//! numerische inverse Kinematik (gedämpfte Pseudoinverse / Levenberg-Marquardt).
//! Längen in mm, Winkel intern in rad.

use serde::{Deserialize, Serialize};

// Ein Vektor im Raum: x, y, z in mm. Mehr braucht man nicht xD
pub type V3 = [f64; 3];
// 4x4-Matrix = Drehung + Verschiebung in einem Rutsch (Mathe-Zauberkasten)
pub type M4 = [[f64; 4]; 4];

pub const IDENT: M4 = [
    [1.0, 0.0, 0.0, 0.0],
    [0.0, 1.0, 0.0, 0.0],
    [0.0, 0.0, 1.0, 0.0],
    [0.0, 0.0, 0.0, 1.0],
];

// Matrix mal Matrix. Reihenfolge ist wichtig, sonst steht der Arm Kopf (lol)
pub fn mul(a: &M4, b: &M4) -> M4 {
    let mut r = [[0.0; 4]; 4];
    for i in 0..4 {
        for j in 0..4 {
            r[i][j] = (0..4).map(|k| a[i][k] * b[k][j]).sum();
        }
    }
    r
}

// Position aus der Matrix fischen (letzte Spalte)
pub fn pos(m: &M4) -> V3 {
    [m[0][3], m[1][3], m[2][3]]
}

// Eine Achse (x=0, y=1, z=2) des Koordinatensystems rausholen
pub fn col(m: &M4, c: usize) -> V3 {
    [m[0][c], m[1][c], m[2][c]]
}

/// Standard-DH: Rz(theta) · Tz(d) · Tx(a) · Rx(alpha)
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

/// Inverse zu `from_xyz_rpy` (rad). Im Gimbal-Lock wird roll = 0 gesetzt.
pub fn to_rpy(m: &M4) -> V3 {
    let sp = -m[2][0];
    let pitch = sp.clamp(-1.0, 1.0).asin();
    if sp.abs() > 0.999_999 {
        let yaw = (-m[0][1]).atan2(m[1][1]);
        [0.0, pitch, yaw]
    } else {
        [m[2][1].atan2(m[2][2]), pitch, m[1][0].atan2(m[0][0])]
    }
}

#[derive(Clone, Copy, Serialize, Deserialize)]
pub struct Joint {
    pub a: f64,
    pub alpha_deg: f64,
    pub d: f64,
    pub offset_deg: f64,
    pub min_deg: f64,
    pub max_deg: f64,
}

pub type Arm = [Joint; 6];

/// Alle Gelenk-Frames: Index 0 = Basis, Index i = Frame nach Gelenk i.
pub fn fk_frames(arm: &Arm, q: &[f64; 6]) -> [M4; 7] {
    let mut f = [IDENT; 7];
    for i in 0..6 {
        let j = &arm[i];
        let t = dh(q[i] + j.offset_deg.to_radians(), j.d, j.a, j.alpha_deg.to_radians());
        f[i + 1] = mul(&f[i], &t);
    }
    f
}

/// Elfin-Cobot (aus dem importierten Modell abgeleitet): DH-Werte und Gesamt-θ der Modell-Ruhepose.
pub fn cobot_arm() -> Arm {
    // Anschläge: schützen vor Kollisionen des Arms mit sich selbst (im Aufbau-Reiter änderbar)
    let j = |a, al, d, off, lim: f64| Joint { a, alpha_deg: al, d, offset_deg: off, min_deg: -lim, max_deg: lim };
    [
        j(0.278, -90.0, 131.673, 0.0, 175.0),
        j(240.342, 0.0, 0.0, -90.0, 120.0),
        j(0.757, 89.157, -1.113, 90.0, 150.0),
        j(3.826, -93.323, 261.045, 180.0, 175.0),
        j(0.133, 91.621, 3.769, 0.0, 120.0),
        j(0.0, 0.0, 83.335, 0.0, 175.0),
    ]
}

// So stand der Arm im Modell, als du ihn runtergeladen hast. Daraus wird das 3D-Bild 'geeicht'
pub const COBOT_REST_THETA_DEG: [f64; 6] = [-57.797, -83.942, -156.059, -175.644, -73.088, 0.0];

/// Inverse einer starren Transformation.
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

// Vorwärts: Motorwinkel rein -> wo ist die Werkzeugspitze? (letztes Frame)
pub fn fk(arm: &Arm, q: &[f64; 6]) -> M4 {
    fk_frames(arm, q)[6]
}

/// Rotationsvektor (Achse·Winkel) der Rotation R_t · R_cᵀ.
fn rot_error(t: &M4, c: &M4) -> V3 {
    let mut re = [[0.0; 3]; 3];
    for i in 0..3 {
        for j in 0..3 {
            re[i][j] = (0..3).map(|k| t[i][k] * c[j][k]).sum();
        }
    }
    let v = [
        (re[2][1] - re[1][2]) / 2.0,
        (re[0][2] - re[2][0]) / 2.0,
        (re[1][0] - re[0][1]) / 2.0,
    ];
    let s = (v[0] * v[0] + v[1] * v[1] + v[2] * v[2]).sqrt();
    let cos = (re[0][0] + re[1][1] + re[2][2] - 1.0) / 2.0;
    let angle = s.atan2(cos);
    if s > 1e-9 {
        let k = angle / s;
        [v[0] * k, v[1] * k, v[2] * k]
    } else if cos > 0.0 {
        [0.0; 3]
    } else {
        // 180°: Achse aus der Diagonale
        let mut ax = [0.0; 3];
        let i = (0..3).max_by(|&a, &b| re[a][a].total_cmp(&re[b][b])).unwrap();
        ax[i] = ((re[i][i] + 1.0) / 2.0).max(0.0).sqrt();
        for j in 0..3 {
            if j != i {
                ax[j] = re[i][j] / (2.0 * ax[i]);
            }
        }
        [ax[0] * angle, ax[1] * angle, ax[2] * angle]
    }
}

// Kleines Gleichungssystem 6x6 lösen (Gauss mit Pivot). Das Herz vom IK-Schritt
fn solve6(mut a: [[f64; 7]; 6]) -> Option<[f64; 6]> {
    for c in 0..6 {
        let p = (c..6).max_by(|&i, &j| a[i][c].abs().total_cmp(&a[j][c].abs()))?;
        if a[p][c].abs() < 1e-14 {
            return None;
        }
        a.swap(c, p);
        for r in 0..6 {
            if r != c {
                let f = a[r][c] / a[c][c];
                for k in c..7 {
                    a[r][k] -= f * a[c][k];
                }
            }
        }
    }
    let mut x = [0.0; 6];
    for i in 0..6 {
        x[i] = a[i][6] / a[i][i];
    }
    Some(x)
}

// Was der Solver zurückgibt: Winkel + wie gut er getroffen hat
pub struct IkResult {
    pub q: [f64; 6],
    pub pos_err: f64,
    pub rot_err: f64,
    pub converged: bool,
}

// Einstellungen: mit/ohne Werkzeugwinkel, Toleranzen, wie oft er neu würfeln darf
pub struct IkOptions {
    pub use_orientation: bool,
    pub pos_tol: f64,
    pub rot_tol: f64,
    pub restarts: usize,
}

// Wie weit weg ist die Spitze noch vom Ziel? (Position in mm, Drehung in rad)
fn errors(arm: &Arm, q: &[f64; 6], target: &M4, orient: bool) -> ([f64; 6], f64, f64) {
    let cur = fk(arm, q);
    let (pc, pt) = (pos(&cur), pos(target));
    let dp = [pt[0] - pc[0], pt[1] - pc[1], pt[2] - pc[2]];
    let dr = if orient { rot_error(target, &cur) } else { [0.0; 3] };
    let pe = (dp[0] * dp[0] + dp[1] * dp[1] + dp[2] * dp[2]).sqrt();
    let re = (dr[0] * dr[0] + dr[1] * dr[1] + dr[2] * dr[2]).sqrt();
    ([dp[0], dp[1], dp[2], dr[0], dr[1], dr[2]], pe, re)
}

// Motoren nicht über ihre Grenzen prügeln
fn clamp_q(arm: &Arm, q: &mut [f64; 6]) {
    for i in 0..6 {
        q[i] = q[i].clamp(arm[i].min_deg.to_radians(), arm[i].max_deg.to_radians());
    }
}

/// Gedämpftes Newton-Verfahren (Levenberg-Marquardt) ab Startwert `q0`.
fn iterate(arm: &Arm, target: &M4, q0: [f64; 6], o: &IkOptions) -> IkResult {
    // Rotationsfehler (rad) wird mit dieser Länge (mm) gewichtet, damit beide Anteile vergleichbar sind.
    const L: f64 = 100.0;
    let mut q = q0;
    clamp_q(arm, &mut q);
    let mut lambda = 1.0;
    let (mut e, mut pe, mut re) = errors(arm, &q, target, o.use_orientation);
    let mut it = 0;
    while it < 300 {
        if pe < o.pos_tol && re < o.rot_tol {
            break;
        }
        it += 1;
        // Jacobi-Matrix per zentraler Differenz
        let mut jac = [[0.0; 6]; 6];
        let h = 1e-6;
        for j in 0..6 {
            let (mut qa, mut qb) = (q, q);
            qa[j] += h;
            qb[j] -= h;
            let (fa, fb) = (fk(arm, &qa), fk(arm, &qb));
            let (pa, pb) = (pos(&fa), pos(&fb));
            for k in 0..3 {
                jac[k][j] = (pa[k] - pb[k]) / (2.0 * h);
            }
            if o.use_orientation {
                // d(rot)/dq aus R(q+h)·R(q-h)ᵀ
                let r = rot_error(&fa, &fb);
                for k in 0..3 {
                    jac[3 + k][j] = L * r[k] / (2.0 * h);
                }
            }
        }
        let mut ew = e;
        for k in 3..6 {
            ew[k] *= L;
        }
        // (J·Jᵀ + λ²I)·y = e  →  dq = Jᵀ·y
        let try_step = |lam: f64| -> Option<[f64; 6]> {
            let mut a = [[0.0; 7]; 6];
            for i in 0..6 {
                for j in 0..6 {
                    a[i][j] = (0..6).map(|k| jac[i][k] * jac[j][k]).sum();
                }
                a[i][i] += lam * lam;
                a[i][6] = ew[i];
            }
            let y = solve6(a)?;
            let mut dq = [0.0; 6];
            for j in 0..6 {
                dq[j] = (0..6).map(|i| jac[i][j] * y[i]).sum();
            }
            let m = dq.iter().fold(0.0f64, |m, v| m.max(v.abs()));
            if m > 0.5 {
                for v in dq.iter_mut() {
                    *v *= 0.5 / m;
                }
            }
            Some(dq)
        };
        let mut improved = false;
        for _ in 0..12 {
            if let Some(dq) = try_step(lambda) {
                let mut qn = q;
                for j in 0..6 {
                    qn[j] += dq[j];
                }
                clamp_q(arm, &mut qn);
                let (en, pn, rn) = errors(arm, &qn, target, o.use_orientation);
                if pn + L * rn < pe + L * re {
                    q = qn;
                    e = en;
                    pe = pn;
                    re = rn;
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
    IkResult { q, pos_err: pe, rot_err: re, converged: pe < o.pos_tol && re < o.rot_tol }
}

/// Löst die IK. Erster Versuch startet am aktuellen Winkelsatz (kleinste Bewegung),
/// danach deterministische Zufalls-Neustarts innerhalb der Gelenkgrenzen.
pub fn solve_ik(arm: &Arm, target: &M4, q_start: [f64; 6], o: &IkOptions) -> IkResult {
    let mut r = search(arm, target, q_start, o);
    if r.converged && !o.use_orientation && o.restarts > 0 {
        r = polish(arm, target, r, o, 25);
    }
    r
}

// Sucht eine Lösung; klappt der erste Versuch nicht, würfelt er neue Startposen
fn search(arm: &Arm, target: &M4, q_start: [f64; 6], o: &IkOptions) -> IkResult {
    let mut best = iterate(arm, target, q_start, o);
    let mut seed = 0x9E37_79B9_7F4A_7C15u64;
    let mut rnd = move || {
        seed ^= seed << 13;
        seed ^= seed >> 7;
        seed ^= seed << 17;
        (seed >> 11) as f64 / (1u64 << 53) as f64
    };
    let mut tries = 0;
    while !best.converged && tries < o.restarts {
        tries += 1;
        let mut q = [0.0; 6];
        for i in 0..6 {
            let (lo, hi) = (arm[i].min_deg.to_radians(), arm[i].max_deg.to_radians());
            q[i] = lo + rnd() * (hi - lo);
        }
        let r = iterate(arm, target, q, o);
        if r.converged || r.pos_err + 100.0 * r.rot_err < best.pos_err + 100.0 * best.rot_err {
            best = r;
        }
    }
    best
}


// 3x3 invertieren, brauchen wir nur im Polish
fn inv3(m: [[f64; 3]; 3]) -> Option<[[f64; 3]; 3]> {
    let c = |i: usize, j: usize| m[(i + 1) % 3][(j + 1) % 3] * m[(i + 2) % 3][(j + 2) % 3] - m[(i + 1) % 3][(j + 2) % 3] * m[(i + 2) % 3][(j + 1) % 3];
    let det = m[0][0] * c(0, 0) + m[0][1] * c(0, 1) + m[0][2] * c(0, 2);
    if det.abs() < 1e-12 {
        return None;
    }
    Some(std::array::from_fn(|i| std::array::from_fn(|j| c(j, i) / det)))
}

/// Nur Position vorgegeben: der Arm hat 3 überzählige Freiheitsgrade. Im Nullraum der
/// Positions-Jacobi-Matrix werden die Handgelenke weg von Anschlägen und Singularitäten
/// zur Komfortstellung gezogen, ohne die Zielposition zu verlassen.
pub fn polish(arm: &Arm, target: &M4, mut r: IkResult, o: &IkOptions, iters: usize) -> IkResult {
    const W: [f64; 6] = [0.0, 0.1, 0.1, 1.0, 0.3, 1.0];
    for _ in 0..iters {
        let mut jac = [[0.0; 6]; 3];
        for j in 0..6 {
            let (mut a, mut b) = (r.q, r.q);
            a[j] += 1e-6;
            b[j] -= 1e-6;
            let (pa, pb) = (pos(&fk(arm, &a)), pos(&fk(arm, &b)));
            for k in 0..3 {
                jac[k][j] = (pa[k] - pb[k]) / 2e-6;
            }
        }
        let mut jjt = [[0.0; 3]; 3];
        for i in 0..3 {
            for j in 0..3 {
                jjt[i][j] = (0..6).map(|k| jac[i][k] * jac[j][k]).sum::<f64>() + if i == j { 1e-3 } else { 0.0 };
            }
        }
        let Some(inv) = inv3(jjt) else { break };
        let g: [f64; 6] = std::array::from_fn(|i| -W[i] * r.q[i]);
        // dq = (I − Jᵀ(JJᵀ)⁻¹J)·g
        let jg: [f64; 3] = std::array::from_fn(|i| (0..6).map(|k| jac[i][k] * g[k]).sum());
        let y: [f64; 3] = std::array::from_fn(|i| (0..3).map(|k| inv[i][k] * jg[k]).sum());
        let mut dq: [f64; 6] = std::array::from_fn(|j| g[j] - (0..3).map(|i| jac[i][j] * y[i]).sum::<f64>());
        let m = dq.iter().fold(0.0f64, |m, v| m.max(v.abs()));
        if m < 1e-5 {
            break;
        }
        let k = 0.03 / m.max(0.03);
        for v in dq.iter_mut() {
            *v *= k * 0.5;
        }
        let q: [f64; 6] = std::array::from_fn(|i| r.q[i] + dq[i]);
        let n = iterate(arm, target, q, o);
        if !n.converged {
            break;
        }
        r = n;
    }
    r
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
