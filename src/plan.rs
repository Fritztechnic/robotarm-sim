//! Playlist-Planer: berechnet VOR dem Start für jede Strecke die komplette Gelenk-Trajektorie.
//! Das Ergebnis hängt nur von Armgeometrie und Punkten ab – nie von Framerate, Geschwindigkeit
//! oder Runde. Dieselbe Strecke fährt also immer exakt denselben Weg.
//!
//! Verfahren pro Strecke A→B (nur Position, die 3 überzähligen Freiheitsgrade bleiben frei):
//!   1. Gerade in Schritten von `STEP` mm unterteilen, p(s) = A + s·(B−A)/|B−A|.
//!   2. Jeder Punkt per gedämpftem Newton (Warmstart vom vorigen) + Nullraum-Zug lösen.
//!   3. Schlägt ein Schritt fehl oder springt ein Gelenk (Singularität/Zweigwechsel),
//!      wird die Strecke als Gelenkfahrt (PTP) geplant: lineare Interpolation im Gelenkraum.
//!   4. Der Schleifenschluss (letzter→erster Punkt) endet exakt in der Startstellung; passt
//!      die Gerade dort nicht hin, wird auch er zur Gelenkfahrt. So ist jede Runde identisch.

use crate::kin::{self, Arm, IkOptions, V3};

pub const STEP: f64 = 2.0;
/// Größter erlaubter Gelenksprung pro Schritt (rad), sonst gilt die Gerade als nicht fahrbar.
const MAX_JUMP: f64 = 0.25;
pub const READY: [f64; 6] = [0.0, -35.0, 70.0, 0.0, 55.0, 0.0];

pub struct Seg {
    pub from: usize,
    pub to: usize,
    /// Gelenkstellungen (rad) von Anfang bis Ende der Strecke
    pub q: Vec<[f64; 6]>,
    /// true = Gelenkfahrt statt gerader Linie
    pub ptp: bool,
    /// true = Schleifenschluss (nur bei aktiver Schleife gefahren)
    pub closing: bool,
}

#[derive(Default)]
pub struct Plan {
    pub pts: Vec<V3>,
    pub first: usize,
    pub start: [f64; 6],
    pub segs: Vec<Seg>,
    pub unreachable: Vec<usize>,
}

fn opt(restarts: usize) -> IkOptions {
    IkOptions { use_orientation: false, pos_tol: 1e-3, rot_tol: 1e-5, restarts }
}

fn target(p: V3) -> kin::M4 {
    kin::from_xyz_rpy(p, [0.0; 3])
}

fn dq_max(a: &[f64; 6], b: &[f64; 6]) -> f64 {
    (0..6).map(|i| (a[i] - b[i]).abs()).fold(0.0, f64::max)
}

/// Gerade von `a` nach `b`, Start bei Stellung `q0`. None = nicht durchgängig fahrbar.
fn march(arm: &Arm, q0: [f64; 6], a: V3, b: V3) -> Option<Vec<[f64; 6]>> {
    let d: V3 = std::array::from_fn(|k| b[k] - a[k]);
    let len = d.iter().map(|v| v * v).sum::<f64>().sqrt();
    let n = (len / STEP).ceil().max(1.0) as usize;
    let mut out = vec![q0];
    let mut q = q0;
    for i in 1..=n {
        let p: V3 = std::array::from_fn(|k| a[k] + d[k] * i as f64 / n as f64);
        let t = target(p);
        let r = kin::solve_ik(arm, &t, q, &opt(0));
        if !r.converged {
            return None;
        }
        let r = kin::polish(arm, &t, r, &opt(0), 2);
        if !r.converged || dq_max(&r.q, &q) > MAX_JUMP {
            return None;
        }
        q = r.q;
        out.push(q);
    }
    Some(out)
}

/// Gelenkfahrt: lineare Interpolation (feiner als 0,5° pro Schritt).
fn lerp_path(a: [f64; 6], b: [f64; 6]) -> Vec<[f64; 6]> {
    let n = (dq_max(&a, &b) / 0.5f64.to_radians()).ceil().max(1.0) as usize;
    (0..=n).map(|i| std::array::from_fn(|k| a[k] + (b[k] - a[k]) * i as f64 / n as f64)).collect()
}

/// Lösung für Punkt `p`, die der Stellung `q` am nächsten liegt.
fn nearest(arm: &Arm, p: V3, q: [f64; 6]) -> Option<[f64; 6]> {
    let t = target(p);
    let r = kin::solve_ik(arm, &t, q, &opt(0));
    let r = if r.converged { r } else { kin::solve_ik(arm, &t, q, &opt(20)) };
    r.converged.then_some(r.q)
}

pub fn build(arm: &Arm, pts: &[V3]) -> Plan {
    let mut plan = Plan { pts: pts.to_vec(), ..Plan::default() };
    let ready = READY.map(f64::to_radians);
    let mut q = ready;
    let mut order: Vec<usize> = vec![];
    let mut sol: Vec<[f64; 6]> = vec![];
    // Erreichbarkeit: jeder Punkt wird von der Bereit-Pose und vom vorigen Punkt aus versucht
    for (i, &p) in pts.iter().enumerate() {
        match nearest(arm, p, q).or_else(|| nearest(arm, p, ready)) {
            Some(s) => {
                q = s;
                order.push(i);
                sol.push(s);
            }
            None => plan.unreachable.push(i),
        }
    }
    let Some(&first) = order.first() else { return plan };
    plan.first = first;
    plan.start = sol[0];
    let mut cur = sol[0];
    for k in 0..order.len() {
        let closing = k + 1 == order.len();
        if closing && order.len() < 2 {
            break;
        }
        let (a, b) = (order[k], order[(k + 1) % order.len()]);
        let goal = if closing { Some(plan.start) } else { None };
        let line = march(arm, cur, pts[a], pts[b]).filter(|l| goal.is_none_or(|g| dq_max(l.last().unwrap(), &g) < 0.02));
        let (q, ptp) = match line {
            Some(mut l) => {
                if let Some(g) = goal {
                    *l.last_mut().unwrap() = g;
                }
                (l, false)
            }
            None => {
                let end = goal.or_else(|| nearest(arm, pts[b], cur)).unwrap_or(cur);
                (lerp_path(cur, end), true)
            }
        };
        cur = *q.last().unwrap();
        plan.segs.push(Seg { from: a, to: b, q, ptp, closing });
    }
    plan
}

/// Zeit (s) für einen Schritt zwischen zwei Samples: je nachdem, was langsamer ist –
/// TCP-Geschwindigkeit (nur bei Geraden) oder maximale Gelenkgeschwindigkeit.
pub fn step_time(arm: &Arm, a: &[f64; 6], b: &[f64; 6], ptp: bool, tcp_speed: f64, speed_dps: f64) -> f64 {
    let tj = dq_max(a, b) / speed_dps.to_radians().max(0.05);
    if ptp {
        return tj;
    }
    let (pa, pb) = (kin::pos(&kin::fk(arm, a)), kin::pos(&kin::fk(arm, b)));
    let mm = (0..3).map(|k| (pa[k] - pb[k]).powi(2)).sum::<f64>().sqrt();
    tj.max(mm / tcp_speed.max(1.0))
}

/// Läuft `dt` Sekunden auf der Strecke weiter. Rückgabe: neuer Index `u` (Bruchteil = Position
/// zwischen zwei Samples) und true, wenn das Ende erreicht ist.
pub fn advance(arm: &Arm, seg: &Seg, mut u: f64, mut dt: f64, tcp_speed: f64, speed_dps: f64) -> (f64, bool) {
    let last = (seg.q.len() - 1) as f64;
    while dt > 0.0 {
        let i = u.floor() as usize;
        if i as f64 >= last {
            return (last, true);
        }
        let full = step_time(arm, &seg.q[i], &seg.q[i + 1], seg.ptp, tcp_speed, speed_dps);
        let left = full * (i as f64 + 1.0 - u);
        if left <= dt {
            dt -= left;
            u = i as f64 + 1.0;
        } else {
            u += dt / full;
            dt = 0.0;
        }
    }
    (u, u >= last)
}

/// Gelenkstellung an der Stelle `u` der Strecke.
pub fn sample(seg: &Seg, u: f64) -> [f64; 6] {
    let i = (u.floor() as usize).min(seg.q.len() - 1);
    let j = (i + 1).min(seg.q.len() - 1);
    let f = u - i as f64;
    std::array::from_fn(|k| seg.q[i][k] + (seg.q[j][k] - seg.q[i][k]) * f)
}
