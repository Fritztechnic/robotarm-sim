//! Playlist-Planer: berechnet VOR dem Start für jede Strecke die komplette Gelenk-Trajektorie.
//! Das Ergebnis hängt nur von Armgeometrie und Punkten ab – nie von Framerate, Geschwindigkeit
//! oder Runde. Dieselbe Strecke fährt also immer exakt denselben Weg.
//!
//! Verfahren:
//!   1. An jedem Punkt alle exakten IK-Lösungen (Haltungen) bestimmen: mit Werkzeugwinkel ≤ 8,
//!      ohne ≤ 4 (Werkzeugrichtung nach der festen Auto-Regel, siehe `kin`).
//!   2. Jede Gerade A→B aus jeder Haltung an A verfolgen: Position linear, Orientierung per Slerp,
//!      Schritte von höchstens `STEP` mm bzw. `ROT_STEP`; je Schritt die nächstgelegene exakte Lösung.
//!      Springt ein Gelenk um mehr als `MAX_JUMP`, wird der Schritt halbiert (bis ~0,1 µm). Ein verbleibender
//!      Sprung ist nur IN einer Singularität erlaubt, wenn die Pose dabei stehen bleibt (Handgelenk dreht durch);
//!      sonst ist die Gerade aus dieser Haltung nicht fahrbar (Anschlag, Haltungswechsel).
//!   3. Graph über alle Punkte × Haltungen: Gerade = Gelenkweg, Gelenkfahrt (PTP) = `PTP_COST` + Gelenkweg.
//!      Kürzester Rundweg per dynamischer Programmierung, exakt (keine Kürzung der Zustände), für jede
//!      Starthaltung. Ergebnis: zuerst möglichst wenige Gelenkfahrten, dann möglichst wenig Gelenkweg.
//!   4. Der Schleifenschluss endet exakt in der Starthaltung, jede Runde ist identisch.
//! J1/J4/J6 drehen endlos: Haltungen gelten modulo 360°, jede Bewegung nimmt den kürzeren Weg.

use crate::kin::{self, Arm, IkOptions, V3, M4};

pub const STEP: f64 = 2.0;
/// Größter Orientierungsschritt (rad) je Abschnitt bei Strecken mit Werkzeugwinkel.
const ROT_STEP: f64 = 0.0175;
/// Größter erlaubter Gelenksprung pro Schritt (rad), sonst gilt die Gerade als nicht fahrbar.
const MAX_JUMP: f64 = 0.25;
/// Kosten einer Gelenkfahrt: mehr als jeder denkbare Gelenkweg auf Geraden.
const PTP_COST: f64 = 1000.0;
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
    /// Positionen mit (aufgelöstem) Werkzeugwinkel – Grundlage für die Prüfung, ob der Plan noch aktuell ist
    pub poses: Vec<Pose>,
    pub first: usize,
    pub start: [f64; 6],
    pub segs: Vec<Seg>,
    pub unreachable: Vec<usize>,
}

/// Zielpose: Position (mm) und optional Werkzeugwinkel (Roll, Pitch, Yaw in rad).
/// Ohne Winkel bestimmt die Auto-Regel die Orientierung; mit Winkel ist die Pose voll vorgegeben.
pub type Pose = (V3, Option<V3>);

fn opt(ori: bool) -> IkOptions {
    IkOptions { use_orientation: ori, pos_tol: 1e-3, rot_tol: 1e-5, restarts: 0 }
}

pub fn target(p: &Pose) -> M4 {
    kin::from_xyz_rpy(p.0, p.1.unwrap_or([0.0; 3]))
}

/// Drehwinkel zwischen den Orientierungen zweier Posen (rad).
fn turn(a: &Pose, b: &Pose) -> f64 {
    match (a.1, b.1) {
        (Some(_), Some(_)) => kin::rot_error(&target(b), &target(a)).iter().map(|v| v * v).sum::<f64>().sqrt(),
        _ => 0.0,
    }
}

/// Pose auf der Strecke a→b bei Anteil s∈[0,1]: Position linear, Orientierung als Drehung um die
/// feste Achse der Relativdrehung (Slerp): R(s) = exp(s·v)·R_a mit exp(v) = R_b·R_aᵀ.
fn pose_at(a: &Pose, b: &Pose, s: f64) -> M4 {
    let p: V3 = std::array::from_fn(|k| a.0[k] + (b.0[k] - a.0[k]) * s);
    let (ta, tb) = (target(a), target(b));
    let mut m = if a.1.is_some() && b.1.is_some() { kin::mul(&kin::rot_exp(kin::rot_error(&tb, &ta).map(|c| c * s)), &ta) } else { ta };
    for k in 0..3 {
        m[k][3] = p[k];
    }
    m
}

fn dq_max(a: &[f64; 6], b: &[f64; 6]) -> f64 {
    (0..6).map(|i| (a[i] - b[i]).abs()).fold(0.0, f64::max)
}

fn path_len(p: &[[f64; 6]]) -> f64 {
    p.windows(2).map(|w| dq_max(&w[0], &w[1])).sum()
}

/// Exakte Lösung am Ziel `t`, die `q` am nächsten liegt (Rückfall: Newton ab `q`).
fn follow(arm: &Arm, t: &M4, ori: bool, q: &[f64; 6]) -> Option<[f64; 6]> {
    let r = kin::solve_ik(arm, t, *q, &opt(ori));
    r.converged.then_some(r.q)
}

/// Gerade von `a` nach `b`, Start in Stellung `q0`. None = aus dieser Haltung nicht stetig fahrbar.
fn track(arm: &Arm, q0: [f64; 6], a: &Pose, b: &Pose) -> Option<Vec<[f64; 6]>> {
    let len = (0..3).map(|k| (b.0[k] - a.0[k]).powi(2)).sum::<f64>().sqrt();
    let n = ((len / STEP).max(turn(a, b) / ROT_STEP)).ceil().max(1.0) as usize;
    let mut out = vec![q0];
    for i in 1..=n {
        refine(arm, a, b, (i - 1) as f64 / n as f64, i as f64 / n as f64, DEPTH, &mut out)?;
    }
    Some(out)
}

/// Größte Verfeinerungstiefe: 2 mm / 2¹⁴ ≈ 0,1 µm.
const DEPTH: u32 = 14;
/// Toleranz der Pose beim Durchdrehen einer Singularität (mm, rad).
const SING_P: f64 = 0.01;
const SING_R: f64 = 1e-4;

/// Abschnitt s0→s1 der Geraden an `out` anhängen. Springt ein Gelenk um mehr als `MAX_JUMP`, wird der
/// Abschnitt halbiert – nahe einer Singularität drehen Gelenke schnell, aber stetig. Bleibt auch bei
/// kleinster Teilung ein Sprung, liegt die Stelle IN einer Singularität (z. B. J5 = 0: nur J4 + J6 zählt).
/// Dort ist die Drehung zulässig, wenn die Pose dabei stehen bleibt: dann dreht nur das Handgelenk durch,
/// die Spitze verlässt die Gerade nicht. Sonst (Anschlag, Haltungswechsel) ist die Gerade nicht fahrbar.
fn refine(arm: &Arm, a: &Pose, b: &Pose, s0: f64, s1: f64, depth: u32, out: &mut Vec<[f64; 6]>) -> Option<()> {
    let q = *out.last().unwrap();
    let t = pose_at(a, b, s1);
    let ori = a.1.is_some();
    let next = follow(arm, &t, ori, &q)?;
    if dq_max(&next, &q) <= MAX_JUMP {
        out.push(next);
        return Some(());
    }
    if depth > 0 {
        let m = 0.5 * (s0 + s1);
        refine(arm, a, b, s0, m, depth - 1, out)?;
        return refine(arm, a, b, m, s1, depth - 1, out);
    }
    let path = lerp_path(arm, q, next);
    let holds = path.iter().all(|x| {
        let f = kin::fk(arm, x);
        let dp = (0..3).map(|k| (f[k][3] - t[k][3]).powi(2)).sum::<f64>().sqrt();
        let dr = if ori { kin::rot_error(&t, &f).iter().map(|v| v * v).sum::<f64>().sqrt() } else { 0.0 };
        dp < SING_P && dr < SING_R
    });
    holds.then(|| out.extend_from_slice(&path[1..]))
}

/// Gelenkfahrt: lineare Interpolation im Gelenkraum (feiner als 0,5° pro Schritt), endlose Gelenke kurz herum.
fn lerp_path(arm: &Arm, a: [f64; 6], b: [f64; 6]) -> Vec<[f64; 6]> {
    let b = kin::wrap_near(arm, b, &a);
    let n = (dq_max(&a, &b) / 0.5f64.to_radians()).ceil().max(1.0) as usize;
    (0..=n).map(|i| std::array::from_fn(|k| a[k] + (b[k] - a[k]) * i as f64 / n as f64)).collect()
}

/// Alle Haltungen am Punkt; gibt es keine exakte (z. B. Auto-Regel nicht erfüllbar), eine Newton-Lösung.
fn candidates(arm: &Arm, p: &Pose) -> Vec<[f64; 6]> {
    let (t, ori) = (target(p), p.1.is_some());
    let c = kin::ik_all(arm, &t, ori);
    if !c.is_empty() {
        return c;
    }
    let r = kin::solve_ik(arm, &t, READY.map(f64::to_radians), &IkOptions { restarts: 20, ..opt(ori) });
    if r.converged { vec![r.q] } else { vec![] }
}

/// Gerade aus Haltung i am Punkt k: Pfad und Haltung j, in der sie am nächsten Punkt ankommt.
type Leg = Option<(Vec<[f64; 6]>, usize)>;

pub fn build(arm: &Arm, pts: &[Pose]) -> Plan {
    let mut plan = Plan { pts: pts.iter().map(|p| p.0).collect(), poses: pts.to_vec(), ..Plan::default() };
    let ready = READY.map(f64::to_radians);
    let (mut order, mut cand): (Vec<usize>, Vec<Vec<[f64; 6]>>) = (vec![], vec![]);
    for (i, p) in pts.iter().enumerate() {
        let c = candidates(arm, p);
        if c.is_empty() {
            plan.unreachable.push(i);
        } else {
            order.push(i);
            cand.push(c);
        }
    }
    let Some(&first) = order.first() else { return plan };
    plan.first = first;
    let op: Vec<Pose> = order.iter().map(|&i| pts[i]).collect();
    let n = op.len();
    if n < 2 {
        plan.start = kin::nearest(arm, &cand[0], &ready).unwrap();
        return plan;
    }
    // Alle Geraden aus allen Haltungen (Strecke k: Punkt k → k+1, die letzte schließt die Schleife)
    let legs: Vec<Vec<Leg>> = (0..n)
        .map(|k| {
            let to = (k + 1) % n;
            cand[k]
                .iter()
                .map(|&q| {
                    let mut path = track(arm, q, &op[k], &op[to])?;
                    let end = path.last_mut().unwrap();
                    let j = (0..cand[to].len()).find(|&j| kin::joint_dist(arm, &cand[to][j], end) < 1e-6)?;
                    // exakt auf die Haltung einrasten (gleiche Umdrehung), damit Folgestrecken nahtlos anschließen
                    *end = kin::wrap_near(arm, cand[to][j], end);
                    Some((path, j))
                })
                .collect()
        })
        .collect();
    // Starthaltungen: die der Bereit-Stellung nächste zuerst (entscheidet bei Gleichstand)
    let mut starts: Vec<usize> = (0..cand[0].len()).collect();
    starts.sort_by(|&a, &b| kin::joint_dist(arm, &cand[0][a], &ready).total_cmp(&kin::joint_dist(arm, &cand[0][b], &ready)));
    // best[k][j] = (Kosten, Vorgänger-Haltung, Gerade?)
    let mut winner: Option<(f64, usize, Vec<(usize, bool)>)> = None;
    for &s0 in &starts {
        let mut best: Vec<Vec<Option<(f64, usize, bool)>>> = cand.iter().map(|c| vec![None; c.len()]).collect();
        best[0][s0] = Some((0.0, usize::MAX, false));
        let relax = |slot: &mut Option<(f64, usize, bool)>, v: (f64, usize, bool)| {
            if slot.is_none_or(|s| v.0 < s.0 - 1e-9) {
                *slot = Some(v);
            }
        };
        for k in 0..n - 1 {
            for i in 0..cand[k].len() {
                let Some((c, _, _)) = best[k][i] else { continue };
                if let Some((path, j)) = &legs[k][i] {
                    relax(&mut best[k + 1][*j], (c + path_len(path), i, true));
                }
                for j in 0..cand[k + 1].len() {
                    relax(&mut best[k + 1][j], (c + PTP_COST + kin::joint_dist(arm, &cand[k][i], &cand[k + 1][j]), i, false));
                }
            }
        }
        // Schleifenschluss zurück in die Starthaltung. Eine Gelenkfahrt hier kostet nur die Hälfte:
        // Sie wird nur bei Endlosschleife gefahren. Kreuzt die Bahn die Handgelenk-Singularität
        // ungerade oft, endet sie in der umgeklappten Haltung – dann gehört der Haltungswechsel
        // an den Schluss statt mitten in die Bahn.
        let mut close: Option<(f64, usize, bool)> = None;
        for i in 0..cand[n - 1].len() {
            let Some((c, _, _)) = best[n - 1][i] else { continue };
            match &legs[n - 1][i] {
                Some((path, j)) if *j == s0 => relax(&mut close, (c + path_len(path), i, true)),
                _ => relax(&mut close, (c + PTP_COST / 2.0 + kin::joint_dist(arm, &cand[n - 1][i], &cand[0][s0]), i, false)),
            }
        }
        let Some((total, mut i, line)) = close else { continue };
        if winner.as_ref().is_some_and(|w| total >= w.0 - 1e-9) {
            continue;
        }
        // Rückwärts: Haltung je Punkt und Art der Strecke, die vom Punkt ausgeht
        let mut route = vec![(0, false); n];
        route[n - 1] = (i, line);
        for k in (1..n).rev() {
            let (_, prev, l) = best[k][i].unwrap();
            route[k - 1] = (prev, l);
            i = prev;
        }
        winner = Some((total, s0, route));
    }
    let (_, s0, route) = winner.unwrap();
    // Vorwärts mit den tatsächlichen (abgewickelten) Winkeln: Geraden werden um ganze Umdrehungen verschoben
    let mut q = cand[0][s0];
    plan.start = q;
    for k in 0..n {
        let (i, line) = route[k];
        let to = (k + 1) % n;
        let path = if line {
            let (p, _) = legs[k][i].as_ref().unwrap();
            let d: [f64; 6] = std::array::from_fn(|m| q[m] - cand[k][i][m]);
            p.iter().map(|x| std::array::from_fn(|m| x[m] + d[m])).collect()
        } else {
            let j = if to == 0 { s0 } else { route[to].0 };
            lerp_path(arm, q, cand[to][j])
        };
        q = *path.last().unwrap();
        plan.segs.push(Seg { from: order[k], to: order[to], q: path, ptp: !line, closing: k + 1 == n });
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

/// Zeit (s), in der die Fahrt auf Nenngeschwindigkeit beschleunigt bzw. wieder abbremst.
pub const RAMP: f64 = 0.35;

/// Geschwindigkeitsfaktor 0..1 bei Nennzeit `tau` einer Fahrt der Nenndauer `total`.
/// Konstante Beschleunigung (v² ∝ zurückgelegter Weg): sanftes Anfahren und Abbremsen
/// statt Ruck von 0 auf volle Geschwindigkeit. Kurze Fahrten erreichen Vollgas gar nicht erst.
pub fn ramp(tau: f64, total: f64, ramp_s: f64) -> f64 {
    ((tau.min(total - tau) / ramp_s.max(0.02)).max(0.0).sqrt()).clamp(0.04, 1.0)
}

/// Läuft `dt` Sekunden auf der Strecke weiter. Rückgabe: neuer Index `u` (Bruchteil = Position
/// zwischen zwei Samples) und true, wenn das Ende erreicht ist.
pub fn advance(arm: &Arm, seg: &Seg, u: f64, dt: f64, tcp_speed: f64, speed_dps: f64, ramp_s: f64) -> (f64, bool) {
    let n = seg.q.len() - 1;
    let times: Vec<f64> = (0..n).map(|i| step_time(arm, &seg.q[i], &seg.q[i + 1], seg.ptp, tcp_speed, speed_dps)).collect();
    let total: f64 = times.iter().sum();
    let at = |u: f64| -> f64 {
        let i = (u.floor() as usize).min(n);
        times[..i].iter().sum::<f64>() + times.get(i).map_or(0.0, |t| t * (u - i as f64))
    };
    // Nennzeit τ in Sample-Index zurückrechnen
    let back = |mut tau: f64| -> f64 {
        for (i, t) in times.iter().enumerate() {
            if tau <= *t {
                return i as f64 + if *t > 0.0 { tau / t } else { 0.0 };
            }
            tau -= t;
        }
        n as f64
    };
    let mut tau = at(u);
    let mut left = dt;
    while left > 0.0 && tau < total {
        let h = left.min(0.004);
        tau = (tau + h * ramp(tau, total, ramp_s)).min(total);
        left -= h;
    }
    if tau >= total - 1e-9 {
        return (n as f64, true);
    }
    (back(tau), false)
}

/// Gelenkstellung an der Stelle `u` der Strecke.
pub fn sample(seg: &Seg, u: f64) -> [f64; 6] {
    let i = (u.floor() as usize).min(seg.q.len() - 1);
    let j = (i + 1).min(seg.q.len() - 1);
    let f = u - i as f64;
    std::array::from_fn(|k| seg.q[i][k] + (seg.q[j][k] - seg.q[i][k]) * f)
}


