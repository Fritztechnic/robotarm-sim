//! Playlist-Planer: berechnet VOR dem Start für jede Strecke die komplette Gelenk-Trajektorie.
//! Das Ergebnis hängt nur von Armgeometrie und Punkten ab – nie von Framerate, Geschwindigkeit
//! oder Runde. Dieselbe Strecke fährt also immer exakt denselben Weg.
//!
//! Verfahren pro Strecke A→B (nur Position, die 3 überzähligen Freiheitsgrade bleiben frei):
//!   1. Gerade in Schritten von `STEP` mm unterteilen, p(s) = A + s·(B−A)/|B−A|.
//!   2. Jeder Punkt per gedämpftem Newton (Warmstart vom vorigen) + Nullraum-Zug lösen.
//!   3. Schlägt ein Schritt fehl oder springt ein Gelenk (Singularität/Zweigwechsel),
//!      wird die Strecke als Gelenkfahrt (PTP) geplant: lineare Interpolation im Gelenkraum.
//!   0. Welche der vielen IK-Lösungen (Ellbogen/Handgelenk-Varianten) an jedem Punkt genommen wird,
//!      entscheidet `route` per dynamischer Programmierung: möglichst wenige Gelenkfahrten.
//!      Geht eine Gerade aus der aktuellen Konfiguration nicht, wird am Startpunkt zuerst umorientiert
//!      (`reorient`: Nullraum-Bewegung, Spitze bleibt auf dem Punkt), dann die Gerade gefahren.
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

/// Alle deutlich verschiedenen IK-Lösungen für `p` (Ellbogen oben/unten, Handgelenk geklappt, Schulter gedreht).
/// Feste Startwerte, daher deterministisch.
fn candidates(arm: &Arm, p: V3) -> Vec<[f64; 6]> {
    let t = target(p);
    let mut out: Vec<[f64; 6]> = vec![];
    let mut seeds = vec![READY.map(f64::to_radians)];
    for j1 in [0.0, 90.0, -90.0, 180.0] {
        for j2 in [-35.0, 35.0] {
            for j3 in [70.0, -70.0] {
                for j5 in [55.0, -55.0] {
                    seeds.push([j1, j2, j3, 0.0, j5, 0.0].map(f64::to_radians));
                }
            }
        }
    }
    for s in seeds {
        let r = kin::solve_ik(arm, &t, s, &opt(0));
        if r.converged && out.iter().all(|o| dq_max(o, &r.q) > 0.1) {
            out.push(r.q);
        }
    }
    out
}

struct State {
    q: [f64; 6],
    cost: f64,
    parent: usize,
    path: Vec<[f64; 6]>,
    ptp: bool,
}

const PTP_COST: f64 = 1000.0;
/// Umorientieren am Punkt kostet etwas, ist aber viel besser als eine Gelenkfahrt.
const REORIENT_COST: f64 = 20.0;
const KEEP: usize = 10;
/// Zustände, die sich weniger als das unterscheiden, gelten als dieselbe Konfiguration (rad).
const SAME: f64 = 0.3;

fn key(k: usize, q: &[f64; 6]) -> (usize, [i64; 6]) {
    (k, q.map(|v| (v * 1e4).round() as i64))
}

fn path_len(p: &[[f64; 6]]) -> f64 {
    p.windows(2).map(|w| dq_max(&w[0], &w[1])).sum()
}

type Cache = std::collections::HashMap<(usize, [i64; 6]), Option<Vec<[f64; 6]>>>;

/// Umorientieren am Punkt: die Spitze bleibt auf `p`, nur die Konfiguration (Handgelenk, Ellbogen…)
/// wechselt von `q0` nach `q1`. Gelenkinterpolation, jeder Schritt wird per IK zurück auf den Punkt
/// projiziert, also Nullraum-Bewegung. None, wenn das ohne Sprung/Verlassen des Punktes nicht geht.
fn reorient(arm: &Arm, p: V3, q0: [f64; 6], q1: [f64; 6]) -> Option<Vec<[f64; 6]>> {
    let n = (dq_max(&q0, &q1) / 2f64.to_radians()).ceil().max(1.0) as usize;
    let t = target(p);
    let mut out = vec![q0];
    for i in 1..n {
        let f = i as f64 / n as f64;
        let qt: [f64; 6] = std::array::from_fn(|k| q0[k] + (q1[k] - q0[k]) * f);
        let r = kin::solve_ik(arm, &t, qt, &opt(0));
        if !r.converged || dq_max(&r.q, out.last().unwrap()) > MAX_JUMP {
            return None;
        }
        out.push(r.q);
    }
    (dq_max(out.last().unwrap(), &q1) <= MAX_JUMP).then(|| {
        out.push(q1);
        out
    })
}

/// Eine Strecke von `pa` nach `pb` ab Stellung `q`: erst direkt als Gerade. Geht das nicht, am Startpunkt
/// erst in eine passende Konfiguration umorientieren (ohne den Punkt zu verlassen), dann Gerade.
/// `fin` prüft/ergänzt das Streckenende (Schleifenschluss). Rückgabe: Pfad und ob umorientiert wurde.
fn leg(arm: &Arm, cache: &mut Cache, k: usize, pa: V3, pb: V3, q: [f64; 6], cand: &[[f64; 6]], fin: &dyn Fn(Vec<[f64; 6]>) -> Option<Vec<[f64; 6]>>) -> Option<(Vec<[f64; 6]>, bool)> {
    let mut line = |q0: [f64; 6]| cache.entry(key(k, &q0)).or_insert_with(|| march(arm, q0, pa, pb)).clone().and_then(fin);
    if let Some(l) = line(q) {
        return Some((l, false));
    }
    let mut cs: Vec<[f64; 6]> = cand.iter().copied().filter(|c| dq_max(c, &q) > 1e-3).collect();
    cs.sort_by(|a, b| dq_max(a, &q).total_cmp(&dq_max(b, &q)));
    for c in cs.into_iter().take(14) {
        if let Some(l) = line(c) {
            if let Some(mut r) = reorient(arm, pa, q, c) {
                r.extend(l.into_iter().skip(1));
                return Some((r, true));
            }
        }
    }
    None
}

/// Bester Weg durch alle Punkte ab Startstellung `s0`: Dynamische Programmierung über die
/// Konfigurationen. Gerade = fast kostenlos, Gelenkfahrt = teuer, also möglichst wenige davon.
fn route(arm: &Arm, pts: &[V3], cand: &[Vec<[f64; 6]>], s0: [f64; 6], cache: &mut Cache) -> (f64, Vec<(Vec<[f64; 6]>, bool)>) {
    let n = pts.len();
    let mut layers: Vec<Vec<State>> = vec![vec![State { q: s0, cost: 0.0, parent: usize::MAX, path: vec![], ptp: false }]];
    for k in 0..n - 1 {
        let mut next: Vec<State> = vec![];
        for (si, s) in layers[k].iter().enumerate() {
            if let Some((path, re)) = leg(arm, cache, k, pts[k], pts[k + 1], s.q, &cand[k], &Some) {
                next.push(State { q: *path.last().unwrap(), cost: s.cost + path_len(&path) + if re { REORIENT_COST } else { 0.0 }, parent: si, path, ptp: false });
            }
            for c in &cand[k + 1] {
                let path = lerp_path(s.q, *c);
                next.push(State { q: *c, cost: s.cost + PTP_COST + path_len(&path), parent: si, path, ptp: true });
            }
        }
        next.sort_by(|a, b| a.cost.total_cmp(&b.cost));
        let mut kept: Vec<State> = vec![];
        for s in next {
            if kept.len() < KEEP && kept.iter().all(|o| dq_max(&o.q, &s.q) > SAME) {
                kept.push(s);
            }
        }
        layers.push(kept);
    }
    // Schleifenschluss: muss exakt in s0 enden, sonst Gelenkfahrt dorthin
    let mut best: Option<(f64, usize, Vec<[f64; 6]>, bool)> = None;
    for (si, s) in layers[n - 1].iter().enumerate() {
        // Gerade zurück zum Startpunkt; endet sie in einer anderen Konfiguration, pendelt der Arm dort
        // kurz in die Startstellung ein (damit jede Runde exakt gleich beginnt)
        let fin = |mut l: Vec<[f64; 6]>| {
            let tail = reorient(arm, pts[0], *l.last().unwrap(), s0)?;
            l.extend(tail.into_iter().skip(1));
            Some(l)
        };
        let line = leg(arm, cache, n - 1, pts[n - 1], pts[0], s.q, &cand[n - 1], &fin);
        let (path, ptp) = match line {
            Some((l, _)) => (l, false),
            None => (lerp_path(s.q, s0), true),
        };
        let cost = s.cost + path_len(&path) + if ptp { PTP_COST } else { 0.0 };
        if best.as_ref().is_none_or(|b| cost < b.0) {
            best = Some((cost, si, path, ptp));
        }
    }
    let (cost, mut si, cpath, cptp) = best.unwrap();
    let mut segs = vec![(cpath, cptp)];
    for k in (1..n).rev() {
        let s = &layers[k][si];
        segs.push((s.path.clone(), s.ptp));
        si = s.parent;
    }
    segs.reverse();
    (cost, segs)
}

pub fn build(arm: &Arm, pts: &[V3]) -> Plan {
    let mut plan = Plan { pts: pts.to_vec(), ..Plan::default() };
    let ready = READY.map(f64::to_radians);
    let (mut order, mut cand): (Vec<usize>, Vec<Vec<[f64; 6]>>) = (vec![], vec![]);
    for (i, &p) in pts.iter().enumerate() {
        let mut c = candidates(arm, p);
        if c.is_empty() {
            c.extend(nearest(arm, p, ready));
        }
        if c.is_empty() {
            plan.unreachable.push(i);
        } else {
            order.push(i);
            cand.push(c);
        }
    }
    let Some(&first) = order.first() else { return plan };
    plan.first = first;
    let op: Vec<V3> = order.iter().map(|&i| pts[i]).collect();
    if op.len() < 2 {
        plan.start = cand[0][0];
        return plan;
    }
    // Startkonfiguration: die mit der besten Gesamtroute (Bereit-Pose am nächsten zuerst: deterministische Reihenfolge)
    let mut starts = cand[0].clone();
    starts.sort_by(|a, b| dq_max(a, &ready).total_cmp(&dq_max(b, &ready)));
    let mut cache = Cache::new();
    let mut best: Option<(f64, [f64; 6], Vec<(Vec<[f64; 6]>, bool)>)> = None;
    for s0 in starts.into_iter().take(6) {
        let (cost, segs) = route(arm, &op, &cand, s0, &mut cache);
        if best.as_ref().is_none_or(|b| cost < b.0 - 1e-9) {
            best = Some((cost, s0, segs));
        }
        if best.as_ref().is_some_and(|b| b.0 < PTP_COST) {
            break;
        }
    }
    let (_, s0, segs) = best.unwrap();
    plan.start = s0;
    let n = op.len();
    for (k, (q, ptp)) in segs.into_iter().enumerate() {
        let closing = k + 1 == n;
        plan.segs.push(Seg { from: order[k], to: order[(k + 1) % n], q, ptp, closing });
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


