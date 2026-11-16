//! Werkstatt: Werkbank, Werkstück, Werkzeug und Bahnpunkte für Bohren, Fräsen und Schweißen.
//!
//! Die Bahnpunkte sind Positionen der WERKZEUGSPITZE. Der Arm wird über den Flansch (TCP) gesteuert;
//! der Flansch sitzt um die Werkzeuglänge L entgegen der Werkzeugachse hinter der Spitze:
//! p_Flansch = p_Spitze − L · z_Werkzeug (z_Werkzeug = dritte Spalte der Drehmatrix).

use crate::kin::{self, M4, V3};
use serde::{Deserialize, Serialize};

#[derive(Clone, Copy, PartialEq, Serialize, Deserialize)]
pub enum Job {
    Drill,
    Mill,
    Weld,
}

pub const JOBS: [(Job, &str); 3] = [(Job::Drill, "Bohren"), (Job::Mill, "Fräsen"), (Job::Weld, "Schweißen")];

/// Mitte des Werkstücks (x, y) und Höhe der Werkbank-Oberfläche (mm).
const CX: f64 = 300.0;
const BENCH_H: f64 = 100.0;
const HOLES: [(f64, f64); 6] = [(-50.0, -35.0), (0.0, -35.0), (50.0, -35.0), (50.0, 35.0), (0.0, 35.0), (-50.0, 35.0)];

pub struct Obj {
    pub cyl: bool,
    pub m: M4,
    pub col: [u8; 3],
    pub metal: f32,
    pub rough: f32,
}

pub enum Mark {
    Line(Vec<V3>, [u8; 4], f32),
    Disc(V3, f64, [u8; 4]),
}

fn rad(rpy: V3) -> V3 {
    rpy.map(f64::to_radians)
}

/// Quader: Mitte der Unterseite `c`, Größe `s`, Drehung `yaw` um z.
fn cuboid(c: V3, s: V3, yaw: f64, col: [u8; 3], metal: f32, rough: f32) -> Obj {
    Obj { cyl: false, m: scaled(c, [s[0], s[1], s[2]], yaw), col, metal, rough }
}

/// Zylinder (Achse z): Mitte der Grundfläche `c`, Radius `r`, Höhe `h`.
fn cylinder(c: V3, r: f64, h: f64, col: [u8; 3], metal: f32, rough: f32) -> Obj {
    Obj { cyl: true, m: scaled(c, [r, r, h], 0.0), col, metal, rough }
}

fn scaled(c: V3, s: V3, yaw: f64) -> M4 {
    let (co, si) = (yaw.cos(), yaw.sin());
    [[co * s[0], -si * s[1], 0.0, c[0]], [si * s[0], co * s[1], 0.0, c[1]], [0.0, 0.0, s[2], c[2]], [0.0, 0.0, 0.0, 1.0]]
}

/// Gleiche Form, aber im Flansch-Frame (Werkzeug).
fn on_tool(o: Obj, fr: &M4) -> Obj {
    Obj { m: kin::mul(fr, &o.m), ..o }
}

pub struct Spec {
    pub rpy_deg: V3,
    pub surface: f64,
    pub tcp_speed: f64,
    /// Spitzenpositionen mit Wartezeit (s)
    pub tips: Vec<(V3, f64)>,
}

impl Job {
    fn plate(self) -> f64 {
        match self {
            Job::Drill => 15.0,
            Job::Mill => 25.0,
            Job::Weld => 8.0,
        }
    }

    pub fn surface(self) -> f64 {
        BENCH_H + self.plate()
    }

    pub fn tool_len(self) -> f64 {
        match self {
            Job::Drill => 120.0,
            Job::Mill => 110.0,
            Job::Weld => 100.0,
        }
    }

    /// Werkzeugwinkel (Roll, Pitch, Yaw in °): Achse nach unten; der Brenner stellt 15° in Fahrtrichtung (stechend).
    pub fn rpy_deg(self) -> V3 {
        match self {
            Job::Weld => [-165.0, 0.0, 0.0],
            _ => [180.0, 0.0, 0.0],
        }
    }

    pub fn spec(self) -> Spec {
        let z0 = self.surface();
        let at = |x: f64, y: f64, h: f64| [CX + x, y, z0 + h];
        let mut tips: Vec<(V3, f64)> = vec![];
        match self {
            Job::Drill => {
                tips.push((at(0.0, 0.0, 60.0), 0.2));
                for (x, y) in HOLES {
                    tips.extend([(at(x, y, 25.0), 0.0), (at(x, y, -10.0), 0.7), (at(x, y, 25.0), 0.0)]);
                }
            }
            Job::Mill => {
                tips.push((at(0.0, 0.0, 60.0), 0.2));
                let (a, b, r) = (70.0, 45.0, 15.0);
                let mut contour = vec![(0.0, -b)];
                for (cx, cy, a0) in [(a - r, -(b - r), -90.0f64), (a - r, b - r, 0.0), (-(a - r), b - r, 90.0), (-(a - r), -(b - r), 180.0)] {
                    for k in 0..=5 {
                        let t = (a0 + 18.0 * k as f64).to_radians();
                        contour.push((cx + r * t.cos(), cy + r * t.sin()));
                    }
                }
                contour.push((0.0, -b));
                let hole: Vec<(f64, f64)> = (0..=24).map(|k| (25.0 * (15.0 * k as f64).to_radians().cos(), 25.0 * (15.0 * k as f64).to_radians().sin())).collect();
                for path in [contour, hole] {
                    let s = path[0];
                    tips.push((at(s.0, s.1, 30.0), 0.0));
                    tips.push((at(s.0, s.1, -3.0), 0.4));
                    tips.extend(path[1..].iter().map(|&(x, y)| (at(x, y, -3.0), 0.0)));
                    tips.push((at(s.0, s.1, 30.0), 0.0));
                }
            }
            Job::Weld => {
                tips.push((at(0.0, 0.0, 80.0), 0.2));
                tips.push((at(0.0, -65.0, 40.0), 0.0));
                tips.push((at(0.0, -65.0, 3.0), 0.6));
                // Pendelnaht: ±2 mm quer zur Naht, alle 4 mm ein Umkehrpunkt
                for k in 1..=32 {
                    tips.push((at(if k % 2 == 1 { 2.0 } else { -2.0 }, -65.0 + 4.0 * k as f64, 3.0), 0.0));
                }
                tips.push((at(0.0, 65.0, 3.0), 0.6));
                tips.push((at(0.0, 65.0, 40.0), 0.0));
            }
        }
        Spec { rpy_deg: self.rpy_deg(), surface: z0, tcp_speed: self.speed(), tips }
    }

    fn speed(self) -> f64 {
        match self {
            Job::Drill => 150.0,
            Job::Mill => 50.0,
            Job::Weld => 20.0,
        }
    }

    /// Flanschposition für eine Spitzenposition.
    pub fn flange(self, tip: V3) -> V3 {
        let z = kin::col(&kin::from_xyz_rpy([0.0; 3], rad(self.rpy_deg())), 2);
        std::array::from_fn(|i| tip[i] - self.tool_len() * z[i])
    }

    /// Werkzeugspitze zu einem Flansch-Frame.
    pub fn tip(self, fr: &M4) -> V3 {
        let (p, z) = (kin::pos(fr), kin::col(fr, 2));
        std::array::from_fn(|i| p[i] + self.tool_len() * z[i])
    }

    /// Arbeitet das Werkzeug gerade am Werkstück?
    pub fn working(self, tip: V3) -> bool {
        let inside = (tip[0] - CX).abs() < 90.0 && tip[1].abs() < 70.0;
        let s = self.surface();
        inside && match self {
            Job::Weld => tip[2] <= s + 4.0,
            _ => tip[2] < s - 0.5,
        }
    }

    pub fn objects(self, fr6: &M4) -> Vec<Obj> {
        let steel = [150, 155, 165];
        let clamp = [200, 60, 40];
        let z0 = BENCH_H;
        let mut o = vec![
            cuboid([CX, 0.0, 0.0], [330.0, 280.0, z0 - 20.0], 0.0, [64, 74, 90], 0.15, 0.6),
            cuboid([CX, 0.0, z0 - 20.0], [350.0, 300.0, 20.0], 0.0, steel, 0.6, 0.4),
        ];
        let t = self.plate();
        let alu = [190, 196, 205];
        match self {
            Job::Drill => {
                o.push(cuboid([CX, 0.0, z0], [160.0, 120.0, t], 0.0, alu, 0.9, 0.35));
                o.push(cuboid([CX - 55.0, 54.0, z0 + t], [34.0, 16.0, 10.0], 0.0, clamp, 0.2, 0.5));
                o.push(cuboid([CX + 55.0, -54.0, z0 + t], [34.0, 16.0, 10.0], 0.0, clamp, 0.2, 0.5));
                o.push(on_tool(cylinder([0.0, 0.0, 0.0], 14.0, 40.0, [60, 65, 75], 0.8, 0.3), fr6));
                o.push(on_tool(cylinder([0.0, 0.0, 40.0], 4.0, 80.0, [150, 155, 165], 1.0, 0.25), fr6));
            }
            Job::Mill => {
                o.push(cuboid([CX, 0.0, z0], [180.0, 130.0, t], 0.0, alu, 0.9, 0.35));
                o.push(cuboid([CX - 70.0, 59.0, z0 + t], [34.0, 16.0, 10.0], 0.0, clamp, 0.2, 0.5));
                o.push(cuboid([CX + 70.0, -59.0, z0 + t], [34.0, 16.0, 10.0], 0.0, clamp, 0.2, 0.5));
                o.push(on_tool(cylinder([0.0, 0.0, 0.0], 18.0, 45.0, [60, 65, 75], 0.8, 0.3), fr6));
                o.push(on_tool(cylinder([0.0, 0.0, 45.0], 6.0, 65.0, [210, 175, 70], 0.9, 0.3), fr6));
            }
            Job::Weld => {
                let dark = [110, 115, 125];
                o.push(cuboid([CX - 37.5, 0.0, z0], [75.0, 140.0, t], 0.0, dark, 0.9, 0.5));
                o.push(cuboid([CX + 37.5, 0.0, z0], [75.0, 140.0, t], 0.0, dark, 0.9, 0.5));
                o.push(cuboid([CX - 62.0, 0.0, z0 + t], [30.0, 16.0, 10.0], 0.0, clamp, 0.2, 0.5));
                o.push(cuboid([CX + 62.0, 0.0, z0 + t], [30.0, 16.0, 10.0], 0.0, clamp, 0.2, 0.5));
                o.push(on_tool(cylinder([0.0, 0.0, 0.0], 11.0, 60.0, [40, 45, 55], 0.4, 0.5), fr6));
                o.push(on_tool(cylinder([0.0, 0.0, 60.0], 7.0, 40.0, [190, 120, 70], 0.9, 0.3), fr6));
            }
        }
        o
    }

    /// Spuren auf dem Werkstück aus den bisherigen Arbeitspunkten `cuts` + Führungslinie.
    pub fn marks(self, cuts: &[V3], tip: V3, welding: bool) -> Vec<Mark> {
        let s = self.surface() + 0.4;
        let mut m = vec![];
        let guide = [0, 120, 212, 70];
        let spec = self.spec();
        match self {
            Job::Drill => {
                for (x, y) in HOLES {
                    let c = [CX + x, y, s];
                    let done = cuts.iter().any(|p| (p[0] - c[0]).hypot(p[1] - c[1]) < 4.0);
                    m.push(Mark::Disc(c, if done { 4.5 } else { 3.0 }, if done { [25, 25, 30, 255] } else { guide }));
                }
            }
            _ => {
                // Führungslinie: zusammenhängende Stücke der Bahn auf Werkstückhöhe
                let mut run: Vec<V3> = vec![];
                for (p, _) in &spec.tips {
                    if p[2] < spec.surface + 5.0 {
                        run.push([p[0], p[1], s]);
                    } else if !run.is_empty() {
                        m.push(Mark::Line(std::mem::take(&mut run), guide, 1.0));
                    }
                }
                if !run.is_empty() {
                    m.push(Mark::Line(run, guide, 1.0));
                }
                let mut seg = vec![];
                let flush = |seg: &mut Vec<V3>, m: &mut Vec<Mark>| {
                    if seg.len() > 1 {
                        let n = seg.len();
                        if self == Job::Weld {
                            m.push(Mark::Line(seg.clone(), [90, 80, 80, 255], 5.0));
                            m.push(Mark::Line(seg[n.saturating_sub(40)..].to_vec(), [255, 140, 30, 255], 3.5));
                        } else {
                            m.push(Mark::Line(seg.clone(), [30, 32, 38, 255], 5.0));
                        }
                    }
                    seg.clear();
                };
                for p in cuts {
                    let p = [p[0], p[1], s];
                    if seg.last().is_some_and(|l: &V3| (l[0] - p[0]).hypot(l[1] - p[1]) > 8.0) {
                        flush(&mut seg, &mut m);
                    }
                    seg.push(p);
                }
                flush(&mut seg, &mut m);
            }
        }
        if welding {
            let p = [tip[0], tip[1], s + 1.0];
            m.push(Mark::Disc(p, 14.0, [255, 170, 40, 60]));
            m.push(Mark::Disc(p, 7.0, [255, 220, 120, 170]));
            m.push(Mark::Disc(p, 3.0, [255, 255, 240, 255]));
        }
        m
    }
}
