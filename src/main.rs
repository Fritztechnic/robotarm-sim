#![allow(deprecated)]
// gpu = 3D-Renderer, icons = SVGs, kin = Mathe, ui = Panels, view = Kamera & Overlays
mod gpu;
mod icons;
mod kin;
mod ui;
mod view;

use eframe::egui::{self, Color32, Stroke};
use kin::{Arm, V3};
use serde::{Deserialize, Serialize};
use view::{Camera, Style};

const OK: Color32 = Color32::from_rgb(40, 160, 90);
const BAD: Color32 = Color32::from_rgb(212, 60, 40);
const WARN: Color32 = Color32::from_rgb(214, 120, 10);
const MUTED: Color32 = Color32::from_gray(105);
const BLUE: Color32 = Color32::from_rgb(0, 120, 212);
const BAR: Color32 = Color32::from_rgb(238, 239, 241);
const BORDER: Color32 = Color32::from_rgb(200, 203, 208);
const TEXT: Color32 = Color32::from_gray(40);

#[derive(Clone, Copy, Serialize, Deserialize)]
struct Servo {
    dir: f64,
    center_deg: f64,
    range_deg: f64,
    pulse_min: f64,
    pulse_max: f64,
}

#[derive(Clone, Serialize, Deserialize)]
struct Config {
    arm: Arm,
    servos: [Servo; 6],
    style: Style,
    speed_dps: f64,
    #[serde(default = "default_tcp_speed")]
    tcp_speed: f64,
    #[serde(default)]
    waypoints: Vec<Waypoint>,
}

fn default_tcp_speed() -> f64 {
    200.0
}

#[derive(Clone, Serialize, Deserialize)]
struct Waypoint {
    pos: V3,
    /// Wartezeit nach Ankunft in Sekunden
    dwell: f64,
}

// Demo-Playlist zum Testen: ein Rechteck und ein Ausreißer nach oben
fn demo_waypoints() -> Vec<Waypoint> {
    [[300.0, -150.0, 250.0], [300.0, 150.0, 250.0], [300.0, 150.0, 450.0], [300.0, -150.0, 450.0], [150.0, 0.0, 600.0]]
        .map(|pos| Waypoint { pos, dwell: 0.3 })
        .to_vec()
}

impl Default for Config {
    fn default() -> Self {
        // Servo deckt ±180° ab; Mitte 1500 µs entspricht Gelenkwinkel 0
        let s = Servo { dir: 1.0, center_deg: 180.0, range_deg: 360.0, pulse_min: 500.0, pulse_max: 2500.0 };
        Self {
            arm: kin::cobot_arm(),
            servos: [s; 6],
            style: Style::default(),
            speed_dps: 180.0,
            tcp_speed: default_tcp_speed(),
            waypoints: demo_waypoints(),
        }
    }
}

// Wo die Einstellungen liegen. Neue Version = neue Datei, damit alte Configs nix zerschießen
fn config_path() -> std::path::PathBuf {
    let home = std::env::var("HOME").unwrap_or_default();
    std::path::Path::new(&home).join(".config/robotarm-sim/config-v3.json")
}

#[derive(PartialEq, Clone, Copy)]
enum Tab {
    Control,
    Playlist,
    Build,
    View,
}

#[derive(PartialEq, Clone, Copy)]
enum Mode {
    Ik,
    Fk,
}

/// Ablauf der Playlist: gerade Linie zum Zielpunkt, dann Wartezeit.
#[derive(Clone, Copy, PartialEq)]
enum Play {
    Idle,
    /// Gelenkbewegung zum ersten Punkt (Start liegt oft nahe einer Singularität)
    Approach,
    Move { from: V3, s: f64 },
    /// Ausweichbewegung im Gelenkraum, wenn die Gerade nicht fahrbar ist (Singularität, Grenze, Schleifenende)
    Joint { from: [f64; 6], to: [f64; 6], s: f64 },
    Dwell(f64),
}

// Der ganze Programmzustand an einem Ort
struct App {
    cfg: Config,
    q_cmd: [f64; 6],
    q_view: [f64; 6],
    mode: Mode,
    tab: Tab,
    target_pos: V3,
    target_rpy_deg: V3,
    /// An: Werkzeugwinkel wird vorgegeben (6D). Aus: IK wählt den Winkel selbst (3D-Position).
    use_orientation: bool,
    ik_msg: Option<(bool, String)>,
    cam: Camera,
    trail: Vec<V3>,
    status: String,
    play: Play,
    looping: bool,
    pl_idx: usize,
    /// Punkte, die außer Reichweite lagen und übersprungen wurden (aktueller Durchlauf)
    skipped: Vec<usize>,
}

impl App {
    fn new(cc: &eframe::CreationContext<'_>) -> Self {
        let cfg: Config = std::fs::read_to_string(config_path())
            .ok()
            .and_then(|s| serde_json::from_str(&s).ok())
            .unwrap_or_default();
        egui_extras::install_image_loaders(&cc.egui_ctx);
        ui::apply_theme(&cc.egui_ctx);
        if let Some(rs) = &cc.wgpu_render_state {
            let gpu = gpu::Gpu::new(&rs.device, rs.target_format);
            rs.renderer.write().callback_resources.insert(gpu);
        }
        let mut app = Self {
            cfg,
            q_cmd: [0.0; 6],
            q_view: [0.0; 6],
            mode: Mode::Ik,
            tab: Tab::Control,
            target_pos: [0.0; 3],
            target_rpy_deg: [0.0; 3],
            use_orientation: false,
            ik_msg: None,
            cam: Camera::default(),
            trail: vec![],
            status: String::new(),
            play: Play::Idle,
            looping: true,
            pl_idx: 0,
            skipped: vec![],
        };
        app.set_pose([0.0, -35.0, 70.0, 0.0, 55.0, 0.0]);
        app.q_view = app.q_cmd;
        // Entwickler-Hilfen für Screenshots
        app.tab = match std::env::var("ROBOTARM_TAB").as_deref() {
            Ok("playlist") => Tab::Playlist,
            Ok("build") => Tab::Build,
            Ok("view") => Tab::View,
            _ => Tab::Control,
        };
        if let Ok(v) = std::env::var("ROBOTARM_VIEW") {
            let n: Vec<f64> = v.split(',').filter_map(|s| s.parse().ok()).collect();
            if n.len() == 2 {
                app.cam.set_view(n[0], n[1]);
            }
        }
        if let Ok(v) = std::env::var("ROBOTARM_POSE") {
            let n: Vec<f64> = v.split(',').filter_map(|s| s.parse().ok()).collect();
            if n.len() == 6 {
                app.set_pose(std::array::from_fn(|i| n[i]));
            }
        }
        if std::env::var("ROBOTARM_PLAY").is_ok() {
            app.play_all();
        }
        app
    }

    fn sync_target_from_q(&mut self) {
        let m = kin::fk(&self.cfg.arm, &self.q_cmd);
        self.target_pos = kin::pos(&m);
        self.target_rpy_deg = kin::to_rpy(&m).map(f64::to_degrees);
    }

    // IK ausführen und Meldung für die Statusleiste bauen
    fn solve(&mut self, restarts: usize) {
        let t = kin::from_xyz_rpy(self.target_pos, self.target_rpy_deg.map(f64::to_radians));
        let opt = kin::IkOptions { use_orientation: self.use_orientation, pos_tol: 1e-3, rot_tol: 1e-5, restarts };
        let r = kin::solve_ik(&self.cfg.arm, &t, self.q_cmd, &opt);
        self.q_cmd = r.q;
        if !self.use_orientation {
            self.target_rpy_deg = kin::to_rpy(&kin::fk(&self.cfg.arm, &r.q)).map(f64::to_degrees);
        }
        self.ik_msg = Some(if r.converged {
            (true, format!("Ziel erreicht · {:.3} mm", r.pos_err))
        } else {
            (false, format!("Nicht erreichbar · Rest {:.1} mm / {:.1}°", r.pos_err, r.rot_err.to_degrees()))
        });
    }

    // Befohlene Winkel in die Grenzen zwingen
    fn clamp_cmd(&mut self) {
        for i in 0..6 {
            let j = &self.cfg.arm[i];
            self.q_cmd[i] = self.q_cmd[i].clamp(j.min_deg.to_radians(), j.max_deg.to_radians());
        }
    }

    // Feste Pose setzen (Null/Bereit/Ruhe), Winkel in Grad
    fn set_pose(&mut self, deg: [f64; 6]) {
        self.q_cmd = deg.map(f64::to_radians);
        self.clamp_cmd();
        self.sync_target_from_q();
        self.ik_msg = None;
    }

    /// (Servo-Winkel, Pulsweite µs, begrenzt?)
    fn motor(&self, i: usize) -> (f64, f64, bool) {
        let s = &self.cfg.servos[i];
        let raw = s.center_deg + s.dir * self.q_cmd[i].to_degrees();
        let a = raw.clamp(0.0, s.range_deg);
        let p = s.pulse_min + a / s.range_deg * (s.pulse_max - s.pulse_min);
        (a, p, (raw - a).abs() > 1e-9)
    }

    // Wo steht die Werkzeugspitze gerade?
    fn tcp(&self) -> V3 {
        kin::pos(&kin::fk(&self.cfg.arm, &self.q_cmd))
    }

    /// Bewegt die Anzeige-Gelenke zur Sollstellung; true solange sie unterwegs sind.
    fn advance(&mut self, dt: f64) -> bool {
        let max = (0..6).map(|i| (self.q_cmd[i] - self.q_view[i]).abs()).fold(0.0, f64::max);
        if max < 1e-6 {
            self.q_view = self.q_cmd;
            return false;
        }
        let step = self.cfg.speed_dps.to_radians() * dt;
        let k = if self.cfg.speed_dps >= 720.0 { 1.0 } else { (step / max).min(1.0) };
        for i in 0..6 {
            self.q_view[i] += (self.q_cmd[i] - self.q_view[i]) * k;
        }
        true
    }

    /// Fährt per Gelenkbewegung zu einem Zielpunkt (kein Playlist-Ablauf).
    fn goto_waypoint(&mut self, i: usize) {
        self.play = Play::Idle;
        self.pl_idx = i;
        self.target_pos = self.cfg.waypoints[i].pos;
        self.mode = Mode::Ik;
        self.solve(20);
    }

    fn playing(&self) -> bool {
        self.play != Play::Idle
    }

    // Gerade Linie von hier zu Punkt i starten
    fn start_move(&mut self, i: usize) {
        self.pl_idx = i;
        self.play = Play::Move { from: self.tcp(), s: 0.0 };
    }

    // Playlist von vorn: erst Bereit-Pose (gute Ellenbogen-Seite), dann zu Punkt 1
    /// Gerade geht nicht: den Punkt per IK mit Neustarts lösen und im Gelenkraum hinfahren.
    /// Nur wenn der Punkt wirklich unerreichbar ist, wird er übersprungen.
    fn joint_fallback(&mut self, wp: &Waypoint) {
        let opt = kin::IkOptions { use_orientation: false, pos_tol: 1e-3, rot_tol: 1e-5, restarts: 20 };
        let r = kin::solve_ik(&self.cfg.arm, &kin::from_xyz_rpy(wp.pos, [0.0; 3]), self.q_cmd, &opt);
        if r.converged {
            self.play = Play::Joint { from: self.q_cmd, to: r.q, s: 0.0 };
            self.ik_msg = Some((true, format!("Playlist · Punkt {} (Gelenkfahrt)", self.pl_idx + 1)));
        } else {
            if !self.skipped.contains(&self.pl_idx) {
                self.skipped.push(self.pl_idx);
            }
            self.ik_msg = Some((false, format!("Punkt {} außer Reichweite", self.pl_idx + 1)));
            self.play = Play::Dwell(0.0);
        }
    }

    /// Statuszeile während der Playlist; rot, wenn Punkte außer Reichweite übersprungen wurden.
    fn play_msg(&self) -> (bool, String) {
        let mut s = format!("Playlist · Punkt {}", self.pl_idx + 1);
        if !self.skipped.is_empty() {
            let l: Vec<String> = self.skipped.iter().map(|i| (i + 1).to_string()).collect();
            s += &format!(" · außer Reichweite: {}", l.join(", "));
        }
        (self.skipped.is_empty(), s)
    }

    fn play_all(&mut self) {
        if !self.cfg.waypoints.is_empty() {
            self.trail.clear();
            self.skipped.clear();
            self.set_pose([0.0, -35.0, 70.0, 0.0, 55.0, 0.0]);
            self.goto_waypoint(0);
            self.play = Play::Approach;
        }
    }

    /// Ein Schritt der Playlist: Gerade zum Zielpunkt mit konstanter TCP-Geschwindigkeit,
    /// begrenzt durch die maximale Gelenkgeschwindigkeit; Gelenkwinkel per IK mit Warmstart.
    fn playlist_step(&mut self, dt: f64) {
        let Some(wp) = self.cfg.waypoints.get(self.pl_idx).cloned() else {
            self.play = Play::Idle;
            return;
        };
        match self.play {
            Play::Idle => {}
            Play::Approach => {
                if (0..6).all(|i| (self.q_cmd[i] - self.q_view[i]).abs() < 1e-6) {
                    self.play = Play::Dwell(wp.dwell);
                }
            }
            Play::Dwell(t) => {
                if t - dt > 0.0 {
                    self.play = Play::Dwell(t - dt);
                } else if self.pl_idx + 1 < self.cfg.waypoints.len() {
                    self.start_move(self.pl_idx + 1);
                } else if self.looping {
                    self.start_move(0);
                } else {
                    self.play = Play::Idle;
                }
            }
            Play::Joint { from, to, s } => {
                let far = (0..6).map(|i| (to[i] - from[i]).abs()).fold(1e-9, f64::max);
                let rate = self.cfg.speed_dps.to_radians().max(1.5);
                let ns = (s + rate * dt / far).min(1.0);
                self.q_cmd = std::array::from_fn(|i| from[i] + (to[i] - from[i]) * ns);
                self.q_view = self.q_cmd;
                self.target_pos = self.tcp();
                self.play = if ns >= 1.0 { Play::Dwell(wp.dwell) } else { Play::Joint { from, to, s: ns } };
            }
            Play::Move { from, s } => {
                let d: V3 = std::array::from_fn(|k| wp.pos[k] - from[k]);
                let len = d.iter().map(|v| v * v).sum::<f64>().sqrt();
                if len < 1e-6 {
                    self.play = Play::Dwell(wp.dwell);
                    return;
                }
                let limit = self.cfg.speed_dps.to_radians().max(0.05) * dt;
                let mut step = self.cfg.tcp_speed.max(1.0) * dt;
                let opt = kin::IkOptions { use_orientation: false, pos_tol: 1e-3, rot_tol: 1e-5, restarts: 0 };
                for attempt in 0..2 {
                    let ns = (s + step).min(len);
                    let p: V3 = std::array::from_fn(|k| from[k] + d[k] * ns / len);
                    let r = kin::solve_ik(&self.cfg.arm, &kin::from_xyz_rpy(p, [0.0; 3]), self.q_cmd, &opt);
                    if !r.converged {
                        self.joint_fallback(&wp);
                        return;
                    }
                    let dq = (0..6).map(|i| (r.q[i] - self.q_cmd[i]).abs()).fold(0.0, f64::max);
                    if dq > limit && attempt == 0 {
                        step *= limit / dq;
                        continue;
                    }
                    if dq > 2.0 * limit {
                        // Sprung im Gelenkraum: Singularität oder Lösungswechsel
                        self.joint_fallback(&wp);
                        return;
                    }
                    self.q_cmd = r.q;
                    self.q_view = r.q;
                    self.target_pos = p;
                    self.play = if ns >= len { Play::Dwell(wp.dwell) } else { Play::Move { from, s: ns } };
                    self.ik_msg = Some(self.play_msg());
                    break;
                }
            }
        }
    }
}

impl eframe::App for App {
    fn ui(&mut self, ui: &mut egui::Ui, _frame: &mut eframe::Frame) {
        let ctx = ui.ctx().clone();
        if let Ok(path) = std::env::var("ROBOTARM_SHOT") {
            let pass = std::env::var("ROBOTARM_PASS").ok().and_then(|p| p.parse().ok()).unwrap_or(10);
            if ctx.cumulative_pass_nr() == pass {
                let c = ctx.clone();
                ctx.send_viewport_cmd(egui::ViewportCommand::Screenshot(egui::ScreenshotCallback::new(move |img| {
                    let mut d = format!("P6\n{} {}\n255\n", img.width(), img.height()).into_bytes();
                    d.extend(img.pixels.iter().flat_map(|p| [p.r(), p.g(), p.b()]));
                    let _ = std::fs::write(path, d);
                    c.send_viewport_cmd(egui::ViewportCommand::Close);
                })));
            }
            ctx.request_repaint();
        }
        let dt = ctx.input(|i| i.stable_dt as f64).min(0.1);
        self.playlist_step(dt);
        let moving = self.advance(dt);
        if moving || self.playing() {
            ctx.request_repaint();
        }
        self.show_panels(ui, moving || self.playing());
    }
}

// Fenster auf und los!
fn main() -> eframe::Result {
    let opts = eframe::NativeOptions {
        viewport: egui::ViewportBuilder::default().with_inner_size([1480.0, 900.0]).with_title("Arm·Sim – 6-DOF Roboterarm"),
        multisampling: 1,
        ..Default::default()
    };
    eframe::run_native("robotarm-sim", opts, Box::new(|cc| Ok(Box::new(App::new(cc)))))
}

#[cfg(test)]
mod tests {
    use super::*;

    fn app() -> App {
        App {
            cfg: Config::default(),
            q_cmd: [0.0; 6],
            q_view: [0.0; 6],
            mode: Mode::Ik,
            tab: Tab::Control,
            target_pos: [0.0; 3],
            target_rpy_deg: [0.0; 3],
            use_orientation: false,
            ik_msg: None,
            cam: Camera::default(),
            trail: vec![],
            status: String::new(),
            play: Play::Idle,
            looping: false,
            pl_idx: 0,
            skipped: vec![],
        }
    }

    #[test]
    fn playlist_visits_all_waypoints_on_straight_lines() {
        let mut a = app();
        a.cfg.tcp_speed = 400.0;
        a.cfg.speed_dps = 720.0;
        a.play_all();
        let (mut visited, mut t) = (vec![false; a.cfg.waypoints.len()], 0.0);
        while a.playing() && t < 120.0 {
            a.playlist_step(0.02);
            a.advance(0.02);
            t += 0.02;
            let tcp = a.tcp();
            for (i, w) in a.cfg.waypoints.iter().enumerate() {
                if (0..3).map(|k| (w.pos[k] - tcp[k]).powi(2)).sum::<f64>().sqrt() < 1.0 {
                    visited[i] = true;
                }
            }
        }
        assert!(!a.playing(), "Playlist endet nicht");
        assert!(visited.iter().all(|v| *v), "nicht alle Punkte erreicht: {visited:?}");
    }

    #[test]
    fn loop_reaches_every_point_even_through_singularities() {
        let mut a = app();
        a.cfg.tcp_speed = 400.0;
        a.cfg.speed_dps = 720.0;
        a.cfg.waypoints = [[300.0, 0.0, 300.0], [-300.0, 0.0, 300.0], [0.0, 300.0, 500.0], [0.0, -300.0, 500.0]]
            .map(|pos| Waypoint { pos, dwell: 0.1 })
            .to_vec();
        a.looping = true;
        a.play_all();
        let (mut hits, mut t) = (0, 0.0);
        let mut last = usize::MAX;
        while t < 90.0 {
            a.playlist_step(0.02);
            a.advance(0.02);
            t += 0.02;
            let tcp = a.tcp();
            for (i, w) in a.cfg.waypoints.iter().enumerate() {
                if i != last && (0..3).map(|k| (w.pos[k] - tcp[k]).powi(2)).sum::<f64>().sqrt() < 1.0 {
                    last = i;
                    hits += 1;
                }
            }
            assert!(a.playing(), "Loop ist stehen geblieben nach {hits} Punkten");
        }
        assert!(hits >= 12, "nur {hits} Punkte erreicht");
    }
}
