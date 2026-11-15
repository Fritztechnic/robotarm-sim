#![allow(deprecated)]
// gpu = 3D-Renderer, icons = SVGs, kin = Mathe, ui = Panels, view = Kamera & Overlays
mod gpu;
mod icons;
mod kin;
mod math;
mod plan;
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

// Einfache Demo: ein Rechteck und ein Ausreißer nach oben (die Tests nutzen sie als Grundlage)
fn demo_simple() -> Vec<Waypoint> {
    [[300.0, -150.0, 250.0], [300.0, 150.0, 250.0], [300.0, 150.0, 450.0], [300.0, -150.0, 450.0], [150.0, 0.0, 600.0]]
        .map(|pos| Waypoint { pos, dwell: 0.3 })
        .to_vec()
}

// Große Demo: Rechteck, Kreis, Zickzack-Raster, Doppel-Spirale und ein Stern, alles in einer Schleife
fn demo_waypoints() -> Vec<Waypoint> {
    let mut v: Vec<(V3, f64)> = vec![([300.0, 0.0, 400.0], 0.4)];
    // Rechteck in der Ebene x = 330
    for (y, z) in [(-130.0, 250.0), (130.0, 250.0), (130.0, 450.0), (-130.0, 450.0)] {
        v.push(([330.0, y, z], 0.2));
    }
    // Kreis (r = 100 mm) in der Ebene x = 340
    for k in 0..12 {
        let t = k as f64 / 12.0 * std::f64::consts::TAU;
        v.push(([340.0, 100.0 * t.sin(), 350.0 + 100.0 * t.cos()], 0.0));
    }
    // Zickzack-Raster knapp über dem Tisch
    for (x, y) in [(250.0, -150.0), (250.0, 150.0), (300.0, 150.0), (300.0, -150.0), (350.0, -150.0), (350.0, 150.0)] {
        v.push(([x, y, 180.0], 0.1));
    }
    // Spirale um die z-Achse, nach oben enger werdend
    for k in 0..10 {
        // nur ±150° um die Z-Achse: J1 hat bei ±175° einen Anschlag
        let (t, f) = (-2.6 + k as f64 * 0.58, k as f64 / 9.0);
        let r = 260.0 - 120.0 * f;
        v.push(([r * t.cos(), r * t.sin(), 250.0 + 300.0 * f], 0.0));
    }
    // Fünfzackiger Stern hoch über dem Arm (jeder zweite Eckpunkt)
    for k in 0..5 {
        let t = std::f64::consts::FRAC_PI_2 + k as f64 * 2.0 * std::f64::consts::TAU / 5.0;
        v.push(([220.0, 160.0 * t.cos(), 500.0 + 110.0 * t.sin()], 0.15));
    }
    v.into_iter().map(|(pos, dwell)| Waypoint { pos, dwell }).collect()
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
    std::path::Path::new(&home).join(".config/robotarm-sim/config-v5.json")
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

/// Ablauf der Playlist: Strecke abfahren, dann Wartezeit am Zielpunkt.
#[derive(Clone, Copy, PartialEq)]
enum Play {
    Idle,
    /// Anfahrt zur ersten Stellung im Gelenkraum
    Joint { from: [f64; 6], to: [f64; 6], s: f64 },
    /// Vorberechnete Strecke k des Plans, u = Position (Sample-Index mit Bruchteil)
    Seg { k: usize, u: f64 },
    Dwell(f64),
}

// Der ganze Programmzustand an einem Ort
enum ReachState {
    None,
    Busy(Arm, std::sync::mpsc::Receiver<kin::Reach>),
    Ready(Arm, std::rc::Rc<kin::Reach>),
}

impl Default for ReachState {
    fn default() -> Self {
        Self::None
    }
}

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
    /// Vorausberechnete Trajektorie der Playlist
    plan: plan::Plan,
    /// Mathe-Fenster sichtbar?
    show_math: bool,
    show_formulas: bool,
    /// Zwischenspeicher der Erreichbarkeit (gilt nur für die gespeicherte Arm-Geometrie)
    reach_cache: std::cell::RefCell<ReachState>,
}

impl App {
    fn new(cc: &eframe::CreationContext<'_>) -> Self {
        let cfg: Config = std::fs::read_to_string(config_path())
            .ok()
            .and_then(|s| serde_json::from_str(&s).ok())
            .unwrap_or_else(|| {
                // Migration aus v4: Playlist/Einstellungen behalten, Gelenkgrenzen auf die Standardwerte zurück
                let mut c: Config = std::fs::read_to_string(config_path().with_file_name("config-v4.json"))
                    .ok()
                    .and_then(|s| serde_json::from_str(&s).ok())
                    .unwrap_or_default();
                for (j, d) in c.arm.iter_mut().zip(kin::cobot_arm()) {
                    (j.min_deg, j.max_deg) = (d.min_deg, d.max_deg);
                }
                c
            });
        egui_extras::install_image_loaders(&cc.egui_ctx);
        // Für Screenshots: Zusatzfenster im Hauptfenster einbetten
        cc.egui_ctx.set_embed_viewports(std::env::var("ROBOTARM_EMBED").is_ok());
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
            plan: plan::Plan::default(),
            show_math: false,
            show_formulas: false,
            reach_cache: Default::default(),
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
        app.show_math = std::env::var("ROBOTARM_MATH").is_ok();
        app.show_formulas = std::env::var("ROBOTARM_FORMULAS").is_ok();
        if std::env::var("ROBOTARM_PLAY").is_ok() {
            app.play_all();
        }
        app
    }

    /// Arbeitsraum-Hülle; wird im Hintergrund berechnet (ca. 1 s) und nur bei geändertem Aufbau neu.
    fn reach(&self, ctx: &egui::Context) -> Option<std::rc::Rc<kin::Reach>> {
        let mut c = self.reach_cache.borrow_mut();
        if let ReachState::Ready(arm, r) = &*c {
            if *arm == self.cfg.arm {
                return Some(r.clone());
            }
        }
        if let ReachState::Busy(arm, rx) = &*c {
            if *arm == self.cfg.arm {
                if let Ok(r) = rx.try_recv() {
                    let r = std::rc::Rc::new(r);
                    *c = ReachState::Ready(self.cfg.arm, r.clone());
                    return Some(r);
                }
                ctx.request_repaint_after(std::time::Duration::from_millis(150));
                return None;
            }
        }
        let (tx, rx) = std::sync::mpsc::channel();
        let arm = self.cfg.arm;
        std::thread::spawn(move || {
            let _ = tx.send(kin::reach(&arm));
        });
        *c = ReachState::Busy(self.cfg.arm, rx);
        ctx.request_repaint_after(std::time::Duration::from_millis(150));
        None
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

    /// Statuszeile während der Playlist; rot, wenn Punkte außer Reichweite übersprungen wurden.
    fn play_msg(&self) -> (bool, String) {
        let mut s = format!("Playlist · Punkt {}", self.pl_idx + 1);
        if let Play::Seg { k, .. } = self.play {
            if self.plan.segs[k].ptp {
                s += " · Gelenkfahrt";
            }
        }
        if !self.skipped.is_empty() {
            let l: Vec<String> = self.skipped.iter().map(|i| (i + 1).to_string()).collect();
            s += &format!(" · außer Reichweite: {}", l.join(", "));
        }
        (self.skipped.is_empty(), s)
    }

    // Playlist von vorn: erst alles vorausberechnen (plan.rs), dann nur noch abspielen.
    fn play_all(&mut self) {
        self.trail.clear();
        self.q_cmd = self.q_view; // Start von der gerade sichtbaren Pose
        let pts: Vec<V3> = self.cfg.waypoints.iter().map(|w| w.pos).collect();
        self.plan = plan::build(&self.cfg.arm, &pts);
        self.skipped = self.plan.unreachable.clone();
        if self.plan.unreachable.len() == pts.len() {
            self.play = Play::Idle;
            self.ik_msg = Some((false, "Kein Punkt erreichbar".into()));
            return;
        }
        self.pl_idx = self.plan.first;
        self.play = Play::Joint { from: self.q_cmd, to: self.plan.start, s: 0.0 };
        self.ik_msg = Some(self.play_msg());
    }

    /// Nächste Strecke nach Punkt `pl_idx` starten (oder Ende).
    fn next_seg(&mut self) {
        let k = self.plan.segs.iter().position(|s| s.from == self.pl_idx);
        match k {
            Some(k) if !self.plan.segs[k].closing || self.looping => {
                self.pl_idx = self.plan.segs[k].to;
                self.play = Play::Seg { k, u: 0.0 };
                self.ik_msg = Some(self.play_msg());
            }
            _ => self.play = Play::Idle,
        }
    }

    /// Ein Schritt der Playlist: nur noch Abspielen der vorberechneten Trajektorie.
    fn playlist_step(&mut self, dt: f64) {
        if self.play == Play::Idle {
            return;
        }
        let pts: Vec<V3> = self.cfg.waypoints.iter().map(|w| w.pos).collect();
        if pts != self.plan.pts {
            // Punkte wurden während der Fahrt geändert: Plan wäre veraltet
            self.play = Play::Idle;
            return;
        }
        let dwell = |i: usize| self.cfg.waypoints[i].dwell;
        match self.play {
            Play::Idle => {}
            Play::Dwell(t) => {
                if t - dt > 0.0 {
                    self.play = Play::Dwell(t - dt);
                } else {
                    self.next_seg();
                }
            }
            Play::Joint { from, to, s } => {
                let far = (0..6).map(|i| (to[i] - from[i]).abs()).fold(1e-9, f64::max);
                let rate = self.cfg.speed_dps.to_radians().max(0.05);
                let ns = (s + rate * dt / far).min(1.0);
                self.q_cmd = std::array::from_fn(|i| from[i] + (to[i] - from[i]) * ns);
                self.q_view = self.q_cmd;
                self.target_pos = self.tcp();
                self.play = if ns >= 1.0 { Play::Dwell(dwell(self.pl_idx)) } else { Play::Joint { from, to, s: ns } };
            }
            Play::Seg { k, u } => {
                let seg = &self.plan.segs[k];
                let (nu, end) = plan::advance(&self.cfg.arm, seg, u, dt, self.cfg.tcp_speed, self.cfg.speed_dps);
                self.q_cmd = plan::sample(seg, nu);
                self.q_view = self.q_cmd;
                self.target_pos = self.tcp();
                self.play = if end { Play::Dwell(dwell(seg.to)) } else { Play::Seg { k, u: nu } };
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
            plan: plan::Plan::default(),
            show_math: false,
            show_formulas: false,
            reach_cache: Default::default(),
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

    #[test]
    fn loop_is_stable_over_many_rounds() {
        let mut pts = demo_simple();
        for pos in [[-50.0, 3.0, 579.0], [171.0, 209.0, 579.0], [236.0, 19.0, 579.0], [222.0, 249.0, 579.0]] {
            pts.push(Waypoint { pos, dwell: 0.3 });
        }
        for dt in [0.004, 0.016, 0.1] {
            let mut a = app();
            a.cfg.waypoints = pts.clone();
            a.cfg.tcp_speed = 230.0;
            a.looping = true;
            a.play_all();
            let (mut t, mut prev, mut dwells, mut worst) = (0.0, a.q_view, 0, 0.0f64);
            let mut in_dwell = false;
            let mut seen: Vec<Option<[f64; 6]>> = vec![None; a.cfg.waypoints.len()];
            while t < 120.0 {
                a.playlist_step(dt);
                a.advance(dt);
                t += dt;
                assert!(a.playing(), "dt {dt}: Playlist gestoppt bei {t:.1}s");
                worst = worst.max((0..6).map(|i| (a.q_view[i] - prev[i]).abs()).fold(0.0, f64::max));
                prev = a.q_view;
                let d = matches!(a.play, Play::Dwell(_));
                if d && !in_dwell {
                    dwells += 1;
                    let w = a.cfg.waypoints[a.pl_idx].pos;
                    let e = (0..3).map(|k| (w[k] - a.tcp()[k]).powi(2)).sum::<f64>().sqrt();
                    assert!(e < 1.0, "dt {dt}: Punkt {} um {e:.1} mm verfehlt", a.pl_idx + 1);
                    // jede Runde identisch: derselbe Punkt wird immer mit denselben Gelenkwinkeln erreicht
                    let q = *seen[a.pl_idx].get_or_insert(a.q_cmd);
                    assert!((0..6).all(|i| (q[i] - a.q_cmd[i]).abs() < 1e-9), "dt {dt}: Punkt {} Runde für Runde verschieden", a.pl_idx + 1);
                }
                in_dwell = d;
            }
            assert!(dwells >= 40, "dt {dt}: nur {dwells} Punkte in 120 s");
            assert!(worst < 0.5, "dt {dt}: Gelenksprung {:.0}° pro Frame", worst.to_degrees());
        }
    }

    #[test]
    fn big_demo_is_fully_reachable() {
        let pts: Vec<V3> = demo_waypoints().iter().map(|w| w.pos).collect();
        let p = plan::build(&kin::cobot_arm(), &pts);
        let ptp = p.segs.iter().filter(|s| s.ptp).count();
        println!("{} Punkte, {} Segmente, {} Gelenkfahrten, unerreichbar {:?}", pts.len(), p.segs.len(), ptp, p.unreachable);
        for s in p.segs.iter().filter(|s| s.ptp) {
            println!("PTP {} -> {}  {:?} -> {:?}", s.from + 1, s.to + 1, pts[s.from], pts[s.to]);
        }
        assert!(p.unreachable.is_empty());
        assert!(ptp * 4 <= p.segs.len(), "zu viele Gelenkfahrten: {ptp}");
    }

    #[test]
    fn plan_is_deterministic_and_closed() {
        let mut s = 7u64;
        let mut rnd = move || { s = s.wrapping_mul(6364136223846793005).wrapping_add(1442695040888963407); (s >> 33) as f64 / (1u64 << 31) as f64 };
        for _ in 0..10 {
            let pts: Vec<V3> = (0..8).map(|_| { let (r, t) = (150.0 + rnd() * 400.0, rnd() * 6.283); [r * t.cos(), r * t.sin(), 100.0 + rnd() * 480.0] }).collect();
            let arm = kin::cobot_arm();
            let (a, b) = (plan::build(&arm, &pts), plan::build(&arm, &pts));
            assert_eq!(a.segs.len(), b.segs.len());
            for (x, y) in a.segs.iter().zip(&b.segs) {
                assert!(x.q == y.q && x.ptp == y.ptp);
            }
            if let Some(c) = a.segs.last().filter(|s| s.closing) {
                assert!((0..6).all(|i| (c.q.last().unwrap()[i] - a.start[i]).abs() < 1e-9), "Schleife schließt nicht");
            }
        }
    }
}
