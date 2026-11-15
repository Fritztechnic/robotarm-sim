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
    /// Pro Punkt die feste Gelenklösung (vom selben Startpunkt aus gelöst), None = außer Reichweite
    wp_q: Vec<Option<[f64; 6]>>,
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
            wp_q: vec![],
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

    /// Nächster erreichbarer Punkt nach `i` (bei Schleife mit Umlauf), sonst None.
    fn next_reachable(&self, i: usize) -> Option<usize> {
        let n = self.cfg.waypoints.len();
        let range = if self.looping { 1..=n } else { 1..=n.saturating_sub(1 + i) };
        range.map(|k| (i + k) % n).find(|&j| self.wp_q.get(j).is_some_and(Option::is_some))
    }

    /// Im Gelenkraum zur festen Lösung von Punkt i fahren, danach Wartezeit.
    fn joint_to(&mut self, i: usize) {
        self.pl_idx = i;
        let to = self.wp_q.get(i).copied().flatten().unwrap_or(self.q_cmd);
        self.play = Play::Joint { from: self.q_cmd, to, s: 0.0 };
        self.ik_msg = Some(self.play_msg());
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

    // Playlist von vorn. Jeder Punkt wird einmal fest gelöst (immer von der Bereit-Pose aus),
    // damit der Arm nie zufällig die Ellenbogenseite wechselt oder abdriftet.
    fn play_all(&mut self) {
        const READY: [f64; 6] = [0.0, -35.0, 70.0, 0.0, 55.0, 0.0];
        self.trail.clear();
        self.skipped.clear();
        let seed = READY.map(f64::to_radians);
        self.q_cmd = self.q_view; // ab hier startet die Bewegung von der gerade sichtbaren Pose
        let opt = kin::IkOptions { use_orientation: false, pos_tol: 1e-3, rot_tol: 1e-5, restarts: 20 };
        self.wp_q = self
            .cfg
            .waypoints
            .iter()
            .map(|w| {
                let r = kin::solve_ik(&self.cfg.arm, &kin::from_xyz_rpy(w.pos, [0.0; 3]), seed, &opt);
                r.converged.then_some(r.q)
            })
            .collect();
        self.skipped = (0..self.wp_q.len()).filter(|&i| self.wp_q[i].is_none()).collect();
        match self.wp_q.iter().position(Option::is_some) {
            Some(first) => self.joint_to(first),
            None => self.ik_msg = Some((false, "Kein Punkt erreichbar".into())),
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
            Play::Dwell(t) => {
                if t - dt > 0.0 {
                    self.play = Play::Dwell(t - dt);
                } else if let Some(n) = self.next_reachable(self.pl_idx) {
                    self.start_move(n);
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
                        self.joint_to(self.pl_idx);
                        return;
                    }
                    let dq = (0..6).map(|i| (r.q[i] - self.q_cmd[i]).abs()).fold(0.0, f64::max);
                    if dq > limit && attempt == 0 {
                        step *= limit / dq;
                        continue;
                    }
                    if dq > 2.0 * limit {
                        // Sprung im Gelenkraum: Singularität oder Lösungswechsel
                        self.joint_to(self.pl_idx);
                        return;
                    }
                    self.q_cmd = r.q;
                    self.q_view = r.q;
                    self.target_pos = p;
                    self.play = Play::Move { from, s: ns };
                    if ns >= len {
                        // Am Ziel exakt auf die feste Lösung einrasten (kurze Gelenkfahrt, falls abweichend)
                        let to = self.wp_q.get(self.pl_idx).copied().flatten().unwrap_or(r.q);
                        let off = (0..6).map(|i| (to[i] - r.q[i]).abs()).fold(0.0, f64::max);
                        self.play = if off > 1e-3 { Play::Joint { from: r.q, to, s: 0.0 } } else { Play::Dwell(wp.dwell) };
                    }
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
            wp_q: vec![],
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
        let mut pts = demo_waypoints();
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
                }
                in_dwell = d;
            }
            assert!(dwells >= 40, "dt {dt}: nur {dwells} Punkte in 120 s");
            assert!(worst < 0.5, "dt {dt}: Gelenksprung {:.0}° pro Frame", worst.to_degrees());
        }
    }
}
