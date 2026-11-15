//! 3D-Ansicht des Arms (GPU-Rendering über `gpu.rs`) inklusive Maus-Steuerung
//! des Zielpunkts direkt im 3D-Raum.

use crate::gpu::{SceneCallback, Uniforms, FLOOR, SHADOW_PASSES, UNIFORM_VEC4};
use crate::kin::{self, Arm, M4, V3};
use eframe::egui::{self, Color32, Pos2, Sense, Stroke};
use eframe::egui_wgpu;
use serde::{Deserialize, Serialize};

#[derive(Clone, PartialEq, Serialize, Deserialize)]
#[serde(default)]
pub struct Style {
    pub accent: [u8; 3],
    pub background: [u8; 3],
    pub grid_step: f64,
    pub fov_deg: f64,
    pub show_grid: bool,
    pub show_frames: bool,
    pub show_labels: bool,
    pub show_target: bool,
    pub show_trail: bool,
    pub show_shadow: bool,
    #[serde(default = "yes")]
    pub show_reach: bool,
}

fn yes() -> bool {
    true
}

impl Default for Style {
    fn default() -> Self {
        Self {
            accent: [226, 112, 24],
            background: [246, 247, 248],
            grid_step: 50.0,
            fov_deg: 40.0,
            show_grid: true,
            show_frames: false,
            show_labels: true,
            show_target: true,
            show_trail: true,
            show_shadow: true,
            show_reach: true,
        }
    }
}

// Orbit-Kamera: Blickpunkt, Abstand, Drehwinkel
pub struct Camera {
    yaw: f64,
    pitch: f64,
    dist: f64,
    center: V3,
    /// (Offset Zielpunkt − Treffpunkt, vertikal?)
    grab: Option<(V3, bool)>,
}

impl Default for Camera {
    fn default() -> Self {
        Self { yaw: -0.9, pitch: 0.4, dist: 1250.0, center: [0.0, 0.0, 300.0], grab: None }
    }
}

impl Camera {
    pub fn basis(&self) -> (V3, V3, V3) {
        let p = Proj::new(self, 40.0, egui::Rect::from_min_size(Pos2::ZERO, egui::vec2(100.0, 100.0)));
        (p.r, p.u, p.f)
    }

    pub fn set_view(&mut self, yaw_deg: f64, pitch_deg: f64) {
        *self = Self { yaw: yaw_deg.to_radians(), pitch: pitch_deg.to_radians(), ..Self::default() };
    }
}

// Was die Maus im 3D-Bereich gemacht hat (Ziel gezogen, View gewechselt ...)
pub struct Interaction {
    pub new_target: Option<V3>,
    pub released: bool,
    pub dragging: bool,
}

fn add(a: V3, b: V3) -> V3 {
    [a[0] + b[0], a[1] + b[1], a[2] + b[2]]
}
fn sub(a: V3, b: V3) -> V3 {
    [a[0] - b[0], a[1] - b[1], a[2] - b[2]]
}
fn scale(a: V3, k: f64) -> V3 {
    [a[0] * k, a[1] * k, a[2] * k]
}
fn dot(a: V3, b: V3) -> f64 {
    a[0] * b[0] + a[1] * b[1] + a[2] * b[2]
}
fn cross(a: V3, b: V3) -> V3 {
    [a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]]
}
fn norm(a: V3) -> V3 {
    let l = dot(a, a).sqrt();
    if l < 1e-12 { [0.0, 0.0, 1.0] } else { scale(a, 1.0 / l) }
}

const NEAR: f64 = 5.0;
const FAR: f64 = 8000.0;

// Perspektive von Hand: 3D-Punkt -> Pixel, und Pixel -> Strahl (für die Maus)
struct Proj {
    eye: V3,
    r: V3,
    u: V3,
    f: V3,
    focal: f64,
    c: Pos2,
}

impl Proj {
    fn new(cam: &Camera, fov_deg: f64, rect: egui::Rect) -> Self {
        let (cp, sp) = (cam.pitch.cos(), cam.pitch.sin());
        let eye = add(cam.center, scale([cp * cam.yaw.cos(), cp * cam.yaw.sin(), sp], cam.dist));
        let f = norm(sub(cam.center, eye));
        let r = norm(cross(f, [0.0, 0.0, 1.0]));
        let u = cross(r, f);
        let focal = rect.height().max(1.0) as f64 / (2.0 * (fov_deg.to_radians() / 2.0).tan());
        Self { eye, r, u, f, focal, c: rect.center() }
    }

    // 3D -> Bildschirm. None, wenn der Punkt hinter der Kamera liegt
    fn pt(&self, p: V3) -> Option<Pos2> {
        let d = sub(p, self.eye);
        let z = dot(d, self.f);
        if z < NEAR {
            return None;
        }
        Some(Pos2::new(
            self.c.x + (dot(d, self.r) / z * self.focal) as f32,
            self.c.y - (dot(d, self.u) / z * self.focal) as f32,
        ))
    }

    // Mausposition -> Blickstrahl in die Szene
    fn ray(&self, p: Pos2) -> V3 {
        let x = (p.x - self.c.x) as f64 / self.focal;
        let y = (p.y - self.c.y) as f64 / self.focal;
        norm(add(self.f, add(scale(self.r, x), scale(self.u, -y))))
    }

    // Strahl mit einer Ebene schneiden: so zieht die Maus den Zielpunkt im Raum
    fn hit(&self, p: Pos2, origin: V3, n: V3) -> Option<V3> {
        let d = self.ray(p);
        let den = dot(d, n);
        if den.abs() < 1e-6 {
            return None;
        }
        let t = dot(sub(origin, self.eye), n) / den;
        (t > 0.0).then(|| add(self.eye, scale(d, t)))
    }
}

// ---------- Modell-Matrizen ----------

/// Modellmatrix je Teil: Frame(q) · Frame(Ruhepose)⁻¹. Teil 0 (Basis) bleibt stehen.
fn part_matrices(arm: &Arm, q: &[f64; 6]) -> [M4; 7] {
    let rest_arm = kin::cobot_arm();
    let q_rest: [f64; 6] = std::array::from_fn(|i| kin::COBOT_REST_THETA_DEG[i].to_radians() - rest_arm[i].offset_deg.to_radians());
    let (cur, rest) = (kin::fk_frames(arm, q), kin::fk_frames(&rest_arm, &q_rest));
    std::array::from_fn(|k| kin::mul(&cur[k], &kin::inv(&rest[k])))
}

fn push_mat(u: &mut Uniforms, base: usize, m: &M4) {
    for c in 0..4 {
        u[base + c] = std::array::from_fn(|r| m[r][c] as f32);
    }
}

struct Mat {
    col: [f32; 3],
    metal: f32,
    rough: f32,
}

fn mat(c: [u8; 3], metal: f32, rough: f32) -> Mat {
    Mat { col: c.map(|v| (v as f32 / 255.0).powf(2.2)), metal, rough }
}

// Zeichnet den ganzen 3D-Bereich: GPU-Szene, Spur, Playlist-Punkte, Gelenk-Scheiben, Ziel. Plus Mausinteraktion
pub fn draw(ui: &mut egui::Ui, cam: &mut Camera, st: &Style, arm: &Arm, q: &[f64; 6], target: &M4, trail: &[V3], wps: &[V3], cur: Option<usize>, reach: (V3, f64, f64)) -> Interaction {
    let rect = ui.available_rect_before_wrap();
    let resp = ui.allocate_rect(rect, Sense::click_and_drag());
    let mut out = Interaction { new_target: None, released: false, dragging: false };

    // --- Maus: Zielpunkt greifen ---
    let tpos = kin::pos(target);
    let pr = Proj::new(cam, st.fov_deg, rect);
    let near = st.show_target && pr.pt(tpos).zip(resp.hover_pos()).is_some_and(|(a, b)| a.distance(b) < 22.0);
    let shift = ui.input(|i| i.modifiers.shift);
    let plane = |vertical: bool| if vertical { norm([pr.f[0], pr.f[1], 0.0]) } else { [0.0, 0.0, 1.0] };
    if resp.drag_started_by(egui::PointerButton::Primary) && near {
        if let Some(p) = resp.interact_pointer_pos() {
            cam.grab = pr.hit(p, tpos, plane(shift)).map(|h| (sub(tpos, h), shift));
        }
    }
    if let Some((off, vertical)) = cam.grab {
        if resp.dragged_by(egui::PointerButton::Primary) {
            out.dragging = true;
            if let Some(h) = resp.interact_pointer_pos().and_then(|p| pr.hit(p, tpos, plane(shift))) {
                if shift != vertical {
                    cam.grab = Some((sub(tpos, h), shift));
                } else {
                    let n = add(h, off);
                    let mut np = if vertical { [tpos[0], tpos[1], n[2]] } else { [n[0], n[1], tpos[2]] };
                    np[2] = np[2].max(0.0);
                    out.new_target = Some(np);
                }
            }
        } else {
            cam.grab = None;
            out.released = true;
        }
    }
    if near || cam.grab.is_some() {
        ui.ctx().set_cursor_icon(if cam.grab.is_some() { egui::CursorIcon::Grabbing } else { egui::CursorIcon::Grab });
    }

    // --- Kamera ---
    if resp.dragged_by(egui::PointerButton::Primary) && cam.grab.is_none() {
        let d = resp.drag_delta();
        cam.yaw -= d.x as f64 * 0.008;
        cam.pitch = (cam.pitch + d.y as f64 * 0.008).clamp(-1.5, 1.5);
    }
    if resp.dragged_by(egui::PointerButton::Secondary) || resp.dragged_by(egui::PointerButton::Middle) {
        let d = resp.drag_delta();
        let k = cam.dist / pr.focal;
        cam.center = add(cam.center, add(scale(pr.r, -d.x as f64 * k), scale(pr.u, d.y as f64 * k)));
    }
    if resp.hovered() {
        let s = ui.input(|i| i.smooth_scroll_delta.y) as f64;
        cam.dist = (cam.dist * (-s * 0.0015).exp()).clamp(150.0, 5000.0);
    }
    let pr = Proj::new(cam, st.fov_deg, rect);
    let painter = ui.painter_at(rect);
    let fr = kin::fk_frames(arm, q);

    // --- GPU-Szene ---
    let light = norm([0.4, -0.5, 0.8]);
    let v3 = |a: V3, w: f64| [a[0] as f32, a[1] as f32, a[2] as f32, w as f32];
    let tan = (st.fov_deg.to_radians() / 2.0).tan();
    let bg = st.background.map(|c| c as f32 / 255.0);
    let mut u: Uniforms = [[0.0; 4]; UNIFORM_VEC4];
    u[..8].copy_from_slice(&[
        v3(pr.eye, 0.0),
        v3(pr.r, 0.0),
        v3(pr.u, 0.0),
        v3(pr.f, 0.0),
        [tan as f32, rect.width() / rect.height().max(1.0), NEAR as f32, FAR as f32],
        v3(light, 0.0),
        [bg[0], bg[1], bg[2], if st.show_grid { 1.0 } else { 0.0 }],
        [st.grid_step as f32, 0.2, 0.0, 0.0],
    ]);
    for k in 0..SHADOW_PASSES {
        let a = k as f64 / SHADOW_PASSES as f64 * std::f64::consts::TAU;
        u[8 + k] = v3(norm([light[0] + 0.03 * a.cos(), light[1] + 0.03 * a.sin(), light[2]]), 0.0);
    }
    let parts = part_matrices(arm, q);
    let body = mat(st.accent, 0.35, 0.34);
    for i in 0..16 {
        let m = parts.get(i).copied().unwrap_or(kin::IDENT);
        push_mat(&mut u, 12 + 4 * i, &m);
        let (c, metal, rough) = if i as u32 == FLOOR { ([0.0; 3], -1.0, 1.0) } else { (body.col, body.metal, body.rough) };
        u[76 + i] = [c[0], c[1], c[2], metal];
        u[92 + i] = [rough, 0.0, 0.0, 0.0];
    }
    painter.add(egui_wgpu::Callback::new_paint_callback(
        rect,
        SceneCallback { size_points: [rect.width(), rect.height()], uniforms: u, bg, shadows: st.show_shadow },
    ));

    let line = |a: V3, b: V3, stroke: Stroke| {
        if let (Some(p), Some(q)) = (pr.pt(a), pr.pt(b)) {
            painter.line_segment([p, q], stroke);
        }
    };

    // Ab hier nur noch 2D-Zeug obendrauf, das Hübsche kommt von der GPU
    // --- Overlays ---
    let triad = |m: &M4, len: f64, a: f32, w: f32| {
        let o = kin::pos(m);
        for (c, col) in [(0, [235, 90, 90]), (1, [100, 210, 120]), (2, [90, 140, 240])] {
            let end = add(o, scale(kin::col(m, c), len));
            line(o, end, Stroke::new(w, Color32::from_rgb(col[0], col[1], col[2]).gamma_multiply(a)));
        }
    };
    let glow = Color32::from_rgb(0, 120, 212);
    if st.show_reach {
        // außen = maximale Reichweite, innen = Totzone (dort kommt die Spitze nicht hin).
        // Bewusst wenig Linien: Umriss (Silhouette) + Äquator, innen zusätzlich leicht gefüllt.
        let (c, inner, outer) = reach;
        let tau = std::f64::consts::TAU;
        let n = 72;
        let d = sub(pr.eye, c);
        let dl = dot(d, d).sqrt().max(1e-6);
        let dn = scale(d, 1.0 / dl);
        let a = norm(cross(dn, [0.0, 0.0, 1.0]));
        let b = cross(a, dn);
        for (r, col, fill) in [(outer, Color32::from_rgb(0, 120, 212), 0u8), (inner, Color32::from_rgb(220, 80, 20), 38)] {
            let k = (r / dl).min(0.99);
            let (sc, sr) = (add(c, scale(dn, r * k)), r * (1.0 - k * k).sqrt());
            let sil: Vec<Pos2> = (0..n).filter_map(|i| {
                let t = i as f64 / n as f64 * tau;
                pr.pt(add(sc, add(scale(a, sr * t.cos()), scale(b, sr * t.sin()))))
            }).collect();
            if sil.len() == n {
                painter.add(egui::Shape::convex_polygon(sil, col.gamma_multiply(fill as f32 / 255.0), Stroke::new(1.6, col.gamma_multiply(0.85))));
            }
            let eq: Vec<Pos2> = (0..=n).filter_map(|i| {
                let t = i as f64 / n as f64 * tau;
                pr.pt([c[0] + r * t.cos(), c[1] + r * t.sin(), c[2]])
            }).collect();
            if eq.len() == n + 1 {
                painter.add(egui::Shape::line(eq, Stroke::new(1.0, col.gamma_multiply(0.5))));
            }
            if let Some(p) = pr.pt([c[0] + r, c[1], c[2]]) {
                let txt = if r == outer { format!("außen {:.0} mm", r) } else { format!("innen {:.0} mm", r) };
                painter.text(p + egui::vec2(6.0, -4.0), egui::Align2::LEFT_BOTTOM, txt, egui::FontId::proportional(10.5), col);
            }
        }
    }
    if st.show_trail && trail.len() > 1 {
        let n = trail.len();
        for (k, w) in trail.windows(2).enumerate() {
            let a = (k as f32 / n as f32).powf(1.5);
            line(w[0], w[1], Stroke::new(1.5, glow.gamma_multiply(0.15 + 0.7 * a)));
        }
    }
    if wps.len() > 1 {
        for w in wps.windows(2) {
            line(w[0], w[1], Stroke::new(1.0, glow.gamma_multiply(0.45)));
        }
    }
    for (i, w) in wps.iter().enumerate() {
        if let Some(p) = pr.pt(*w) {
            let on = cur == Some(i);
            painter.circle(p, 8.0, if on { glow } else { Color32::WHITE }, Stroke::new(1.5, glow));
            painter.text(p, egui::Align2::CENTER_CENTER, i + 1, egui::FontId::proportional(9.5), if on { Color32::WHITE } else { Color32::from_gray(50) });
        }
    }
    if st.show_frames {
        for m in fr.iter() {
            triad(m, 30.0, 0.9, 2.0);
        }
    }
    triad(&fr[6], 40.0, 0.9, 2.0);
    if st.show_labels {
        let ink = Color32::from_gray(60);
        for i in 0..6 {
            let ax = kin::col(&fr[i], 2);
            let (o0, o1) = (kin::pos(&fr[i]), kin::pos(&fr[i + 1]));
            // Scheibe auf die Achse legen, dort wo das Gelenk-Gehäuse sitzt (J1/J6 mittig, J4 an der Trennfuge des Unterarms)
            let frac = [0.5, 0.0, 0.0, 0.42, 0.0, 0.5][i];
            let o = add(o0, scale(ax, dot(sub(o1, o0), ax) * frac));
            let off = arm[i].offset_deg.to_radians();
            let spin = |v: V3, a: f64| add(scale(v, a.cos()), scale(cross(ax, v), a.sin()));
            let zero = spin(kin::col(&fr[i], 0), off);
            let ptr = kin::col(&fr[i + 1], 0);
            let r = if i == 5 { 42.0 } else { 62.0 };
            let at = |dir: V3, k: f64| add(o, scale(dir, r * k));
            line(add(o, scale(ax, -r * 1.25)), add(o, scale(ax, r * 1.25)), Stroke::new(1.0, Color32::from_rgb(0, 120, 212).gamma_multiply(0.55)));
            let ring = |k: f64, a0: f64, a1: f64, n: usize| -> Vec<Pos2> {
                (0..=n).filter_map(|s| pr.pt(at(spin(zero, a0 + (a1 - a0) * s as f64 / n as f64), k))).collect()
            };
            let tau = std::f64::consts::TAU;
            let full = ring(1.0, 0.0, tau, 64);
            if full.len() > 2 {
                painter.add(egui::Shape::convex_polygon(full.clone(), Color32::from_white_alpha(70), Stroke::NONE));
                painter.add(egui::Shape::line(full, Stroke::new(1.0, Color32::from_gray(110))));
            }
            for t in 0..12 {
                let d = spin(zero, tau * t as f64 / 12.0);
                line(at(d, 0.88), at(d, 1.0), Stroke::new(1.0, Color32::from_gray(130)));
            }
            let q_rad = q[i];
            let arc = ring(0.7, 0.0, q_rad, ((q_rad.abs() / tau * 64.0) as usize).max(2));
            if arc.len() > 1 {
                painter.add(egui::Shape::line(arc, Stroke::new(3.0, Color32::from_rgb(0, 120, 212).gamma_multiply(0.8))));
            }
            line(o, at(zero, 1.0), Stroke::new(1.0, Color32::from_gray(150)));
            line(o, at(ptr, 1.0), Stroke::new(2.0, Color32::from_rgb(230, 90, 20)));
            if let (Some(c), Some(e)) = (pr.pt(o), pr.pt(at(ptr, 1.0))) {
                painter.circle_filled(e, 3.0, Color32::from_rgb(230, 90, 20));
                painter.circle_filled(c, 2.0, ink);
                let s = painter.layout_no_wrap(format!("J{}  {:+.1}°", i + 1, q_rad.to_degrees()), egui::FontId::monospace(10.5), ink);
                let side = if i % 2 == 0 { 1.0 } else { -1.0 };
                let tp = Pos2::new(c.x + side * ((e - c).length().max(30.0) + 12.0), c.y);
                let rc = egui::Rect::from_min_size(
                    Pos2::new(if side > 0.0 { tp.x } else { tp.x - s.size().x }, tp.y - s.size().y / 2.0),
                    s.size(),
                );
                painter.rect_filled(rc.expand(2.0), 3.0, Color32::from_white_alpha(215));
                painter.galley(rc.min, s, ink);
            }
        }
    }
    if st.show_target {
        let ground = [tpos[0], tpos[1], 0.0];
        line(tpos, ground, Stroke::new(1.0, glow.gamma_multiply(0.5)));
        if let Some(p) = pr.pt(ground) {
            painter.circle_stroke(p, 5.0, Stroke::new(1.0, glow.gamma_multiply(0.7)));
        }
        triad(target, 35.0, 0.6, 1.5);
        if let Some(p) = pr.pt(tpos) {
            let active = near || cam.grab.is_some();
            painter.circle_filled(p, if active { 13.0 } else { 10.0 }, glow.gamma_multiply(if active { 0.30 } else { 0.14 }));
            painter.circle_stroke(p, 9.0, Stroke::new(if active { 2.5 } else { 1.5 }, glow));
            painter.circle_filled(p, 2.5, glow);
        }
    }
    painter.text(
        rect.left_bottom() + egui::vec2(12.0, -10.0),
        egui::Align2::LEFT_BOTTOM,
        "Zielpunkt ziehen: Arm steuern (Umschalt = Höhe) · Ziehen: drehen · Rechtsklick: verschieben · Mausrad: Zoom",
        egui::FontId::proportional(11.0),
        Color32::from_gray(120),
    );
    out
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn mouse_ray_hits_projected_point() {
        let rect = egui::Rect::from_min_size(Pos2::ZERO, egui::vec2(1000.0, 700.0));
        let mut cam = Camera::default();
        cam.set_view(-40.0, 30.0);
        let pr = Proj::new(&cam, 40.0, rect);
        let p = [80.0, -50.0, 210.0];
        let h = pr.hit(pr.pt(p).unwrap(), p, [0.0, 0.0, 1.0]).unwrap();
        let v = pr.hit(pr.pt(p).unwrap(), p, norm([pr.f[0], pr.f[1], 0.0])).unwrap();
        for i in 0..3 {
            assert!((h[i] - p[i]).abs() < 1e-3 && (v[i] - p[i]).abs() < 1e-3);
        }
    }
}
