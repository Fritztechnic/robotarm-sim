//! Oberfläche: Panels, Werkzeugleiste, Tabs, Playlist. Alle Symbole sind SVGs (`icons.rs`).

use super::*;
use crate::icons::{self, Icon};
use eframe::egui::{RichText, Sense, vec2};

// Helles Fusion-360-Theme
pub fn apply_theme(ctx: &egui::Context) {
    let mut v = egui::Visuals::light();
    v.panel_fill = Color32::from_rgb(246, 246, 247);
    v.window_fill = Color32::WHITE;
    v.extreme_bg_color = Color32::WHITE;
    v.faint_bg_color = Color32::from_rgb(240, 241, 243);
    v.override_text_color = Some(TEXT);
    v.selection.bg_fill = BLUE.gamma_multiply(0.22);
    v.selection.stroke = Stroke::new(1.0, BLUE);
    v.hyperlink_color = BLUE;
    let r = egui::CornerRadius::same(2);
    let w = &mut v.widgets;
    for s in [&mut w.noninteractive, &mut w.inactive, &mut w.hovered, &mut w.active, &mut w.open] {
        s.corner_radius = r;
    }
    w.inactive.bg_fill = Color32::from_rgb(232, 234, 237);
    w.inactive.weak_bg_fill = Color32::from_rgb(232, 234, 237);
    w.inactive.bg_stroke = Stroke::new(1.0, Color32::from_rgb(214, 216, 220));
    w.hovered.bg_fill = Color32::from_rgb(214, 232, 247);
    w.hovered.weak_bg_fill = Color32::from_rgb(214, 232, 247);
    w.hovered.bg_stroke = Stroke::new(1.0, BLUE);
    w.noninteractive.bg_stroke = Stroke::new(1.0, Color32::from_rgb(220, 222, 226));
    ctx.set_visuals(v);
    ctx.global_style_mut(|s| {
        s.spacing.item_spacing = vec2(6.0, 5.0);
        s.spacing.slider_width = 130.0;
        s.spacing.interact_size.y = 22.0;
    });
}

fn img(i: Icon, tint: Color32) -> egui::Image<'static> {
    egui::Image::new(icons::source(i)).tint(tint).fit_to_exact_size(vec2(15.0, 15.0))
}

/// Symbol-Schalter mit Tooltip.
fn ibtn(ui: &mut egui::Ui, i: Icon, tip: &str, on: bool) -> egui::Response {
    let tint = if on { BLUE } else { Color32::from_gray(60) };
    ui.add(egui::Button::image(img(i, tint)).selected(on).min_size(vec2(26.0, 24.0))).on_hover_text(tip)
}

/// Schaltfläche mit Symbol und Text.
fn tbtn(ui: &mut egui::Ui, i: Icon, text: &str) -> egui::Response {
    ui.add(egui::Button::image_and_text(img(i, Color32::from_gray(60)), text))
}

fn dv(ui: &mut egui::Ui, v: &mut f64, suffix: &str, speed: f64) -> bool {
    ui.add(egui::DragValue::new(v).speed(speed).suffix(suffix).max_decimals(3)).changed()
}

fn card(ui: &mut egui::Ui, title: Option<&str>, f: impl FnOnce(&mut egui::Ui)) {
    egui::Frame::new()
        .fill(Color32::WHITE)
        .stroke(Stroke::new(1.0, Color32::from_rgb(222, 224, 228)))
        .corner_radius(2)
        .inner_margin(9)
        .show(ui, |ui| {
            ui.set_width(ui.available_width());
            if let Some(t) = title {
                ui.label(RichText::new(t.to_uppercase()).size(10.5).strong().color(MUTED));
                ui.add_space(2.0);
            }
            f(ui);
        });
    ui.add_space(6.0);
}

fn panel_title(ui: &mut egui::Ui, text: &str) {
    egui::Frame::new().fill(Color32::from_rgb(226, 228, 231)).inner_margin(egui::Margin::symmetric(8, 4)).show(ui, |ui| {
        ui.set_width(ui.available_width());
        ui.label(RichText::new(text).size(10.5).strong().color(MUTED));
    });
}

fn toggle(ui: &mut egui::Ui, on: &mut bool) -> egui::Response {
    let (rect, mut resp) = ui.allocate_exact_size(vec2(36.0, 20.0), Sense::click());
    if resp.clicked() {
        *on = !*on;
        resp.mark_changed();
    }
    let t = ui.ctx().animate_bool_responsive(resp.id, *on);
    ui.painter().rect_filled(rect, 10.0, Color32::from_rgb(190, 194, 200).lerp_to_gamma(BLUE, t));
    let x = egui::lerp(rect.left() + 10.0..=rect.right() - 10.0, t);
    ui.painter().circle_filled(egui::pos2(x, rect.center().y), 7.0, Color32::WHITE);
    resp
}

fn axis_slider(ui: &mut egui::Ui, label: &str, color: Color32, v: &mut f64, range: std::ops::RangeInclusive<f64>, unit: &str) -> bool {
    ui.horizontal(|ui| {
        ui.label(RichText::new(label).strong().monospace().color(color));
        let r = ui.add(egui::Slider::new(v, range).suffix(unit).fixed_decimals(1));
        // egui meldet auch Klemm-Korrekturen als changed – nur echte Eingaben zählen (sonst stoppt die Playlist)
        r.changed() && (r.is_pointer_button_down_on() || r.dragged() || r.has_focus())
    })
    .inner
}

fn group(ui: &mut egui::Ui, label: &str, f: impl FnOnce(&mut egui::Ui)) {
    ui.vertical(|ui| {
        ui.horizontal(|ui| f(ui));
        ui.label(RichText::new(label).size(9.5).color(MUTED));
    });
    ui.separator();
}

impl App {
    // Baut das komplette Layout: Menü, Toolbar, links/rechts, unten Zeitleiste, Mitte 3D
    pub fn show_panels(&mut self, ui: &mut egui::Ui, active: bool) {
        let bar = |fill: Color32| egui::Frame::new().fill(fill).inner_margin(egui::Margin::symmetric(8, 4));
        let side = || egui::Frame::new().fill(Color32::from_rgb(246, 246, 247)).stroke(Stroke::new(1.0, BORDER));

        egui::Panel::top("menu").frame(bar(Color32::from_rgb(226, 228, 231))).show(ui, |ui| {
            ui.horizontal(|ui| {
                ui.label(RichText::new("ARM·SIM").strong().color(Color32::from_rgb(226, 112, 24)));
                ui.label(RichText::new("6-DOF · Elfin").size(11.0).color(MUTED));
                ui.add_space(16.0);
                for (t, n, ic) in [(Tab::Control, "STEUERUNG", Icon::Goto), (Tab::Playlist, "PLAYLIST", Icon::List), (Tab::Build, "AUFBAU", Icon::Sliders), (Tab::View, "ANSICHT", Icon::Eye)] {
                    let sel = self.tab == t;
                    let col = if sel { BLUE } else { Color32::from_gray(70) };
                    let r = ui
                        .horizontal(|ui| {
                            ui.add(img(ic, col));
                            ui.label(RichText::new(n).size(11.0).strong().color(col));
                        })
                        .response;
                    let r = ui.interact(r.rect, ui.id().with(n), Sense::click()).on_hover_cursor(egui::CursorIcon::PointingHand);
                    if sel {
                        ui.painter().line_segment([r.rect.left_bottom() + vec2(0.0, 3.0), r.rect.right_bottom() + vec2(0.0, 3.0)], Stroke::new(2.0, BLUE));
                    }
                    if r.clicked() {
                        self.tab = t;
                    }
                    ui.add_space(10.0);
                }
            });
        });
        egui::Panel::top("toolbar").frame(bar(BAR).stroke(Stroke::new(1.0, BORDER))).show(ui, |ui| {
            ui.horizontal(|ui| self.toolbar(ui));
        });
        egui::Panel::bottom("timeline").frame(bar(BAR).stroke(Stroke::new(1.0, BORDER))).show(ui, |ui| {
            ui.horizontal(|ui| self.timeline(ui));
        });
        egui::Panel::left("left").exact_size(362.0).frame(side()).show(ui, |ui| {
            panel_title(ui, match self.tab {
                Tab::Control => "STEUERUNG",
                Tab::Playlist => "PLAYLIST",
                Tab::Build => "AUFBAU · DH-PARAMETER & SERVOS",
                Tab::View => "ANSICHT",
            });
            egui::Frame::new().inner_margin(8).show(ui, |ui| {
                egui::ScrollArea::vertical().show(ui, |ui| match self.tab {
                    Tab::Control => self.control_tab(ui),
                    Tab::Playlist => self.playlist_tab(ui),
                    Tab::Build => self.build_tab(ui),
                    Tab::View => self.view_tab(ui),
                });
            });
        });
        egui::Panel::right("right").exact_size(230.0).frame(side()).show(ui, |ui| {
            panel_title(ui, "MOTOREN");
            egui::Frame::new().inner_margin(8).show(ui, |ui| self.motors(ui));
        });
        egui::CentralPanel::default().frame(egui::Frame::NONE).show(ui, |ui| {
            let rect = ui.max_rect();
            let tcp = self.tcp();
            if active || self.trail.is_empty() {
                let far = self.trail.last().is_none_or(|l| (0..3).map(|i| (l[i] - tcp[i]).powi(2)).sum::<f64>() > 4.0);
                if far {
                    self.trail.push(tcp);
                    if self.trail.len() > 1500 {
                        self.trail.remove(0);
                    }
                }
            }
            let target = kin::from_xyz_rpy(self.target_pos, self.target_rpy_deg.map(f64::to_radians));
            let wps: Vec<V3> = self.cfg.waypoints.iter().map(|w| w.pos).collect();
            let cur = (self.tab == Tab::Playlist || self.playing()).then_some(self.pl_idx);
            let res = view::draw(ui, &mut self.cam, &self.cfg.style, &self.cfg.arm, &self.q_view, &target, &self.trail, &wps, cur);
            if let Some(np) = res.new_target {
                self.play = Play::Idle;
                self.mode = Mode::Ik;
                self.target_pos = np;
                self.solve(0);
            }
            if res.released {
                self.solve(40);
            }
            self.view_cube(ui, rect);
        });
    }

    // Toolbar mit Posen, Ansichten, Anzeige-Schaltern
    fn toolbar(&mut self, ui: &mut egui::Ui) {
        group(ui, "POSEN", |ui| {
            for (n, q) in [("Null", [0.0; 6]), ("Bereit", [0.0, -35.0, 70.0, 0.0, 55.0, 0.0]), ("Ruhe", [0.0, -100.0, 135.0, 0.0, 60.0, 0.0])] {
                if ui.button(n).clicked() {
                    self.play = Play::Idle;
                    self.set_pose(q);
                }
            }
        });
        group(ui, "ANSICHT", |ui| {
            for (ic, n, y, p) in [(Icon::Iso, "Isometrisch", -52.0, 26.0), (Icon::Front, "Vorne", -90.0, 0.0), (Icon::Side, "Seite", 0.0, 0.0), (Icon::Top, "Oben", -90.0, 89.0)] {
                if ibtn(ui, ic, n, false).clicked() {
                    self.cam.set_view(y, p);
                }
            }
            if ibtn(ui, Icon::Home, "Kamera zurücksetzen", false).clicked() {
                self.cam = Camera::default();
            }
        });
        let st = &mut self.cfg.style;
        group(ui, "ANZEIGE", |ui| {
            for (ic, n, v) in [
                (Icon::Grid, "Raster", &mut st.show_grid),
                (Icon::Shadow, "Schatten", &mut st.show_shadow),
                (Icon::Trail, "TCP-Spur", &mut st.show_trail),
                (Icon::Axes, "Koordinatensysteme", &mut st.show_frames),
                (Icon::Tag, "Gelenk-Beschriftung", &mut st.show_labels),
                (Icon::Marker, "Zielmarker", &mut st.show_target),
            ] {
                if ibtn(ui, ic, n, *v).clicked() {
                    *v = !*v;
                }
            }
        });
        group(ui, "KONFIGURATION", |ui| {
            if ibtn(ui, Icon::Save, "Konfiguration und Playlist speichern", false).clicked() {
                let p = config_path();
                let r = std::fs::create_dir_all(p.parent().unwrap()).and_then(|_| std::fs::write(&p, serde_json::to_string_pretty(&self.cfg).unwrap()));
                self.status = match r {
                    Ok(_) => "Gespeichert".into(),
                    Err(e) => format!("Fehler: {e}"),
                };
            }
            if !self.status.is_empty() {
                ui.label(RichText::new(&self.status).size(11.0).color(MUTED));
            }
        });
    }

    /// Fusion-artige Zeitleiste: Abspielen, Schleife, Zielpunkte, Status.
    fn timeline(&mut self, ui: &mut egui::Ui) {
        let n = self.cfg.waypoints.len();
        if self.playing() {
            if ibtn(ui, Icon::Stop, "Stopp", true).clicked() {
                self.play = Play::Idle;
            }
        } else if ibtn(ui, Icon::Play, "Playlist abspielen", false).clicked() {
            self.play_all();
        }
        if ibtn(ui, Icon::Repeat, "Schleife", self.looping).clicked() {
            self.looping = !self.looping;
        }
        let w = (ui.available_width() - 470.0).max(120.0);
        let (rect, _) = ui.allocate_exact_size(vec2(w, 24.0), Sense::hover());
        let y = rect.center().y;
        ui.painter().line_segment([egui::pos2(rect.left() + 8.0, y), egui::pos2(rect.right() - 8.0, y)], Stroke::new(2.0, Color32::from_gray(200)));
        for i in 0..n {
            let x = if n == 1 { rect.center().x } else { egui::lerp((rect.left() + 12.0)..=(rect.right() - 12.0), i as f32 / (n - 1) as f32) };
            let c = egui::pos2(x, y);
            let r = ui.interact(egui::Rect::from_center_size(c, vec2(18.0, 20.0)), ui.id().with(("tl", i)), Sense::click()).on_hover_text(format!("Punkt {} anfahren", i + 1));
            let cur = self.pl_idx == i && (self.playing() || self.tab == Tab::Playlist);
            ui.painter().circle(c, if cur { 8.0 } else { 6.5 }, if cur { BLUE } else { Color32::WHITE }, Stroke::new(1.5, if cur || r.hovered() { BLUE } else { Color32::from_gray(140) }));
            ui.painter().text(c, egui::Align2::CENTER_CENTER, i + 1, egui::FontId::proportional(8.5), if cur { Color32::WHITE } else { Color32::from_gray(60) });
            if r.clicked() {
                self.goto_waypoint(i);
            }
        }
        ui.separator();
        let m = kin::fk(&self.cfg.arm, &self.q_view);
        let (p, r) = (kin::pos(&m), kin::to_rpy(&m).map(f64::to_degrees));
        let (ok, msg) = self.ik_msg.clone().unwrap_or((true, "Bereit".into()));
        ui.label(RichText::new("●").color(if ok { OK } else { BAD }));
        ui.label(RichText::new(msg).size(12.0));
        ui.separator();
        ui.label(RichText::new(format!("X {:+.0}  Y {:+.0}  Z {:+.0} mm", p[0], p[1], p[2])).monospace().size(12.0));
        ui.label(RichText::new(format!("R {:+.0}° P {:+.0}° Y {:+.0}°", r[0], r[1], r[2])).monospace().size(12.0).color(MUTED));
    }

    // Reiter Steuerung: Ziel (X/Y/Z), Werkzeugwinkel an/aus, Gelenke
    fn control_tab(&mut self, ui: &mut egui::Ui) {
        ui.horizontal(|ui| {
            ui.selectable_value(&mut self.mode, Mode::Ik, "Zielpunkt");
            ui.selectable_value(&mut self.mode, Mode::Fk, "Gelenke");
        });
        ui.add_space(4.0);
        if self.mode == Mode::Ik {
            let mut ch = false;
            card(ui, Some("Zielposition"), |ui| {
                for (i, (n, c, r)) in [("X", Color32::from_rgb(220, 50, 50), -600.0..=600.0), ("Y", Color32::from_rgb(40, 160, 70), -600.0..=600.0), ("Z", Color32::from_rgb(40, 90, 220), 0.0..=750.0)].into_iter().enumerate() {
                    ch |= axis_slider(ui, n, c, &mut self.target_pos[i], r, " mm");
                }
                if tbtn(ui, Icon::Marker, "Ziel = aktueller TCP").clicked() {
                    self.sync_target_from_q();
                    self.ik_msg = None;
                }
            });
            card(ui, Some("Werkzeugwinkel"), |ui| {
                ui.horizontal(|ui| {
                    ui.label("Winkel vorgeben");
                    ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
                        ch |= toggle(ui, &mut self.use_orientation).changed();
                    });
                });
                let names = [("Roll", -180.0..=180.0), ("Pitch", -90.0..=90.0), ("Yaw", -180.0..=180.0)];
                if self.use_orientation {
                    for (i, (n, r)) in names.into_iter().enumerate() {
                        ch |= axis_slider(ui, &n[..1], MUTED, &mut self.target_rpy_deg[i], r, "°");
                    }
                    ui.horizontal(|ui| {
                        if ui.button("Senkrecht").clicked() {
                            self.target_rpy_deg = [180.0, 0.0, self.target_rpy_deg[2]];
                            ch = true;
                        }
                        if ui.button("Waagerecht").clicked() {
                            self.target_rpy_deg = [0.0, 90.0, 0.0];
                            ch = true;
                        }
                    });
                } else {
                    ui.label(RichText::new("Wird automatisch berechnet:").size(11.0).color(MUTED));
                    for (i, (n, _)) in names.into_iter().enumerate() {
                        ui.label(RichText::new(format!("{n:<6}{:+8.1}°", self.target_rpy_deg[i])).monospace());
                    }
                }
            });
            if ch {
                self.play = Play::Idle;
                self.solve(40);
            }
        } else {
            card(ui, Some("Gelenkwinkel"), |ui| {
                let mut ch = false;
                for i in 0..6 {
                    let j = self.cfg.arm[i];
                    let mut deg = self.q_cmd[i].to_degrees();
                    ch |= axis_slider(ui, &format!("J{}", i + 1), MUTED, &mut deg, j.min_deg..=j.max_deg, "°");
                    self.q_cmd[i] = deg.to_radians();
                }
                if ch {
                    self.play = Play::Idle;
                    self.sync_target_from_q();
                    self.ik_msg = None;
                }
            });
        }
        card(ui, Some("Gelenkgeschwindigkeit"), |ui| {
            ui.horizontal(|ui| {
                dv(ui, &mut self.cfg.speed_dps, " °/s", 5.0);
                ui.label(RichText::new("≥ 720 = sofort").size(11.0).color(MUTED));
            });
        });
    }

    // Reiter Playlist: Punkte tippen, duplizieren, verschieben, löschen. Alles editierbar
    fn playlist_tab(&mut self, ui: &mut egui::Ui) {
        card(ui, Some("Ablauf"), |ui| {
            ui.horizontal(|ui| {
                ui.label("TCP-Geschwindigkeit");
                dv(ui, &mut self.cfg.tcp_speed, " mm/s", 5.0);
                self.cfg.tcp_speed = self.cfg.tcp_speed.clamp(1.0, 2000.0);
            });
            ui.label(RichText::new("Gerade Linien zwischen den Punkten, Werkzeugwinkel automatisch. Start/Stopp in der Zeitleiste unten.").size(11.0).color(MUTED));
        });
        let (mut del, mut up, mut go, mut dup) = (None, None, None, None);
        card(ui, Some("Zielpunkte (mm)"), |ui| {
            ui.spacing_mut().item_spacing.x = 3.0;
            ui.horizontal(|ui| {
                ui.add_space(19.0);
                for (t, w) in [("X", 44.0), ("Y", 44.0), ("Z", 44.0), ("Pause", 44.0)] {
                    ui.add_sized([w, 12.0], egui::Label::new(RichText::new(t).size(10.0).color(MUTED)));
                }
            });
            for i in 0..self.cfg.waypoints.len() {
                let cur = self.pl_idx == i && self.playing();
                let w = &mut self.cfg.waypoints[i];
                egui::Frame::new().fill(if cur { BLUE.gamma_multiply(0.12) } else { Color32::TRANSPARENT }).show(ui, |ui| {
                    ui.horizontal(|ui| {
                        let skipped = self.skipped.contains(&i);
                        ui.add_sized([16.0, 20.0], egui::Label::new(RichText::new((i + 1).to_string()).strong().color(if skipped { Color32::from_rgb(200, 40, 40) } else if cur { BLUE } else { MUTED })))
                            .on_hover_text_at_pointer(if skipped { "Außer Reichweite, wird übersprungen" } else { "" });
                        for (k, v) in w.pos.iter_mut().enumerate() {
                            let r = if k == 2 { 0.0..=900.0 } else { -700.0..=700.0 };
                            ui.add_sized([44.0, 20.0], egui::DragValue::new(v).speed(1.0).range(r).max_decimals(0));
                        }
                        ui.add_sized([44.0, 20.0], egui::DragValue::new(&mut w.dwell).speed(0.05).range(0.0..=60.0).suffix(" s").max_decimals(1));
                        if ibtn(ui, Icon::Goto, "Hierhin fahren", false).clicked() {
                            go = Some(i);
                        }
                        if ibtn(ui, Icon::Copy, "Darunter duplizieren", false).clicked() {
                            dup = Some(i);
                        }
                        if ibtn(ui, Icon::Up, "Nach oben", false).clicked() && i > 0 {
                            up = Some(i);
                        }
                        if ibtn(ui, Icon::Close, "Löschen", false).clicked() {
                            del = Some(i);
                        }
                    });
                });
            }
            if self.cfg.waypoints.is_empty() {
                ui.label(RichText::new("Noch keine Zielpunkte").color(MUTED));
            }
        });
        if let Some(i) = dup {
            let w = self.cfg.waypoints[i].clone();
            self.cfg.waypoints.insert(i + 1, w);
        }
        if let Some(i) = up {
            self.cfg.waypoints.swap(i, i - 1);
        }
        if let Some(i) = del {
            self.cfg.waypoints.remove(i);
            self.play = Play::Idle;
            self.pl_idx = self.pl_idx.min(self.cfg.waypoints.len().saturating_sub(1));
        }
        if let Some(i) = go {
            self.goto_waypoint(i);
        }
        ui.horizontal_wrapped(|ui| {
            if tbtn(ui, Icon::Plus, "Punkt").clicked() {
                let pos = self.cfg.waypoints.last().map_or(self.target_pos, |w| [w.pos[0], w.pos[1], w.pos[2] + 50.0]);
                self.cfg.waypoints.push(Waypoint { pos, dwell: 0.3 });
            }
            if tbtn(ui, Icon::Goto, "Aktuelles Ziel").clicked() {
                self.cfg.waypoints.push(Waypoint { pos: self.target_pos, dwell: 0.3 });
            }
            if tbtn(ui, Icon::Reset, "Demo").clicked() {
                self.cfg.waypoints = demo_waypoints();
                self.play = Play::Idle;
            }
            if tbtn(ui, Icon::Close, "Leeren").clicked() {
                self.cfg.waypoints.clear();
                self.play = Play::Idle;
            }
        });
    }

    // Reiter Aufbau: DH-Parameter und Servo-Zuordnung
    fn build_tab(&mut self, ui: &mut egui::Ui) {
        ui.label(RichText::new("Standard-DH: Rz(θ+Offset)·Tz(d)·Tx(a)·Rx(α)").size(11.0).color(MUTED));
        ui.add_space(4.0);
        let mut ch = false;
        for i in 0..6 {
            card(ui, None, |ui| {
                egui::CollapsingHeader::new(RichText::new(format!("Gelenk {}", i + 1)).strong()).id_salt(("j", i)).show(ui, |ui| {
                    let j = &mut self.cfg.arm[i];
                    let s = &mut self.cfg.servos[i];
                    egui::Grid::new(("g", i)).num_columns(2).show(ui, |ui| {
                        for (l, v, u, sp) in [
                            ("a (Länge)", &mut j.a, " mm", 0.5),
                            ("α (Verdrehung)", &mut j.alpha_deg, "°", 0.5),
                            ("d (Versatz)", &mut j.d, " mm", 0.5),
                            ("θ-Offset", &mut j.offset_deg, "°", 0.5),
                            ("Limit min", &mut j.min_deg, "°", 0.5),
                            ("Limit max", &mut j.max_deg, "°", 0.5),
                        ] {
                            ui.label(l);
                            ch |= dv(ui, v, u, sp);
                            ui.end_row();
                        }
                        ui.label(RichText::new("Servo").color(MUTED));
                        ui.end_row();
                        ui.label("Richtung");
                        ui.horizontal(|ui| {
                            ch |= ui.selectable_value(&mut s.dir, 1.0, "+").changed();
                            ch |= ui.selectable_value(&mut s.dir, -1.0, "−").changed();
                        });
                        ui.end_row();
                        for (l, v, u, sp) in [
                            ("Mitte", &mut s.center_deg, "°", 0.5),
                            ("Servo-Bereich", &mut s.range_deg, "°", 1.0),
                            ("Puls min", &mut s.pulse_min, " µs", 5.0),
                            ("Puls max", &mut s.pulse_max, " µs", 5.0),
                        ] {
                            ui.label(l);
                            ch |= dv(ui, v, u, sp);
                            ui.end_row();
                        }
                    });
                    j.max_deg = j.max_deg.max(j.min_deg);
                    s.range_deg = s.range_deg.max(1.0);
                });
            });
        }
        if ch {
            self.play = Play::Idle;
            self.clamp_cmd();
            self.q_view = self.q_cmd;
            self.sync_target_from_q();
            self.ik_msg = None;
        }
        if tbtn(ui, Icon::Reset, "Auf Elfin-Standard zurücksetzen").clicked() {
            let (style, waypoints) = (self.cfg.style.clone(), std::mem::take(&mut self.cfg.waypoints));
            self.cfg = Config { style, waypoints, ..Config::default() };
            self.set_pose([0.0; 6]);
            self.q_view = self.q_cmd;
        }
    }

    // Reiter Ansicht: Farben und Anzeigeoptionen
    fn view_tab(&mut self, ui: &mut egui::Ui) {
        let st = &mut self.cfg.style;
        card(ui, Some("Darstellung"), |ui| {
            egui::Grid::new("view").num_columns(2).show(ui, |ui| {
                ui.label("Lackfarbe");
                ui.color_edit_button_srgb(&mut st.accent);
                ui.end_row();
                ui.label("Hintergrund");
                ui.color_edit_button_srgb(&mut st.background);
                ui.end_row();
                ui.label("Rasterweite");
                dv(ui, &mut st.grid_step, " mm", 1.0);
                ui.end_row();
                ui.label("Sichtfeld");
                ui.add(egui::Slider::new(&mut st.fov_deg, 20.0..=90.0).suffix("°"));
                ui.end_row();
            });
            st.grid_step = st.grid_step.max(5.0);
        });
        if tbtn(ui, Icon::Trail, "Spur löschen").clicked() {
            self.trail.clear();
        }
    }

    // Rechte Spalte: Winkel + Servo-Winkel + Puls pro Motor
    fn motors(&mut self, ui: &mut egui::Ui) {
        for i in 0..6 {
            let (a, p, clamped) = self.motor(i);
            let j = self.cfg.arm[i];
            let deg = self.q_cmd[i].to_degrees();
            ui.horizontal(|ui| {
                ui.label(RichText::new(format!("J{}", i + 1)).strong().color(BLUE));
                ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
                    ui.label(RichText::new(format!("{deg:+.1}°")).monospace().size(15.0));
                });
            });
            let (r, _) = ui.allocate_exact_size(vec2(ui.available_width(), 8.0), Sense::hover());
            let bar = r.shrink2(vec2(0.0, 2.0));
            ui.painter().rect_filled(bar, 2.0, Color32::from_rgb(222, 225, 229));
            let span = (j.max_deg - j.min_deg).max(1e-9);
            let f = ((deg - j.min_deg) / span).clamp(0.0, 1.0) as f32;
            let zero = ((0.0 - j.min_deg) / span).clamp(0.0, 1.0) as f32;
            let (x0, x1) = (egui::lerp(bar.x_range(), zero), egui::lerp(bar.x_range(), f));
            ui.painter().rect_filled(egui::Rect::from_x_y_ranges(x0.min(x1)..=x0.max(x1), bar.y_range()), 2.0, BLUE.gamma_multiply(0.8));
            ui.painter().circle(egui::pos2(x1, r.center().y), 4.5, Color32::WHITE, Stroke::new(1.5, BLUE));
            let c = if clamped { WARN } else { Color32::from_gray(70) };
            ui.horizontal(|ui| {
                ui.label(RichText::new(format!("Servo {a:5.1}°")).monospace().size(11.5).color(c));
                ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
                    ui.label(RichText::new(format!("{p:.0} µs")).monospace().size(11.5).color(c));
                });
            });
            ui.separator();
        }
        if (0..6).any(|i| self.motor(i).2) {
            ui.label(RichText::new("Orange: Servo-Bereich überschritten").size(11.0).color(WARN));
        }
        ui.horizontal(|ui| {
            if tbtn(ui, Icon::Copy, "Pulse").clicked() {
                let s: Vec<String> = (0..6).map(|i| format!("{:.0}", self.motor(i).1)).collect();
                ui.ctx().copy_text(s.join(","));
            }
            if tbtn(ui, Icon::Copy, "Winkel").clicked() {
                let s: Vec<String> = self.q_cmd.iter().map(|q| format!("{:.3}", q.to_degrees())).collect();
                ui.ctx().copy_text(s.join(","));
            }
        });
    }

    /// Würfel zur Ansichtswahl (wie in Fusion).
    fn view_cube(&mut self, ui: &mut egui::Ui, rect: egui::Rect) {
        let (r, u, f) = self.cam.basis();
        let c = rect.right_top() + vec2(-70.0, 72.0);
        let s = 26.0;
        let area = egui::Rect::from_center_size(c, vec2(100.0, 100.0));
        let resp = ui.interact(area, egui::Id::new("viewcube"), Sense::click());
        let proj = |p: V3| c + vec2((p[0] * r[0] + p[1] * r[1] + p[2] * r[2]) as f32, -(p[0] * u[0] + p[1] * u[1] + p[2] * u[2]) as f32) * s;
        let faces: [(V3, &str, f64, f64); 6] = [
            ([0.0, 0.0, 1.0], "OBEN", -90.0, 89.0),
            ([0.0, 0.0, -1.0], "UNTEN", -90.0, -89.0),
            ([0.0, -1.0, 0.0], "VORNE", -90.0, 0.0),
            ([0.0, 1.0, 0.0], "HINTEN", 90.0, 0.0),
            ([1.0, 0.0, 0.0], "RECHTS", 0.0, 0.0),
            ([-1.0, 0.0, 0.0], "LINKS", 180.0, 0.0),
        ];
        let hover = resp.hover_pos();
        let inside = |p: egui::Pos2, poly: &[egui::Pos2]| {
            let sg: Vec<f32> = (0..4)
                .map(|i| {
                    let (a, b) = (poly[i], poly[(i + 1) % 4]);
                    (b.x - a.x) * (p.y - a.y) - (b.y - a.y) * (p.x - a.x)
                })
                .collect();
            sg.iter().all(|v| *v >= 0.0) || sg.iter().all(|v| *v <= 0.0)
        };
        for (n, name, yaw, pitch) in faces {
            if n[0] * f[0] + n[1] * f[1] + n[2] * f[2] >= 0.0 {
                continue;
            }
            let t1 = if n[2].abs() > 0.5 { [1.0, 0.0, 0.0] } else { [0.0, 0.0, 1.0] };
            let t2 = [n[1] * t1[2] - n[2] * t1[1], n[2] * t1[0] - n[0] * t1[2], n[0] * t1[1] - n[1] * t1[0]];
            let pts: Vec<egui::Pos2> = [(-1.0, -1.0), (1.0, -1.0), (1.0, 1.0), (-1.0, 1.0)]
                .iter()
                .map(|(a, b)| proj([n[0] + a * t1[0] + b * t2[0], n[1] + a * t1[1] + b * t2[1], n[2] + a * t1[2] + b * t2[2]]))
                .collect();
            let hot = hover.is_some_and(|h| inside(h, &pts));
            let fill = if hot { Color32::from_rgb(190, 220, 245) } else { Color32::from_rgb(244, 245, 247) };
            let painter = ui.painter();
            painter.add(egui::Shape::convex_polygon(pts.clone(), fill, Stroke::new(1.0, Color32::from_gray(150))));
            let ctr = pts.iter().fold(egui::Vec2::ZERO, |a, p| a + p.to_vec2()) / 4.0;
            painter.text(ctr.to_pos2(), egui::Align2::CENTER_CENTER, name, egui::FontId::proportional(8.5), Color32::from_gray(70));
            if hot && resp.clicked() {
                self.cam.set_view(yaw, pitch);
            }
        }
        let painter = ui.painter();
        for (d, col, l) in [([1.0, 0.0, 0.0], Color32::from_rgb(220, 50, 50), "X"), ([0.0, 1.0, 0.0], Color32::from_rgb(40, 160, 70), "Y"), ([0.0, 0.0, 1.0], Color32::from_rgb(40, 90, 220), "Z")] {
            let p = proj([-1.0 + 2.9 * d[0], -1.0 + 2.9 * d[1], -1.0 + 2.9 * d[2]]);
            painter.line_segment([proj([-1.0, -1.0, -1.0]), p], Stroke::new(1.5, col));
            painter.text(p, egui::Align2::CENTER_CENTER, l, egui::FontId::proportional(10.0), col);
        }
    }
}
