//! Mathe-Fenster: zeigt live, was unter der Haube passiert (DH-Tabelle, Matrizen, Jacobi, IK, Servos, Planer).

use crate::{kin, App, BLUE, MUTED, WARN};
use eframe::egui::{self, Color32, RichText};

fn f(v: f64) -> String {
    // -0.000 vermeiden, sieht in Matrizen unruhig aus
    let v = if v.abs() < 5e-4 { 0.0 } else { v };
    format!("{v:+9.3}")
}

fn mono(s: impl Into<String>) -> RichText {
    RichText::new(s).monospace().size(12.5)
}

/// Matrix mit Klammerbalken links/rechts.
fn matrix(ui: &mut egui::Ui, id: &str, rows: &[Vec<String>]) {
    let r = egui::Frame::new().inner_margin(egui::Margin::symmetric(8, 3)).show(ui, |ui| {
        egui::Grid::new(id).spacing([10.0, 1.0]).show(ui, |ui| {
            for row in rows {
                for c in row {
                    ui.label(mono(c.clone()));
                }
                ui.end_row();
            }
        });
    });
    let (rect, st) = (r.response.rect, egui::Stroke::new(1.5, Color32::from_gray(90)));
    let p = ui.painter();
    for (x, dx) in [(rect.left(), 5.0), (rect.right(), -5.0)] {
        p.line_segment([egui::pos2(x, rect.top()), egui::pos2(x, rect.bottom())], st);
        p.line_segment([egui::pos2(x, rect.top()), egui::pos2(x + dx, rect.top())], st);
        p.line_segment([egui::pos2(x, rect.bottom()), egui::pos2(x + dx, rect.bottom())], st);
    }
}

fn m4_rows(m: &kin::M4) -> Vec<Vec<String>> {
    m.iter().map(|r| r.iter().map(|v| f(*v)).collect()).collect()
}

/// Hintergrund + Scrollen: ohne Panel bleibt ein eigenes Fenster transparent (schwarz).
fn panel(ui: &mut egui::Ui, body: impl FnOnce(&mut egui::Ui)) {
    egui::CentralPanel::default().frame(egui::Frame::new().fill(Color32::from_rgb(246, 247, 249)).inner_margin(egui::Margin::same(16))).show_inside(ui, |ui| {
        egui::ScrollArea::vertical().auto_shrink([false, false]).show(ui, body);
    });
}

/// LaTeX-Formel als SVG (erzeugt mit tools/gen_formulas.mjs).
macro_rules! tex {
    ($ui:expr, $k:literal) => {
        tex($ui, egui::Image::from_bytes(concat!("bytes://tex/", $k, ".svg"), include_bytes!(concat!("../assets/tex/", $k, ".svg"))))
    };
}

fn tex(ui: &mut egui::Ui, img: egui::Image<'_>) {
    egui::Frame::new().fill(Color32::WHITE).stroke(egui::Stroke::new(1.0, Color32::from_gray(226))).corner_radius(6.0).inner_margin(egui::Margin::symmetric(14, 6)).show(ui, |ui| {
        ui.add(img.fit_to_original_size(1.0).max_width(ui.available_width()));
    });
}

#[allow(dead_code)]
fn formula(ui: &mut egui::Ui, s: &str) {
    ui.label(RichText::new(s).monospace().size(13.0).color(BLUE));
}

fn note(ui: &mut egui::Ui, s: &str) {
    ui.label(RichText::new(s).size(11.5).color(MUTED));
}

fn det3(m: [[f64; 3]; 3]) -> f64 {
    m[0][0] * (m[1][1] * m[2][2] - m[1][2] * m[2][1]) - m[0][1] * (m[1][0] * m[2][2] - m[1][2] * m[2][0]) + m[0][2] * (m[1][0] * m[2][1] - m[1][1] * m[2][0])
}

impl App {
    /// Beide Mathe-Fenster sind echte, frei verschiebbare Betriebssystem-Fenster (auch auf zweitem Monitor).
    pub fn math_window(&mut self, ctx: &egui::Context) {
        if self.show_math {
            let b = egui::ViewportBuilder::default().with_title("Mathematik · live").with_inner_size([620.0, 720.0]);
            self.show_math = !ctx.show_viewport_immediate(egui::ViewportId::from_hash_of("math-live"), b, |ui, _| {
                panel(ui, |ui| self.math_body(ui));
                ui.ctx().input(|i| i.viewport().close_requested())
            });
        }
        if self.show_formulas {
            let b = egui::ViewportBuilder::default().with_title("Mathematik · Formeln").with_inner_size([700.0, 780.0]);
            self.show_formulas = !ctx.show_viewport_immediate(egui::ViewportId::from_hash_of("math-static"), b, |ui, _| {
                panel(ui, formulas_body);
                ui.ctx().input(|i| i.viewport().close_requested())
            });
        }
    }

    fn math_body(&mut self, ui: &mut egui::Ui) {
        let arm = self.cfg.arm;
        let q = self.q_cmd;
        let frames = kin::fk_frames(&arm, &q);

        egui::CollapsingHeader::new(RichText::new("1 · Vorwärtskinematik (DH)").strong()).default_open(true).show(ui, |ui| {
            tex!(ui, "dh_a");
            tex!(ui, "t06");
            note(ui, "θ_i = q_i + Offset_i. Längen in mm, Winkel in Grad.");
            egui::Grid::new("dh").striped(true).spacing([14.0, 2.0]).show(ui, |ui| {
                for h in ["i", "q", "Offset", "θ = q+Off", "d", "a", "α"] {
                    ui.label(RichText::new(h).strong().size(11.5));
                }
                ui.end_row();
                for i in 0..6 {
                    let j = &arm[i];
                    ui.label(mono(format!("{}", i + 1)));
                    ui.label(mono(format!("{:+8.2}°", q[i].to_degrees())));
                    ui.label(mono(format!("{:+5.0}°", j.offset_deg)));
                    ui.label(mono(format!("{:+8.2}°", q[i].to_degrees() + j.offset_deg)));
                    ui.label(mono(format!("{:8.3}", j.d)));
                    ui.label(mono(format!("{:8.3}", j.a)));
                    ui.label(mono(format!("{:+7.2}°", j.alpha_deg)));
                    ui.end_row();
                }
            });
            ui.add_space(4.0);
            tex!(ui, "dh_m");
            for i in 0..6 {
                egui::CollapsingHeader::new(mono(format!("T_0→{}  (Frame nach Gelenk {})", i + 1, i + 1))).id_salt(("t", i)).show(ui, |ui| {
                    matrix(ui, &format!("tm{i}"), &m4_rows(&frames[i + 1]));
                });
            }
            ui.label(RichText::new("Werkzeug-Pose T_0→6").strong());
            matrix(ui, "t06", &m4_rows(&frames[6]));
            let (p, r) = (kin::pos(&frames[6]), kin::to_rpy(&frames[6]).map(f64::to_degrees));
            ui.label(mono(format!("p = ({:+.2}, {:+.2}, {:+.2}) mm", p[0], p[1], p[2])));
            ui.label(mono(format!("RPY = ({:+.1}°, {:+.1}°, {:+.1}°)   R = Rz(Y)·Ry(P)·Rx(R)", r[0], r[1], r[2])));
        });

        egui::CollapsingHeader::new(RichText::new("2 · Jacobi-Matrix & Singularität").strong()).default_open(true).show(ui, |ui| {
            let j = kin::jacobian(&arm, &q);
            tex!(ui, "jac1");
            note(ui, "Analytisch berechnet (exakt): oben 3 Zeilen Spitzengeschwindigkeit in mm/rad, unten 3 Zeilen Winkelgeschwindigkeit (Drehachsen z_(i−1)).");
            matrix(ui, "jac", &j.iter().map(|r| r.iter().map(|v| format!("{v:+8.1}")).collect()).collect::<Vec<_>>());
            let jjt: [[f64; 3]; 3] = std::array::from_fn(|a| std::array::from_fn(|b| (0..6).map(|k| j[a][k] * j[b][k]).sum()));
            let w = det3(jjt).max(0.0).sqrt();
            tex!(ui, "manip");
            let (txt, col) = if w < 2.0e4 { ("nahe Singularität: IK instabil, Gelenke können stark schwingen", WARN) } else { ("gut konditioniert", Color32::from_rgb(40, 160, 90)) };
            ui.horizontal(|ui| {
                ui.label(mono(format!("w = {w:10.0}")).color(col));
                ui.label(RichText::new(txt).color(col).size(12.0));
            });
        });

        egui::CollapsingHeader::new(RichText::new("3 · Inverse Kinematik").strong()).default_open(true).show(ui, |ui| {
            tex!(ui, "poe");
            tex!(ui, "ikchain");
            note(ui, "Geschlossene Lösung (Paden-Kahan): Schulter J1∩J2 und Handgelenk J4∩J5∩J6 schneiden sich in Punkten, daher zerfällt die IK in Teilprobleme mit je 1–2 exakten Lösungen. Ergebnis: alle Haltungen (bis 8), auf Rechengenauigkeit, ohne Iteration und ohne Abhängigkeit von der aktuellen Stellung. Gewählt wird die zur Ist-Stellung nächstgelegene.");
            note(ui, "Ohne Winkelvorgabe legt eine feste Regel die Werkzeugrichtung fest (gestreckt von der Schulter zum Ziel, nah an der Schulter weich abgeknickt), J4 ≈ 0, J6 = 0. Die Haltung hängt damit NUR von der Position ab: A→B und B→A fahren dieselben Stellungen.");
            tex!(ui, "auto");
            note(ui, "Mit Winkelvorgabe ist die Pose vollständig festgelegt (6 Gleichungen, 6 Gelenke). Orientierungsfehler als Rotationsvektor (nahe 180° stabil aus der Diagonale):");
            tex!(ui, "roterr");
            note(ui, "Nur bei selbst geänderter Geometrie (Achsen schneiden sich nicht mehr) dient die geschlossene Form als Startwert für gedämpftes Newton (Levenberg-Marquardt, Cholesky):");
            tex!(ui, "ik_dq");
            tex!(ui, "ik6");
            note(ui, "Anschläge: J1, J4 und J6 haben intern ±720° Spielraum, damit der kürzeste Weg nie an einer Grenze scheitert; die Anzeige springt bei ±360° auf 0 zurück.");
            let exact = kin::ik_all(&arm, &kin::from_xyz_rpy(self.target_pos, [0.0; 3]), false).len();
            ui.label(mono(format!("Haltungen am Ziel (ohne Winkel): {exact}")));
            let t = self.target_pos;
            let p = kin::pos(&frames[6]);
            let e: kin::V3 = std::array::from_fn(|k| t[k] - p[k]);
            ui.label(mono(format!("p* = ({:+.2}, {:+.2}, {:+.2})", t[0], t[1], t[2])));
            ui.label(mono(format!("e  = ({:+.3}, {:+.3}, {:+.3})  |e| = {:.4} mm", e[0], e[1], e[2], e.iter().map(|v| v * v).sum::<f64>().sqrt())));
            if let Some((ok, m)) = &self.ik_msg {
                ui.label(RichText::new(m).size(12.0).color(if *ok { Color32::from_rgb(40, 160, 90) } else { WARN }));
            }
        });

        egui::CollapsingHeader::new(RichText::new("4 · Servo-Abbildung").strong()).show(ui, |ui| {
            tex!(ui, "servo");
            egui::Grid::new("servo").striped(true).spacing([14.0, 2.0]).show(ui, |ui| {
                for h in ["Gelenk", "q", "a (Servo)", "µs"] {
                    ui.label(RichText::new(h).strong().size(11.5));
                }
                ui.end_row();
                for i in 0..6 {
                    let (a, p, c) = self.motor(i);
                    ui.label(mono(format!("J{}", i + 1)));
                    ui.label(mono(format!("{:+8.2}°", q[i].to_degrees())));
                    ui.label(mono(format!("{a:7.2}°")).color(if c { WARN } else { Color32::from_gray(40) }));
                    ui.label(mono(format!("{p:6.0}")));
                    ui.end_row();
                }
            });
        });

        egui::CollapsingHeader::new(RichText::new("5 · Playlist-Planer").strong()).show(ui, |ui| {
            note(ui, "Die gesamte Bahn wird VOR dem Start berechnet und beim Abspielen nur noch abgetastet. Dadurch ist jede Runde exakt gleich, unabhängig von Framerate und Geschwindigkeit.");
            tex!(ui, "line");
            tex!(ui, "step");
            tex!(ui, "time");
            tex!(ui, "ramp");
            note(ui, "Anfahren/Abbremsen mit konstanter Beschleunigung (Rampe 0,35 s), kurze Fahrten erreichen die Nenngeschwindigkeit nicht.");
            tex!(ui, "dp");
            note(ui, "Graph über alle Punkte × Haltungen (exakte IK-Lösungen): jede Gerade wird aus jeder Haltung verfolgt; die dynamische Programmierung wählt exakt die Route mit den wenigsten Gelenkfahrten, dann dem kürzesten Gelenkweg. Kein Umorientieren am Punkt nötig: ohne Winkel hängt die Haltung nur von der Position ab.");
            tex!(ui, "wrap");
            note(ui, "Kürzester Weg: J1, J4 und J6 werden um ganze Umdrehungen verschoben, bis das Ziel der aktuellen Stellung am nächsten liegt (359° → 0° ist +1°, 450° wird als 90° angefahren). Gilt für Gelenkfahrten, Anfahrt und Zielbewegung; Geraden bleiben kontinuierlich. Die Planung läuft im Hintergrund-Thread.");
            tex!(ui, "rst");
            note(ui, "Anzeige: Erreicht ein Wert ±360°, springt er auf 0 zurück (gleiche Pose). Intern haben die drei Gelenke ±720° Spielraum, damit eine Gerade oder der kürzeste Weg nie an einer Grenze scheitert.");
            tex!(ui, "slerp");
            note(ui, "Mit Werkzeugwinkel (global oder pro Punkt): Position linear, Orientierung per Slerp überblendet; jeder Schritt wird mit vollständiger 6D-IK gelöst. Schrittzahl richtet sich nach Weg (2 mm) UND Drehwinkel (1°), auch reine Drehungen werden fein abgefahren.");
            note(ui, "Springt ein Gelenk um mehr als 14° pro Schritt, wird der Schritt rekursiv halbiert (bis ≈ 0,1 µm): nahe einer Singularität drehen J4/J6 schnell, aber stetig. Bleibt ein Sprung, liegt die Stelle genau in der Singularität (J5 = 0) – zulässig, wenn die Spitze dabei stehen bleibt (Handgelenk dreht durch, < 0,01 mm). Sonst (Gelenkanschlag, keine Lösung) ist die Gerade aus dieser Haltung nicht fahrbar: dann Gelenkfahrt (PTP) mit linearer Interpolation im Gelenkraum. Der Schleifenschluss endet exakt in der Startstellung. Kreuzt die Bahn die Handgelenk-Singularität ungerade oft, endet sie umgeklappt (J5 mit anderem Vorzeichen): Dieser Haltungswechsel kommt auf die Schließstrecke (nur bei Endlosschleife gefahren, halbe Kosten), nicht mitten in die Bahn. Bleibt trotz optimaler Haltungswahl eine Gelenkfahrt, ist sie unvermeidbar: Jede Haltung stößt irgendwo an eine Grenze (Elfin: J2 ±134°, J3 ±149°, J5 ±146°). Beispiel: Werkzeug fest waagrecht nach +x – das Raster bei z = 180 geht nur mit Schulter vorn/Ellbogen oben (oder hinten/unten), Punkte hinter dem Sockel nur mit den anderen beiden (sonst J5 ≈ 166°). Mit „Gier mitdrehen“ (Yaw relativ zur Richtung Sockel → Punkt) zeigt das Werkzeug radial nach außen und die Demo läuft ohne Gelenkfahrt.");
            if self.plan.segs.is_empty() {
                note(ui, "Noch kein Plan: Playlist starten.");
            } else {
                egui::Grid::new("segs").striped(true).spacing([14.0, 2.0]).show(ui, |ui| {
                    for h in ["Strecke", "Typ", "Samples", "max Δq"] {
                        ui.label(RichText::new(h).strong().size(11.5));
                    }
                    ui.end_row();
                    for s in &self.plan.segs {
                        let dq = s.q.windows(2).map(|w| (0..6).map(|i| (w[1][i] - w[0][i]).abs()).fold(0.0, f64::max)).fold(0.0, f64::max);
                        ui.label(mono(format!("{} → {}{}", s.from + 1, s.to + 1, if s.closing { "  (Schluss)" } else { "" })));
                        ui.label(mono(if s.ptp { "Gelenkfahrt" } else { "Gerade" }).color(if s.ptp { WARN } else { Color32::from_gray(40) }));
                        ui.label(mono(format!("{}", s.q.len())));
                        ui.label(mono(format!("{:.2}°", dq.to_degrees())));
                        ui.end_row();
                    }
                });
                if !self.plan.unreachable.is_empty() {
                    let l: Vec<String> = self.plan.unreachable.iter().map(|i| (i + 1).to_string()).collect();
                    ui.label(RichText::new(format!("Außer Reichweite: {}", l.join(", "))).color(WARN));
                }
            }
        });
    }
}

/// Reine Theorie, ohne Live-Werte: so funktioniert alles in diesem Programm.
fn formulas_body(ui: &mut egui::Ui) {
    let h = |ui: &mut egui::Ui, n: &str, s: &str| {
        ui.add_space(14.0);
        ui.horizontal(|ui| {
            ui.label(RichText::new(n).size(20.0).color(BLUE).strong());
            ui.label(RichText::new(s).size(17.0).strong().color(Color32::from_gray(30)));
        });
        ui.add_space(2.0);
        ui.separator();
        ui.add_space(4.0);
    };
    let p = |ui: &mut egui::Ui, s: &str| {
        ui.add_space(3.0);
        ui.label(RichText::new(s).size(13.0).color(Color32::from_gray(70)));
        ui.add_space(3.0);
    };
    ui.label(RichText::new("Mathematik hinter dem Arm").size(24.0).strong().color(Color32::from_gray(25)));
    ui.label(RichText::new("Reine Formeln, unabhängig von der aktuellen Stellung. Live-Werte stehen im Σ-Fenster.").size(13.0).color(MUTED));

    h(ui, "1", "Vorwärtskinematik (Denavit-Hartenberg)");
    p(ui, "Jedes Gelenk i hat vier Parameter: Drehung θ (Variable), Versatz d entlang z, Länge a entlang x und Verdrehung α um x.");
    tex!(ui, "dh_a");
    p(ui, "Daraus die homogene 4×4-Matrix eines Gelenks:");
    tex!(ui, "dh_m");
    p(ui, "Alle sechs Gelenke hintereinander ergeben die Pose der Werkzeugspitze. R ist die 3×3-Drehmatrix, p = (x, y, z) die Position (TCP).");
    tex!(ui, "t06");

    h(ui, "2", "Orientierung (Roll · Pitch · Yaw)");
    tex!(ui, "rpy");
    p(ui, "Bei P = ±90° fallen Roll und Yaw zusammen (Gimbal Lock): dort ist nur ihre Summe bestimmt.");

    h(ui, "3", "Jacobi-Matrix und Singularitäten");
    p(ui, "Verknüpft Gelenkgeschwindigkeiten mit der Spitzengeschwindigkeit. z_(i−1) ist die Drehachse von Gelenk i, p_(i−1) dessen Ursprung.");
    tex!(ui, "jac1");
    p(ui, "Vollständig (analytisch, exakt – keine Differenzenquotienten) mit den Drehachsen für die Orientierung; die IK nutzt genau diese Matrix:");
    tex!(ui, "jac6");
    p(ui, "Die Manipulierbarkeit w misst, wie gut sich die Spitze in alle Richtungen bewegen lässt:");
    tex!(ui, "manip");
    p(ui, "w → 0 ist eine Singularität: Der Arm verliert eine Bewegungsrichtung, kleine Spitzenbewegungen brauchen riesige Gelenkwinkel (typisch: Arm gestreckt oder Handgelenk über der Schulterachse).");

    h(ui, "4", "Inverse Kinematik (geschlossen, Paden-Kahan)");
    p(ui, "Die Kinematik wird als Produkt von Exponentialen geschrieben: ω_i ist die Drehachse von Gelenk i, r_i ein Punkt darauf (beide bei q = 0), M die Werkzeugpose bei q = 0. Das ist dieselbe Abbildung wie die DH-Kette, nur in Achsen statt Frames:");
    tex!(ui, "poe");
    p(ui, "Beim Elfin schneiden sich J1 und J2 im Schulterpunkt s, J4, J5 und J6 im Handgelenkpunkt w. Dadurch zerfällt die IK in drei geometrische Teilprobleme mit geschlossener Lösung:");
    tex!(ui, "sp1");
    tex!(ui, "sp2");
    tex!(ui, "sp3");
    p(ui, "Kette: Der Handgelenkpunkt W hängt nur von J1–J3 ab. Sein Abstand zur Schulter legt J3 fest (zwei Lösungen: Ellbogen oben/unten), dann J1/J2 (vorn/hinten), zuletzt die Handgelenksdrehung (geklappt/nicht geklappt):");
    tex!(ui, "ikchain");
    p(ui, "Ergebnis: bis zu 8 exakte Haltungen, unabhängig vom Startwert. Die IK wählt die zur Ist-Stellung nächstgelegene (Vollkreis-Gelenke über den kürzeren Weg). Lösungen außerhalb der Anschläge entfallen.");
    p(ui, "Ohne Winkelvorgabe fehlen 3 Gleichungen. Statt einer Nullraum-Suche (deren Ergebnis vom Startwert abhängt) bestimmt eine feste Regel die Werkzeugrichtung u: gestreckt von der Schulter zum Ziel; nah an der Schulter, wo der Ellbogen nicht weiter einklappen kann, knickt das Werkzeug in der Armebene um ψ ab, sodass der Handgelenkabstand d erreichbar bleibt. J4 so nah an 0 wie möglich, J6 = 0:");
    tex!(ui, "auto");
    p(ui, "σ ist ein weiches max(0, y): Wert und Steigung stetig, der Knick setzt mit endlicher Geschwindigkeit ein (eine harte Grenze hätte wegen arccos eine unendliche Steigung). Gedreht wird um die J2-Achse der jeweiligen Schulterseite, damit bleibt die Regel auch beim Überqueren der J1-Achse stetig. Grenze: direkt über der Schulter (Abstand < ~150 mm) erzwingt die Regel teils J2 am Anschlag; dort wählt der Planer eine Gelenkfahrt.");
    tex!(ui, "soft");
    p(ui, "Folge: Die Haltung ohne Winkel hängt NUR von der Position ab (je Schulterseite × Ellbogen eine, bis zu 4). Hin- und Rückweg zwischen zwei Punkten fahren exakt dieselben Stellungen, es gibt keine pfadabhängigen Umwege.");
    p(ui, "Orientierungsfehler (für Prüfung und Rückfall) als Rotationsvektor der Restdrehung R* Rᵀ; nahe 180° wird die Achse aus der Diagonale (R = 2 n nᵀ − I) bestimmt:");
    tex!(ui, "roterr");
    p(ui, "Rückfall nur für selbst geänderte Geometrie (Achsen schneiden sich nicht mehr): die geschlossene Form liefert Startwerte, gedämpftes Newton (Levenberg-Marquardt, Cholesky-Zerlegung) macht sie exakt; außer Reichweite streckt es den Arm so weit wie möglich zum Ziel:");
    tex!(ui, "ik_e");
    tex!(ui, "ik_dq");
    tex!(ui, "ik6");
    p(ui, "Geometrie: d₁ = 130, a₂ = 240, d₄ = 260, d₆ = 84,09 mm (Flanschfläche) – ein Elfin E10 im Maßstab 1:2. Die Gelenkachsen wurden aus dem 3D-Netz gemessen: Ausgleich über die Normalengeraden der Flächen an jeder Teilefuge (Linienkomplex-Fit). Ergebnis: J1⊥J2, J3⊥J4, J4⊥J5, J5⊥J6 schneiden sich auf < 0,2 mm und < 0,2° – die Achsen sind also ideal. Die Modellteile sitzen exakt darauf (Fugenspalt ≈ 2 mm = Netzauflösung; vorher bis 12 mm durch 1,5° Fehler an J4 und 2,6° an J5). Ältere Geometrie und Teile-Versätze werden beim Laden ersetzt.");
    p(ui, "Anschläge: J1, J4 und J6 haben intern ±720° Spielraum (damit der kürzeste Weg nie an einer Grenze scheitert, z. B. 340° → 10° = +30°); die Anzeige springt bei ±360° auf 0 zurück.");

    h(ui, "5", "Servo-Abbildung");
    p(ui, "c = Servo-Winkel bei q = 0, d = ±1 (Drehrichtung), range = Stellweg, p_min/p_max = Pulsbreiten der Endlagen.");
    tex!(ui, "servo");

    h(ui, "6", "Bahnplanung (Playlist)");
    p(ui, "Eine Gerade wird in Schritten von höchstens 2 mm bzw. 1° Drehung abgefahren; an jedem Schritt wird unter den exakten IK-Lösungen die der vorigen Stellung nächste genommen:");
    tex!(ui, "line");
    tex!(ui, "step");
    p(ui, "Fahrbar ist sie, wenn es an jedem Schritt eine Lösung gibt und kein Gelenk mehr als 0,25 rad pro Schritt springt (ein großer Sprung wird rekursiv halbiert, bis ≈ 0,1 µm: nahe einer Singularität drehen Gelenke schnell, aber stetig; genau in der Singularität J5 = 0 darf das Handgelenk durchdrehen, solange die Spitze stehen bleibt). Sonst Gelenkfahrt (PTP):");
    tex!(ui, "ptp");
    p(ui, "Die Zeit je Schritt begrenzt entweder die Spitzengeschwindigkeit oder die schnellste Gelenkbewegung:");
    tex!(ui, "time");
    p(ui, "Anfahren und Abbremsen laufen mit konstanter Beschleunigung (v² ∝ Weg) statt als Ruck; kurze Fahrten erreichen die Nenngeschwindigkeit gar nicht erst:");
    tex!(ui, "ramp");
    p(ui, "Welche Haltung an jedem Punkt gilt, entscheidet ein kürzester Weg im Graphen Punkte × Haltungen. Jede Gerade wird aus JEDER Haltung am Startpunkt verfolgt und endet in einer bestimmten Haltung am Ziel; zusätzlich ist jede Gelenkfahrt zwischen zwei Haltungen eine Kante. Die dynamische Programmierung ist exakt (keine Zustände gekürzt) und wird für jede Starthaltung gelöst; die Schleife muss in der Starthaltung schließen:");
    tex!(ui, "dp");
    p(ui, "Damit gilt: zuerst so wenige Gelenkfahrten wie überhaupt möglich, bei Gleichstand der kürzeste Gelenkweg, bei weiterem Gleichstand die Starthaltung nahe der Bereit-Stellung. Ein Umorientieren am Punkt ist nicht mehr nötig: ohne Winkel ist die Haltung durch die Position festgelegt, mit Winkel durch die Pose.");
    h(ui, "6b", "Kürzester Weg bei Vollkreis-Gelenken (±360°)");
    p(ui, "J1, J4 und J6 drehen mehr als eine Umdrehung: Eine Stellung q und q ± 2π sind dieselbe Pose. Jedes Ziel wird deshalb vor der Bewegung um ganze Umdrehungen verschoben, bis es der Ist-Stellung am nächsten liegt (dabei bleiben die internen Grenzen ±720° gewahrt, sodass nie ein langer Umweg nötig wird):");
    tex!(ui, "wrap");
    p(ui, "Beispiele: 359° → 0° dreht +1° (nicht −359°); 450° wird als 90° angefahren. Das gilt für Gelenkfahrten des Planers, die Anfahrt zur ersten Pose und die normale Bewegung zum Zielwert. Gerade Strecken bleiben dagegen stetig – dort würde ein Zurückdrehen die Gerade verlassen. Ein Schleifenschluss kann dadurch um ganze Umdrehungen versetzt enden; die Pose ist identisch.");
    p(ui, "Rückstellung: Die angezeigte und gesteuerte Stellung wird bei ±360° auf 0 zurückgesetzt (gleiche Pose, nur ein anderer Zahlenwert). Weil die Planung intern bis ±720° rechnet, ist der kürzeste Weg nie durch eine Grenze blockiert – ein hartes Limit bei ±360° würde z. B. 340° → 10° als −330° statt +30° erzwingen:");
    tex!(ui, "rst");
    h(ui, "6c", "Werkzeugwinkel im Playbook");
    p(ui, "Jeder Punkt kann einen eigenen Winkel (Roll, Pitch, Yaw) tragen, sonst gilt der globale Winkel (\"Winkel vorgeben\"); sobald irgendein Punkt einen Winkel hat, erben Punkte ohne Winkel den zuletzt gültigen (zyklisch), der Winkel bleibt also im ganzen Ablauf vorgegeben. Zwischen zwei Punkten wird die Position linear und die Orientierung per Slerp (kürzeste Drehung) überblendet:");
    tex!(ui, "slerp");
    p(ui, "Mit festem Winkel hat die Pose nur diskrete IK-Lösungen. Eine Gerade ist fahrbar, wenn dieselbe Konfiguration stetig mitläuft, andernfalls folgt eine Gelenkfahrt. J1/J4/J6 werden über den kürzeren Weg (modulo 360°) verglichen. Der Schleifenschluss endet direkt in der Startstellung.");
    p(ui, "Bewegungsprofil: Anfahren und Abbremsen mit konstanter Beschleunigung (siehe oben). Die Planung läuft in einem Hintergrund-Thread, die Oberfläche bleibt bedienbar.");
    p(ui, "Die ganze Bahn wird vor dem Start berechnet: Sie hängt nur von Arm und Punkten ab, nicht von Framerate, Geschwindigkeit oder Runde.");

    h(ui, "7", "Arbeitsraum (Hüllflächen)");
    p(ui, "Zufallsstichprobe im Gelenkraum (900 000 Stück, 20 % direkt an den Anschlägen). Jede Position wird relativ zur Schulterachse c nach Richtung (Azimut φ, Polarwinkel ϑ) einsortiert:");
    tex!(ui, "reach1");
    tex!(ui, "reach2");
    tex!(ui, "reach3");
    p(ui, "Zwischen innerer und äußerer Fläche erreicht die Spitze den Punkt (Position, Winkel frei). Selbstkollisionen und Boden sind nicht berücksichtigt.");
}
