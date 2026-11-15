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
            tex!(ui, "ik_e");
            tex!(ui, "ik_dq");
            tex!(ui, "null");
            note(ui, "Mit Winkelvorgabe kommt der Orientierungsfehler als Rotationsvektor dazu (nahe 180° wird die Achse aus der Diagonale bestimmt, damit sie stabil bleibt):");
            tex!(ui, "roterr");
            tex!(ui, "ik6");
            note(ui, "Anschläge: J1, J4 und J6 laufen von −360° bis +360° (360° = 0°, kürzester Weg), die übrigen Gelenke haben ihre Grenzen. Zufalls-Neustarts nutzen für diese Gelenke eine Umdrehung (±180°).");
            note(ui, "Ohne Winkelvorgabe hat der Arm 3 überzählige Freiheitsgrade (6 Gelenke, 3 Positionsgleichungen). Im Nullraum bewegt sich der Arm, ohne die Spitze zu verschieben: das nutzt der Zug der Handgelenke J4/J6 zur Komfortstellung. W = diag(0, 0.1, 0.1, 1, 0.3, 1).");
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
            note(ui, "Welche IK-Lösung (Ellbogen, Handgelenk, Schulter, J1/J4 ±360° versetzt) an jedem Punkt gilt, wählt eine dynamische Programmierung: möglichst wenige Gelenkfahrten, dann kürzester Gelenkweg.");
            tex!(ui, "reor");
            note(ui, "Umorientieren am Punkt: Gelenkinterpolation, jeder Schritt per IK auf den Punkt zurückprojiziert (Spitze bleibt stehen), danach die Gerade.");
            tex!(ui, "wrap");
            note(ui, "Kürzester Weg: Endlos drehende Gelenke (J1, J4) werden um ganze Umdrehungen verschoben, bis das Ziel der aktuellen Stellung am nächsten liegt (359° → 0° ist +1°, 450° wird als 90° angefahren). Gilt für Gelenkfahrten, Anfahrt und Zielbewegung; Geraden bleiben kontinuierlich. Die Planung läuft im Hintergrund-Thread.");
            note(ui, "Springt ein Gelenk (> 14° pro 2 mm) oder findet die IK keine Lösung, ist die Gerade nicht fahrbar: dann Gelenkfahrt (PTP) mit linearer Interpolation im Gelenkraum. Der Schleifenschluss endet exakt in der Startstellung.");
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

    h(ui, "4", "Inverse Kinematik");
    p(ui, "Gesucht ist q mit p(q) = p*. Für beliebige Gelenke gibt es keine geschlossene Lösung, deshalb iterativ per gedämpftem Newton-Verfahren (Levenberg-Marquardt) mit Anschlägen:");
    tex!(ui, "ik_e");
    tex!(ui, "ik_dq");
    p(ui, "λ dämpft die Schritte nahe Singularitäten. Mit Winkelvorgabe wird e um den Orientierungsfehler erweitert (6 Gleichungen, 6 Unbekannte). Ohne Vorgabe bleiben 3 überzählige Freiheitsgrade; der Nullraum verschiebt die Spitze nicht und zieht die Gelenke zur Komfortstellung:");
    tex!(ui, "null");
    p(ui, "Mit Winkelvorgabe kommt der Orientierungsfehler als Rotationsvektor (Achse · Winkel) der Restdrehung R* Rᵀ hinzu. Nahe 180° verschwindet der antisymmetrische Anteil, dort wird die Achse stattdessen aus der Diagonale (R = 2 n nᵀ − I) bestimmt, ihr Vorzeichen aus v. Beide Anteile werden mit L vergleichbar gemacht:");
    tex!(ui, "roterr");
    tex!(ui, "ik6");
    p(ui, "Anschläge: J1, J4 und J6 laufen von −360° bis +360° (360° entspricht 0°, es gilt der kürzeste Weg), die anderen Gelenke haben eigene Grenzen, auf die jeder Schritt geklemmt wird. Zufalls-Neustarts nutzen für diese Gelenke eine Umdrehung (±180°).");
    p(ui, "Mehrere Startwerte (Restarts) finden verschiedene Lösungen: Ellbogen oben/unten, Handgelenk geklappt.");

    h(ui, "5", "Servo-Abbildung");
    p(ui, "c = Servo-Winkel bei q = 0, d = ±1 (Drehrichtung), range = Stellweg, p_min/p_max = Pulsbreiten der Endlagen.");
    tex!(ui, "servo");

    h(ui, "6", "Bahnplanung (Playlist)");
    p(ui, "Eine Gerade wird in 2-mm-Schritten abgefahren, jeder Schritt per IK mit Warmstart gelöst:");
    tex!(ui, "line");
    tex!(ui, "step");
    p(ui, "Fahrbar ist sie, wenn die IK an jedem Schritt konvergiert und kein Gelenk mehr als 0,25 rad pro Schritt springt. Sonst Gelenkfahrt (PTP):");
    tex!(ui, "ptp");
    p(ui, "Die Zeit je Schritt begrenzt entweder die Spitzengeschwindigkeit oder die schnellste Gelenkbewegung:");
    tex!(ui, "time");
    p(ui, "Anfahren und Abbremsen laufen mit konstanter Beschleunigung (v² ∝ Weg) statt als Ruck; kurze Fahrten erreichen die Nenngeschwindigkeit gar nicht erst:");
    tex!(ui, "ramp");
    p(ui, "Welche Konfiguration (Ellbogen, Handgelenk, Schulter) an jedem Punkt gewählt wird, entscheidet eine dynamische Programmierung über alle IK-Lösungen: Gesamtroute mit möglichst wenigen Gelenkfahrten, bei gleich vielen mit kürzestem Gelenkweg. Geht eine Gerade aus der aktuellen Konfiguration nicht, orientiert sich der Arm zuerst AM PUNKT um (Nullraum-Bewegung: Gelenkinterpolation, jeder Schritt per IK zurück auf den Punkt projiziert, die Spitze bleibt stehen) und fährt dann die Gerade. Auch die Schleife schließt so in die Startstellung.");
    p(ui, "Kandidaten: pro Punkt bis zu ~30 verschiedene IK-Lösungen aus festen Startwerten (deterministisch), bei J1/J4 zusätzlich um ±360° versetzt. Die Route ist ein kürzester Pfad (dynamische Programmierung) über diese Zustände; Kosten = Gelenkweg + 1000 je Gelenkfahrt + 20 je Umorientieren:");
    tex!(ui, "dp");
    p(ui, "Umorientieren am Punkt (Nullraum-Bewegung): Gelenkinterpolation, jeder Schritt per IK zurück auf den Punkt projiziert, bei höchstens 0,25 rad Sprung:");
    tex!(ui, "reor");
    h(ui, "6b", "Kürzester Weg bei Vollkreis-Gelenken (±360°)");
    p(ui, "J1 und J4 haben keinen Anschlag: Eine Stellung q und q ± 2π sind dieselbe Pose. Jedes Ziel wird deshalb vor der Bewegung um ganze Umdrehungen verschoben, bis es der Ist-Stellung am nächsten liegt (dabei bleiben die Grenzen gewahrt):");
    tex!(ui, "wrap");
    p(ui, "Beispiele: 359° → 0° dreht +1° (nicht −359°); 450° wird als 90° angefahren. Das gilt für Gelenkfahrten des Planers, die Anfahrt zur ersten Pose und die normale Bewegung zum Zielwert. Gerade Strecken bleiben dagegen stetig – dort würde ein Zurückdrehen die Gerade verlassen. Ein Schleifenschluss kann dadurch um ganze Umdrehungen versetzt enden; die Pose ist identisch.");
    p(ui, "Bewegungsprofil: Anfahren und Abbremsen mit konstanter Beschleunigung (siehe oben). Die Planung läuft in einem Hintergrund-Thread, die Oberfläche bleibt bedienbar.");
    p(ui, "Die ganze Bahn wird vor dem Start berechnet: Sie hängt nur von Arm und Punkten ab, nicht von Framerate, Geschwindigkeit oder Runde.");

    h(ui, "7", "Arbeitsraum (Hüllflächen)");
    p(ui, "Zufallsstichprobe im Gelenkraum (900 000 Stück, 20 % direkt an den Anschlägen). Jede Position wird relativ zur Schulterachse c nach Richtung (Azimut φ, Polarwinkel ϑ) einsortiert:");
    tex!(ui, "reach1");
    tex!(ui, "reach2");
    tex!(ui, "reach3");
    p(ui, "Zwischen innerer und äußerer Fläche erreicht die Spitze den Punkt (Position, Winkel frei). Selbstkollisionen und Boden sind nicht berücksichtigt.");
}
