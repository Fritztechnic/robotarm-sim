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

fn formula(ui: &mut egui::Ui, s: &str) {
    ui.label(RichText::new(s).monospace().size(13.0).color(BLUE));
}

fn note(ui: &mut egui::Ui, s: &str) {
    ui.label(RichText::new(s).size(11.5).color(MUTED));
}

/// Positions-Jacobi 3×6 per zentraler Differenz (mm/rad).
fn jacobian(arm: &kin::Arm, q: &[f64; 6]) -> [[f64; 6]; 3] {
    let mut j = [[0.0; 6]; 3];
    for c in 0..6 {
        let (mut a, mut b) = (*q, *q);
        a[c] += 1e-6;
        b[c] -= 1e-6;
        let (pa, pb) = (kin::pos(&kin::fk(arm, &a)), kin::pos(&kin::fk(arm, &b)));
        for r in 0..3 {
            j[r][c] = (pa[r] - pb[r]) / 2e-6;
        }
    }
    j
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
                egui::ScrollArea::vertical().show(ui, |ui| self.math_body(ui));
                ui.ctx().input(|i| i.viewport().close_requested())
            });
        }
        if self.show_formulas {
            let b = egui::ViewportBuilder::default().with_title("Mathematik · Formeln").with_inner_size([700.0, 780.0]);
            self.show_formulas = !ctx.show_viewport_immediate(egui::ViewportId::from_hash_of("math-static"), b, |ui, _| {
                egui::ScrollArea::vertical().show(ui, |ui| formulas_body(ui));
                ui.ctx().input(|i| i.viewport().close_requested())
            });
        }
    }

    fn math_body(&mut self, ui: &mut egui::Ui) {
        let arm = self.cfg.arm;
        let q = self.q_cmd;
        let frames = kin::fk_frames(&arm, &q);

        egui::CollapsingHeader::new(RichText::new("1 · Vorwärtskinematik (DH)").strong()).default_open(true).show(ui, |ui| {
            formula(ui, "A_i = Rz(θ_i) · Tz(d_i) · Tx(a_i) · Rx(α_i)");
            formula(ui, "T_0→6 = A_1 · A_2 · A_3 · A_4 · A_5 · A_6");
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
            formula(ui, "       [ cθ  -sθ·cα   sθ·sα   a·cθ ]\nA_i =  [ sθ   cθ·cα  -cθ·sα   a·sθ ]\n       [ 0    sα       cα      d    ]\n       [ 0    0        0       1    ]");
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
            let j = jacobian(&arm, &q);
            formula(ui, "J = ∂p/∂q   (3×6, mm/rad)     ṗ = J · q̇");
            matrix(ui, "jac", &j.iter().map(|r| r.iter().map(|v| format!("{v:+8.1}")).collect()).collect::<Vec<_>>());
            let jjt: [[f64; 3]; 3] = std::array::from_fn(|a| std::array::from_fn(|b| (0..6).map(|k| j[a][k] * j[b][k]).sum()));
            let w = det3(jjt).max(0.0).sqrt();
            formula(ui, "w = √det(J·Jᵀ)   (Manipulierbarkeit)");
            let (txt, col) = if w < 2.0e4 { ("nahe Singularität: IK instabil, Gelenke können stark schwingen", WARN) } else { ("gut konditioniert", Color32::from_rgb(40, 160, 90)) };
            ui.horizontal(|ui| {
                ui.label(mono(format!("w = {w:10.0}")).color(col));
                ui.label(RichText::new(txt).color(col).size(12.0));
            });
        });

        egui::CollapsingHeader::new(RichText::new("3 · Inverse Kinematik").strong()).default_open(true).show(ui, |ui| {
            formula(ui, "e = p* − p(q)                         (Positionsfehler)");
            formula(ui, "Δq = Jᵀ · (J·Jᵀ + λ²·I)^-1 · e         (gedämpftes Newton / LM)");
            formula(ui, "Δq_N = (I − J⁺J) · g,  g = −W·q       (Nullraum-Zug)");
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
            formula(ui, "a = clamp(center + dir · q, 0, range)");
            formula(ui, "µs = pmin + a / range · (pmax − pmin)");
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
            formula(ui, "p(s) = A + s · (B − A)/|B − A|,   Schritt 2 mm");
            formula(ui, "q(s_k) = IK(p(s_k); Warmstart q(s_k−1)) + Nullraum-Zug");
            formula(ui, "Zeit/Schritt = max(Δs / v_TCP,  max|Δq_i| / ω_max)");
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
    let h = |ui: &mut egui::Ui, s: &str| {
        ui.add_space(8.0);
        ui.label(RichText::new(s).strong().size(15.0));
        ui.separator();
    };
    let p = |ui: &mut egui::Ui, s: &str| {
        ui.label(RichText::new(s).size(12.5));
    };
    h(ui, "1 · Vorwärtskinematik (Denavit-Hartenberg)");
    p(ui, "Jedes Gelenk i hat vier Parameter: Drehung θ_i (Variable), Versatz d_i entlang z, Länge a_i entlang x und Verdrehung α_i um x.");
    formula(ui, "A_i = Rz(θ_i) · Tz(d_i) · Tx(a_i) · Rx(α_i)\nθ_i = q_i + Offset_i");
    formula(ui, "       [ cθ  -sθ·cα   sθ·sα   a·cθ ]\nA_i =  [ sθ   cθ·cα  -cθ·sα   a·sθ ]\n       [ 0    sα       cα      d    ]\n       [ 0    0        0       1    ]");
    formula(ui, "T_0→6 = A_1 · A_2 · A_3 · A_4 · A_5 · A_6 = [ R  p ; 0  1 ]");
    p(ui, "R ist die 3×3-Drehmatrix der Werkzeugorientierung, p = (x, y, z) die Position der Werkzeugspitze (TCP).");

    h(ui, "2 · Orientierung (Roll · Pitch · Yaw)");
    formula(ui, "R = Rz(Y) · Ry(P) · Rx(R)\nP = −asin(R₃₁)\nR = atan2(R₃₂, R₃₃)\nY = atan2(R₂₁, R₁₁)");
    p(ui, "Bei P = ±90° fallen Roll und Yaw zusammen (Gimbal Lock): dort ist nur ihre Summe bestimmt.");

    h(ui, "3 · Jacobi-Matrix");
    p(ui, "Verknüpft Gelenkgeschwindigkeiten mit der Spitzengeschwindigkeit. Für Drehgelenke:");
    formula(ui, "ṗ = J · q̇\nJ_i = z_(i−1) × (p_6 − p_(i−1))        (Spalte i, 3×6)\nz_(i−1) = Drehachse von Gelenk i,  p_(i−1) = dessen Ursprung");
    formula(ui, "Manipulierbarkeit   w = √det(J · Jᵀ)");
    p(ui, "w → 0 bedeutet Singularität: der Arm verliert eine Bewegungsrichtung, kleine Spitzenbewegungen brauchen riesige Gelenkwinkel (typisch: Arm ganz gestreckt, Handgelenk gestreckt, Handgelenk über der Schulterachse).");

    h(ui, "4 · Inverse Kinematik");
    p(ui, "Gesucht: q mit p(q) = p*. Es gibt keine geschlossene Lösung für beliebige Gelenke, deshalb iterativ (Newton mit Dämpfung, Levenberg-Marquardt):");
    formula(ui, "e = p* − p(q)\nΔq = Jᵀ · (J·Jᵀ + λ²·I)⁻¹ · e\nq ← clamp(q + Δq, q_min, q_max)       Wiederholen bis |e| < ε");
    p(ui, "λ dämpft die Schritte nahe Singularitäten. Mit Orientierungsvorgabe wird e um den Winkelfehler erweitert (6 Gleichungen, 6 Unbekannte).");
    p(ui, "Ohne Winkelvorgabe bleiben 3 überzählige Freiheitsgrade. Der Nullraum verschiebt die Spitze nicht:");
    formula(ui, "Δq_N = (I − J⁺J) · g,   g = −W · q        J⁺ = Jᵀ(J·Jᵀ)⁻¹\nq ← q + α · Δq_N");
    p(ui, "g zieht die Gelenke zur Komfortstellung (W gewichtet je Gelenk), ohne den Zielpunkt zu verlassen. Mehrere Startwerte (Restarts) finden verschiedene Lösungen (Ellbogen oben/unten, Handgelenk geklappt).");

    h(ui, "5 · Servo-Abbildung");
    formula(ui, "a = clamp(center + dir · q, 0, range)        (Servowinkel)\nµs = pmin + a / range · (pmax − pmin)   (Pulsbreite)");
    p(ui, "center = Servo-Winkel bei q = 0, dir = ±1 (Drehrichtung), range = Stellweg des Servos, pmin/pmax = Pulsbreiten der Endlagen.");

    h(ui, "6 · Bahnplanung (Playlist)");
    formula(ui, "Gerade:      p(s) = A + s · (B − A)/|B − A|,   s ∈ [0, |B − A|],  Schritt 2 mm\nJe Schritt:  q(s_k) = IK(p(s_k); Warmstart q(s_k−1)) + Nullraum-Zug");
    formula(ui, "Gelenkfahrt: q(u) = q_A + u · (q_B − q_A),   u ∈ [0, 1]");
    formula(ui, "Zeit je Schritt = max( Δs / v_TCP ,  max_i |Δq_i| / ω_max )");
    p(ui, "Eine Gerade gilt als fahrbar, wenn die IK an jedem Schritt konvergiert und kein Gelenk mehr als 0,25 rad pro Schritt springt. Sonst Gelenkfahrt. Die gesamte Bahn wird vor dem Start berechnet: sie hängt nur von Arm und Punkten ab, nicht von Framerate, Geschwindigkeit oder Runde.");

    h(ui, "7 · Arbeitsraum (Hüllflächen)");
    p(ui, "Zufallsstichprobe q ~ U[q_min, q_max]^6 (900 000 Stück, 20 % direkt an den Anschlägen). Jede Position wird relativ zur Schulterachse c in Kugelkoordinaten (Azimut, Polarwinkel) einsortiert:");
    formula(ui, "r = |p(q) − c|\nr_max(Richtung) = größter r  → äußere Fläche\nr_min(Richtung) = kleinster r → innere Fläche (Totzone)");
    formula(ui, "r_außen = max r_max       r_innen = min r_min       r_überall = min r_max");
    p(ui, "Zwischen innerer und äußerer Fläche erreicht die Spitze den Punkt (Position, Winkel frei). Selbstkollisionen und Boden sind nicht berücksichtigt.");
}
