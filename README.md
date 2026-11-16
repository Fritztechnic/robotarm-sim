# robotarm-sim

Desktop-Simulator für einen 6-DOF-Roboterarm (Elfin-Cobot) in Rust (eframe/egui + wgpu).

- Vorwärts- und inverse Kinematik: ideale Elfin-Geometrie, geschlossene IK (Paden-Kahan, alle bis zu 8 Haltungen exakt), Levenberg-Marquardt nur als Rückfall für selbst geänderte Geometrie
- Automatischer Werkzeugwinkel (schaltbar) nach fester Regel: die Haltung hängt nur von der Zielposition ab, Hin- und Rückweg sind identisch
- Zielpunkt per Eingabe oder Maus im 3D-Raum, Winkel und Servo-Pulse werden berechnet
- Playlist: mehrere Zielpunkte, Bahn wird vorausberechnet (jede Runde exakt gleich), gerade Linien oder Gelenkfahrt, Loop
- Mathe-Fenster (eigene OS-Fenster) und Formel-Fenster (Σ in der Toolbar): DH-Tabelle, Matrizen, Jacobi, IK-Formeln, Servo-Abbildung, Planer live
- GPU-Rendering des Cobot-Modells (`assets/cobot.bin`), Gelenk-Scheiben mit exakten Winkeln

```
cargo run --release
cargo test --release
```

Dev-Hooks: `ROBOTARM_SHOT=/pfad.ppm` (Screenshot), `ROBOTARM_TAB`, `ROBOTARM_VIEW=yaw,pitch`, `ROBOTARM_POSE=j1,..,j6`, `ROBOTARM_PLAY=1`.

## Werkzeugwinkel im Playbook, Vollkreis-Gelenke
- Winkel pro Punkt (Achsen-Symbol in der Playbook-Zeile) oder global („Winkel vorgeben“ in der Steuerung, gilt für den ganzen Ablauf). Zwischen Punkten mit verschiedenen Winkeln wird per Slerp überblendet. Der Planer verfolgt jede Gerade aus jeder exakten Haltung und wählt per Graph-Suche die Route mit den wenigsten Gelenkfahrten; ist ein Pfad nicht stetig fahrbar, folgt eine Gelenkfahrt.
- J1, J4 und J6: Anzeige springt bei ±360° auf 0, intern ±720° Spielraum; es wird stets der kürzeste Weg gewählt (359° → 0° = +1°).
- Linkes und rechtes Panel lassen sich über die Gruppe PANELS in der Werkzeugleiste ein-/ausklappen.
