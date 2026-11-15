# robotarm-sim

Desktop-Simulator für einen 6-DOF-Roboterarm (Elfin-Cobot) in Rust (eframe/egui + wgpu).

- Vorwärts- und inverse Kinematik (DH, Levenberg-Marquardt) mit automatischem Werkzeugwinkel (schaltbar)
- Zielpunkt per Eingabe oder Maus im 3D-Raum, Winkel und Servo-Pulse werden berechnet
- Playlist: mehrere Zielpunkte, Bahn wird vorausberechnet (jede Runde exakt gleich), gerade Linien oder Gelenkfahrt, Loop
- Mathe-Fenster (eigene OS-Fenster) und Formel-Fenster (Σ in der Toolbar): DH-Tabelle, Matrizen, Jacobi, IK-Formeln, Servo-Abbildung, Planer live
- GPU-Rendering des Cobot-Modells (`assets/cobot.bin`), Gelenk-Scheiben mit exakten Winkeln

```
cargo run --release
cargo test --release
```

Dev-Hooks: `ROBOTARM_SHOT=/pfad.ppm` (Screenshot), `ROBOTARM_TAB`, `ROBOTARM_VIEW=yaw,pitch`, `ROBOTARM_POSE=j1,..,j6`, `ROBOTARM_PLAY=1`.
