// Abbildungen als Inline-SVG (zur Bauzeit erzeugt). Farben kommen aus docs/style.css (Klassen).
const f1 = (x) => (Math.round(x * 10) / 10).toString();
const svg = (w, h, body, cap) =>
  `<figure><svg viewBox="0 0 ${w} ${h}" role="img" xmlns="http://www.w3.org/2000/svg"><defs>
<marker id="ah" viewBox="0 0 10 10" refX="9" refY="5" markerWidth="7" markerHeight="7" orient="auto-start-reverse"><path d="M0 0L10 5L0 10z" fill="context-stroke"/></marker></defs>${body}</svg><figcaption>${cap}</figcaption></figure>`;
const line = (x1, y1, x2, y2, c = 'ax', arrow = false) =>
  `<line class="${c}" x1="${f1(x1)}" y1="${f1(y1)}" x2="${f1(x2)}" y2="${f1(y2)}"${arrow ? ' marker-end="url(#ah)"' : ''}/>`;
const text = (x, y, s, c = 'lbl', anchor = 'middle') => `<text class="${c}" x="${f1(x)}" y="${f1(y)}" text-anchor="${anchor}">${s}</text>`;
const dot = (x, y, r = 3.5, c = 'pt') => `<circle class="${c}" cx="${f1(x)}" cy="${f1(y)}" r="${r}"/>`;
const arc = (cx, cy, r, a0, a1, c = 'arcl') => {
  // Winkel in Grad, mathematisch (gegen den Uhrzeigersinn, y nach oben)
  const p = (a) => [cx + r * Math.cos((a * Math.PI) / 180), cy - r * Math.sin((a * Math.PI) / 180)];
  const [x0, y0] = p(a0), [x1, y1] = p(a1);
  return `<path class="${c}" d="M${f1(x0)} ${f1(y0)}A${r} ${r} 0 ${a1 - a0 > 180 ? 1 : 0} 0 ${f1(x1)} ${f1(y1)}"/>`;
};
const grid = (w, h, step, ox, oy) => {
  let s = '';
  for (let x = ox % step; x <= w; x += step) s += line(x, 0, x, h, 'grid');
  for (let y = oy % step; y <= h; y += step) s += line(0, y, w, y, 'grid');
  return s;
};

export const figures = {
  axes2d() {
    const u = 40, ox = 60, oy = 190;
    const P = (x, y) => [ox + x * u, oy - y * u];
    const [px, py] = P(3, 2);
    return svg(420, 230, `${grid(420, 230, u, ox, oy)}${line(ox - 20, oy, 400, oy, 'ax', true)}${line(ox, oy + 20, ox, 12, 'ax', true)}
${text(405, oy + 16, 'x')}${text(ox - 12, 14, 'y')}
${line(px, oy, px, py, 'dash')}${line(ox, py, px, py, 'dash')}${line(ox, oy, px, py, 'vA', true)}${dot(px, py)}
${text(px + 8, py - 8, 'P = (3 | 2)', 'lbl', 'start')}${text(px, oy + 16, '3')}${text(ox - 10, py + 4, '2')}`,
      'Ein Punkt in der Ebene: Zuerst 3 nach rechts (x), dann 2 nach oben (y). Der Pfeil vom Ursprung zu P ist der zugehörige <b>Vektor</b>.');
  },
  vecadd() {
    const u = 40, ox = 40, oy = 190;
    const P = (x, y) => [ox + x * u, oy - y * u];
    const [ax, ay] = P(3, 1), [bx, by] = P(1, 2), [cx, cy] = P(4, 3);
    return svg(420, 230, `${grid(420, 230, u, ox, oy)}
${line(ox, oy, ax, ay, 'vA', true)}${line(ox, oy, bx, by, 'vB', true)}
${line(ax, ay, cx, cy, 'dash')}${line(bx, by, cx, cy, 'dash')}${line(ox, oy, cx, cy, 'vC', true)}
${text(ax + 4, ay + 16, 'a = (3|1)', 'lblA', 'start')}${text(bx - 8, by - 6, 'b = (1|2)', 'lblB', 'end')}${text(cx + 8, cy - 6, 'a + b = (4|3)', 'lblC', 'start')}`,
      'Vektoren addiert man komponentenweise. Geometrisch: Pfeil b an die Spitze von a hängen (Parallelogramm).');
  },
  dot() {
    const ox = 50, oy = 180, s = 1;
    const a = [250, 0], b = [170, 120];
    const ab = (a[0] * b[0] + a[1] * b[1]) / (a[0] ** 2 + a[1] ** 2);
    const pr = [a[0] * ab, a[1] * ab];
    const ang = (Math.atan2(b[1], b[0]) * 180) / Math.PI;
    return svg(420, 220, `${line(ox, oy, ox + a[0], oy - a[1], 'vA', true)}${line(ox, oy, ox + b[0], oy - b[1], 'vB', true)}
${line(ox + b[0], oy - b[1], ox + pr[0], oy - pr[1], 'dash')}${line(ox, oy + 12, ox + pr[0], oy + 12, 'vC')}
${arc(ox, oy, 38, 0, ang)}${text(ox + 52, oy - 12, 'θ', 'lbl')}
${text(ox + a[0] + 6, oy + 4, 'a', 'lblA', 'start')}${text(ox + b[0] + 6, oy - b[1] - 4, 'b', 'lblB', 'start')}
${text(ox + pr[0] / 2, oy + 30, 'Länge der Projektion von b auf a', 'lblC')}`,
      'Das Skalarprodukt a·b = |a|·|b|·cos θ. Es misst, „wie viel von b in Richtung a zeigt“ (grüne Projektion), multipliziert mit |a|.');
  },
  cross() {
    // Schiefe Parallelprojektion: x nach rechts, y nach hinten-rechts-oben, z nach oben
    const ox = 150, oy = 190;
    const P = (x, y, z) => [ox + x * 90 + y * 42, oy - z * 90 - y * 26];
    const O = P(0, 0, 0), A = P(1, 0, 0), B = P(0, 1, 0), AB = P(1, 1, 0), C = P(0, 0, 1);
    return svg(420, 230, `<polygon class="fillA" points="${[O, A, AB, B].map((p) => p.map(f1).join(',')).join(' ')}"/>
${line(...O, ...A, 'vA', true)}${line(...O, ...B, 'vB', true)}${line(...O, ...C, 'vC', true)}
${text(A[0] + 6, A[1] + 14, 'a', 'lblA', 'start')}${text(B[0] + 6, B[1] - 4, 'b', 'lblB', 'start')}${text(C[0] + 8, C[1] + 4, 'a × b', 'lblC', 'start')}
${text(AB[0] + 30, AB[1] - 6, 'Fläche = |a × b|', 'lbl', 'start')}${dot(...O)}`,
      'Das Kreuzprodukt a × b steht senkrecht auf a <i>und</i> b (Rechte-Hand-Regel). Seine Länge ist die Fläche des aufgespannten Parallelogramms.');
  },
  unit() {
    const cx = 130, cy = 130, r = 100, th = 35, t = (th * Math.PI) / 180;
    const px = cx + r * Math.cos(t), py = cy - r * Math.sin(t);
    return svg(420, 260, `<circle class="circ" cx="${cx}" cy="${cy}" r="${r}"/>${line(cx - 115, cy, cx + 115, cy, 'ax', true)}${line(cx, cy + 115, cx, cy - 115, 'ax', true)}
${line(cx, cy, px, py, 'vA', true)}${line(px, py, px, cy, 'vB')}${line(cx, cy, px, cy, 'vC')}${arc(cx, cy, 30, 0, th)}${dot(px, py)}
${text(cx + 44, cy - 8, 'θ')}${text((cx + px) / 2, cy + 18, 'cos θ', 'lblC')}${text(px + 10, (cy + py) / 2, 'sin θ', 'lblB', 'start')}${text(px + 8, py - 8, '(cos θ | sin θ)', 'lbl', 'start')}${text(cx + 40, cy - 55, '1', 'lblA')}`,
      'Einheitskreis (Radius 1): Der Punkt zum Winkel θ hat die Koordinaten (cos θ | sin θ). Daraus folgt sofort cos²θ + sin²θ = 1.');
  },
  rot2d() {
    const cx = 120, cy = 150, r = 95, a0 = 20, th = 50;
    const p = (a) => [cx + r * Math.cos((a * Math.PI) / 180), cy - r * Math.sin((a * Math.PI) / 180)];
    const P = p(a0), Q = p(a0 + th);
    return svg(420, 230, `${line(cx - 20, cy, cx + 130, cy, 'ax', true)}${line(cx, cy + 20, cx, cy - 135, 'ax', true)}${arc(cx, cy, r, a0, a0 + th, 'dash')}
${line(cx, cy, ...P, 'vA', true)}${line(cx, cy, ...Q, 'vC', true)}${arc(cx, cy, 55, a0, a0 + th)}${text(cx + 66, cy - 55, 'θ')}
${text(P[0] + 8, P[1] + 4, 'P', 'lblA', 'start')}${text(Q[0] + 4, Q[1] - 8, "P' = R(θ)·P", 'lblC', 'start')}`,
      'Eine Drehung um den Ursprung: P wird um den Winkel θ gegen den Uhrzeigersinn auf P′ gedreht. Länge und Abstand zum Ursprung bleiben gleich.');
  },
  lawcos() {
    const A = [40, 190], B = [350, 190], C = [250, 50];
    return svg(420, 230, `<polygon class="fillA" points="${[A, B, C].map((p) => p.join(',')).join(' ')}"/>
${text(195, 210, 'c', 'lblC')}${text(130, 105, 'b', 'lblB', 'end')}${text(315, 105, 'a', 'lblA', 'start')}
${text(C[0] - 4, C[1] + 44, 'γ')}${text(...[A[0] + 48, A[1] - 8], 'α')}${text(B[0] - 50, B[1] - 8, 'β')}`,
      'Kosinussatz: c² = a² + b² − 2ab·cos γ. Kennt man drei Seiten, folgt jeder Winkel – so berechnet die IK den Ellbogenwinkel (J3).');
  },
  twolink() {
    const O = [90, 190], l1 = 130, l2 = 110, T = [230, 120];
    const d = Math.hypot(T[0] - O[0], O[1] - T[1]);
    const a = Math.atan2(O[1] - T[1], T[0] - O[0]);
    const cb = (l1 * l1 + d * d - l2 * l2) / (2 * l1 * d), b = Math.acos(cb);
    const E = (s) => [O[0] + l1 * Math.cos(a + s * b), O[1] - l1 * Math.sin(a + s * b)];
    const Eu = E(1), Ed = E(-1);
    return svg(420, 270, `${grid(420, 270, 40, O[0], O[1])}${line(O[0], O[1], ...Eu, 'vA')}${line(...Eu, ...T, 'vA')}${line(O[0], O[1], ...Ed, 'vB')}${line(...Ed, ...T, 'vB')}
${dot(...O)}${dot(...Eu, 4, 'ptA')}${dot(...Ed, 4, 'ptB')}${dot(...T, 5, 'ptC')}${text(T[0] + 10, T[1] + 4, 'Ziel', 'lblC', 'start')}
${text(Eu[0] - 6, Eu[1] - 8, 'Ellbogen oben', 'lblA', 'end')}${text(Ed[0] + 10, Ed[1] + 14, 'Ellbogen unten', 'lblB', 'start')}`,
      'Zwei Glieder, ein Ziel: Es gibt zwei Lösungen (Ellbogen oben / unten). Beim 6-Achser kommt das ebenfalls vor – deshalb gibt es mehrere <b>Haltungen</b>.');
  },
  armzero() {
    // Seitenansicht Nullstellung (x nach rechts, z nach oben), Maßstab 0,42
    const s = 0.42, ox = 150, oy = 330;
    const X = (x) => ox + x * s, Z = (z) => oy - z * s;
    const pts = [[0, 0], [0, 130], [0, 130], [240 * 0, 130 + 0]];
    return svg(420, 360, `${line(ox - 60, oy, ox + 130, oy, 'ground')}
${line(X(0), Z(0), X(0), Z(130), 'link', false)}${line(X(0), Z(130), X(0), Z(370), 'link')}${line(X(0), Z(370), X(0), Z(630), 'link')}${line(X(0), Z(630), X(0), Z(714.09), 'linkT')}
${[0, 130, 370, 630, 714.09].map((z) => dot(X(0), Z(z), 5)).join('')}
${line(X(70), Z(0), X(70), Z(130), 'dim')}${text(X(70) + 6, Z(65), 'd₁ = 130', 'lbl', 'start')}
${line(X(70), Z(130), X(70), Z(370), 'dim')}${text(X(70) + 6, Z(250), 'a₂ = 240', 'lbl', 'start')}
${line(X(70), Z(370), X(70), Z(630), 'dim')}${text(X(70) + 6, Z(500), 'd₄ = 260', 'lbl', 'start')}
${line(X(70), Z(630), X(70), Z(714.09), 'dim')}${text(X(70) + 6, Z(672), 'd₆ = 84,09', 'lbl', 'start')}
${text(X(-14), Z(130) + 4, 'J1/J2', 'lbl', 'end')}${text(X(-14), Z(370) + 4, 'J3', 'lbl', 'end')}${text(X(-14), Z(630) + 4, 'J4–J6', 'lbl', 'end')}${text(X(-14), Z(714) + 4, 'TCP', 'lblC', 'end')}`,
      'Der Elfin in Nullstellung (alle Gelenke 0): Er steht senkrecht, der TCP liegt bei (0 | 0 | 714,09) mm. Alle Maße in Millimetern.');
  },
  wristsing() {
    return svg(420, 200, `${line(40, 150, 140, 150, 'link')}${line(140, 150, 140, 70, 'link')}${line(140, 70, 380, 70, 'link')}${dot(140, 70, 5)}${dot(260, 70, 5)}${dot(380, 70, 5, 'ptC')}
${line(260, 70, 260, 150, 'dash')}${text(140, 52, 'J4-Achse', 'lbl')}${text(260, 52, 'J5', 'lbl')}${text(380, 52, 'J6-Achse', 'lbl')}
${text(210, 118, 'J5 = 0: J4- und J6-Achse fallen zusammen', 'lblB')}${text(210, 138, '→ nur die Summe J4 + J6 ist bestimmt', 'lblB')}`,
      'Handgelenk-Singularität: Bei J5 = 0 liegen die Achsen von J4 und J6 auf einer Geraden. Man kann dann nur noch die Summe der beiden Drehungen festlegen, nicht jede einzeln.');
  },
  linevsptp() {
    return svg(420, 220, `${grid(420, 220, 40, 20, 200)}${dot(60, 160, 5, 'ptA')}${dot(360, 60, 5, 'ptB')}
${line(60, 160, 360, 60, 'vA')}<path class="vB" d="M60 160C130 60 280 30 360 60"/>
${text(200, 126, 'Linie (Bahn)', 'lblA', 'start')}${text(160, 40, 'Gelenkfahrt (PTP)', 'lblB', 'start')}${text(60, 182, 'Start', 'lbl')}${text(360, 82, 'Ziel', 'lbl')}`,
      'Linearbahn: die Spitze fährt exakt gerade. Gelenkfahrt (PTP): Jedes Gelenk interpoliert für sich linear – die Spitze fährt einen Bogen. Daher ist PTP „teuer“ und wird vermieden.');
  },
  ramp() {
    const ox = 50, oy = 180, w = 320, h = 130;
    let d = `M${ox} ${oy}`;
    for (let i = 1; i <= 100; i++) {
      const t = i / 100, v = Math.sqrt(Math.min(t, 1 - t) / 0.35);
      d += `L${f1(ox + t * w)} ${f1(oy - Math.min(1, v) * h)}`;
    }
    return svg(420, 220, `${line(ox, oy, ox + w + 20, oy, 'ax', true)}${line(ox, oy, ox, 30, 'ax', true)}<path class="vA" d="${d}"/>
${line(ox, oy - h, ox + w, oy - h, 'dash')}${text(ox - 6, oy - h + 4, 'v', 'lbl', 'end')}${text(ox + w + 14, oy + 16, 't', 'lbl')}${text(ox + 50, oy + 18, 'Rampe', 'lbl')}${text(ox + w - 50, oy + 18, 'Rampe', 'lbl')}${text(ox + w / 2, oy - h - 8, 'Vorschubgeschwindigkeit', 'lbl')}`,
      'Geschwindigkeitsprofil einer Linie: sanft anfahren (v ∝ √τ), konstant fahren, sanft abbremsen. So gibt es keinen Ruck am Anfang und Ende.');
  },
  wrap() {
    const ox = 30, w = 360, y = 100, X = (a) => ox + ((a + 720) / 1440) * w;
    const ticks = [-720, -360, 0, 360, 720].map((a) => `${line(X(a), y - 6, X(a), y + 6)}${text(X(a), y + 24, a + '°')}`).join('');
    return svg(420, 190, `${line(ox, y, ox + w, y, 'ax', true)}${ticks}
${dot(X(340), y, 5, 'ptA')}${text(X(340) - 14, y + 44, '340°', 'lblA')}${dot(X(360), y, 5, 'ptB')}${text(X(360) + 20, y + 44, '360° = 0°', 'lblB', 'start')}
<path class="vC" d="M${f1(X(360))} ${y - 28}L${f1(X(0))} ${y - 28}" marker-end="url(#ah)"/>${text(X(180), y - 38, 'Rücksetzen auf 0°', 'lblC')}`,
      'J1, J4 und J6 dürfen intern bis ±720° laufen und werden bei ±360° auf 0° zurückgesetzt. Der Weg zum nächsten Ziel wird immer auf den <b>kurzen Weg</b> (≤ 180°) normiert.');
  },
  dpgraph() {
    const cols = [60, 160, 260, 360], rows = [50, 100, 150];
    let s = '';
    for (let c = 0; c < 3; c++) for (const y0 of rows) for (const y1 of rows) s += line(cols[c], y0, cols[c + 1], y1, y0 === y1 ? 'edge2' : 'edge');
    for (const x of cols) for (const y of rows) s += dot(x, y, 6);
    s += `<polyline class="best" points="60,100 160,100 260,150 360,150" fill="none"/>`;
    return svg(420, 220, `${s}${cols.map((x, i) => text(x, 190, 'Punkt ' + (i + 1))).join('')}${text(210, 18, 'jeder Knoten = eine Haltung des Arms', 'lbl')}`,
      'Planung als kürzester Weg: Pro Punkt gibt es mehrere Haltungen (Knoten). Eine Kante kostet wenig, wenn die Linie ohne Sprung fahrbar ist, und sehr viel (PTP-Strafe), wenn nicht. Dynamische Programmierung findet den günstigsten Gesamtweg.');
  },
  tooltip() {
    return svg(420, 240, `<rect class="fillB" x="40" y="190" width="340" height="24"/>${line(210, 40, 240, 168, 'link')}
<rect class="tool" x="196" y="40" width="30" height="20" transform="rotate(-8 210 50)"/>${dot(210, 40, 5)}${dot(240, 168, 5, 'ptC')}
${text(240, 36, 'Flansch', 'lbl', 'start')}${text(252, 172, 'Werkzeugspitze', 'lblC', 'start')}
${text(160, 112, 'L', 'lblB', 'end')}${line(170, 40, 170, 168, 'dim')}`,
      'Die Spitze liegt um die Werkzeuglänge L entlang der Werkzeugachse vor dem Flansch: p<sub>Flansch</sub> = p<sub>Spitze</sub> − L·z<sub>Werkzeug</sub>.');
  },
  sp1() {
    const c = [200, 130], r = 70;
    const p = (a) => [c[0] + r * Math.cos((a * Math.PI) / 180), c[1] - r * Math.sin((a * Math.PI) / 180)];
    const P = p(20), Q = p(110);
    return svg(420, 240, `<circle class="circ" cx="${c[0]}" cy="${c[1]}" r="${r}"/>${line(c[0], c[1] - 110, c[0], c[1] + 110, 'dash')}${text(c[0] + 8, c[1] - 100, 'Drehachse', 'lbl', 'start')}
${line(...c, ...P, 'vA')}${line(...c, ...Q, 'vC')}${arc(c[0], c[1], 30, 20, 110)}${text(c[0] + 18, c[1] - 40, 'θ?')}${dot(...P, 5, 'ptA')}${dot(...Q, 5, 'ptC')}
${text(P[0] + 10, P[1], 'p', 'lblA', 'start')}${text(Q[0] - 10, Q[1] - 6, 'q', 'lblC', 'end')}`,
      'Paden-Kahan Teilproblem 1: Um welchen Winkel θ muss man p um eine Achse drehen, damit es auf q fällt? Beide Punkte werden zuerst senkrecht zur Achse projiziert; dann ist θ = atan2(Kreuzprodukt, Skalarprodukt).');
  },
};
