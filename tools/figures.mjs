// Abbildungen als Inline-SVG im Stil eines CAD-Viewports (Fusion 360). Alle Geometrien werden aus den
// echten Formeln berechnet (DH-Vorwärtskinematik, Kosinussatz, Rodrigues, …) – nichts ist „frei Hand“ gezeichnet.
const rad = (d) => (d * Math.PI) / 180, deg = (r) => (r * 180) / Math.PI;
const n2 = (x) => Math.round(x * 100) / 100;
const fm = (x, n = 1) => x.toFixed(n).replace('.', ',').replace('-', '−');

/* ---------- Text-Helfer ---------- */
const v = (s) => `<tspan class="v">${s}</tspan>`;
const vb = (s) => `<tspan class="v b">${s}</tspan>`;

/* ---------- SVG-Primitive ---------- */
const pt = (p) => `${n2(p[0])},${n2(p[1])}`;
const L = (a, b, c = 'ln', arrow = false, extra = '') =>
  `<line class="${c}" x1="${n2(a[0])}" y1="${n2(a[1])}" x2="${n2(b[0])}" y2="${n2(b[1])}"${arrow ? ' marker-end="url(#ah)"' : ''} ${extra}/>`;
const SUB = '₀₁₂₃₄₅₆₇₈₉', SUP = { '²': '2', '³': '3' };
const mk = (s) => s
  .replace(/[₀-₉]/g, (c) => `<tspan class="sb" baseline-shift="sub">${SUB.indexOf(c)}</tspan>`)
  .replace(/[²³]/g, (c) => `<tspan class="sb" baseline-shift="super">${SUP[c]}</tspan>`)
  .replace(/_([A-Za-z]+)/g, '<tspan class="sb" baseline-shift="sub">$1</tspan>');
const T = (p, s, c = 'tx', anchor = 'middle', dx = 0, dy = 0) =>
  `<text class="${c}" x="${n2(p[0] + dx)}" y="${n2(p[1] + dy)}" text-anchor="${anchor}">${mk(s)}</text>`;
const D = (p, r = 4.5, c = 'dt') => `<circle class="${c}" cx="${n2(p[0])}" cy="${n2(p[1])}" r="${r > 6 ? r : n2(r * 0.7)}"/>`;
const poly = (pts, c = 'fl') => `<polygon class="${c}" points="${pts.map(pt).join(' ')}"/>`;
const path = (d, c = 'ln', extra = '') => `<path class="${c}" d="${d}" ${extra}/>`;
const polyline = (pts, c = 'ln') => `<polyline class="${c}" points="${pts.map(pt).join(' ')}"/>`;

function arcPts(map, m, r, a0, a1, steps = 40) {
  const p = [];
  for (let i = 0; i <= steps; i++) {
    const a = rad(a0 + ((a1 - a0) * i) / steps);
    p.push(map([m[0] + r * Math.cos(a), m[1] + r * Math.sin(a)]));
  }
  return p;
}
const arcPath = (map, m, r, a0, a1, steps) => 'M' + arcPts(map, m, r, a0, a1, steps).map(pt).join('L');
const arcArrow = (map, m, r, a0, a1) => path(arcPath(map, m, r, a0, a1), 'ln arc', 'marker-end="url(#ah)"');
const plane = (ox, oy, u) => (p) => [ox + p[0] * u, oy - p[1] * u];
const unitv = (a, b) => { const d = Math.hypot(b[0] - a[0], b[1] - a[1]) || 1; return [(b[0] - a[0]) / d, (b[1] - a[1]) / d]; };
const rightMark = (o, d1, d2, s = 10) => polyline([[o[0] + d1[0] * s, o[1] + d1[1] * s], [o[0] + (d1[0] + d2[0]) * s, o[1] + (d1[1] + d2[1]) * s], [o[0] + d2[0] * s, o[1] + d2[1] * s]], 'ln thin');

function grid2d(map, x0, x1, y0, y1, step = 1, labels = true) {
  let s = L(map([x0, 0]), map([x1, 0]), 'axl') + L(map([0, y0]), map([0, y1]), 'axl');
  for (let x = x0; x <= x1 + 1e-9; x += step) if (x !== 0) { const p = map([x, 0]); s += L([p[0], p[1] - 3], [p[0], p[1] + 3], 'axl') + (labels ? T(p, fm(x, 0), 'tk', 'middle', 0, 16) : ''); }
  for (let y = y0; y <= y1 + 1e-9; y += step) if (y !== 0) { const p = map([0, y]); s += L([p[0] - 3, p[1]], [p[0] + 3, p[1]], 'axl') + (labels ? T(p, fm(y, 0), 'tk', 'end', -7, 4) : ''); }
  return s;
}

/* ---------- 3D: Z-oben-Kamera, echte Parallelprojektion ---------- */
function cam(az, el, s, ox, oy) {
  const ca = Math.cos(rad(az)), sa = Math.sin(rad(az)), ce = Math.cos(rad(el)), se = Math.sin(rad(el));
  return (p) => [ox + s * (p[0] * ca - p[1] * sa), oy - s * (p[2] * ce + (p[0] * sa + p[1] * ca) * se)];
}
function fitCam(az, el, pts, w, h, pad) {
  const u = cam(az, el, 1, 0, 0), q = pts.map(u);
  const x0 = Math.min(...q.map((p) => p[0])), x1 = Math.max(...q.map((p) => p[0])), y0 = Math.min(...q.map((p) => p[1])), y1 = Math.max(...q.map((p) => p[1]));
  const sc = Math.min((w - 2 * pad) / (x1 - x0), (h - 2 * pad) / (y1 - y0));
  return cam(az, el, sc, w / 2 - (sc * (x0 + x1)) / 2, h / 2 - (sc * (y0 + y1)) / 2);
}
function floor3d(pr, ext, step) {
  let s = '';
  for (let a = -ext; a <= ext + 1e-9; a += step) {
    s += L(pr([a, -ext, 0]), pr([a, ext, 0]), a === 0 ? 'axl' : 'gr') + L(pr([-ext, a, 0]), pr([ext, a, 0]), a === 0 ? 'axl' : 'gr');
  }
  return s;
}
const axes3d = (pr, len) => L(pr([0, 0, 0]), pr([len, 0, 0]), 'ax x', true) + L(pr([0, 0, 0]), pr([0, len, 0]), 'ax y', true) + L(pr([0, 0, 0]), pr([0, 0, len]), 'ax z', true);
const arrow3d = (pr, a, b, c) => L(pr(a), pr(b), c, true);

/* ---------- Rahmen ---------- */
const DEFS = `<defs>
<marker id="ah" viewBox="0 0 10 10" refX="9" refY="5" markerUnits="userSpaceOnUse" markerWidth="11" markerHeight="11" orient="auto-start-reverse"><path d="M0 1.8L10 5L0 8.2L2.2 5z" fill="context-stroke"/></marker>
<marker id="dm" viewBox="0 0 10 10" refX="5" refY="5" markerUnits="userSpaceOnUse" markerWidth="9" markerHeight="9" orient="auto-start-reverse"><path d="M0 2L10 5L0 8L2 5z" fill="context-stroke"/></marker></defs>`;
function fig(w, h, title, body, cap) {
  return `<figure class="vp"><svg viewBox="0 0 ${w} ${h}" role="img" xmlns="http://www.w3.org/2000/svg">${DEFS}${body}</svg>
<figcaption><b>${title}.</b> ${cap}</figcaption></figure>`;
}

/* ---------- Die echte Kinematik des Elfin ---------- */
const COBOT = [[0, -90, 130, 0], [240, 0, 0, -90], [0, 90, 0, 90], [0, -90, 260, 180], [0, 90, 0, 0], [0, 0, 84.09, 0]];
const mul = (A, B) => A.map((r, i) => B[0].map((_, j) => r.reduce((s, _, k) => s + A[i][k] * B[k][j], 0)));
function dh(a, al, d, th) {
  const ct = Math.cos(th), st = Math.sin(th), ca = Math.cos(al), sa = Math.sin(al);
  return [[ct, -st * ca, st * sa, a * ct], [st, ct * ca, -ct * sa, a * st], [0, sa, ca, d], [0, 0, 0, 1]];
}
function fk(q) {
  let T0 = [[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]];
  const out = [T0];
  COBOT.forEach(([a, al, d, o], i) => { T0 = mul(T0, dh(a, rad(al), d, rad(q[i] + o))); out.push(T0); });
  return out;
}
const org = (M) => [M[0][3], M[1][3], M[2][3]];
const col = (M, k) => [M[0][k], M[1][k], M[2][k]];
const add = (a, b, s = 1) => [a[0] + s * b[0], a[1] + s * b[1], a[2] + s * b[2]];
const sub3 = (i) => String.fromCharCode(8320 + i);

function armDrawing(pr, q, { frames = false, tri = 70, labels = true } = {}) {
  const Ts = fk(q), P = Ts.map(org);
  let s = '';
  const chain = P.filter((p, i) => i === 0 || Math.hypot(p[0] - P[i - 1][0], p[1] - P[i - 1][1], p[2] - P[i - 1][2]) > 1e-6);
  const pts = chain.map(pr);
  s += polyline(pts, 'tube out') + polyline(pts, 'tube in');
  for (let i = 0; i < 6; i++) {
    const z = col(Ts[i], 2), o = org(Ts[i]);
    s += L(pr(add(o, z, -26)), pr(add(o, z, 26)), 'jnt out') + L(pr(add(o, z, -26)), pr(add(o, z, 26)), 'jnt in');
  }
  if (frames) {
    for (let i = 0; i <= 6; i++) {
      const o = org(Ts[i]);
      s += arrow3d(pr, o, add(o, col(Ts[i], 0), tri), 'ax x') + arrow3d(pr, o, add(o, col(Ts[i], 1), tri), 'ax y') + arrow3d(pr, o, add(o, col(Ts[i], 2), tri), 'ax z');
      if (labels) s += T(pr(o), i === 6 ? 'TCP' : 'KS' + sub3(i), 'tx s', 'start', 8, 16);
    }
  }
  return { svg: s, Ts, P };
}

/* =================================================================== */
export const figures = {
  rtri() {
    const m = plane(120, 270, 1), A = [0, 0], B = [330, 0], C = [330, 200], th = Math.atan2(200, 330) * 180 / Math.PI;
    return fig(640, 320, 'Das rechtwinklige Dreieck',
      `${poly([m(A), m(B), m(C)], 'fl')}${rightMark(m(B), [-1, 0], [0, 1], 14)}
${path(arcPath(m, A, 70, 0, th), 'ln arc')}${T(m([86, 14]), 'θ', 'tx', 'start')}
${T(m([165, 0]), 'Ankathete  b', 'tx', 'middle', 0, 24)}${T(m([330, 100]), 'Gegenkathete  a', 'tx', 'start', 14, 4)}${T(m([165, 100]), 'Hypotenuse  c', 'tx', 'end', -14, -24)}
${T(m(A), 'A', 'tx', 'end', -10, 6)}${T(m(B), 'B', 'tx', 'start', 8, 22)}${T(m(C), 'C', 'tx', 'start', 8, -6)}
${T([20, 40], 'sin θ = a / c', 'tx s', 'start')}${T([20, 66], 'cos θ = b / c', 'tx s', 'start')}${T([20, 92], 'tan θ = a / b', 'tx s', 'start')}${T([20, 118], 'a² + b² = c²', 'tx s', 'start')}`,
      'Die Seite gegenüber vom rechten Winkel heißt Hypotenuse (<i>c</i>), die anliegende Seite des Winkels θ Ankathete (<i>b</i>), die gegenüberliegende Gegenkathete (<i>a</i>). Daraus folgen Sinus, Kosinus, Tangens und der Satz des Pythagoras.');
  },

  pyth() {
    const m = plane(120, 260, 30), a = 3, b = 4;
    const P = [[0, 0], [b, 0], [b, a]];
    return fig(640, 400, 'Satz des Pythagoras mit Zahlen',
      `${poly([m([0, 0]), m([b, 0]), m([b, a])], 'fl')}${rightMark(m([b, 0]), [-1, 0], [0, 1], 12)}
${poly([m([0, 0]), m([4, 3]), m([1, 7]), m([-3, 4])], 'fl A')}${T(m([-0.5, 3.5]), 'c² = 5² = 25', 'tx', 'middle', 0, 6)}${poly([m([0, 0]), m([b, 0]), m([b, -b]), m([0, -b])], 'fl B')}${poly([m([b, 0]), m([b + a, 0]), m([b + a, a]), m([b, a])], 'fl B')}
${T(m([b / 2, -b / 2]), 'b² = 4² = 16', 'tx', 'middle', 0, 6)}${T(m([b + a / 2, a / 2]), 'a² = 3² = 9', 'tx', 'middle', 0, 6)}
${T([420, 60], '3² + 4² = 9 + 16 = 25 = 5²', 'tx s', 'start')}${T([420, 90], 'c = √25 = 5', 'tx s', 'start')}`,
      'Über den beiden Katheten stehen Quadrate mit den Flächen 16 und 9. Zusammen sind sie 25 – genau die Fläche des Quadrats über der Hypotenuse, deren Seitenlänge also √25 = 5 ist.');
  },
  axes2d() {
    const m = plane(70, 250, 70), P = [3, 2];
    return fig(560, 300, 'Koordinatensystem (Ebene)',
      `${grid2d(m, 0, 6, 0, 3, 1)}${L(m([0, 0]), m([6.6, 0]), 'axl x', true)}${L(m([0, 0]), m([0, 3.5]), 'axl y', true)}${T(m([6.6, 0]), 'x', 'tx', 'start', 8, 5)}${T(m([0, 3.5]), 'y', 'tx', 'middle', 0, -8)}
${L(m([3, 0]), m(P), 'ln dash')}${L(m([0, 2]), m(P), 'ln dash')}${L(m([0, 0]), m(P), 'vec A', true)}${D(m(P))}${D(m([0, 0]), 4, 'dt o')}
${T(m(P), `${v('P')} = (3 | 2)`, 'tx', 'start', 12, -10)}${T(m([1.5, 1]), vb('p'), 'tx a', 'end', -6, -10)}`,
      'Der Punkt <i>P</i> hat die Koordinaten (3 | 2): erst 3 Einheiten nach rechts, dann 2 nach oben. Der Pfeil vom Ursprung zu <i>P</i> ist der Ortsvektor <b>p</b>.');
  },

  unit() {
    const m = plane(200, 175, 120), th = 35, t = rad(th), c = Math.cos(t), s = Math.sin(t), Pp = [c, s];
    return fig(640, 350, 'Einheitskreis',
      `${grid2d(m, -1.5, 1.5, -1.5, 1.5, 0.5, false)}${path(arcPath(m, [0, 0], 1, 0, 360, 120), 'ln circ')}
${L(m([-1.5, 0]), m([1.5, 0]), 'axl x', true)}${L(m([0, -1.4]), m([0, 1.4]), 'axl y', true)}
${L(m([c, 0]), m(Pp), 'vec B')}${L(m([0, 0]), m([c, 0]), 'vec C')}${L(m([0, 0]), m(Pp), 'vec A', true)}
${path(arcPath(m, [0, 0], 0.3, 0, th), 'ln arc')}${T(m([0.34, 0.07]), v('θ') + ' = 35°', 'tx', 'start', 4, 0)}
${D(m(Pp))}${T(m(Pp), `(cos θ | sin θ) = (${fm(c, 3)} | ${fm(s, 3)})`, 'tx', 'start', 10, -8)}
${T(m([c / 2, 0]), 'cos θ = ' + fm(c, 3), 'tx c', 'middle', 0, 20)}${T(m([c, s / 2]), 'sin θ = ' + fm(s, 3), 'tx b', 'start', 8, 4)}
${T(m([-0.35, 0.45]), 'r = 1', 'tx a', 'end')}
${T(m([1, 0]), '1', 'tk', 'middle', 0, 16)}${T(m([-1, 0]), '−1', 'tk', 'middle', 0, 16)}${T(m([0, 1]), '1', 'tk', 'end', -8, 4)}${T(m([0, -1]), '−1', 'tk', 'end', -8, 4)}`,
      'Der Punkt auf dem Einheitskreis zum Winkel θ = 35° hat die Koordinaten (cos θ | sin θ). Pythagoras im markierten Dreieck liefert cos²θ + sin²θ = 1.');
  },

  vecadd() {
    const m = plane(70, 270, 70), a = [3, 1], b = [1, 2], c = [4, 3];
    return fig(560, 320, 'Vektoraddition',
      `${grid2d(m, 0, 6, 0, 3.5, 1)}${L(m(a), m(c), 'ln dash B')}${L(m(b), m(c), 'ln dash A')}
${L(m([0, 0]), m(a), 'vec A', true)}${L(m([0, 0]), m(b), 'vec B', true)}${L(m([0, 0]), m(c), 'vec C', true)}
${T(m(a), vb('a') + ' = (3 | 1)', 'tx a', 'start', 8, 18)}${T(m(b), vb('b') + ' = (1 | 2)', 'tx b', 'start', 14, -16)}${T(m(c), vb('a') + ' + ' + vb('b') + ' = (4 | 3)', 'tx c', 'start', 10, -6)}
${D(m([0, 0]), 4, 'dt o')}`,
      'Komponentenweise addieren: (3 | 1) + (1 | 2) = (4 | 3). Geometrisch hängt man <b>b</b> an die Spitze von <b>a</b>; es entsteht ein Parallelogramm, dessen Diagonale die Summe ist.');
  },

  dot() {
    const m = plane(70, 270, 80), a = [4, 1], b = [1.5, 3];
    const ab = a[0] * b[0] + a[1] * b[1], aa = a[0] * a[0] + a[1] * a[1];
    const pr = [a[0] * (ab / aa), a[1] * (ab / aa)];
    const na = Math.hypot(...a), nb = Math.hypot(...b), th = deg(Math.acos(ab / (na * nb)));
    const aAng = deg(Math.atan2(a[1], a[0])), bAng = deg(Math.atan2(b[1], b[0]));
    const foot = m(pr), bt = m(b), o = m([0, 0]);
    return fig(660, 340, 'Skalarprodukt und Projektion',
      `${grid2d(m, 0, 6, 0, 3.5, 1)}
${L(o, m(a), 'vec A', true)}${L(o, bt, 'vec B', true)}${L(bt, foot, 'ln dash')}${L(o, foot, 'vec C thick')}
${rightMark(foot, unitv(foot, o), unitv(foot, bt), 10)}
${path(arcPath(m, [0, 0], 0.9, aAng, bAng), 'ln arc')}${T(m([1.0, 0.85]), 'θ = ' + fm(th, 1) + '°', 'tx', 'start', 6, 0)}
${T(m(a), vb('a') + ' = (4 | 1)', 'tx a', 'start', 8, 4)}${T(m(b), vb('b') + ' = (1,5 | 3)', 'tx b', 'start', 8, -6)}
${T(m([pr[0] / 2, pr[1] / 2]), 'Projektion', 'tx c', 'middle', 4, -12)}
${T([430, 80], vb('a') + '·' + vb('b') + ' = 4·1,5 + 1·3 = ' + fm(ab, 1), 'tx', 'start')}
${T([430, 108], '|' + vb('a') + '| = ' + fm(na, 3) + ',  |' + vb('b') + '| = ' + fm(nb, 3), 'tx s', 'start')}
${T([430, 132], 'cos θ = ' + fm(ab / (na * nb), 3) + ' → θ = ' + fm(th, 1) + '°', 'tx s', 'start')}
${T([430, 156], 'Projektionslänge = ' + fm(ab / na, 3), 'tx s c', 'start')}`,
      'Das Skalarprodukt ist 9; daraus folgt der Winkel zwischen den Vektoren. Die blaue Strecke ist die Projektion von <b>b</b> auf <b>a</b> (Lotfuß mit rechtem Winkel).');
  },

  cross() {
    const pr = cam(-38, 28, 80, 200, 305);
    const a = [2, 0, 0], b = [0.6, 1.6, 0], cz = a[0] * b[1] - a[1] * b[0];
    const O = [0, 0, 0], AB = add(a, b);
    const ex = pr([1, 0, 0]), o = pr(O), ux = unitv(o, ex);
    return fig(640, 400, 'Kreuzprodukt (Rechte-Hand-Regel)',
      `${floor3d(pr, 3, 1)}${poly([O, a, AB, b].map(pr), 'fl A')}${L(pr(a), pr(AB), 'ln dash')}${L(pr(b), pr(AB), 'ln dash')}
${arrow3d(pr, O, a, 'vec A')}${arrow3d(pr, O, b, 'vec B')}${arrow3d(pr, O, [0, 0, cz], 'vec C')}${D(o, 4, 'dt o')}
${rightMark(o, ux, [0, -1], 12)}
${T(pr(a), vb('a') + ' = (2 | 0 | 0)', 'tx a', 'start', 8, 18)}${T(pr(b), vb('b') + ' = (0,6 | 1,6 | 0)', 'tx b', 'start', 8, -6)}
${T(pr([0, 0, cz]), vb('a') + ' × ' + vb('b') + ' = (0 | 0 | ' + fm(cz, 1) + ')', 'tx c', 'start', 10, 0)}
${T(pr([(a[0] + b[0]) / 2 + 0.2, (a[1] + b[1]) / 2, 0]), 'Fläche = ' + fm(cz, 1), 'tx', 'middle', 0, 5)}`,
      'a × b = (0 | 0 | 2·1,6 − 0·0,6) = (0 | 0 | 3,2). Der Vektor steht senkrecht auf der Fläche; seine Länge 3,2 ist genau die Fläche des Parallelogramms (Grundlinie 2 · Höhe 1,6).');
  },

  lawcos() {
    const a = 3.2, b = 4.0, c = 5.0;
    const A = Math.acos((b * b + c * c - a * a) / (2 * b * c)), B = Math.acos((a * a + c * c - b * b) / (2 * a * c)), g = Math.acos((a * a + b * b - c * c) / (2 * a * b));
    const m = plane(70, 300, 100), PA = [0, 0], PB = [c, 0], PC = [b * Math.cos(A), b * Math.sin(A)];
    return fig(660, 340, 'Kosinussatz',
      `${poly([PA, PB, PC].map(m), 'fl B')}${L(m(PA), m(PB), 'vec C')}${L(m(PB), m(PC), 'vec A')}${L(m(PC), m(PA), 'vec B')}
${path(arcPath(m, PA, 0.6, 0, deg(A)), 'ln arc')}${path(arcPath(m, PB, 0.6, 180 - deg(B), 180), 'ln arc')}${path(arcPath(m, PC, 0.45, 180 + deg(A), 360 - deg(B)), 'ln arc')}
${T(m(PA), 'α = ' + fm(deg(A), 1) + '°', 'tx', 'start', 66, -10)}${T(m(PB), 'β = ' + fm(deg(B), 1) + '°', 'tx', 'end', -66, -10)}${T(m(PC), 'γ = ' + fm(deg(g), 1) + '°', 'tx', 'middle', 0, 52)}
${T(m([c / 2, 0]), 'c = 5,0', 'tx c', 'middle', 0, 22)}${T(m([(PB[0] + PC[0]) / 2, (PB[1] + PC[1]) / 2]), 'a = 3,2', 'tx a', 'start', 10, 0)}${T(m([PC[0] / 2, PC[1] / 2]), 'b = 4,0', 'tx b', 'end', -10, -4)}
${D(m(PA))}${D(m(PB))}${D(m(PC))}
${T([20, 50], 'c² = a² + b² − 2ab·cos γ', 'tx', 'start')}${T([20, 76], '25,00 = 10,24 + 16,00 − 25,6·cos γ', 'tx s', 'start')}${T([20, 100], 'cos γ = ' + fm(Math.cos(g), 4) + ' → γ = ' + fm(deg(g), 2) + '°', 'tx s c', 'start')}`,
      'Alle drei Seiten sind bekannt (3,2 / 4,0 / 5,0), damit liegen alle Winkel fest. Genau so bestimmt die IK den Ellbogenwinkel aus Gliedlängen und Abstand.');
  },

  rot2d() {
    const m = plane(110, 290, 80), P = [2.4, 0.8], th = 50, t = rad(th), c = Math.cos(t), s = Math.sin(t);
    const Q = [c * P[0] - s * P[1], s * P[0] + c * P[1]], r = Math.hypot(...P), a0 = deg(Math.atan2(P[1], P[0]));
    return fig(680, 340, 'Drehung um den Ursprung',
      `${grid2d(m, -1, 3, 0, 3, 1)}${path(arcPath(m, [0, 0], r, a0, a0 + th), 'ln dash')}
${L(m([0, 0]), m(P), 'vec A', true)}${L(m([0, 0]), m(Q), 'vec C', true)}${path(arcPath(m, [0, 0], 0.9, a0, a0 + th), 'ln arc')}
${T(m([0.55, 1.05]), 'θ = 50°', 'tx', 'start', 4, -2)}${D(m(P))}${D(m(Q))}
${T(m(P), vb('P') + ' = (2,4 | 0,8)', 'tx a', 'start', 10, 20)}${T(m(Q), 'P’ = (' + fm(Q[0], 3) + ' | ' + fm(Q[1], 3) + ')', 'tx c', 'start', 10, -8)}
${T([430, 80], 'R(50°) = [ cos θ  −sin θ ; sin θ  cos θ ]', 'tx s', 'start')}${T([430, 106], '= [ ' + fm(c, 3) + '  ' + fm(-s, 3) + ' ; ' + fm(s, 3) + '  ' + fm(c, 3) + ' ]', 'tx s', 'start')}
${T([430, 140], '|P| = ' + fm(r, 3) + ',  |P’| = ' + fm(Math.hypot(...Q), 3), 'tx s c', 'start')}`,
      'P’ = R(θ)·P. Der gestrichelte Kreis zeigt: Die Länge bleibt erhalten, nur die Richtung ändert sich. Alle Zahlen sind exakt nachgerechnet.');
  },

  rodrigues() {
    const pr = cam(-35, 24, 150, 290, 290), vv = [1.0, 0.2, 0.9], th = 70;
    const par = [0, 0, vv[2]], rp = Math.hypot(vv[0], vv[1]), ph = Math.atan2(vv[1], vv[0]);
    const rot = [rp * Math.cos(ph + rad(th)), rp * Math.sin(ph + rad(th)), vv[2]];
    const arc = [], full = [];
    for (let i = 0; i <= 80; i++) { const a = ph + rad(th) * (i / 80); arc.push(pr([rp * Math.cos(a), rp * Math.sin(a), vv[2]])); }
    for (let i = 0; i <= 120; i++) { const a = (i / 120) * 2 * Math.PI; full.push(pr([rp * Math.cos(a), rp * Math.sin(a), vv[2]])); }
    const hz = [0, 0, vv[2]];
    return fig(640, 390, 'Rodrigues-Formel: Drehung um eine Achse',
      `${floor3d(pr, 1.5, 0.5)}${L(pr([0, 0, -0.4]), pr([0, 0, 1.7]), 'ln axk', true)}
${polyline(full, 'ln circ')}${polyline(arc, 'ln arc thick')}
${L(pr(hz), pr([vv[0], vv[1], vv[2]]), 'ln dash')}${L(pr(hz), pr(rot), 'ln dash C')}
${arrow3d(pr, [0, 0, 0], vv, 'vec A')}${arrow3d(pr, [0, 0, 0], rot, 'vec C')}${arrow3d(pr, [0, 0, 0], par, 'vec B thick')}
${D(pr([0, 0, 0]), 4, 'dt o')}
${T(pr(vv), vb('v'), 'tx a', 'start', 8, 14)}${T(pr(rot), vb('v') + '’', 'tx c', 'start', 8, -4)}${T(pr([0, 0, 1.7]), vb('k') + '  (Drehachse)', 'tx', 'start', 8, 0)}${T(pr([0, 0, 0.45]), 'Anteil längs k', 'tx b', 'end', -10, 0)}
${T(pr([rp * Math.cos(ph + rad(th / 2)) * 0.6, rp * Math.sin(ph + rad(th / 2)) * 0.6, vv[2]]), 'θ = 70°', 'tx', 'middle', 0, -8)}`,
      '<b>v</b> wird um die Achse <b>k</b> um 70° gedreht. Der Anteil längs der Achse bleibt (senkrechter Pfeil), der senkrechte Anteil läuft auf dem Kreis; die Länge |<b>v</b>’| = |<b>v</b>| bleibt erhalten.');
  },

  armzero() {
    const s = 0.5, ox = 220, oy = 400, m = (x, z) => [ox + x * s, oy - z * s];
    const zs = [0, 130, 370, 630, 714.09];
    let g = L(m(-200, 0), m(280, 0), 'axl') + L(m(0, 0), m(0, 780), 'ln dash');
    const pts = zs.map((z) => m(0, z));
    g += polyline(pts, 'tube out') + polyline(pts, 'tube in');
    g += `<rect class="jbox" x="${n2(m(-34, 130)[0])}" y="${n2(m(0, 130)[1])}" width="${n2(68 * s)}" height="${n2(130 * s)}"/>`;
    for (const [z, lab] of [[130, 'J2'], [370, 'J3'], [630, 'J5']]) g += `<circle class="jc" cx="${n2(m(0, z)[0])}" cy="${n2(m(0, z)[1])}" r="12"/>${T(m(0, z), '×', 'tx s', 'middle', 0, 5)}${T(m(0, z), lab, 'tx a', 'end', -22, 5)}`;
    g += T(m(0, 60), 'J1', 'tx a', 'end', -22, 4) + T(m(0, 500), 'J4', 'tx a', 'end', -22, 5) + T(m(0, 672), 'J6', 'tx a', 'end', -22, 4);
    g += D(m(0, 714.09), 6, 'dt C') + T(m(0, 714.09), 'TCP (0 | 0 | 714,09)', 'tx c', 'start', 22, 4);
    const dim = (z0, z1, x, lab) => L(m(x, z0), m(x, z1), 'dim', false, 'marker-start="url(#dm)" marker-end="url(#dm)"') + L(m(x - 8, z0), m(x + 8, z0), 'dim') + L(m(x - 8, z1), m(x + 8, z1), 'dim') + T(m(x, (z0 + z1) / 2), lab, 'tx', 'start', 10, 4);
    g += dim(0, 130, 120, 'd₁ = 130') + dim(130, 370, 120, 'a₂ = 240') + dim(370, 630, 120, 'd₄ = 260') + dim(630, 714.09, 120, 'd₆ = 84,09');
    g += T(m(190, 450), 'Σ = 714,09 mm', 'tx b', 'start');
    return fig(580, 440, 'Elfin in Nullstellung (Seitenansicht, alle q = 0)', g, 'Alle Gelenke auf 0: Der Arm steht senkrecht. Die Gesamthöhe ist die Summe der vier Maße: 130 + 240 + 260 + 84,09 = 714,09 mm. Die Längen sind maßstäblich gezeichnet (Gelenke J2, J3, J5 drehen um Achsen senkrecht zur Zeichenebene).');
  },

  dhframes() {
    const q = [35, 50, -70, 40, 60, 30], Ts = fk(q), pts = [[0, 0, 0], [150, 0, 0], [0, 150, 0], [0, 0, 150]];
    Ts.forEach((M) => { const o = org(M); pts.push(o, add(o, col(M, 0), 70), add(o, col(M, 1), 70), add(o, col(M, 2), 70)); });
    const pr = fitCam(-50, 22, pts, 720, 500, 40);
    const arm = armDrawing(pr, q, { frames: true, tri: 70 }), tcp = org(Ts[6]);
    return fig(720, 560, 'DH-Koordinatensysteme am Elfin (q = 35°, 50°, −70°, 40°, 60°, 30°)',
      `${floor3d(pr, 300, 100)}${axes3d(pr, 150)}${arm.svg}
${T([20, 540], 'x rot · y grün · z blau (Gelenkachse)', 'tx s', 'start')}${T([700, 540], 'TCP = (' + fm(tcp[0], 1) + ' | ' + fm(tcp[1], 1) + ' | ' + fm(tcp[2], 1) + ') mm', 'tx c s', 'end')}`,
      'Jedes Glied trägt ein eigenes Koordinatensystem; z ist stets die Drehachse des Gelenks. Die Lage jedes Systems ist das Matrizenprodukt A₁···Aᵢ – mit der echten Elfin-Tabelle berechnet.');
  },

  sp1() {
    const pr = cam(-40, 30, 140, 300, 300), r = 1.2, h = 0.7, a1 = 15, a2 = 112;
    const pp = (a) => [r * Math.cos(rad(a)), r * Math.sin(rad(a)), h];
    const full = [], arc = [];
    for (let i = 0; i <= 120; i++) full.push(pr([r * Math.cos(rad(i * 3)), r * Math.sin(rad(i * 3)), h]));
    for (let i = 0; i <= 50; i++) arc.push(pr(pp(a1 + ((a2 - a1) * i) / 50)));
    const C = [0, 0, h];
    return fig(640, 400, 'Teilproblem 1: Drehwinkel um eine Achse',
      `${floor3d(pr, 1.5, 0.5)}${L(pr([0, 0, 0]), pr([0, 0, 1.7]), 'ln axk', true)}${T(pr([0, 0, 1.7]), vb('k'), 'tx', 'start', 8, 4)}
${polyline(full, 'ln circ')}${polyline(arc, 'ln arc thick')}
${L(pr(C), pr(pp(a1)), 'ln A thick')}${L(pr(C), pr(pp(a2)), 'ln C thick')}
${D(pr(pp(a1)), 6, 'dt A')}${D(pr(pp(a2)), 6, 'dt C')}${D(pr(C), 3.5, 'dt')}
${T(pr(pp(a1)), vb('p'), 'tx a', 'start', 10, 14)}${T(pr(pp(a2)), vb('q'), 'tx c', 'end', -10, -6)}
${T(pr(pp((a1 + a2) / 2)), 'θ = ?', 'tx', 'middle', 0, -14)}
${T([20, 330], 'θ = atan2( k·(u×v), u·v ) = ' + fm(a2 - a1, 1) + '°', 'tx s', 'start')}`,
      'Beide Punkte liegen im gleichen Abstand von der Achse <b>k</b> (Kreis). Gesucht ist der Winkel θ, der <b>p</b> in <b>q</b> dreht. Das Kreuzprodukt liefert sin θ, das Skalarprodukt cos θ.');
  },

  twolink() {
    const L1 = 240, L2 = 260, sc = 0.75, m = plane(80, 400, sc), Tg = [330, 260], d = Math.hypot(...Tg);
    const c2 = (d * d - L1 * L1 - L2 * L2) / (2 * L1 * L2), ang = Math.atan2(Tg[1], Tg[0]);
    const sol = (sg) => { const q2 = sg * Math.acos(c2), q1 = ang - Math.atan2(L2 * Math.sin(q2), L1 + L2 * Math.cos(q2)); return { q1, q2, E: [L1 * Math.cos(q1), L1 * Math.sin(q1)] }; };
    const up = sol(1), dn = sol(-1);
    const dimv = (p, q, off, lab) => {
      const [u0, u1] = unitv(p, q), n = [-u1, u0], a = [p[0] + n[0] * off, p[1] + n[1] * off], b = [q[0] + n[0] * off, q[1] + n[1] * off];
      return L(a, b, 'dim', false, 'marker-start="url(#dm)" marker-end="url(#dm)"') + L(p, a, 'dim') + L(q, b, 'dim') + T([(a[0] + b[0]) / 2 + n[0] * 12, (a[1] + b[1]) / 2 + n[1] * 12 + 5], lab, 'tx');
    };
    let g = L(m([-30, 0]), m([540, 0]), 'axl', true) + L(m([0, -30]), m([0, 440]), 'axl', true) + T(m([540, 0]), v('x'), 'tx', 'start', 8, 5) + T(m([0, 440]), v('y'), 'tx', 'middle', 0, -8);
    g += path(arcPath(m, [0, 0], L1 + L2, 0, 90, 90), 'circ');
    g += polyline([m([0, 0]), m(dn.E), m(Tg)], 'ln dash') + D(m(dn.E), 4.5, 'dt A');
    g += polyline([m([0, 0]), m(up.E), m(Tg)], 'ln thick');
    g += L(m([0, 0]), m(Tg), 'ln dash') + L(m(Tg), m([Tg[0], 0]), 'ln dash') + L(m(Tg), m([0, Tg[1]]), 'ln dash');
    const e = up.E, ext = [e[0] + 0.55 * L2 * Math.cos(up.q1), e[1] + 0.55 * L2 * Math.sin(up.q1)];
    g += L(m(e), m(ext), 'ln dash');
    g += arcArrow(m, [0, 0], 85, 0, deg(up.q1)) + T(m([105 * Math.cos(up.q1 / 2), 105 * Math.sin(up.q1 / 2)]), v('q') + '₁', 'tx', 'middle', 0, 5);
    g += arcArrow(m, e, 70, deg(up.q1), deg(up.q1 + up.q2)) + T(m([e[0] + 95 * Math.cos(up.q1 + up.q2 / 2), e[1] + 95 * Math.sin(up.q1 + up.q2 / 2)]), v('q') + '₂', 'tx', 'middle', 0, 5);
    g += dimv(m([0, 0]), m(e), -22, v('L') + '₁') + dimv(m(e), m(Tg), -22, v('L') + '₂');
    g += T(m([Tg[0] / 2, Tg[1] / 2]), v('d'), 'tx', 'middle', 14, 16);
    g += D(m([0, 0]), 5, 'dt') + D(m(e), 4.5, 'dt A') + D(m(Tg), 4.5, 'dt A');
    g += T(m(Tg), vb('P') + ' (' + v('x') + ', ' + v('y') + ')', 'tx', 'start', 12, -6) + T(m([Tg[0], 0]), v('x') + '_P', 'tx', 'middle', 0, 20) + T(m([0, Tg[1]]), v('y') + '_P', 'tx', 'end', -8, 5);
    g += T(m(dn.E), 'zweite Lösung', 'tx s', 'end', -10, 4);
    g += T([40, 452], v('L') + '₁ = 240,  ' + v('L') + '₂ = 260,  ' + v('d') + ' = ' + fm(d, 1), 'tx s', 'start');
    g += T([40, 478], 'cos ' + v('q') + '₂ = (' + v('d') + '² − ' + v('L') + '₁² − ' + v('L') + '₂²) / (2' + v('L') + '₁' + v('L') + '₂) = ' + fm(c2, 3), 'tx s', 'start');
    g += T([40, 504], v('q') + '₂ = ±' + fm(deg(Math.acos(c2)), 1) + '°  →  zwei Lösungen', 'tx s c', 'start');
    return fig(700, 520, 'Zwei Glieder, ein Ziel – zwei Lösungen', g, 'Der Kosinussatz liefert den Ellbogenwinkel q₂ = ±arccos(…). Durchgezogen: eine Lösung, gestrichelt: die gespiegelte. Der Kreis ist die maximale Reichweite L₁ + L₂.');
  },

  wristsing() {
    const make = (q5, ox, label) => {
      const q = [0, 70, -70, 0, q5, 0], F = fk(q), wq = org(F[4]), pts = [[0, 0, 0], wq, add(wq, col(F[3], 2), 200), add(wq, col(F[3], 2), -150), add(wq, col(F[5], 2), 200), add(wq, col(F[5], 2), -150), [-150, 0, 0], [150, 0, 0]], pr0 = fitCam(-35, 12, pts, 340, 400, 20), pr = (p) => { const r = pr0(p); return [r[0] + ox - 170, r[1] + 85]; };
      const arm = armDrawing(pr, q), Ts = arm.Ts, wp = org(Ts[4]);
      const z3 = col(Ts[3], 2), z5 = col(Ts[5], 2);
      const ax = (z, c) => L(pr(add(wp, z, -150)), pr(add(wp, z, 200)), 'ln dash ' + c);
      const dp = Math.abs(z3[0] * z5[0] + z3[1] * z5[1] + z3[2] * z5[2]);
      const lab = dp > 0.9999 ? T(pr(add(wp, z3, 205)), 'J4- und J6-Achse', 'tx b', 'middle', 0, -6)
        : T(pr(add(wp, z3, 205)), 'J4-Achse', 'tx a', 'middle', 0, -6) + T(pr(add(wp, z5, 205)), 'J6-Achse', 'tx b', 'start', 8, 4);
      return `${L(pr([-150, 0, 0]), pr([150, 0, 0]), 'axl')}${arm.svg}${ax(z3, 'A')}${ax(z5, 'B')}${lab}${D(pr(wp), 5, 'dt o')}
${T([ox, 18], label, 'tx b', 'middle')}${T([ox, 504], '|z₃·z₅| = ' + fm(dp, 3) + (dp > 0.9999 ? ' → Achsen fallen zusammen' : ''), 'tx s', 'middle')}`;
    };
    return fig(760, 520, 'Handgelenk-Singularität', make(0, 200, 'J5 = 0°  (Singularität)') + make(50, 540, 'J5 = 50°  (regulär)'),
      'Links: Bei J5 = 0 liegen die Drehachsen von J4 und J6 exakt übereinander (|z₃·z₅| = 1); die Hand kann nur noch die Summe J4 + J6 festlegen. Rechts: Bei J5 = 50° sind beide Achsen verschieden. Die Achsen sind aus der echten Elfin-Kinematik berechnet.');
  },

  linevsptp() {
    const L1 = 240, L2 = 260, s = 0.78, m = plane(230, 380, s);
    const ik = (p) => { const d = Math.hypot(...p), c2 = (d * d - L1 * L1 - L2 * L2) / (2 * L1 * L2), q2 = -Math.acos(c2); return [Math.atan2(p[1], p[0]) - Math.atan2(L2 * Math.sin(q2), L1 + L2 * Math.cos(q2)), q2]; };
    const tipOf = (q) => [L1 * Math.cos(q[0]) + L2 * Math.cos(q[0] + q[1]), L1 * Math.sin(q[0]) + L2 * Math.sin(q[0] + q[1])];
    const elb = (q) => [L1 * Math.cos(q[0]), L1 * Math.sin(q[0])];
    const A = [330, 60], B = [100, 400], qa = ik(A), qb = ik(B);
    const lerp = (t) => [qa[0] + t * (qb[0] - qa[0]), qa[1] + t * (qb[1] - qa[1])];
    const ptp = []; for (let i = 0; i <= 60; i++) ptp.push(m(tipOf(lerp(i / 60))));
    const pose = (q, cls) => polyline([m([0, 0]), m(elb(q)), m(tipOf(q))], 'tube out ghost') + polyline([m([0, 0]), m(elb(q)), m(tipOf(q))], 'tube in ghost ' + cls);
    let dev = 0;
    const nx = B[1] - A[1], ny = -(B[0] - A[0]), nl = Math.hypot(nx, ny);
    for (let i = 0; i <= 200; i++) { const p = tipOf(lerp(i / 200)); dev = Math.max(dev, Math.abs(((p[0] - A[0]) * nx + (p[1] - A[1]) * ny) / nl)); }
    return fig(760, 450, 'Linearbahn gegen Gelenkfahrt (PTP)',
      `${grid2d(m, -200, 500, 0, 450, 100)}${pose(qa, 'A')}${pose(lerp(0.5), 'C')}${pose(qb, 'B')}
${L(m(A), m(B), 'vec C thick')}${polyline(ptp, 'ln B thick')}${D(m(A), 6, 'dt A')}${D(m(B), 6, 'dt B')}${D(m([0, 0]), 7, 'dt o')}
${T(m(A), 'Start', 'tx a', 'start', 10, 16)}${T(m(B), 'Ziel', 'tx b', 'start', 12, 0)}
${T([480, 60], 'Linie: Spitze exakt gerade', 'tx c', 'start')}${T([480, 84], 'PTP: Bogen, bis ' + fm(dev, 0) + ' mm neben der Linie', 'tx b', 'start')}`,
      'Planarer Arm (L₁ = 240, L₂ = 260). Bei der Gelenkfahrt interpoliert jedes Gelenk linear; die Spitze weicht bis zu ' + fm(dev, 0) + ' mm von der Geraden ab. Die Zwischenstellung nach der halben Zeit ist als Geist eingezeichnet.');
  },

  dpgraph() {
    const cols = [130, 260, 390, 520], rows = [90, 170, 250], names = ['Haltung A', 'Haltung B', 'Haltung C'];
    const cost = [[1.2, 0.9, 1.5], [1.0, 0.7, 1.1], [1.4, 1.3, 0.8]]; // je Schicht und Haltung (gleiche Haltung → gleiche Haltung)
    let g = '';
    for (let c = 0; c < 3; c++) for (let i = 0; i < 3; i++) for (let j = 0; j < 3; j++) {
      const a = [cols[c], rows[i]], b = [cols[c + 1], rows[j]];
      if (i === j) g += L(a, b, 'ln edge') + T([(a[0] + b[0]) / 2, a[1] - 8], fm(cost[c][i], 1), 'tk', 'middle');
      else g += L(a, b, 'ln edge bad');
    }
    g += polyline(cols.map((x) => [x, rows[1]]), 'ln best');
    for (const x of cols) for (const y of rows) g += D([x, y], 9, 'dt node');
    g += cols.map((x, i) => T([x, 300], 'Punkt ' + (i + 1), 'tx')).join('');
    g += rows.map((y, i) => T([cols[0] - 22, y], names[i], 'tx s', 'end', 0, 4)).join('');
    const sums = [0, 1, 2].map((i) => cost[0][i] + cost[1][i] + cost[2][i]);
    g += T([330, 28], 'Gesamtkosten:  A = ' + fm(sums[0], 1) + '   B = ' + fm(sums[1], 1) + '   C = ' + fm(sums[2], 1) + '  →  B gewinnt', 'tx c', 'middle');
    g += L([340, 332], [376, 332], 'ln edge bad') + T([384, 336], 'Haltungswechsel: nur als PTP, Kosten 1000', 'tx s', 'start');
    return fig(680, 350, 'Haltungsgraph und kürzester Weg', g, 'Jeder Knoten ist eine Haltung an einem Wegpunkt. Fahrbar sind Übergänge in derselben Haltung (Kosten = Gelenkweg in rad, Zahlen an den Kanten); ein Wechsel kostet die PTP-Strafe 1000. Die grüne Folge hat die kleinste Summe (B: 0,9 + 0,7 + 1,3 = 2,9).');
  },

  wrap() {
    const ox = 60, w = 560, y = 170, X = (a) => ox + ((a + 720) / 1440) * w;
    let g = `<rect class="band" x="${X(-720)}" y="${y - 22}" width="${X(720) - X(-720)}" height="44"/><rect class="band2" x="${X(-360)}" y="${y - 22}" width="${X(360) - X(-360)}" height="44"/>`;
    g += L([X(-720) - 10, y], [X(720) + 14, y], 'axl', true);
    for (const a of [-720, -540, -360, -180, 0, 180, 360, 540, 720]) g += L([X(a), y - 8], [X(a), y + 8], 'ln') + T([X(a), y + 34], fm(a, 0) + '°', a % 360 === 0 ? 'tx s' : 'tk', 'middle');
    for (const a of [-360, 360]) g += L([X(a), y - 70], [X(a), y + 22], 'ln dash B');
    g += T([X(0), y - 30], 'Anzeigebereich ±360°', 'tx s', 'middle') + T([X(-540), y - 30], 'interner Puffer', 'tk', 'middle') + T([X(540), y - 30], 'interner Puffer', 'tk', 'middle');
    g += path(`M${X(360)} ${y - 70}Q${X(180)} ${y - 125} ${X(6)} ${y - 72}`, 'ln C', 'marker-end="url(#ah)"');
    g += path(`M${X(-360)} ${y - 70}Q${X(-180)} ${y - 125} ${X(-6)} ${y - 72}`, 'ln C', 'marker-end="url(#ah)"');
    g += T([X(180), y - 112], '360° → 0°', 'tx c', 'middle') + T([X(-180), y - 112], '−360° → 0°', 'tx c', 'middle');
    g += D([X(32), y], 6, 'dt A') + D([X(33), y], 6, 'dt B') + T([X(32), y + 70], 'von 32° nach 33°: Weg = +1° (nie −359°)', 'tx s', 'middle');
    return fig(680, 270, 'Rücksetzen der Winkel J1, J4, J6', g, 'J1, J4 und J6 laufen intern bis ±720°. Erreicht die Anzeige ±360°, springt sie auf 0° (gleiche Stellung). Der Weg zwischen zwei Zielen wird immer als kürzester Weg (≤ 180°) berechnet.');
  },

  ramp() {
    const Tt = 2.4, Tr = 0.35, ox = 80, oy = 270, w = 500, h = 190;
    const X = (t) => ox + (t / Tt) * w, Y = (f) => oy - f * h, f = (t) => Math.min(1, Math.sqrt(Math.min(t, Tt - t) / Tr));
    const pts = []; for (let i = 0; i <= 240; i++) { const t = (i / 240) * Tt; pts.push([X(t), Y(f(t))]); }
    let g = '';
    for (let t = 0; t <= Tt + 1e-9; t += 0.4) g += L([X(t), oy], [X(t), Y(1.15)], 'gr') + T([X(t), oy + 18], fm(t, 1), 'tk', 'middle');
    for (const fv of [0, 0.5, 1]) g += L([ox, Y(fv)], [ox + w, Y(fv)], fv === 0 ? 'axl' : 'gr') + T([ox - 8, Y(fv)], fm(fv, 1), 'tk', 'end', 0, 4);
    g += poly([[X(0), Y(0)], ...pts, [X(Tt), Y(0)]], 'fl C') + polyline(pts, 'ln C thick');
    g += L([X(Tr), Y(1)], [X(Tr), oy], 'ln dash') + L([X(Tt - Tr), Y(1)], [X(Tt - Tr), oy], 'ln dash');
    g += T([X(Tr), oy + 38], 'T_r = 0,35 s', 'tx s', 'middle') + T([X(Tt - Tr), oy + 38], 'T − T_r', 'tx s', 'middle');
    g += T([X(Tt / 2), Y(1) - 10], 'v = v_max (konstant)', 'tx c', 'middle');
    g += T([ox - 44, 30], 'v / v_max', 'tx s', 'start') + T([ox + w, oy + 62], 'Zeit τ in s', 'tx s', 'end');
    return fig(660, 340, 'Geschwindigkeitsprofil einer Linie (T = 2,4 s)', g, 'Exakte Kurve v/v_max = min(1, √(min(τ, T−τ)/T_r)): Anfahren und Bremsen über je 0,35 s mit Wurzelverlauf, dazwischen konstanter Vorschub. Die Fläche unter der Kurve ist der zurückgelegte Weg.');
  },

  tooltip() {
    const Lt = 100, s = 2.2, m = plane(300, 330, s);
    const phi = rad(-165), zW = [-Math.sin(phi), Math.cos(phi)]; // z-Achse (y, z) der Werkzeugdrehung Rx(−165°)
    const tip = [0, 0], fl = [tip[0] - Lt * zW[0], tip[1] - Lt * zW[1]], nrm = [-zW[1], zW[0]];
    const a2 = (p, d, k) => [p[0] + k * d[0], p[1] + k * d[1]];
    const rect = (a, b, w) => [a2(a, nrm, -w), a2(a, nrm, w), a2(b, nrm, w), a2(b, nrm, -w)];
    const hk = a2(fl, zW, -24);
    let g = poly([[-80, -8], [80, -8], [80, 0], [-80, 0]].map(m), 'fl B') + L(m([-80, 0]), m([80, 0]), 'ln');
    g += poly(rect(hk, fl, 13).map(m), 'part') + poly(rect(fl, tip, 5).map(m), 'tool');
    g += L(m(tip), m([0, 130]), 'ln dash') + L(m(fl), m(a2(fl, zW, -30)), 'ln dash');
    g += path(arcPath(m, tip, 70, 90, 105, 12), 'ln arc') + T(m([-4, 78]), '15°', 'tx', 'end', -6, 0);
g += D(m(fl), 6, 'dt A') + D(m(tip), 6, 'dt C');
    g += T(m(fl), 'Flansch (wird gerechnet)', 'tx a', 'end', -16, -30) + T(m(tip), 'Spitze (Aufgabe)', 'tx c', 'start', 14, -14);
    const o1 = a2(fl, nrm, -22), o2 = a2(tip, nrm, -22);
    g += L(m(o1), m(o2), 'dim', false, 'marker-start="url(#dm)" marker-end="url(#dm)"') + T(m([(o1[0] + o2[0]) / 2, (o1[1] + o2[1]) / 2]), 'L = 100', 'tx', 'end', -10, 0);
    g += T([440, 120], 'p_Flansch = p_Spitze − L·z_W', 'tx', 'start') + T([440, 146], 'z_W = (0 | ' + fm(zW[0], 4) + ' | ' + fm(zW[1], 4) + ')', 'tx s', 'start');
    g += T([440, 168], 'p_Flansch − p_Spitze = (0 | ' + fm(-Lt * zW[0], 2) + ' | ' + fm(-Lt * zW[1], 2) + ') mm', 'tx s c', 'start');
    return fig(780, 380, 'Schweißen: Spitze und Flansch (Seitenansicht y–z)', g, 'Der Brenner ist um 15° geneigt. Der Flansch liegt L = 100 mm entgegen der Werkzeugachse z_W hinter der Spitze: seitlich um 100·sin 15° = 25,88 mm versetzt und 96,59 mm höher. Alle Werte sind aus R = Rₓ(−165°) berechnet.');
  },
};
