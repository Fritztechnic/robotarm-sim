// Interaktive Bausteine der Mathematik-Dokumentation (reines JavaScript, keine Abhängigkeiten).
const NS = 'http://www.w3.org/2000/svg';
const el = (tag, attrs = {}, parent) => {
  const e = document.createElementNS(NS, tag);
  for (const k in attrs) e.setAttribute(k, attrs[k]);
  parent && parent.appendChild(e);
  return e;
};
const h = (html) => { const d = document.createElement('div'); d.innerHTML = html; return d.firstElementChild; };
const fmt = (x, n = 2) => (Math.abs(x) < 5e-10 ? 0 : x).toFixed(n).replace('.', ',');
const rad = (d) => (d * Math.PI) / 180, deg = (r) => (r * 180) / Math.PI;
const col = (n) => getComputedStyle(document.documentElement).getPropertyValue(n).trim();

function widget(name, title, build) {
  document.querySelectorAll(`[data-widget="${name}"]`).forEach((host) => {
    host.className = 'widget';
    host.innerHTML = `<h4>${title}</h4>`;
    build(host);
  });
}
// Ziehbarer Punkt in einem SVG
function draggable(svg, circle, toWorld, onMove) {
  let on = false;
  const pos = (e) => {
    const r = svg.getBoundingClientRect(), vb = svg.viewBox.baseVal;
    return [((e.clientX - r.left) / r.width) * vb.width, ((e.clientY - r.top) / r.height) * vb.height];
  };
  circle.style.cursor = 'grab';
  circle.addEventListener('pointerdown', (e) => { on = true; circle.setPointerCapture(e.pointerId); });
  circle.addEventListener('pointerup', () => (on = false));
  circle.addEventListener('pointermove', (e) => { if (on) onMove(...toWorld(...pos(e))); });
}

/* ---------- 1. Skalar- und Kreuzprodukt ---------- */
widget('dot', 'Ausprobieren: Skalarprodukt und Kreuzprodukt (Punkte ziehen)', (host) => {
  const u = 40, ox = 60, oy = 190;
  const svg = el('svg', { viewBox: '0 0 460 260', width: 460 }, host);
  for (let x = ox % u; x < 460; x += u) el('line', { x1: x, y1: 0, x2: x, y2: 260, stroke: col('--line') }, svg);
  for (let y = oy % u; y < 260; y += u) el('line', { x1: 0, y1: y, x2: 460, y2: y, stroke: col('--line') }, svg);
  el('line', { x1: 0, y1: oy, x2: 460, y2: oy, stroke: col('--mut') }, svg);
  el('line', { x1: ox, y1: 0, x2: ox, y2: 260, stroke: col('--mut') }, svg);
  const a = [4, 1], b = [1, 3];
  const la = el('line', { stroke: col('--A'), 'stroke-width': 3 }, svg), lb = el('line', { stroke: col('--B'), 'stroke-width': 3 }, svg);
  const pa = el('circle', { r: 9, fill: col('--A') }, svg), pb = el('circle', { r: 9, fill: col('--B') }, svg);
  const out = h('<span class="res"></span>');
  host.appendChild(out);
  const S = (v) => [ox + v[0] * u, oy - v[1] * u];
  const W = (x, y) => [Math.round(((x - ox) / u) * 2) / 2, Math.round(((oy - y) / u) * 2) / 2];
  const draw = () => {
    for (const [l, p, v] of [[la, pa, a], [lb, pb, b]]) {
      const [x, y] = S(v);
      l.setAttribute('x1', ox); l.setAttribute('y1', oy); l.setAttribute('x2', x); l.setAttribute('y2', y);
      p.setAttribute('cx', x); p.setAttribute('cy', y);
    }
    const d = a[0] * b[0] + a[1] * b[1], c = a[0] * b[1] - a[1] * b[0];
    const na = Math.hypot(...a), nb = Math.hypot(...b);
    const ang = na && nb ? deg(Math.acos(Math.max(-1, Math.min(1, d / (na * nb))))) : NaN;
    out.textContent = `a = (${fmt(a[0], 1)} | ${fmt(a[1], 1)})   b = (${fmt(b[0], 1)} | ${fmt(b[1], 1)})\n` +
      `a·b = ${fmt(a[0], 1)}·${fmt(b[0], 1)} + ${fmt(a[1], 1)}·${fmt(b[1], 1)} = ${fmt(d, 2)}\n` +
      `|a| = ${fmt(na)}, |b| = ${fmt(nb)}  →  Winkel θ = arccos(a·b / (|a||b|)) = ${fmt(ang, 1)}°\n` +
      `Kreuzprodukt (z-Komponente) a×b = ${fmt(a[0], 1)}·${fmt(b[1], 1)} − ${fmt(a[1], 1)}·${fmt(b[0], 1)} = ${fmt(c, 2)}  (${c > 0 ? 'b liegt links von a' : c < 0 ? 'b liegt rechts von a' : 'parallel'})\n` +
      (Math.abs(d) < 1e-9 ? '→ a·b = 0: die Vektoren stehen senkrecht aufeinander!' : '');
  };
  draggable(svg, pa, W, (x, y) => { a[0] = x; a[1] = y; draw(); });
  draggable(svg, pb, W, (x, y) => { b[0] = x; b[1] = y; draw(); });
  draw();
});

/* ---------- 2. Drehmatrix ---------- */
widget('rot', 'Ausprobieren: 2D-Drehung mit Drehmatrix', (host) => {
  host.insertAdjacentHTML('beforeend', '<label>θ <input type="range" min="-180" max="180" value="40"><output></output></label>');
  const svg = el('svg', { viewBox: '0 0 300 240', width: 300 }, host);
  const out = h('<span class="res"></span>'); host.appendChild(out);
  const slider = host.querySelector('input'), o = host.querySelector('output');
  const cx = 110, cy = 120, u = 70;
  el('line', { x1: 0, y1: cy, x2: 300, y2: cy, stroke: col('--mut') }, svg);
  el('line', { x1: cx, y1: 0, x2: cx, y2: 240, stroke: col('--mut') }, svg);
  const P = [1.5, 0.5];
  const l0 = el('line', { stroke: col('--A'), 'stroke-width': 3 }, svg), l1 = el('line', { stroke: col('--C'), 'stroke-width': 3 }, svg);
  const draw = () => {
    const t = rad(+slider.value), c = Math.cos(t), s = Math.sin(t);
    const Q = [c * P[0] - s * P[1], s * P[0] + c * P[1]];
    for (const [l, v] of [[l0, P], [l1, Q]]) { l.setAttribute('x1', cx); l.setAttribute('y1', cy); l.setAttribute('x2', cx + v[0] * u); l.setAttribute('y2', cy - v[1] * u); }
    o.textContent = slider.value + '°';
    out.textContent = `R = [ cos θ  −sin θ ]   = [ ${fmt(c, 3)}  ${fmt(-s, 3)} ]\n    [ sin θ   cos θ ]     [ ${fmt(s, 3)}  ${fmt(c, 3)} ]\n` +
      `P  = (${fmt(P[0])} | ${fmt(P[1])})  (rot)\nP' = R·P = (${fmt(Q[0], 3)} | ${fmt(Q[1], 3)})  (grün)\n|P| = ${fmt(Math.hypot(...P), 4)}, |P'| = ${fmt(Math.hypot(...Q), 4)}  – die Länge bleibt gleich.`;
  };
  slider.oninput = draw; draw();
});

/* ---------- 3. Zwei-Glieder-IK ---------- */
widget('ik2', 'Ausprobieren: Inverse Kinematik mit zwei Gliedern (Ziel ziehen)', (host) => {
  const L1 = 130, L2 = 110, ox = 100, oy = 200;
  const svg = el('svg', { viewBox: '0 0 460 250', width: 460 }, host);
  const lines = [col('--A'), col('--B')].map((c) => [el('polyline', { fill: 'none', stroke: c, 'stroke-width': 5, 'stroke-linecap': 'round', 'stroke-linejoin': 'round', opacity: 0.9 }, svg), c]);
  el('circle', { cx: ox, cy: oy, r: 6, fill: col('--fg') }, svg);
  const tgt = el('circle', { r: 10, fill: col('--C') }, svg);
  const out = h('<span class="res"></span>'); host.appendChild(out);
  let T = [180, 110];
  const draw = () => {
    const d = Math.hypot(...T);
    const rr = Math.min(Math.max(d, Math.abs(L1 - L2) + 1e-6), L1 + L2);
    // Kosinussatz: cos(q2) = (d² − L1² − L2²) / (2 L1 L2)
    const c2 = Math.max(-1, Math.min(1, (d * d - L1 * L1 - L2 * L2) / (2 * L1 * L2)));
    let txt = `Ziel = (${fmt(T[0], 0)} | ${fmt(T[1], 0)}),  d = ${fmt(d, 1)}\n`;
    [1, -1].forEach((s, i) => {
      const q2 = s * Math.acos(c2);
      const q1 = Math.atan2(T[1], T[0]) - Math.atan2(L2 * Math.sin(q2), L1 + L2 * Math.cos(q2));
      const E = [L1 * Math.cos(q1), L1 * Math.sin(q1)], Tp = [E[0] + L2 * Math.cos(q1 + q2), E[1] + L2 * Math.sin(q1 + q2)];
      lines[i][0].setAttribute('points', `${ox},${oy} ${ox + E[0]},${oy - E[1]} ${ox + Tp[0]},${oy - Tp[1]}`);
      txt += `Lösung ${i + 1} (${s > 0 ? 'Ellbogen unten' : 'Ellbogen oben'}):  q1 = ${fmt(deg(q1), 1)}°, q2 = ${fmt(deg(q2), 1)}°\n`;
    });
    if (d > L1 + L2 || d < Math.abs(L1 - L2)) txt += 'Ziel außerhalb der Reichweite – das Ergebnis ist nur die nächstmögliche Pose.';
    void rr;
    tgt.setAttribute('cx', ox + T[0]); tgt.setAttribute('cy', oy - T[1]);
    out.textContent = txt;
  };
  draggable(svg, tgt, (x, y) => [x - ox, oy - y], (x, y) => { T = [x, y]; draw(); });
  draw();
});

/* ---------- 4. Vorwärtskinematik des Elfin ---------- */
const COBOT = [[0, -90, 130, 0], [240, 0, 0, -90], [0, 90, 0, 90], [0, -90, 260, 180], [0, 90, 0, 0], [0, 0, 84.09, 0]]; // a, α, d, Offset
const LIM = [[-720, 720], [-134, 134], [-149, 149], [-720, 720], [-146, 146], [-720, 720]];
const mul = (A, B) => A.map((r, i) => B[0].map((_, j) => r.reduce((s, _, k) => s + A[i][k] * B[k][j], 0)));
function dh(a, al, d, th) {
  const ct = Math.cos(th), st = Math.sin(th), ca = Math.cos(al), sa = Math.sin(al);
  return [[ct, -st * ca, st * sa, a * ct], [st, ct * ca, -ct * sa, a * st], [0, sa, ca, d], [0, 0, 0, 1]];
}
function fk(qdeg) {
  let T = [[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]];
  const all = [];
  COBOT.forEach(([a, al, d, off], i) => { T = mul(T, dh(a, rad(al), d, rad(qdeg[i] + off))); all.push(T); });
  return all;
}
widget('fk', 'Ausprobieren: Vorwärtskinematik des Elfin (Gelenke einstellen)', (host) => {
  const q = [0, 0, 0, 0, 0, 0];
  const names = ['J1', 'J2', 'J3', 'J4', 'J5', 'J6'];
  const sl = names.map((n, i) => {
    const lab = h(`<label>${n} <input type="range" min="${Math.max(LIM[i][0], -180)}" max="${Math.min(LIM[i][1], 180)}" value="0"><output>0°</output></label>`);
    host.appendChild(lab);
    const inp = lab.querySelector('input'), out = lab.querySelector('output');
    inp.oninput = () => { q[i] = +inp.value; out.textContent = q[i] + '°'; draw(); };
    return inp;
  });
  const btn = h('<button style="margin-left:6px">Alle auf 0</button>'); host.appendChild(btn);
  btn.onclick = () => { sl.forEach((s, i) => { s.value = 0; s.oninput(); }); };
  const svg = el('svg', { viewBox: '0 0 460 320', width: 460 }, host);
  svg.style.cursor = 'grab';
  let az = -50, el_ = 22;
  const gl = el('g', {}, svg), body = el('g', {}, svg);
  const out = h('<span class="res"></span>'); host.appendChild(out);
  let drag = null;
  svg.addEventListener('pointerdown', (e) => { drag = [e.clientX, e.clientY, az, el_]; svg.setPointerCapture(e.pointerId); });
  svg.addEventListener('pointerup', () => (drag = null));
  svg.addEventListener('pointermove', (e) => { if (drag) { az = drag[2] + (e.clientX - drag[0]) * 0.5; el_ = Math.max(-10, Math.min(80, drag[3] + (e.clientY - drag[1]) * 0.4)); draw(); } });
  const draw = () => {
    const Ts = [[[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, 0]], ...fk(q)];
    const P = Ts.map((T) => [T[0][3], T[1][3], T[2][3]]);
    const ca = Math.cos(rad(az)), sa = Math.sin(rad(az)), ce = Math.cos(rad(el_)), se = Math.sin(rad(el_)), s = 0.3, ox = 230, oy = 270;
    const pr = (p) => [ox + s * (p[0] * ca - p[1] * sa), oy - s * (p[2] * ce + (p[0] * sa + p[1] * ca) * se)];
    const ln = (a, b, attrs) => el('line', { x1: pr(a)[0], y1: pr(a)[1], x2: pr(b)[0], y2: pr(b)[1], 'stroke-linecap': 'round', ...attrs }, body);
    gl.innerHTML = ''; body.innerHTML = '';
    for (let a = -500; a <= 500; a += 100) {
      for (const [p0, p1] of [[[a, -500, 0], [a, 500, 0]], [[-500, a, 0], [500, a, 0]]]) el('line', { x1: pr(p0)[0], y1: pr(p0)[1], x2: pr(p1)[0], y2: pr(p1)[1], stroke: col('--line') }, gl);
    }
    [['#e5483b', [150, 0, 0]], ['#4dae4a', [0, 150, 0]], ['#3a78d6', [0, 0, 150]]].forEach(([c, d]) => ln([0, 0, 0], d, { stroke: c, 'stroke-width': 2 }));
    for (let i = 0; i < 6; i++) if (Math.hypot(...P[i + 1].map((v, k) => v - P[i][k])) > 1e-6) { ln(P[i], P[i + 1], { stroke: '#2b3138', 'stroke-width': 11 }); ln(P[i], P[i + 1], { stroke: '#e07b00', 'stroke-width': 7 }); }
    for (let i = 0; i < 6; i++) { const T = Ts[i], z = [T[0][2], T[1][2], T[2][2]], o = P[i]; ln(o.map((v, k) => v - 26 * z[k]), o.map((v, k) => v + 26 * z[k]), { stroke: '#3b3f45', 'stroke-width': 13 }); }
    const T6 = Ts[6];
    [['#e5483b', 0], ['#4dae4a', 1], ['#3a78d6', 2]].forEach(([c, k]) => ln(P[6], [0, 1, 2].map((j) => P[6][j] + 60 * T6[j][k]), { stroke: c, 'stroke-width': 2.5 }));
    const T = Ts[6], R = T;
    const pitch = -Math.asin(Math.max(-1, Math.min(1, R[2][0])));
    const yaw = Math.atan2(R[1][0], R[0][0]), roll = Math.atan2(R[2][1], R[2][2]);
    out.textContent = `TCP-Position:  x = ${fmt(T[0][3])}  y = ${fmt(T[1][3])}  z = ${fmt(T[2][3])}  mm\n` +
      `Werkzeugwinkel: Roll = ${fmt(deg(roll), 1)}°  Pitch = ${fmt(deg(pitch), 1)}°  Yaw = ${fmt(deg(yaw), 1)}°\n` +
      `Matrix  T = [ ${T[0].slice(0, 3).map((v) => fmt(v, 3)).join('  ')} | ${fmt(T[0][3])} ]\n` +
      `            [ ${T[1].slice(0, 3).map((v) => fmt(v, 3)).join('  ')} | ${fmt(T[1][3])} ]\n` +
      `            [ ${T[2].slice(0, 3).map((v) => fmt(v, 3)).join('  ')} | ${fmt(T[2][3])} ]\n` +
      'Bei allen Gelenken 0 muss (0 | 0 | 714,09) herauskommen. Ansicht: mit der Maus ziehen, um zu drehen.';
  };
  draw();
});

/* ---------- Scroll-Spy für Browser-Baum und Zeitleiste ---------- */
(() => {
  const links = [...document.querySelectorAll('#toc a')];
  const map = new Map(links.map((a) => [a.getAttribute('href').slice(1), a]));
  const heads = [...document.querySelectorAll('main h2[id],main h3[id]')];
  const bar = document.getElementById('prog');
  const upd = () => {
    let cur = heads[0];
    for (const hd of heads) if (hd.getBoundingClientRect().top < 120) cur = hd;
    links.forEach((a) => a.classList.remove('on'));
    const a = cur && map.get(cur.id);
    if (a) { a.classList.add('on'); a.scrollIntoView({ block: 'nearest' }); }
    const max = document.documentElement.scrollHeight - innerHeight;
    if (bar) bar.style.width = (max > 0 ? (scrollY / max) * 100 : 0) + '%';
    document.querySelectorAll('#tl a').forEach((d) => d.classList.toggle('on', cur && d.dataset.h2 === (cur.tagName === 'H2' ? cur.id : cur.dataset.parent)));
  };
  let h2 = null; heads.forEach((hd) => { if (hd.tagName === 'H2') h2 = hd.id; else hd.dataset.parent = h2; });
  addEventListener('scroll', upd, { passive: true }); upd();
})();
