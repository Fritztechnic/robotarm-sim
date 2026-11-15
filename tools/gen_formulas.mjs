// Erzeugt aus LaTeX die SVGs in assets/tex/ (MathJax). Aufruf: npm i mathjax-full && node tools/gen_formulas.mjs
import { mathjax } from 'mathjax-full/js/mathjax.js';
import { TeX } from 'mathjax-full/js/input/tex.js';
import { SVG } from 'mathjax-full/js/output/svg.js';
import { liteAdaptor } from 'mathjax-full/js/adaptors/liteAdaptor.js';
import { RegisterHTMLHandler } from 'mathjax-full/js/handlers/html.js';
import { AllPackages } from 'mathjax-full/js/input/tex/AllPackages.js';
import fs from 'fs';

const F = {
  dh_a: String.raw`A_i = R_z(\theta_i)\,T_z(d_i)\,T_x(a_i)\,R_x(\alpha_i),\qquad \theta_i = q_i + \theta_{0,i}`,
  dh_m: String.raw`A_i=\begin{bmatrix}\cos\theta_i & -\sin\theta_i\cos\alpha_i & \sin\theta_i\sin\alpha_i & a_i\cos\theta_i\\ \sin\theta_i & \cos\theta_i\cos\alpha_i & -\cos\theta_i\sin\alpha_i & a_i\sin\theta_i\\ 0 & \sin\alpha_i & \cos\alpha_i & d_i\\ 0&0&0&1\end{bmatrix}`,
  t06: String.raw`T_{0\to6}=A_1A_2A_3A_4A_5A_6=\begin{bmatrix}R & \mathbf p\\ \mathbf 0^{T} & 1\end{bmatrix}`,
  rpy: String.raw`\begin{aligned}R&=R_z(Y)\,R_y(P)\,R_x(R_{oll})\\ P&=-\arcsin R_{31},\quad R_{oll}=\operatorname{atan2}(R_{32},R_{33}),\quad Y=\operatorname{atan2}(R_{21},R_{11})\end{aligned}`,
  jac1: String.raw`\dot{\mathbf p}=J\,\dot{\mathbf q},\qquad J_i=\mathbf z_{i-1}\times(\mathbf p_6-\mathbf p_{i-1})\quad(3\times6)`,
  jac6: String.raw`J=\begin{bmatrix}\mathbf z_{0}\times(\mathbf p_6-\mathbf p_{0}) & \cdots & \mathbf z_{5}\times(\mathbf p_6-\mathbf p_{5})\\ \mathbf z_{0} & \cdots & \mathbf z_{5}\end{bmatrix},\qquad \begin{bmatrix}\mathbf v\\ \boldsymbol\omega\end{bmatrix}=J\,\dot{\mathbf q}`,
  ramp: String.raw`\dot s(\tau)=v\cdot\min\!\left(1,\ \sqrt{\frac{\min(\tau,\,T-\tau)}{T_r}}\right),\qquad T_r=0{,}35\,\mathrm{s}`,
  roterr: String.raw`\begin{aligned}R_e&=R^{*}R^{T},\quad \mathbf v=\tfrac12\begin{bmatrix}R_{e,32}-R_{e,23}\\ R_{e,13}-R_{e,31}\\ R_{e,21}-R_{e,12}\end{bmatrix}\\ \mathbf e_R&=\theta\,\frac{\mathbf v}{\lVert\mathbf v\rVert},\qquad \theta=\operatorname{atan2}\!\left(\lVert\mathbf v\rVert,\ \tfrac{\operatorname{tr}R_e-1}{2}\right)\end{aligned}`,
  ik6: String.raw`\mathbf e=\begin{bmatrix}\mathbf p^{*}-\mathbf p\\ L\,\mathbf e_R\end{bmatrix},\qquad L=100\,\mathrm{mm}`,
  wrap: String.raw`q_i'=q_i-2\pi\,\operatorname{round}\!\left(\frac{q_i-q_i^{\mathrm{ist}}}{2\pi}\right),\qquad q_{min}\le q_i'\le q_{max}`,
  dp: String.raw`C_k(s)=\min_{s'}\Big[\,C_{k-1}(s')+\sum\lvert\Delta\mathbf q\rvert+1000\,[\text{Gelenkfahrt}]+20\,[\text{Umorientieren}]\Big]`,
  reor: String.raw`\mathbf q_{k+1}=\mathrm{IK}\!\left(\mathbf p_A;\ (1-u_k)\,\mathbf q_A+u_k\,\mathbf q_B\right),\quad \mathbf p_A=\text{const},\quad \lVert\Delta\mathbf q\rVert_\infty\le 0{,}25\,\mathrm{rad}`,
  manip: String.raw`w=\sqrt{\det\!\left(J\,J^{T}\right)}`,
  ik_e: String.raw`\mathbf e=\mathbf p^{*}-\mathbf p(\mathbf q)`,
  ik_dq: String.raw`\Delta\mathbf q=J^{T}\!\left(J\,J^{T}+\lambda^{2}I\right)^{-1}\mathbf e,\qquad \mathbf q\leftarrow\operatorname{clamp}(\mathbf q+\Delta\mathbf q,\;\mathbf q_{min},\,\mathbf q_{max})`,
  null: String.raw`\Delta\mathbf q_N=\left(I-J^{+}J\right)\mathbf g,\qquad \mathbf g=-W\mathbf q,\qquad J^{+}=J^{T}\!\left(J\,J^{T}\right)^{-1}`,
  servo: String.raw`a=\operatorname{clamp}(c+d\cdot q,\;0,\;\mathrm{range}),\qquad \mu s=p_{min}+\frac{a}{\mathrm{range}}\,(p_{max}-p_{min})`,
  line: String.raw`\mathbf p(s)=\mathbf A+s\,\frac{\mathbf B-\mathbf A}{\lVert\mathbf B-\mathbf A\rVert},\quad s\in[0,\lVert\mathbf B-\mathbf A\rVert],\quad \Delta s=2\,\mathrm{mm}`,
  step: String.raw`\mathbf q(s_k)=\mathrm{IK}\!\left(\mathbf p(s_k);\;\mathbf q(s_{k-1})\right)+\text{Nullraum-Zug}`,
  ptp: String.raw`\mathbf q(u)=\mathbf q_A+u\,(\mathbf q_B-\mathbf q_A),\qquad u\in[0,1]`,
  time: String.raw`\Delta t=\max\!\left(\frac{\Delta s}{v_{TCP}},\;\frac{\max_i|\Delta q_i|}{\omega_{max}}\right)`,
  reach1: String.raw`r=\lVert\mathbf p(\mathbf q)-\mathbf c\rVert,\qquad \mathbf q\sim\mathcal U\!\left[\mathbf q_{min},\mathbf q_{max}\right]`,
  reach2: String.raw`r_{max}(\varphi,\vartheta)=\max r,\qquad r_{min}(\varphi,\vartheta)=\min r`,
  reach3: String.raw`r_{\text{außen}}=\max r_{max},\qquad r_{\text{innen}}=\min r_{min},\qquad r_{\text{überall}}=\min r_{max}`,
};

const adaptor = liteAdaptor();
RegisterHTMLHandler(adaptor);
const doc = mathjax.document('', { InputJax: new TeX({ packages: AllPackages }), OutputJax: new SVG({ fontCache: 'none' }) });
fs.mkdirSync('assets/tex', { recursive: true });
const PX = 9.0; // Pixel pro ex
for (const [k, tex] of Object.entries(F)) {
  let svg = adaptor.innerHTML(doc.convert(tex, { display: true }));
  svg = svg.replace(/<svg[^>]*>/, (t) => t.replace(/width="([\d.]+)ex"/, (_, v) => `width="${(v * PX).toFixed(1)}"`).replace(/height="([\d.]+)ex"/, (_, v) => `height="${(v * PX).toFixed(1)}"`).replace(/style="[^"]*"/, ''));
  svg = svg.replaceAll('currentColor', '#1f2a3a');
  fs.writeFileSync(`assets/tex/${k}.svg`, svg);
}
console.log(Object.keys(F).length, 'SVGs');
