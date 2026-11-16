// Baut docs/Mathematik.pdf (A4, Skriptstil) aus docs/parts/*.html.
//   cd tools && npm i --no-save katex pagedjs puppeteer-core && node build_pdf.mjs
// Seitenumbruch und Seitenzahlen im Inhaltsverzeichnis übernimmt Paged.js, gedruckt wird mit Chromium.
import fs from 'node:fs';
import path from 'node:path';
import katex from 'katex';
import puppeteer from 'puppeteer-core';
import { figures } from './figures.mjs';

const root = path.resolve(import.meta.dirname, '..');
const kdist = path.join(import.meta.dirname, 'node_modules/katex/dist');
const chrome = process.env.CHROME ||
  fs.globSync(path.join(process.env.HOME, '.cache/ms-playwright/chromium-*/chrome-linux64/chrome'))[0];

const parts = fs.readdirSync(path.join(root, 'docs/parts')).sort()
  .map((f) => fs.readFileSync(path.join(root, 'docs/parts', f), 'utf8')).join('\n');
const tex = (s, d) => katex.renderToString(s.trim(), { displayMode: d, throwOnError: true, strict: false });
const body = parts
  .replace(/<div data-widget="[^"]*"><\/div>/g, '')
  .replace(/\$\$([\s\S]+?)\$\$/g, (_, m) => `<div class="eq">${tex(m, true)}</div>`)
  .replace(/\$([^$\n]+?)\$/g, (_, m) => tex(m, false))
  .replace(/\{\{fig:([\w-]+)\}\}/g, (_, n) => {
    if (!figures[n]) throw new Error('Abbildung fehlt: ' + n);
    return figures[n]();
  });

const heads = [...body.matchAll(/<h([23]) id="([^"]+)">([\s\S]*?)<\/h\1>/g)]
  .map(([, l, id, t]) => ({ l: +l, id, t: t.replace(/<[^>]+>/g, '') }));
const toc = heads.map((x) => `<a class="l${x.l}" href="#${x.id}"><span>${x.t}</span></a>`).join('');

let css = fs.readFileSync(path.join(kdist, 'katex.min.css'), 'utf8');
css = css.replace(/url\(fonts\/([^)]+?\.woff2)\) format\("woff2"\)(,url\([^;}]*?)?(?=[;}])/g, (_, f) =>
  `url(data:font/woff2;base64,${fs.readFileSync(path.join(kdist, 'fonts', f)).toString('base64')}) format("woff2")`);
const style = fs.readFileSync(path.join(root, 'docs/style.css'), 'utf8');
const print = fs.readFileSync(path.join(root, 'docs/print.css'), 'utf8');
const paged = fs.readFileSync(path.join(import.meta.dirname, 'node_modules/pagedjs/dist/paged.polyfill.min.js'), 'utf8');

const title = 'Die Mathematik des Roboterarms';
const html = `<!doctype html><html lang="de"><head><meta charset="utf-8"><title>${title}</title>
<style>${css}</style><style>${style}</style><style>${print}</style>
<script>window.PagedConfig={auto:true,after:()=>{window.__done=true}}</script></head><body>
<section id="titel"><div class="tk">Skript</div><h1>${title}</h1>
<p class="sub">Lineare Algebra, Vorwärts- und inverse Kinematik und Bahnplanung eines 6-Achs-Roboterarms – Schritt für Schritt von Grund auf erklärt</p>
<p class="meta">Begleitdokument zum Programm <i>robotarm-sim</i></p></section>
<nav id="inhalt"><h2 class="plain">Inhaltsverzeichnis</h2>${toc}</nav>
<main>${body}</main><script>document.querySelectorAll('figure.vp svg').forEach((sv)=>{const v=sv.viewBox.baseVal,b=sv.getBBox(),p=6,x0=Math.min(0,b.x-p),y0=Math.min(0,b.y-p),x1=Math.max(v.width,b.x+b.width+p),y1=Math.max(v.height,b.y+b.height+p);sv.setAttribute('viewBox',[x0,y0,x1-x0,y1-y0].join(' '));});</script><script>${paged}</script></body></html>`;

const tmp = path.join(root, 'docs/.pdf-build.html');
fs.writeFileSync(tmp, html);
const br = await puppeteer.launch({ executablePath: chrome, args: ['--no-sandbox'] });
const pg = await br.newPage();
await pg.goto('file://' + tmp, { waitUntil: 'load' });
await pg.waitForFunction('window.__done===true', { timeout: 240000 });
const over = await pg.evaluate(() => [...document.querySelectorAll('.pagedjs_page_content .eq, .pagedjs_page_content .box, .pagedjs_page_content table, .pagedjs_page_content p')].filter((e) => e.scrollWidth > e.clientWidth + 2).map((e) => (e.textContent || '').slice(0, 60)));
if (over.length) console.log('Überlauf:', over);
const clip = await pg.evaluate(() => [...document.querySelectorAll('.pagedjs_page_content figure svg')].flatMap((sv) => {
  const R = sv.getBoundingClientRect(), t = sv.parentElement.querySelector('figcaption b')?.textContent || '?';
  return [...sv.querySelectorAll('text,line,path,polygon,circle')].filter((e) => { const r = e.getBoundingClientRect(); return r.width + r.height > 0 && (r.left < R.left - 1 || r.right > R.right + 1 || r.top < R.top - 1 || r.bottom > R.bottom + 1); }).slice(0, 1).map(() => t);
}));
if (clip.length) console.log('Abgeschnitten:', clip);
await pg.pdf({ path: path.join(root, 'docs/Mathematik.pdf'), preferCSSPageSize: true, printBackground: true });
await br.close();
fs.rmSync(tmp);
console.log('docs/Mathematik.pdf', (fs.statSync(path.join(root, 'docs/Mathematik.pdf')).size / 1024).toFixed(0), 'KiB');
