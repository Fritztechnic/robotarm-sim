// Baut docs/Mathematik.html: eine einzige, offline lesbare Datei.
//   cd tools && npm i --no-save katex && node build_docs.mjs
// Quelle: docs/parts/*.html (in Dateinamen-Reihenfolge). In den Texten gilt
//   $…$ inline, $$…$$ abgesetzt (KaTeX, zur Bauzeit gerendert) und {{fig:name}} für Abbildungen aus figures.mjs.
import fs from 'node:fs';
import path from 'node:path';
import katex from 'katex';
import { figures } from './figures.mjs';

const root = path.resolve(import.meta.dirname, '..');
const kdist = path.join(import.meta.dirname, 'node_modules/katex/dist');

const parts = fs.readdirSync(path.join(root, 'docs/parts')).sort()
  .map((f) => fs.readFileSync(path.join(root, 'docs/parts', f), 'utf8')).join('\n');

const tex = (s, display) => katex.renderToString(s.trim(), { displayMode: display, throwOnError: true, strict: false, trust: false });
// Abgesetzte Formeln zuerst; Dollarzeichen in Code-Blöcken gibt es im Text nicht.
let body = parts
  .replace(/\$\$([\s\S]+?)\$\$/g, (_, m) => `<div class="eq">${tex(m, true)}</div>`)
  .replace(/\$([^$\n]+?)\$/g, (_, m) => tex(m, false))
  .replace(/\{\{fig:([\w-]+)\}\}/g, (_, n) => {
    if (!figures[n]) throw new Error('Abbildung fehlt: ' + n);
    return figures[n]();
  });

// Inhaltsverzeichnis aus <h2 id> / <h3 id>
const toc = [...body.matchAll(/<h([23]) id="([^"]+)">([\s\S]*?)<\/h\1>/g)]
  .map(([, l, id, t]) => `<li class="l${l}"><a href="#${id}">${t.replace(/<[^>]+>/g, '')}</a></li>`).join('\n');

// KaTeX-CSS mit eingebetteten Schriften (nur woff2)
let css = fs.readFileSync(path.join(kdist, 'katex.min.css'), 'utf8');
css = css.replace(/url\(fonts\/([^)]+?\.woff2)\) format\("woff2"\)(,url\([^;}]*?)?(?=[;}])/g, (_, f) => {
  const b = fs.readFileSync(path.join(kdist, 'fonts', f)).toString('base64');
  return `url(data:font/woff2;base64,${b}) format("woff2")`;
});

const style = fs.readFileSync(path.join(root, 'docs/style.css'), 'utf8');
const script = fs.readFileSync(path.join(root, 'docs/widgets.js'), 'utf8');

const html = `<!doctype html>
<html lang="de"><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>Die Mathematik des Roboterarms – von Null an erklärt</title>
<style>${css}</style><style>${style}</style></head>
<body><nav id="toc"><div class="brand">ARM-SIM<small>Mathematik</small></div><ol>${toc}</ol></nav>
<main>${body}</main><script>${script}</script></body></html>`;

fs.mkdirSync(path.join(root, 'docs'), { recursive: true });
fs.writeFileSync(path.join(root, 'docs/Mathematik.html'), html);
console.log('docs/Mathematik.html', (html.length / 1024).toFixed(0), 'KiB');
