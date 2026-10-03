import fs from 'node:fs';
import path from 'node:path';
import { pathToFileURL } from 'node:url';

const [archifyRoot, ...files] = process.argv.slice(2);
const { ChromeVisualBrowser, findChrome } = await import(pathToFileURL(path.join(archifyRoot, 'bin/visual-check.mjs')).href);
const browser = new ChromeVisualBrowser(findChrome());
try {
  const session = await browser.sessionPromise;
  const send = (method, params = {}) => browser.cdp.send(method, params, session);
  const run = async expression => {
    const r = await send('Runtime.evaluate', { expression, awaitPromise: true, returnByValue: true });
    if (r.exceptionDetails) throw new Error(r.exceptionDetails.exception?.description || JSON.stringify(r.exceptionDetails));
    return r.result?.value;
  };
  await send('Page.addScriptToEvaluateOnNewDocument', { source: `
    window.exportBlobs = new Map(); window.exportDownloads = [];
    const create = URL.createObjectURL.bind(URL);
    URL.createObjectURL = blob => { const url = create(blob); exportBlobs.set(url, blob); return url; };
    const click = HTMLAnchorElement.prototype.click;
    HTMLAnchorElement.prototype.click = function() {
      if (this.download) { exportDownloads.push({ name: this.download, blob: exportBlobs.get(this.href) }); return; }
      return click.call(this);
    };
  ` });
  await send('Emulation.setDeviceMetricsOverride', { width: 1920, height: 1080, deviceScaleFactor: 1, mobile: false });
  await send('Emulation.setEmulatedMedia', { features: [{ name: 'prefers-reduced-motion', value: 'reduce' }] });
  for (const file of files) {
    const loaded = browser.cdp.waitFor('Page.loadEventFired', session);
    await send('Page.navigate', { url: pathToFileURL(path.resolve(file)).href + '?theme=light' });
    await loaded;
    await run('document.fonts.ready');
    await run('Archify.readerLayout.whenStable()');
    await run('Archify.viewerChromeLayout.whenStable()');
    await run("Archify.exportMenu.run('svg-light')");
    const svg = await run("exportDownloads.at(-1).blob.text()");
    if (!svg || !svg.includes('<svg')) throw new Error('Official SVG export unavailable');
    const out = file.replace(/\.html$/, '.svg');
    fs.writeFileSync(out, svg);
    const audit = await run(`(() => {
      const labels = [...document.querySelectorAll('svg g[data-edge-id][data-detail="context"]')].map(g => {
        const r = g.querySelector('rect'); if (!r) return null;
        const b = r.getBBox(); return {id:g.dataset.edgeId,x:b.x,y:b.y,width:b.width,height:b.height};
      }).filter(Boolean);
      const overlaps=[];
      for(let i=0;i<labels.length;i++)for(let j=i+1;j<labels.length;j++){
        const a=labels[i],b=labels[j];
        if(Math.min(a.x+a.width,b.x+b.width)>Math.max(a.x,b.x)&&Math.min(a.y+a.height,b.y+b.height)>Math.max(a.y,b.y))overlaps.push([a.id,b.id]);
      }
      return {labels, labelBackgroundOverlaps:overlaps, exportReceipt:Object.fromEntries([...document.documentElement.attributes].filter(a=>a.name.startsWith('data-last-export-')).map(a=>[a.name,a.value]))};
    })()`);
    fs.writeFileSync(file.replace(/\.html$/, '.export-audit.json'), JSON.stringify({method:'official Archify.exportMenu.run(svg-light), Chrome CDP; label rectangle geometry only',...audit},null,2)+'\n');
    const shot = await send('Page.captureScreenshot', {format:'png'});
    fs.writeFileSync(file.replace(/\.html$/, '.desktop.png'), Buffer.from(shot.data,'base64'));
    console.log(JSON.stringify({file,svg:out,labelBackgroundOverlaps:audit.labelBackgroundOverlaps}));
    if (audit.labelBackgroundOverlaps.length) throw new Error('Overlapping edge label backgrounds');
  }
} finally { await browser.close(); }
