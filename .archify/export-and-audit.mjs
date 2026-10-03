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
  const template = fs.readFileSync('.archify/portfolio-wrapper.template.html','utf8');
  const payloads = {system:fs.readFileSync(files[0]).toString('base64'),ros:fs.readFileSync(files[1]).toString('base64')};
  const combined = template.replace('PAYLOADS',JSON.stringify(payloads)).replace('DOC_COMMIT',process.env.GITHUB_SHA.slice(0,7));
  fs.mkdirSync('.archify/final-combined',{recursive:true});
  const combinedPath='.archify/final-combined/03_combined_portfolio_architecture.html';
  fs.writeFileSync(combinedPath,combined);
  const loaded = browser.cdp.waitFor('Page.loadEventFired', session);
  await send('Page.navigate',{url:pathToFileURL(path.resolve(combinedPath)).href});
  await loaded;
  await run(`Promise.all([...document.querySelectorAll('iframe')].map(async f=>{
    const started=Date.now();
    while(!f.contentWindow?.Archify?.readerLayout){if(Date.now()-started>15000)throw new Error('Combined iframe timeout');await new Promise(r=>setTimeout(r,20));}
    await f.contentDocument.fonts.ready;await f.contentWindow.Archify.readerLayout.whenStable();
  }))`);
  const states=[];
  for(const mode of ['system','ros','both']){
    await run(`document.querySelector('[data-mode="${mode}"]').click()`);
    await run(`Promise.all([...document.querySelectorAll('iframe')].filter(f=>f.getClientRects().length).map(f=>f.contentWindow.Archify.readerLayout.whenStable()))`);
    await run(`new Promise((resolve,reject)=>{const started=Date.now();function poll(){const visible=[...document.querySelectorAll('iframe')].filter(f=>f.getClientRects().length);if(visible.every(f=>f.getBoundingClientRect().height>=f.contentDocument.documentElement.scrollHeight))return resolve();if(Date.now()-started>15000)return reject(new Error('Combined frame sizing timeout'));requestAnimationFrame(poll)}requestAnimationFrame(poll)})`);
    const state=await run(`({mode:${JSON.stringify(mode)},panels:Object.fromEntries(['system','ros'].map(id=>[id,!document.getElementById(id).hidden])),frames:[...document.querySelectorAll('iframe')].map(f=>({view:f.dataset.view,nodes:f.contentDocument.querySelectorAll('[data-node-id]').length,hasArchify:!!f.contentWindow.Archify,scrollHeight:f.contentDocument.documentElement.scrollHeight,frameHeight:f.getBoundingClientRect().height})),active:[...document.querySelectorAll('[aria-pressed="true"]')].map(b=>b.dataset.mode)})`);
    const expected={system:mode!=='ros',ros:mode!=='system'};
    if(JSON.stringify(state.panels)!==JSON.stringify(expected)||state.active[0]!==mode||state.frames.some(f=>!f.hasArchify||!f.nodes||(state.panels[f.view]&&f.frameHeight<f.scrollHeight)))throw new Error('Combined tab/frame validation failed '+JSON.stringify(state));
    states.push(state);
    const shot=await send('Page.captureScreenshot',{format:'png'});
    fs.writeFileSync('.archify/final-combined/combined-'+mode+'.desktop.png',Buffer.from(shot.data,'base64'));
  }
  fs.writeFileSync('.archify/final-combined/combined-browser-audit.json',JSON.stringify({status:'pass',method:'Chrome CDP, srcdoc loads, tab visibility, frame content/height; viewport 1920x1080',states},null,2)+'\n');
} finally { await browser.close(); }
