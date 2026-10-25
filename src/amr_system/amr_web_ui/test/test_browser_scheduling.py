"""Run the app against a small DOM/canvas double; no browser or ROS process."""
from pathlib import Path
import shutil
import subprocess

import pytest


def test_browser_draw_invalidation_buckets_and_visibility():
    node = shutil.which('node')
    if not node:
        pytest.skip('Node is needed for the DOM/canvas unit test')
    app = Path(__file__).resolve().parents[1] / 'web' / 'app.js'
    script = r'''
const fs = require('fs'), vm = require('vm'), assert = require('assert');
const elements = new Map(), events = {}, frames = new Map(), timers = new Map();
let serial = 0, clears = 0, fills = [], rects = [], resize;
const context = new Proxy({
  clearRect(){clears++;},
  fillRect(){throw Error('Scan points must use bucket paths');},
  beginPath(){rects=[];},
  rect(...args){rects.push(args);},
  fill(){fills.push({color:this.fillStyle, rects:[...rects]});},
}, {get(target, key){return key in target ? target[key] : ()=>{};}});
function element(id) {
  if (!elements.has(id)) elements.set(id, {
    textContent:'', hidden:false, disabled:false, value:0, style:{}, handlers:{}, dataset:{},
    clientWidth:640, clientHeight:480, width:640, height:480,
    classList:{add(){},remove(){},toggle(){}},
    addEventListener(event, fn){this.handlers[event]=fn;},
    setAttribute(){}, getContext(){return context;},
    getBoundingClientRect(){return {width:640,height:480,left:0,top:0};},
    querySelector(selector){return element(id+selector);},
    setPointerCapture(){}, replaceChildren(){}, focus(){},
  });
  return elements.get(id);
}
const scan = element('scan-layer'); scan.dataset.layer='scan'; scan.checked=true;
const pan = element('pan-mode'); pan.dataset.mode='pan';
global.document = {
  hidden:false, getElementById:element, querySelector:element,
  querySelectorAll(selector){return selector==='[data-layer]'?[scan]:selector==='[data-mode]'?[pan]:[];},
  addEventListener(event, fn){events[event]=fn;},
};
global.window = {AMRGeometry:{
  worldToScreen(p){return {x:320+p.x*16,y:240-p.y*16};},
  screenToWorld(p){return {x:(p.x-320)/16,y:(240-p.y)/16};},
}, addEventListener(event, fn){events['window:'+event]=fn;}};
global.ResizeObserver = class {constructor(fn){resize=fn;} observe(){}};
global.devicePixelRatio=1;
global.requestAnimationFrame = fn=>{frames.set(++serial,fn);return serial;};
global.cancelAnimationFrame = id=>frames.delete(id);
global.setTimeout = (fn,delay)=>{assert.equal(delay,200);timers.set(++serial,fn);return serial;};
global.clearTimeout = id=>timers.delete(id);
global.AbortSignal = {timeout(){return {};}};
let state = {preview:true, mode:'AUTO', navigation:'IDLE', layers:{},
  scan:{age:0, points:[[0,0,0],[1,1,0],[2,2,31],[3,3,null]]}};
const requests=[];
global.fetch = async url=>{
  requests.push(url);
  if(url==='/api/state')return {ok:true,json:async()=>JSON.parse(JSON.stringify(state))};
  return {ok:false};
};
async function settle(){for(let i=0;i<12;i++)await Promise.resolve();}
function draw(){
  const queued=[...frames.values()];frames.clear();queued.forEach(fn=>fn());
}
(async()=>{
  vm.runInThisContext(fs.readFileSync(process.argv[1],'utf8'));
  await settle();
  assert.equal(requests.filter(x=>x==='/api/state').length,1);
  assert.equal(frames.size,1); draw();
  assert.equal(clears,1);assert.equal(frames.size,0,'Drawing must not reschedule itself');
  assert.equal(fills.length,3,'Two intensity buckets and the missing-intensity color');
  assert.deepEqual(fills.map(x=>x.rects.length),[2,1,1]);
  assert.deepEqual(fills.map(x=>x.color),['hsl(240,78%,48%)','hsl(0,78%,48%)','#009bb9']);
  resize();resize();assert.equal(frames.size,1);draw();assert.equal(clears,2);
  pan.handlers.click();draw();
  element('map').handlers.pointerdown({button:0,pointerId:1,clientX:20,clientY:30});
  element('map').handlers.pointermove({pointerId:1,clientX:40,clientY:50});
  assert.equal(frames.size,1);draw();
  element('map').handlers.pointerup({pointerId:1});draw();
  assert.equal(frames.size,0);
  const pending=[...timers.values()][0];
  document.hidden=true;events.visibilitychange();
  assert.equal(timers.size,0);assert.equal(frames.size,0);
  await pending();await settle();
  assert.equal(requests.filter(x=>x==='/api/state').length,1,'Hidden tab must not poll');
  resize();assert.equal(frames.size,0);
  document.hidden=false;events.visibilitychange();await settle();
  assert.equal(requests.filter(x=>x==='/api/state').length,2,'Visibility resumes immediately');
  assert.equal(timers.size,1);assert.equal(frames.size,1);draw();
  assert.equal(frames.size,0);
  const next=[...timers.values()][0];await next();await settle();
  assert.equal(requests.filter(x=>x==='/api/state').length,3);
  assert.equal(frames.size,1,'A new state must invalidate the canvas');
  assert(requests.every(url=>url==='/api/state'||url==='/api/floors'));
})().catch(error=>{console.error(error);process.exitCode=1;});
'''
    result = subprocess.run([node, '-e', script, str(app)], capture_output=True, text=True, timeout=10)
    assert result.returncode == 0, result.stdout + result.stderr
