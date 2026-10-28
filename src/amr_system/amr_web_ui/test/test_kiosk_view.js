'use strict';
const assert = require('node:assert/strict');
const fs = require('node:fs');
const path = require('node:path');
const vm = require('node:vm');
const {test} = require('node:test');
const source = fs.readFileSync(path.join(__dirname, '../web/kiosk.js'), 'utf8');
const sandbox = {window:{}, console, Date, Math, Set, performance};
vm.runInNewContext(source, sandbox);
const K = sandbox.window.AMRKiosk;
const copy = x => JSON.parse(JSON.stringify(x));
const ready = overrides => ({mode:'AUTO', map_version:101, action_ready:true, floor_action_ready:true,
  localization:{ready:true, age:0}, floor:{floor:'4F', confidence:'CONFIRMED', zone:1},
  estop:{required:true, state:'READY'}, goal:null, ...overrides});
const goal = (status='ACCEPTED', extra={}) => ({status, request_id:'kiosk_goal_123', floor:'4F', room:'402', ...extra});
const own = {ownedRequestIds:['kiosk_goal_123']};

test('connection overlay wins all overlaps, including initial load and stale state', () => {
  assert.equal(K.selectScreen(null), 'connection');
  const all = ready({estop:{state:'ESTOP'}, mode:'MANUAL', goal:goal('ACCEPTED',{kind:'floor_goal'})});
  assert.equal(K.selectScreen(all,{fetchFailed:true}), 'connection');
  assert.equal(K.selectScreen(all,{stateAgeMs:3001}), 'connection');
  assert.equal(K.selectScreen(ready(),{stateAgeMs:3000}), 'idle');
});
test('E-stop overrides manual, active floor mission, delay, and terminal result', () => {
  for (const g of [goal(), goal('ACCEPTED',{kind:'floor_goal'}), goal('SUCCEEDED')]) {
    assert.equal(K.selectScreen(ready({goal:g, mode:'MANUAL', estop:{required:true,error:'정지'}}), {...own,startPending:true}), 'estop');
    assert.equal(K.selectScreen(ready({goal:g, estop:{required:false,state:'ESTOP'}}), own), 'estop');
  }
  assert.equal(K.selectScreen(ready({estop:{required:false,error:'ignored'}})), 'idle');
});
test('manual overrides floor mission and local delay', () => {
  assert.equal(K.selectScreen(ready({mode:'MANUAL', goal:goal('ACCEPTED',{kind:'floor_goal'})}), {startPending:true}), 'manual');
});
test('active floor missions use elevator view; terminal floor missions use result', () => {
  assert.equal(K.selectScreen(ready({goal:goal('SENDING',{kind:'floor_goal'})})), 'transit');
  assert.equal(K.selectScreen(ready({goal:goal(), floor_mission:{phase:'RIDING'}})), 'transit');
  assert.equal(K.selectScreen(ready({goal:goal('SUCCEEDED',{kind:'floor_goal'}), floor_mission:{phase:'RIDING'}}),own), 'arrived');
  assert.equal(K.selectScreen(ready({floor_mission:{phase:'RIDING'}})), 'localizing');
});
test('all active goal statuses and unsent start delay use delivery view', () => {
  for (const status of ['SENDING','ACCEPTED','CANCELING']) assert.equal(K.selectScreen(ready({goal:goal(status)})), 'driving');
  assert.equal(K.selectScreen(ready(),{startPending:true}), 'driving');
  assert.equal(K.selectScreen(ready({localization:{ready:false},goal:goal()})), 'driving');
});
test('only owned, unacknowledged room results appear, before readiness', () => {
  for (const status of ['SUCCEEDED','FAILED','REJECTED','CANCELED']) {
    assert.equal(K.selectScreen(ready({goal:goal(status),localization:{ready:false}}),own), 'arrived');
    assert.equal(K.selectScreen(ready({goal:goal(status)})), 'idle');
    assert.equal(K.selectScreen(ready({goal:goal(status),acknowledged_request:'kiosk_goal_123'}),own), 'idle');
    assert.equal(K.selectScreen(ready({goal:goal(status,{room:''})}),own), 'idle');
  }
});
test('every readiness gate locks picker; confirmed, beacon and assumed floors work', () => {
  for (const change of [{localization:null},{localization:{ready:false}}, {localization:{ready:true,age:2}},
    {action_ready:false}, {floor:null}, {floor:{floor:'4F',confidence:'ASSUMED',zone:2}},
    {floor:{floor:'4F',confidence:'TRANSIT',zone:2}}, {floor:{floor:'4F',confidence:'UNKNOWN',zone:0}},
    {floor:{floor:null,confidence:'TRANSIT'}},{tf_error:'missing'}, {localization_reset_pending:true},
    {pending_request:{request_id:'admin_123'}}]) assert.equal(K.selectScreen(ready(change)), 'localizing');
  assert.equal(K.selectScreen(ready()), 'idle');
  assert.equal(K.selectScreen(ready({floor:{floor:'3F',confidence:'BEACON'}})), 'idle');
  // No hall beacon since startup: same-floor rooms open, other floors wait for confirmation.
  assert.equal(K.selectScreen(ready({floor:{floor:'4F',confidence:'ASSUMED',zone:1}})), 'idle');
  assert.equal(K.floorConfirmed(ready({floor:{floor:'4F',confidence:'ASSUMED',zone:1}})), false);
  assert.equal(K.floorConfirmed(ready()), true);
});
test('every transit.js phase is mapped, including recovery branches and terminal phases', () => {
  const expected = {NAV_TO_HALL:1,CALL_ELEVATOR:2,WAIT_DOOR_OPEN:2,BOARDING:3,BACKING_OUT:3,
    IN_CAR_ALIGN:3,SELECT_FLOOR:3,RIDING:4,SWITCH_MAP:4,ALIGHTING:5,RETURN_TO_CAR:5,
    RELOCALIZE:5,NAV_TO_GOAL:6,SUCCEEDED:6,FAILED_SAFE:0,CANCELED:0,IDLE:0};
  const transit = fs.readFileSync(path.join(__dirname, '../web/transit.js'),'utf8');
  const labels = transit.match(/window\.AMRMissionLabels\s*=\s*\{([\s\S]*?)\};/)[1];
  const phases = [...labels.matchAll(/([A-Z_]+):/g)].map(m=>m[1]);
  assert.deepEqual(phases.sort(), Object.keys(expected).sort(), 'Update coverage when transit phases change');
  for (const [phase, step] of Object.entries(expected)) assert.equal(K.phaseToStep(phase),step,phase);
  assert.equal(K.phaseToStep('unknown'),0);
});
test('progress clamps, never decreases, and resets for each new request', () => {
  let p = K.updateProgress(null,'a',10,0);
  assert.equal(p.percent,0);
  p = K.updateProgress(p,'a',5,5000); assert.equal(p.percent,50);
  p = K.updateProgress(p,'a',8,6000); assert.equal(p.percent,50);
  p = K.updateProgress(p,'a',0,7000); assert.equal(p.percent,100);
  p = K.updateProgress(p,'a',null,8000); assert.equal(p.percent,100); assert.equal(p.eta,'');
  p = K.updateProgress(p,'b',20,9000); assert.equal(p.percent,0); assert.equal(p.initial,20);
  assert.equal(K.updateProgress(p,'b',-1,10000).percent,0);
  assert.equal(K.updateProgress(null,'c',0,0).percent,0);
});
test('ETA uses observed recent speed, falls back to 0.3, and hides unknown values', () => {
  let p = K.updateProgress(null,'a',9,0); assert.equal(p.eta,'약 30초');
  p = K.updateProgress(p,'a',6,1000); assert.equal(p.eta,'약 2초');
  p = K.updateProgress(p,'a',3,12000); assert.equal(p.eta,'약 10초');
  assert.equal(p.samples.length,1,'old samples must leave the ten-second window');
  for (const v of [null,undefined,NaN,Infinity,-1]) assert.equal(K.formatETA(v),'');
  for (const [v, text] of [[0,'약 0초'],[1.2,'약 2초'],[59,'약 59초'],[59.9,'약 1분'],[60,'약 1분'],[61,'약 2분'],[120,'약 2분']]) assert.equal(K.formatETA(v),text);
});
test('room numbers and door suffixes preserve supplied spelling and order', () => {
  assert.deepEqual(copy(K.splitRoomId('413-앞문')),{number:'413',suffix:'앞문'});
  assert.deepEqual(copy(K.splitRoomId('413-뒷문')),{number:'413',suffix:'뒷문'});
  assert.deepEqual(copy(K.splitRoomId('401')),{number:'401',suffix:''});
  assert.equal(K.roomLabel('413-앞문'),'413호 앞문');
  assert.equal(K.roomLabel('401'),'401호');
  assert.deepEqual(copy(K.splitRoomId('413-앞문-가')),{number:'413',suffix:'앞문-가'});
});

function flowFixture() {
  const posts=[], timers=new Map(), errors=[], owned=[];
  let s=ready(), enabled=true, time=0, number=0, postImpl=async()=>({});
  const flow=K.createDeliveryFlow({
    post:async(kind,payload)=>{ posts.push({kind,payload}); return postImpl(kind,payload); },
    changed(){}, error:e=>errors.push(e), remember:id=>owned.push(id),
    ready:()=>enabled, now:()=>time, state:()=>s,
    setTimer(fn,ms){assert.equal(ms,3000);timers.set(++number,fn);return number;}, clearTimer:id=>timers.delete(id),
  });
  return {flow,posts,timers,errors,owned, setState:v=>{s=v;}, disable:()=>{enabled=false;},
    setPost:fn=>{postImpl=fn;}, setTime:v=>{time=v;},
    async fire(){const [id,fn]=timers.entries().next().value;timers.delete(id);time=3000;await fn();}};
}
const roomPayload={floor:'3F',room:'301',x:1.123456789,y:-2.5,yaw:Math.PI/2,map_version:17};
test('departure waits three seconds; payload copies exact room coordinates and new ID', async()=>{
  const f=flowFixture(); f.flow.start(roomPayload);
  assert.equal(f.posts.length,0); assert.equal(f.flow.pending.phase,'delay');
  await f.fire(); assert.equal(f.posts.length,1); assert.equal(f.posts[0].kind,'goal');
  const {request_id,...sent}=f.posts[0].payload;
  assert.match(request_id,/^[A-Za-z0-9_-]{8,80}$/); assert.deepEqual(sent,roomPayload);
  assert.deepEqual(f.owned,[request_id]);
});
test('stop before send removes timer and sends nothing',()=>{
  const f=flowFixture();f.flow.start(roomPayload);f.flow.stop();
  assert.equal(f.timers.size,0);assert.equal(f.flow.pending,null);assert.equal(f.posts.length,0);
});
test('readiness loss during delay prevents sending, including stale timer callbacks',async()=>{
  const f=flowFixture();f.flow.start(roomPayload);const late=[...f.timers.values()][0];
  f.disable();f.flow.reconcile(ready());await late();assert.equal(f.posts.length,0);assert.equal(f.flow.pending,null);
  assert.deepEqual(f.errors,['출발할 수 없습니다. 상태를 확인한 뒤 다시 선택하세요.']);
});
test('POST error releases local start state; no automatic retry; ownership retained',async()=>{
  const f=flowFixture();f.setPost(async()=>{throw Error('서버 오류');});f.flow.start(roomPayload);await f.fire();
  assert.equal(f.flow.pending,null);assert.deepEqual(f.errors,['서버 오류']);assert.equal(f.posts.length,1);assert.equal(f.owned.length,1);
});
test('stop during sending waits for ACCEPTED, cancels once, and uses a new request ID',async()=>{
  const f=flowFixture();let resolve; f.setPost(kind=>kind==='goal'?new Promise(r=>{resolve=r;}):Promise.resolve({}));
  f.flow.start(roomPayload);const running=f.fire();const id=f.posts[0].payload.request_id;
  f.flow.stop(); assert.equal(f.posts.length,1);
  f.setState(ready({goal:goal('SENDING',{request_id:id})}));resolve({});await running;
  assert.equal(f.posts.length,1,'server only accepts cancel after goal acceptance');
  f.setState(ready({goal:goal('ACCEPTED',{request_id:id})}));f.flow.reconcile(ready({goal:goal('ACCEPTED',{request_id:id})}));
  await Promise.resolve();assert.equal(f.posts.length,2);assert.equal(f.posts[1].kind,'cancel');
  assert.notEqual(f.posts[1].payload.request_id,id);
});
test('any active goal stops with a single tap, even one this kiosk did not send',async()=>{
  // Ownership is lost when Chromium restarts mid-delivery; the stop button must still work.
  const f=flowFixture();f.setState(ready({goal:goal()}));f.flow.stop();
  assert.equal(f.posts.length,1);assert.equal(f.posts[0].kind,'cancel');
  const idle=flowFixture();idle.setState(ready({goal:goal('SUCCEEDED')}));idle.flow.stop();assert.equal(idle.posts.length,0);
});
test('request with no observed result eventually releases local pending state',async()=>{
  const f=flowFixture();f.flow.start(roomPayload);await f.fire();f.setTime(14000);f.flow.reconcile(ready());
  assert.equal(f.flow.pending,null);assert.deepEqual(f.errors,['연결 대기']);assert.equal(f.posts.length,1);
});

// Small DOM/HTTP double catches startup, wiring, caching and admin auth regressions.
// No sockets, Chromium, X11, ROS, or physical device access.
function browserFixture() {
  const elements=new Map(), handlers={}, timers=new Map(), intervals=[], frames=new Map(), requests=[];
  let serial=0, time=0, draws=0, storage='[]', state=ready({kiosk:{pin_required:true}});
  const contexts=new Proxy({clearRect(){draws++;},measureText(text){return {width:text.length*10};}}, {get:(obj,k)=>obj[k] || (()=>{})});
  class Element {
    constructor(id,tag='div',attributes='') {
      this.id=id;this.tag=tag;this.dataset={};this.hidden=false;this.disabled=false;this.style={};
      this.textContent='';this.className='';this.listeners={};this.children=[];this.html='';this.width=0;this.height=0;
      for(const m of attributes.matchAll(/data-([\w-]+)="([^"]*)"/g)) this.dataset[m[1].replace(/-([a-z])/g,(_,c)=>c.toUpperCase())]=m[2];
    }
    get innerHTML(){return this.html;}
    set innerHTML(value){
      this.htmlWrites=(this.htmlWrites || 0)+1;
      this.children.forEach(id=>elements.delete(id));this.children=[];this.html=value;
      for(const m of value.matchAll(/<(\w+)\b([^>]*)>/g)) {
        const id=m[2].match(/\bid="([^"]*)"/)?.[1] || (m[1]==='button'?`button-${++serial}`:null);
        if(!id)continue;
        const element=new Element(id,m[1],m[2]);elements.set(id,element);this.children.push(id);
      }
    }
    addEventListener(name,fn){this.listeners[name]=fn;}
    closest(selector){return selector==='button' && this.tag==='button'?this:null;}
    getBoundingClientRect(){return {width:600,height:400,left:0,top:0};}
    getContext(){return contexts;}
    setPointerCapture(){}
  }
  elements.set('app',new Element('app'));
  const floors=[{name:'4F',level:4,selectable:true,map:{width:100,height:100,resolution:0.1,
    origin:[0,0,0],render_origin:[0,0,0],transform_valid:true,version:1},rooms:[{id:'402',x:2,y:3,yaw:0}]}];
  floors.push({...copy(floors[0]), name:'3F',level:3,map:{...copy(floors[0].map),version:17},rooms:[{id:'301',x:4,y:5,yaw:0.7}]});
  let failState=false, rejectAdmin=false, patrolError=null;
  const env={window:{}, document:{hidden:false,getElementById:id=>elements.get(id)||null,
    addEventListener:(name,fn)=>{handlers[name]=fn;},
    querySelectorAll(selector){const attr=selector.slice(6,-1).replace(/-([a-z])/g,(_,c)=>c.toUpperCase());return [...elements.values()].filter(e=>e.dataset[attr]!==undefined);}},
    console, Date, Math, Set, Map, performance:{now:()=>time},
    sessionStorage:{getItem:()=>storage,setItem:(key,v)=>{storage=v;}},
    setTimeout:(fn,ms)=>{timers.set(++serial,{fn,ms});return serial;},clearTimeout:id=>timers.delete(id),
    setInterval:fn=>{intervals.push(fn);},requestAnimationFrame:fn=>{frames.set(++serial,fn);return serial;},cancelAnimationFrame:id=>frames.delete(id),
    ResizeObserver:class{observe(){}disconnect(){}},AbortController,AbortSignal,
    createImageBitmap:async()=>({close(){}}),
    fetch:async(url,options={})=>{
      requests.push({url,options});
      if(url==='/api/kiosk/state'){if(failState)throw Error('offline');return {ok:true,json:async()=>copy(state)};}
      if(url==='/api/floors')return {ok:true,json:async()=>({floors,current_floor:'4F'})};
      if(url.includes('/map.png'))return {ok:true,headers:{get:()=> '1'},blob:async()=>({})};
      if(url==='/api/acknowledge'){state.acknowledged_request=JSON.parse(options.body).request_id;return {ok:true,json:async()=>({})};}
      if(url==='/api/kiosk/unlock')return {ok:true,json:async()=>({token:'memory-only',expires_in:600})};
      if(url==='/api/patrol/start' || url==='/api/patrol/stop') {
        if(patrolError)return {ok:false,status:patrolError.status,json:async()=>({error:patrolError.detail})};
        const body=JSON.parse(options.body), active=url.endsWith('/start');
        state.patrol={...state.patrol,active,route:body.route || state.patrol?.route,phase:active?'NAV':'ABORTED',index:1,total:12,age:0};
        state.notice={request_id:body.request_id,status:'SUCCEEDED',detail:''};
        return {ok:true,status:202,json:async()=>({request_id:body.request_id})};
      }
      if(url==='/api/initialpose' && rejectAdmin)return {ok:false,status:403,json:async()=>({error:'토큰 만료'})};
      return {ok:true,json:async()=>({})};
    },
  };
  vm.runInNewContext(fs.readFileSync(path.join(__dirname,'../web/geometry.js'),'utf8'),env);
  vm.runInNewContext(fs.readFileSync(path.join(__dirname,'../web/battery-profile.js'),'utf8'),env);
  vm.runInNewContext(source,env);
  const settle=async()=>{for(let i=0;i<30;i++)await Promise.resolve();};
  return {env,elements,requests,timers,frames,handlers,intervals,settle,
    get draws(){return draws;}, get storage(){return storage;},
    setState:s=>{state=s;},fail:()=>{failState=true;},rejectAdmin:()=>{rejectAdmin=true;},
    rejectPatrol:(status,detail)=>{patrolError={status,detail};},
    setTime:n=>{time=n;},
    draw(){const queued=[...frames.values()];frames.clear();queued.forEach(fn=>fn());},
    async poll(){const entry=[...timers.entries()].find(([,v])=>v.ms===500);assert(entry,'scheduled polling');timers.delete(entry[0]);await entry[1].fn();await settle();},
    async click(data,value){const b=[...elements.values()].find(e=>e.dataset[data]===value);assert(b,`button ${data}=${value}`);assert(!b.disabled);await elements.get('app').listeners.click({target:b});await settle();},
  };
}
test('DOM startup polls only kiosk state; unchanged pose/path/size does not redraw; hidden polling stops',async()=>{
  const b=browserFixture();await b.settle();b.draw();const draws=b.draws;
  assert(draws>0);assert.equal(b.elements.get('status-label').textContent,'정상');
  await b.poll();b.draw();assert.equal(b.draws,draws);
  assert(b.requests.every(r=>r.url!=='/api/state'));
  b.env.document.hidden=true;b.handlers.visibilitychange();await b.settle();
  assert.equal([...b.timers.values()].filter(t=>t.ms===500).length,0);
  const count=b.requests.length;await b.env.window.AMRKiosk.selectScreen(ready());assert.equal(b.requests.length,count);
  b.env.document.hidden=false;b.handlers.visibilitychange();await b.settle();
  assert(b.requests.length>count);
  b.fail();await b.poll();assert.equal(b.elements.get('connection').hidden,false);
});
test('PIN unlock and initialpose use kiosk headers; token stays in memory and 403 exits admin',async()=>{
  const b=browserFixture();await b.settle();b.draw();
  await b.click('action','admin');await b.click('pin','1');await b.click('pin','2');await b.click('action','unlock');b.draw();
  assert.equal(b.elements.get('app').className,'app admin');
  const canvas=b.elements.get('map');canvas.listeners.pointerdown({button:0,isPrimary:true,pointerId:1,clientX:300,clientY:200});
  canvas.listeners.pointermove({pointerId:1,clientX:350,clientY:200});b.draw();
  b.rejectAdmin();await b.click('action','apply-pose');
  const requests=b.requests.filter(r=>r.options.method==='POST');assert.equal(requests.length,2);
  for(const r of requests){assert.equal(r.options.headers['X-AMR-Client'],'kiosk');assert.equal(r.options.headers['X-AMR-Request'],'1');assert.equal(r.options.headers['Content-Type'],'application/json');}
  assert.equal(JSON.parse(requests[0].options.body).pin,'12');
  assert.equal(requests[1].options.headers['X-AMR-Kiosk-Token'],'memory-only');
  assert.equal(JSON.parse(requests[1].options.body).map_version,101,'picked pose uses live version, not floor image version 1');
  assert.match(JSON.parse(requests[1].options.body).request_id,/^[A-Za-z0-9_-]{8,80}$/);
  assert.equal(b.storage,'[]');assert.equal(b.elements.get('app').className,'app');assert.equal(b.elements.get('toast').textContent,'토큰 만료');
});
test('admin close-kiosk asks the server with the admin token',async()=>{
  const b=browserFixture();await b.settle();b.draw();
  await b.click('action','admin');await b.click('pin','1');await b.click('action','unlock');
  await b.click('action','close-kiosk');
  const req=b.requests.find(r=>r.url==='/api/kiosk/exit');assert(req,'kiosk exit request');
  assert.equal(req.options.method,'POST');assert.equal(req.options.headers['X-AMR-Client'],'kiosk');
  assert.equal(req.options.headers['X-AMR-Kiosk-Token'],'memory-only');assert.equal(req.options.body,'{}');
});
test('PIN-free admin warns, uses saved pose, and exits after five idle minutes',async()=>{
  const b=browserFixture();b.setState(ready({kiosk:{pin_required:false},localization:{ready:true,age:0,last_pose:{x:2,y:3,yaw:1,saved_at:1700000000}}}));
  await b.settle();await b.poll();b.draw();await b.click('action','admin');
  assert.equal(b.elements.get('pin-warning').hidden,false);assert.match(b.elements.get('last-pose').textContent,/4층 .* 저장/);
  await b.click('action','apply-last');
  const req=b.requests.find(r=>r.url==='/api/initialpose');const body=JSON.parse(req.options.body);
  assert.deepEqual({x:body.x,y:body.y,yaw:body.yaw,map_version:body.map_version},{x:2,y:3,yaw:1,map_version:101});
  assert.equal(req.options.headers['X-AMR-Kiosk-Token'],undefined);
  b.setTime(300001);b.intervals.forEach(fn=>fn());assert.equal(b.elements.get('app').className,'app');
});
test('room selection, delayed departure, persistent receipt and acknowledgement are wired',async()=>{
  const b=browserFixture();await b.settle();b.draw();
  await b.click('room','402');assert.equal(b.elements.get('sheet').hidden,false);
  await b.click('action','depart');assert.equal(b.elements.get('status-label').textContent,'배송 중');
  assert.equal(b.elements.get('progress-value').textContent,0);assert.equal(b.elements.get('eta-box').hidden,true);
  assert.equal(b.requests.filter(r=>r.options.method==='POST').length,0);
  const [timerId,timer]=[...b.timers].find(([,t])=>t.ms===3000);b.timers.delete(timerId);await timer.fn();await b.settle();
  const sent=b.requests.find(r=>r.url==='/api/goal'), body=JSON.parse(sent.options.body);
  assert.deepEqual({x:body.x,y:body.y,yaw:body.yaw,map_version:body.map_version,floor:body.floor,room:body.room},
    {x:2,y:3,yaw:0,map_version:101,floor:'4F',room:'402'});
  assert(JSON.parse(b.storage).includes(body.request_id));
  b.setState(ready({goal:goal('SUCCEEDED',{request_id:body.request_id})}));await b.poll();
  assert.equal(b.elements.get('result-title').textContent,'402호 도착');
  b.setTime(2000);b.intervals.forEach(fn=>fn());assert.equal(b.elements.get('ack').textContent,'수령 확인');
  await b.click('action','ack');assert.equal(b.elements.get('status-label').textContent,'정상');
  const ack=b.requests.find(r=>r.url==='/api/acknowledge');assert.equal(JSON.parse(ack.options.body).request_id,body.request_id);
});
test('battery uses fresh BMS or existing dashboard voltage scale and hides stale data',async()=>{
  const b=browserFixture();await b.settle();
  b.setState(ready({telemetry:{battery:{soc:82,age:0}}}));await b.poll();assert.equal(b.elements.get('battery-percent').textContent,'82%');
  b.setState(ready({telemetry:{motor_left_power:{voltage:26.5,age:0},motor_right_power:{voltage:26.5,age:0}}}));await b.poll();
  assert.equal(b.elements.get('battery-percent').textContent,'50%');assert.equal(b.elements.get('battery').hidden,false);
  b.setState(ready({telemetry:{battery:{soc:82,age:3}}}));await b.poll();assert.equal(b.elements.get('battery').hidden,true);
});

test('progress ignores zero/tiny initial feedback and calibrates against first three seconds maximum',()=>{
  let p=null;
  for(const [t,r] of [[0,0],[500,0.01],[1000,0.05]]) {
    p=K.updateProgress(p,'new',r,t);
    assert.equal(p.percent,0);assert.equal(p.initial,null);assert.equal(p.eta,'');assert.equal(p.samples.length,0);
  }
  p=K.updateProgress(p,'new',1,1500);assert.equal(p.initial,1);assert.equal(p.percent,0);
  p=K.updateProgress(p,'new',10,2000);assert.equal(p.initial,10);assert.equal(p.percent,0);
  p=K.updateProgress(p,'new',8,3000);assert(Math.abs(p.percent-20)<1e-8);
  p=K.updateProgress(p,'new',20,4500);assert.equal(p.initial,20);assert.equal(p.percent,0);
  p=K.updateProgress(p,'new',25,4501);assert.equal(p.initial,20);assert.equal(p.percent,0);
  p=K.updateProgress(p,'new',15,4600);assert.equal(p.percent,25);
  p=K.updateProgress(p,'new',18,4700);assert.equal(p.percent,25);
  p=K.updateProgress(p,'new',10,5000);assert.equal(p.percent,50);
  p=K.updateProgress(p,'new',0,6000);assert.equal(p.percent,100);
  p=K.updateProgress(p,'next',0.01,7000);assert.equal(p.initial,null);assert.equal(p.percent,0);
});
test('AUTO and E-stop readiness gates apply independently of screen selection',()=>{
  for(const mode of ['MANUAL','UNKNOWN',null,undefined]) assert.equal(K.readyForGoal(ready({mode})),false);
  assert.equal(K.readyForGoal(ready({estop:{required:true,error:'정지'}})),false);
  assert.equal(K.readyForGoal(ready({estop:{required:false,error:'ignored'}})),true);
  for(const [phase,step] of Object.entries(K.PHASE_STEPS)) {
    if(step && phase!=='SUCCEEDED') assert.equal(K.selectScreen(ready({floor_mission:{phase}})),'localizing',phase);
  }
});
test('E-stop cancellation detail excludes foreign, completed and acknowledged goals',()=>{
  assert.equal(K.estopDeliveryRoom(ready({goal:goal()}),own.ownedRequestIds),'402');
  assert.equal(K.estopDeliveryRoom(ready({goal:goal()}),[]),'');
  for(const status of ['SUCCEEDED','FAILED','REJECTED','CANCELED']) {
    assert.equal(K.estopDeliveryRoom(ready({goal:goal(status)}),own.ownedRequestIds),'');
  }
  assert.equal(K.estopDeliveryRoom(ready({goal:goal(),acknowledged_request:'kiosk_goal_123'}),own.ownedRequestIds),'');
  assert.equal(K.estopDeliveryRoom(ready({goal:goal('SUCCEEDED')}),[],'301'),'301');
});
test('readiness loss at timer expiry cancels departure and explains why',async()=>{
  const f=flowFixture();f.flow.start(roomPayload);f.disable();await f.fire();
  assert.equal(f.posts.length,0);assert.equal(f.flow.pending,null);
  assert.deepEqual(f.errors,['출발할 수 없습니다. 상태를 확인한 뒤 다시 선택하세요.']);
});
test('successful cancel remains busy until CANCELING or terminal, preventing a second POST',async()=>{
  for(const status of ['CANCELING','SUCCEEDED','FAILED','REJECTED','CANCELED']) {
    const f=flowFixture();f.owned.push('kiosk_goal_123');f.setState(ready({goal:goal()}));
    f.flow.stop();for(let i=0;i<10;i++)await Promise.resolve();
    assert.equal(f.flow.stopping,true);f.flow.stop();assert.equal(f.posts.length,1);
    f.flow.reconcile(ready({goal:goal(status)}));assert.equal(f.flow.stopping,false,status);
  }
});
test('successful cancel releases after five seconds; failure permits an explicit retry',async()=>{
  const f=flowFixture();f.owned.push('kiosk_goal_123');f.setState(ready({goal:goal()}));
  f.flow.stop();for(let i=0;i<10;i++)await Promise.resolve();
  f.setTime(4999);assert.equal(f.flow.stopping,true);f.flow.stop();assert.equal(f.posts.length,1);
  f.setTime(5000);assert.equal(f.flow.stopping,false);
  f.setPost(async()=>{throw Error('취소 실패');});f.flow.stop();for(let i=0;i<10;i++)await Promise.resolve();
  assert.equal(f.flow.stopping,false);assert.deepEqual(f.errors,['취소 실패']);assert.equal(f.posts.length,2);
});
async function fireDeparture(b) {
  const entry=[...b.timers].find(([,t])=>t.ms===3000);assert(entry,'departure timer');
  b.timers.delete(entry[0]);await entry[1].fn();await b.settle();
}
test('other-floor goal uses static floor version even when live version is null',async()=>{
  const b=browserFixture();await b.settle();b.setState(ready({map_version:null}));await b.poll();
  assert.equal(b.elements.get('status-label').textContent,'정상');
  const room=[...b.elements.values()].find(e=>e.dataset.room==='402');assert.equal(room.disabled,true);
  await b.click('floor','3F');await b.click('room','301');await b.click('action','depart');await fireDeparture(b);
  const body=JSON.parse(b.requests.find(r=>r.url==='/api/goal').options.body);
  assert.deepEqual({floor:body.floor,room:body.room,x:body.x,y:body.y,yaw:body.yaw,map_version:body.map_version},
    {floor:'3F',room:'301',x:4,y:5,yaw:0.7,map_version:17});
});
test('assumed floor opens same-floor rooms and locks other floors until confirmed',async()=>{
  const b=browserFixture();await b.settle();b.setState(ready({floor:{floor:'4F',confidence:'ASSUMED',zone:1}}));await b.poll();
  assert.equal(b.elements.get('status-label').textContent,'정상');
  const room=id=>[...b.elements.values()].find(e=>e.dataset.room===id);
  assert.equal(room('402').disabled,false);
  await b.click('floor','3F');
  assert.equal(room('301').disabled,true);assert.match(b.elements.get('rooms').innerHTML,/현재 층 확인 후 다른 층/);
  b.setState(ready({floor:{floor:'4F',confidence:'BEACON',zone:1}}));await b.poll();
  assert.equal(room('301').disabled,false);assert.doesNotMatch(b.elements.get('rooms').innerHTML,/lock-note/);
  await b.click('floor','4F');await b.click('room','402');await b.click('action','depart');await fireDeparture(b);
  assert.equal(JSON.parse(b.requests.find(r=>r.url==='/api/goal').options.body).room,'402');
});
test('same-floor goal uses latest live version after the start delay',async()=>{
  const b=browserFixture();await b.settle();await b.click('room','402');await b.click('action','depart');
  b.setState(ready({map_version:202}));await b.poll();await fireDeparture(b);
  assert.equal(JSON.parse(b.requests.find(r=>r.url==='/api/goal').options.body).map_version,202);
});
test('null live map disables both pose actions without changing the admin screen',async()=>{
  const b=browserFixture();await b.settle();b.setState(ready({kiosk:{pin_required:false},localization:{ready:true,age:0,last_pose:{x:2,y:3,yaw:1}}}));await b.poll();
  await b.click('action','admin');b.draw();
  b.elements.get('map').listeners.pointerdown({button:0,isPrimary:true,pointerId:1,clientX:300,clientY:200});
  assert.equal(b.elements.get('apply-pose').disabled,false);assert.equal(b.elements.get('apply-last').disabled,false);
  b.setState(ready({map_version:null,kiosk:{pin_required:false},localization:{ready:true,age:0,last_pose:{x:2,y:3,yaw:1}}}));await b.poll();
  assert.equal(b.elements.get('app').className,'app admin');
  assert.equal(b.elements.get('apply-pose').disabled,true);assert.equal(b.elements.get('apply-last').disabled,true);
  for(const id of ['apply-pose','apply-last'])await b.elements.get('app').listeners.click({target:b.elements.get(id)});
  assert.equal(b.requests.filter(r=>r.options.method==='POST').length,0);
});
test('live version loss during delay sends nothing and displays readiness toast',async()=>{
  const b=browserFixture();await b.settle();await b.click('room','402');await b.click('action','depart');
  b.setState(ready({map_version:null}));await b.poll();
  assert.equal(b.elements.get('status-label').textContent,'정상');
  assert.equal(b.elements.get('toast').textContent,'출발할 수 없습니다. 상태를 확인한 뒤 다시 선택하세요.');
  assert.equal([...b.timers.values()].filter(t=>t.ms===3000).length,0);
  assert.equal(b.requests.filter(r=>r.options.method==='POST').length,0);
});
test('E-stop retains canceled local start room but never claims foreign or finished delivery',async()=>{
  const b=browserFixture();await b.settle();await b.click('room','402');await b.click('action','depart');
  b.setState(ready({estop:{state:'ESTOP'}}));await b.poll();
  assert.equal(b.elements.get('estop-detail').textContent,'비상 정지 버튼이 눌려 있습니다. 402호 배송은 취소되었습니다.');
  await b.poll();assert.match(b.elements.get('estop-detail').textContent,/402호 배송은 취소/);
  b.setState(ready());await b.poll();
  for(const status of ['ACCEPTED','SUCCEEDED']) {
    b.setState(ready({estop:{state:'ESTOP'},goal:goal(status)}));await b.poll();
    assert.equal(b.elements.get('estop-detail').textContent,'비상 정지 버튼이 눌려 있습니다.');
  }
});
test('gear stays disabled through start and active delivery; incoming goal closes admin or PIN pad',async()=>{
  const b=browserFixture();await b.settle();await b.click('room','402');await b.click('action','depart');
  assert.equal(b.elements.get('admin-button').disabled,true);
  await b.click('action','stop');assert.equal(b.elements.get('admin-button').disabled,false);
  await b.click('action','admin');assert.equal(b.elements.get('sheet').hidden,false);
  b.setState(ready({goal:goal()}));await b.poll();assert.equal(b.elements.get('sheet').hidden,true);
  assert.equal(b.elements.get('admin-button').disabled,true);assert(b.elements.get('stop'));
  b.setState(ready({kiosk:{pin_required:false}}));await b.poll();await b.click('action','admin');
  assert.equal(b.elements.get('app').className,'app admin');
  b.setState(ready({goal:goal()}));await b.poll();
  assert.equal(b.elements.get('app').className,'app');assert.equal(b.elements.get('status-label').textContent,'배송 중');
  assert(b.elements.get('stop'));assert.equal(b.elements.get('admin-button').disabled,true);
});

test('operator hold follows manual and precedes every goal screen', () => {
  for (const status of ['SENDING','ACCEPTED','CANCELING','SUCCEEDED']) {
    const s = ready({hold:{held:true}, goal:goal(status,{kind:'floor_goal'})});
    assert.equal(K.selectScreen(s,own),'held');
    assert.equal(K.selectScreen({...s,mode:'MANUAL'},own),'manual');
    assert.equal(K.selectScreen({...s,estop:{state:'ESTOP'}},own),'estop');
    assert.equal(K.readyForGoal(s),false);
  }
});
test('return home never needs a receipt and task remains server supplied', () => {
  for (const status of ['SUCCEEDED','FAILED','REJECTED','CANCELED']) {
    const s = ready({goal:goal(status,{origin:'return_home'}),task:'서버에서 전달한 작업'});
    assert.equal(K.selectScreen(s,own),'idle');
    assert.equal(K.selectScreen({...s,localization:{ready:false}},own),'localizing');
    assert.equal(s.task,'서버에서 전달한 작업');
  }
  assert.equal(K.task_summary,undefined);
  assert.equal(K.taskSummary,undefined);
});
test('held admin remains reachable and return home hides delivery stop', async () => {
  const b=browserFixture();await b.settle();
  b.setState(ready({hold:{held:true},kiosk:{pin_required:false},goal:goal()}));await b.poll();
  assert.equal(b.elements.get('status-label').textContent,'관리자 정지');
  assert.equal(b.elements.get('admin-button').disabled,false);
  await b.click('action','admin');
  assert.equal(b.elements.get('app').className,'app admin');
  for(const kind of ['goal','floor_goal']) {
    b.setState(ready({goal:goal('ACCEPTED',{kind,origin:'return_home'})}));await b.poll();
    assert.equal(b.elements.get('status-label').textContent,'관리자 호출로 복귀 중');
    assert.equal(b.elements.get('stop-wrap').hidden,true);
    assert.equal(b.elements.get(kind==='goal'?'drive-label':'transit-label').textContent,'관리자 호출로 복귀 중');
  }
});

test('fresh patrol locks deliveries after held and before transit', () => {
  const patrol={active:true,route:'4F_full',phase:'NAV',index:1,total:12,age:0.2};
  const s=ready({patrol,goal:goal('ACCEPTED',{kind:'floor_goal'})});
  assert.equal(K.selectScreen(s),'patrol');
  assert.equal(K.readyForGoal(s),false);
  assert.equal(K.selectScreen({...s,hold:{held:true}}),'held');
  assert.equal(K.selectScreen({...s,mode:'MANUAL'}),'manual');
  assert.equal(K.selectScreen({...s,estop:{state:'ESTOP'}}),'estop');
  for(const age of [3,4,null,undefined,'0',NaN,-1]) {
    const stale=ready({patrol:{...patrol,age}});
    assert.equal(K.selectScreen(stale),'idle');
    assert.equal(K.readyForGoal(stale),true);
  }
  for(const phase of ['DONE','ABORTED']) assert.equal(K.selectScreen(ready({patrol:{...patrol,active:false,phase}})),'idle');
});

test('patrol warning renders progress, keeps admin reachable and removes delivery controls', async () => {
  const b=browserFixture();await b.settle();
  b.setState(ready({patrol:{active:true,route:'4F_full',phase:'DWELL',index:3,total:12,age:0},
    kiosk:{pin_required:false},goal:goal()}));await b.poll();
  assert.equal(b.elements.get('status-label').textContent,'야간 순찰 중');
  assert(b.elements.get('status-pill').className.includes('warn'));
  assert(b.elements.get('screen').innerHTML.includes('순찰이 끝날 때까지 배송을 받지 않습니다.'));
  assert.equal(b.elements.get('patrol-progress').textContent,'4F_full · 3/12 · 관측');
  assert.equal(b.elements.get('stop'),undefined);
  assert.equal(b.elements.get('admin-button').disabled,false);
  await b.click('action','admin');
  assert.equal(b.elements.get('app').className,'app admin');
});

test('patrol admin status reports waiting, progress and terminal results defensively', () => {
  assert.equal(K.patrolAdminStatus(null),'대기');
  for(const [phase,label] of [['NAV','이동'],['DWELL','관측'],['RETURN','복귀']]) {
    const s=ready({patrol:{active:true,age:0,route:'4F_full',index:2,total:12,phase}});
    assert.equal(K.patrolAdminStatus(s),`4F_full 2/12 · ${label}`);
    assert.equal(K.patrolAdminStatus({...s,patrol:{...s.patrol,age:3}}),'대기');
  }
  assert.equal(K.patrolAdminStatus({patrol:{phase:'DONE'}}),'완료');
  assert.equal(K.patrolAdminStatus({patrol:{phase:'ABORTED',detail:'관리자 요청'}}),'중단: 관리자 요청');
});

test('patrol admin start and stop have independent enable rules', () => {
  const s=ready({patrol:{active:false,routes:['4F_full'],age:0}}), live={online:true};
  assert.deepEqual(copy(K.patrolAdminControls(s,live)),{start:true,stop:false});
  for(const options of [{}, {online:false}, {...live,busy:true}, {...live,blocked:true}])
    assert.equal(K.patrolAdminControls(s,options).start,false);
  for(const changes of [{hold:{held:true}}, {goal:goal()}, {patrol:{routes:[]}}, {patrol:{routes:null}}])
    assert.equal(K.patrolAdminControls({...s,...changes},live).start,false);
  const active={...s,goal:goal(),hold:{held:true},patrol:{...s.patrol,active:true}};
  assert.deepEqual(copy(K.patrolAdminControls(active,{...live,blocked:true})),{start:false,stop:true});
  assert.equal(K.patrolAdminControls(active,{...live,busy:true}).stop,false);
  assert.equal(K.patrolAdminControls(active,{online:false}).stop,false);
  assert.equal(K.patrolAdminControls({...active,patrol:{...active.patrol,age:3}},live).stop,false);
});

test('patrol route choices retain selection and only rewrite options when routes change', async () => {
  const b=browserFixture();await b.settle();
  const s=ready({kiosk:{pin_required:false},patrol:{routes:['4F_full','4F_short'],active:false}});
  b.setState(s);await b.poll();await b.click('action','admin');
  const route=b.elements.get('patrol-route'), writes=route.htmlWrites;
  assert.equal(route.value,'4F_full');
  route.value='4F_short';b.elements.get('app').listeners.change({target:route});
  await b.poll();assert.equal(route.htmlWrites,writes);assert.equal(route.value,'4F_short');
  b.setState({...s,patrol:{...s.patrol,routes:['4F_short','new_route']}});await b.poll();
  assert.equal(route.htmlWrites,writes+1);assert.equal(route.value,'4F_short');
  await b.click('action','exit');await b.click('action','admin');
  assert.equal(b.elements.get('patrol-route').value,'4F_short');
  b.setState({...s,patrol:{routes:[]}});await b.poll();
  assert.equal(b.elements.get('patrol-start').disabled,true);
  assert.equal(b.elements.get('patrol-route').value,'');
});

for(const pin of [false,true]) test(`patrol admin sends kiosk commands with PIN=${pin} and exits only on start`, async () => {
  const b=browserFixture();await b.settle();
  b.setState(ready({kiosk:{pin_required:pin},patrol:{routes:['4F_full'],active:false,phase:'IDLE'}}));await b.poll();
  const enter=async()=>{await b.click('action','admin');if(pin){await b.click('pin','1');await b.click('action','unlock');}};
  await enter();assert.equal(b.elements.get('patrol-admin-status').textContent,'대기');
  await b.click('action','patrol-start');
  assert.equal(b.elements.get('app').className,'app');
  assert.equal(b.elements.get('status-label').textContent,'야간 순찰 중');
  // Stop must work while the existing adminBlocked() rejects moving/active goals.
  b.setState(ready({kiosk:{pin_required:pin},navigation:'RUNNING',goal:goal(),
    patrol:{routes:['4F_full'],active:true,phase:'NAV',age:0,route:'4F_full',index:1,total:12}}));await b.poll();
  await enter();assert.equal(b.elements.get('patrol-start').disabled,true);
  assert.equal(b.elements.get('patrol-stop').disabled,false);
  assert.equal(b.elements.get('patrol-admin-status').textContent,'4F_full 1/12 · 이동');
  // The fake server will finish the navigation before the next state update.
  b.setState(ready({kiosk:{pin_required:pin},patrol:{routes:['4F_full'],active:true,phase:'NAV',age:0,route:'4F_full'}}));
  await b.click('action','patrol-stop');
  assert.equal(b.elements.get('app').className,'app admin');
  assert.equal(b.elements.get('patrol-stop').disabled,true);
  const sent=b.requests.filter(r=>r.url.startsWith('/api/patrol/'));
  assert.deepEqual(sent.map(r=>r.url),['/api/patrol/start','/api/patrol/stop']);
  for(const [i,r] of sent.entries()) {
    assert.equal(r.options.headers['X-AMR-Client'],'kiosk');
    assert.equal(r.options.headers['X-AMR-Kiosk-Token'],pin?'memory-only':undefined);
    const payload=JSON.parse(r.options.body);assert.match(payload.request_id,/^[A-Za-z0-9_-]{8,80}$/);
    assert.deepEqual(Object.keys(payload).sort(),i===0?['request_id','route']:['request_id']);
    if(i===0)assert.equal(payload.route,'4F_full');
  }
});

for(const [status,detail] of [[409,'등록되지 않은 순찰 경로입니다.'],[403,'관리 모드 잠금을 먼저 해제하세요.']])
  test(`patrol admin surfaces HTTP ${status} and handles rejected start`, async () => {
    const b=browserFixture();await b.settle();
    b.setState(ready({kiosk:{pin_required:false},patrol:{routes:['4F_full'],active:false}}));await b.poll();
    await b.click('action','admin');b.rejectPatrol(status,detail);await b.click('action','patrol-start');
    assert.equal(b.elements.get('toast').textContent,detail);
    assert.equal(b.elements.get('app').className,status===403?'app':'app admin');
  });
