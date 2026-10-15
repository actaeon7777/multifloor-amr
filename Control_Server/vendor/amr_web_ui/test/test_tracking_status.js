/* Run with gjs test_tracking_status.js web/app.js; exercise the real app poll/render path. */
const GLib = imports.gi.GLib;
const System = imports.system;
const [ok, bytes] = GLib.file_get_contents(ARGV[0]);
if (!ok) throw new Error('Cannot read app.js');

const elements = new Map();
function element(id) {
  if (!elements.has(id)) elements.set(id, {
    textContent:'', className:'', hidden:false, disabled:false, value:0, style:{}, handlers:{},
    classList:{add(){}, remove(){}, toggle(){}},
    addEventListener(event, handler) { this.handlers[event] = handler; },
    setAttribute(){}, getContext() { return {}; },
    querySelector(selector) { return element(`${id} ${selector}`); },
  });
  return elements.get(id);
}
globalThis.document = {
  getElementById:element,
  querySelector:element,
  querySelectorAll() { return []; },
};
globalThis.window = {AMRGeometry:{}};
globalThis.performance = {now() { return 1000; }};
globalThis.requestAnimationFrame = () => {};
globalThis.AbortSignal = {timeout() { return {}; }};
let nextPoll = null;
globalThis.setTimeout = callback => { nextPoll = callback; };

const warning = '지도와 LiDAR 스캔이 충분히 정렬되지 않았습니다.';
let state = {
  preview:false, navigation:'RUNNING', action_ready:true, goal:null, notice:null,
  localization_reset_pending:false, pending_request:null, tf_error:null,
  localization:{state:'READY', ready:true, age:0, detail:'주행 준비 완료', tracking_warning:warning},
  robot:{x:0, y:0, yaw:0, age:0}, velocity:{linear:0, angular:0, age:0},
  layers:{map:{version:1, width:3, height:2, resolution:1}},
};
const requests = [];
globalThis.fetch = async url => {
  requests.push(url);
  if (url === '/api/state') return {ok:true, async json() { return state; }};
  if (url.startsWith('/api/layers/')) return {ok:false};
  throw new Error(`Rendering must not issue commands: ${url}`);
};

function assert(condition, detail) {
  if (!condition) throw new Error(detail);
}

async function run() {
  new Function(new TextDecoder().decode(bytes))();
  // The initial poll awaits fetch and JSON separately.
  await Promise.resolve();
  await Promise.resolve();
  assert(nextPoll !== null, 'Initial poll must finish');
  element('map').handlers.keydown({key:'ArrowUp', preventDefault(){}});
  assert(element('nav-status').textContent === '이동 중', 'Warning must preserve navigation status');
  assert(element('nav-detail').textContent.includes(warning), 'READY warning must be rendered');
  assert(element('nav-detail').textContent.includes('위치 추적 경고'), 'Warning must have a clear label');
  assert(!element('send').disabled, 'READY warning alone must not close the goal gate');

  state.localization = {...state.localization, tracking_warning:''};
  await nextPoll();
  assert(!element('nav-detail').textContent.includes(warning), 'Recovery must remove the old warning');
  assert(!element('send').disabled, 'Recovered READY must keep the goal gate open');

  state.localization = {...state.localization, state:'VERIFYING', ready:false,
    detail:'현재 위치 추정을 유지하며 새 스캔으로 확인합니다.', tracking_warning:warning};
  await nextPoll();
  assert(element('nav-status').textContent === '위치 확인 중', 'Confirmed loss must show VERIFYING');
  assert(element('nav-detail').textContent === state.localization.detail, 'VERIFYING must show supervisor detail');
  assert(element('send').disabled, 'VERIFYING must close the goal gate');

  state.localization = {...state.localization, state:'READY', ready:true, age:3};
  await nextPoll();
  assert(element('nav-status').textContent === '위치 상태 수신 대기', 'Stale READY must remain blocked');
  assert(!element('nav-detail').textContent.includes(warning), 'Stale warning must not mask stale status');
  assert(element('send').disabled, 'Stale READY must close the goal gate');
  assert(requests.every(url => url === '/api/state' || url.startsWith('/api/layers/')),
    'Status rendering must never send or resume a goal');
}

run().then(
  () => print('READY warning, recovery, VERIFYING and stale-status rendering/gates: OK'),
  error => { printerr(error.stack || error); System.exit(1); },
);
