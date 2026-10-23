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
globalThis.crypto = {randomUUID() { return 'gjs_request_0001'; }};
let nextPoll = null;
globalThis.setTimeout = callback => { nextPoll = callback; };

const warning = '지도와 LiDAR 스캔이 충분히 정렬되지 않았습니다.';
let state = {
  preview:false, mode:'AUTO', navigation:'RUNNING', action_ready:true, goal:null, notice:null,
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

  // An LB takeover aborts the goal: show the override, not FAILED, and block goals.
  state = {...state, mode:'MANUAL', navigation:'MANUAL', goal:{x:1, y:1, yaw:0, status:'FAILED'}};
  await nextPoll();
  assert(element('nav-status').textContent === '수동 조작 중', 'MANUAL must outrank the aborted goal');
  assert(element('send').disabled, 'An LB takeover must close the goal gate');
  state = {...state, mode:'AUTO', navigation:'OVERRIDDEN'};
  await nextPoll();
  assert(element('nav-status').textContent === '수동 조작으로 중단됨', 'OVERRIDDEN must be explained');
  assert(!element('send').disabled, 'Releasing LB must reopen the goal gate for a new goal');
  state = {...state, navigation:'RUNNING', goal:null};
  await nextPoll();

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
  assert(requests.every(url => url === '/api/state' || url === '/api/floors' || url.startsWith('/api/layers/')),
    'Status rendering must never send or resume a goal');

  // E-stop: a released STM32 still needs approval; the banner never approves by itself.
  state.localization = {...state.localization, age:0};
  const estop = (values) => ({required:true, received:true, fresh:true, age:0, timeout:.6, healthy:false,
    approval_required:false, reason:'', detail:'', error:'E-stop 차단', stm32:{}, ...values});
  state = {...state, estop:estop({state:'READY', healthy:true})};
  await nextPoll();
  assert(element('estop-banner').hidden, 'READY must hide the E-stop banner');
  assert(!element('send').disabled, 'READY E-stop must keep the goal gate open');
  state = {...state, estop:estop({state:'ESTOP', reason:'stm32_stopped'})};
  await nextPoll();
  assert(!element('estop-banner').hidden && element('estop-title').textContent === 'E-stop 눌림', 'ESTOP banner');
  assert(element('estop-banner').className.includes('danger'), 'ESTOP must be shown as danger');
  assert(element('estop-approve').disabled, 'A pressed E-stop cannot be approved');
  assert(element('send').disabled, 'ESTOP must close the goal gate');
  state = {...state, estop:estop({state:'DISARMED', approval_required:true})};
  await nextPoll();
  assert(element('estop-title').textContent.includes('재개 승인 필요'), 'DISARMED must ask for approval');
  assert(!element('estop-approve').disabled && !element('estop-approve').hidden, 'DISARMED enables approval');
  assert(element('send').disabled, 'DISARMED must keep goals blocked until approval');
  state = {...state, estop:estop({state:'DISARMED', approval_required:true, age:1})};
  await nextPoll();
  assert(element('estop-title').textContent === 'E-stop 상태 수신 없음', 'A stale lease is shown as missing');
  assert(element('estop-approve').disabled, 'A stale DISARMED view must not allow approval');
  state = {...state, estop:estop({state:'ARMING'})};
  await nextPoll();
  assert(element('estop-title').textContent === '주행 준비 중', 'ARMING banner');
  assert(element('estop-approve').hidden, 'ARMING hides the approve button');
  state = {...state, estop:estop({state:'BYPASS', healthy:true, required:false})};
  await nextPoll();
  assert(element('estop-banner').hidden, 'Not required must hide the banner');
  assert(requests.every(url => url === '/api/state' || url === '/api/floors' || url.startsWith('/api/layers/')),
    'E-stop rendering must never approve or send commands');

  // A remote (relay) view shows the state but never offers approval.
  state = {...state, estop:estop({state:'DISARMED', approval_required:true}), remote:{robot_id:'amr', age:0, timeout:2}};
  await nextPoll();
  assert(element('estop-approve').hidden, 'Remote views must hide the approve button');
  assert(element('estop-detail').textContent.includes('현장'), 'Remote views must point to the robot');
  element('estop-approve').handlers.click();
  assert(!requests.includes('/api/estop_approve'), 'A remote view must never request approval');

  // Locally, approval needs a second press within the confirmation window.
  delete state.remote;
  state = {...state};
  await nextPoll();
  assert(!element('estop-approve').hidden && element('estop-approve').textContent === '재개 승인', 'Local approve button');
  element('estop-approve').handlers.click();
  assert(!requests.includes('/api/estop_approve'), 'The first press only arms the approval');
  assert(element('estop-approve').textContent === '다시 눌러 승인', 'The armed button asks for confirmation');
  element('estop-approve').handlers.click();
  await Promise.resolve();
  assert(requests.includes('/api/estop_approve'), 'The confirming press requests approval');
}

run().then(
  () => print('READY warning, recovery, LB takeover, VERIFYING, stale-status, E-stop rendering/gates and approval: OK'),
  error => { printerr(error.stack || error); System.exit(1); },
);
