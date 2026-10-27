'use strict';
const cards = new Map();
const busy = new Set();
const labels = {stop: '정지', resume: '해제', return_home: '복귀', patrol_start: '순찰 시작', patrol_stop: '순찰 정지'};
const numeric = (value, digits = 2) => typeof value === 'number' && Number.isFinite(value) ? value.toFixed(digits) : '—';
function element(tag, text, className) {
  const node = document.createElement(tag);
  if (text !== undefined) node.textContent = text;
  if (className) node.className = className;
  return node;
}
function makeCard(id) {
  const node = element('article', undefined, 'robot-card');
  const title = element('h2', id);
  const link = element('p', '', 'link');
  const fields = element('dl');
  const values = {};
  for (const [key, label] of Object.entries({floor: '층', pose: '위치 (m / °)', task: '작업', battery: '배터리', mode: '모드', hold: '관리자 정지'})) {
    values[key] = element('dd', '—', key);
    fields.append(element('dt', label), values[key]);
  }
  const badges = element('div', undefined, 'badges');
  const actions = element('div', undefined, 'actions');
  const patrolRow = element('div', undefined, 'patrol-row');
  const route = element('select');
  route.setAttribute('aria-label', `${id} 순찰 경로`);
  const patrolProgress = element('p', '', 'patrol-progress');
  patrolRow.append(element('strong', '순찰'), route);
  const buttons = {};
  for (const [kind, label] of Object.entries(labels)) {
    const button = element('button', label, kind);
    button.type = 'button';
    button.addEventListener('click', () => command(id, kind));
    (kind.startsWith('patrol_') ? patrolRow : actions).append(button);
    buttons[kind] = button;
  }
  const result = element('p', '', 'result');
  result.setAttribute('role', 'status');
  node.append(title, link, fields, badges, actions, patrolRow, patrolProgress, result);
  document.querySelector('#robots').append(node);
  const card = {node, link, values, badges, buttons, result, route, patrolProgress, robot: {robot_id: id, online: false}};
  cards.set(id, card);
  return card;
}
function update(card, robot) {
  card.robot = robot;
  const s = robot.summary || {};
  card.node.classList.toggle('offline', !robot.online);
  card.link.textContent = `${robot.online ? '온라인' : '오프라인'} · 마지막 연결 ${numeric(robot.age, 1)}초 전${robot.local_api_available === false ? ' · 로컬 API 연결 없음' : ''}`;
  card.values.floor.textContent = s.floor || '—';
  card.values.pose.textContent = `x ${numeric(s.robot?.x)} / y ${numeric(s.robot?.y)} / yaw ${numeric(typeof s.robot?.yaw === 'number' ? s.robot.yaw * 180 / Math.PI : null)}`;
  card.values.task.textContent = s.task || '—';
  card.values.task.title = s.task || '';
  const b = s.battery;
  card.values.battery.textContent = b ? (b.valid && typeof b.soc === 'number' ? `${numeric(b.soc, 0)} %` : `${numeric(b.voltage)} V`) + (b.fresh === false ? ' (지연)' : '') : '—';
  card.values.mode.textContent = s.mode || '—';
  card.values.hold.textContent = !robot.online ? '—' : s.hold?.held ? '정지 중' : s.hold?.pending ? '하차 후 정지 예정' : '해제됨';
  const p = s.patrol || {};
  const patrolOn = p.active === true && typeof p.age === 'number' && p.age < 3;
  const routes = Array.isArray(p.routes) ? p.routes.filter(v => typeof v === 'string') : [];
  if (JSON.stringify(routes) !== card.route.dataset.routes) {
    const selected = card.route.value;
    card.route.replaceChildren(...routes.map(name => { const option = element('option', name); option.value = name; return option; }));
    if (routes.includes(selected)) card.route.value = selected;
    card.route.dataset.routes = JSON.stringify(routes);
  }
  card.route.disabled = !robot.online || busy.has(robot.robot_id) || !routes.length;
  const phase = {NAV:'이동', DWELL:'관측', RETURN:'복귀', IDLE:'대기', DONE:'완료', ABORTED:'중단'}[p.phase] || p.phase || '대기';
  card.patrolProgress.textContent = robot.online ? `${p.route || '—'} ${p.index ?? 0}/${p.total ?? 0} · ${phase}` : '—';
  card.badges.replaceChildren();
  if (robot.online) {
    if (s.estop?.state === 'ESTOP' || s.estop?.healthy === false || s.estop?.error) {
      card.badges.append(element('span', `E-stop ${s.estop?.state || '확인 필요'}`, 'badge danger'));
    } else if (s.estop?.healthy !== true) {
      card.badges.append(element('span', 'E-stop 확인 필요', 'badge'));
    } else {
      card.badges.append(element('span', `E-stop ${s.estop.state || '정상'}`, 'badge safe'));
    }
    if (patrolOn) card.badges.append(element('span', '순찰 중', 'badge'));
    if (s.mode === 'MANUAL') card.badges.append(element('span', '수동', 'badge'));
    if (s.hold?.held) card.badges.append(element('span', '관리자 정지', 'badge danger'));
    else if (s.hold?.pending) card.badges.append(element('span', '하차 후 정지 예정', 'badge danger'));
    if (s.localization?.ready !== true) card.badges.append(element('span', '위치 확인 필요', 'badge'));
  }
  for (const [kind, button] of Object.entries(card.buttons)) {
    button.disabled = !robot.online || busy.has(robot.robot_id) || (kind === 'resume' && s.hold?.held !== true && s.hold?.pending !== true) ||
      (kind === 'patrol_start' && (!routes.length || s.hold?.held || patrolOn ||
        ['SENDING', 'ACCEPTED', 'CANCELING'].includes(s.goal?.status))) ||
      (kind === 'patrol_stop' && !patrolOn);
  }
}
// AbortSignal.timeout is missing in older browsers; without it every fetch would throw.
const timeoutSignal = ms => typeof AbortSignal.timeout === 'function' ? AbortSignal.timeout(ms) :
  (() => { const c = new AbortController(); setTimeout(() => c.abort(), ms); return c.signal; })();
function requestId() {
  if (typeof crypto.randomUUID === 'function') return crypto.randomUUID().replaceAll('-', '');
  // Tailscale's plain HTTP is not a secure context in some browsers.
  return Array.from(crypto.getRandomValues(new Uint8Array(16)), byte => byte.toString(16).padStart(2, '0')).join('');
}
async function command(id, kind) {
  const card = cards.get(id);
  if (busy.has(id) || !card.robot.online || card.buttons[kind].disabled) return;
  const confirmation = {stop:`${id} 로봇을 정지할까요?`, return_home:`${id} 로봇을 홈으로 복귀시킬까요?`,
    patrol_start:`${id} 로봇의 ${card.route.value} 순찰을 시작할까요?`, patrol_stop:`${id} 로봇의 순찰을 정지할까요?`};
  if (kind !== 'resume' && !window.confirm(confirmation[kind])) return;
  const payload = {request_id: requestId()};
  if (kind === 'patrol_start') payload.route = card.route.value;
  busy.add(id);
  card.result.textContent = `${labels[kind]} 요청 중…`;
  update(card, card.robot);
  try {
    const response = await fetch(`/robots/${encodeURIComponent(id)}/api/${kind}`, {
      method: 'POST', headers: {'Content-Type': 'application/json', 'X-AMR-Request': '1'},
      body: JSON.stringify(payload), signal: timeoutSignal(15000)
    });
    const body = await response.json();
    card.result.textContent = response.ok ? `${labels[kind]} 요청 접수 (${response.status})` : (body.error || `요청 실패 (${response.status})`);
  } catch (_) {
    card.result.textContent = '명령 결과를 확인하지 못했습니다. 현재 상태를 확인해 주세요.';
  } finally {
    busy.delete(id);
    update(card, card.robot);
  }
}
let polling = false;
async function poll() {
  if (polling) return;
  polling = true;
  try {
    const response = await fetch('/api/robots', {cache: 'no-store', signal: timeoutSignal(2000)});
    if (!response.ok) throw new Error('connection');
    const robots = await response.json();
    if (!Array.isArray(robots)) throw new Error('format');
    const ids = new Set(robots.map(robot => robot.robot_id));
    for (const [id, card] of cards) {
      if (!ids.has(id)) { card.node.remove(); cards.delete(id); }
    }
    for (const robot of robots) update(cards.get(robot.robot_id) || makeCard(robot.robot_id), robot);
    document.querySelector('#empty').hidden = robots.length > 0;
    document.querySelector('#connection').textContent = '1초마다 갱신';
  } catch (_) {
    document.querySelector('#connection').textContent = '관제 서버 연결 끊김';
    for (const card of cards.values()) update(card, {...card.robot, online: false, summary: null});
  } finally { polling = false; }
}
poll();
setInterval(poll, 1000);


const eventRows = new Map();
const seenEvents = new Set();
let eventCursor = 0;
let eventsInitialized = false;
let eventPolling = false;
let securityOnly = false;
let eventFilterVersion = 0;
let loadedFilterVersion = -1;
let visibleEvents = '';
const eventOrder = (a, b) => b.wall_time - a.wall_time || b.id.localeCompare(a.id);
let soundEnabled = false;
let audioContext;
let titleTimer;
const normalTitle = document.title;
const soundToggle = document.querySelector('#sound-toggle');
try { soundEnabled = localStorage.getItem('amr-event-sound') === 'on'; } catch (_) {}
function updateSoundToggle() {
  soundToggle.textContent = soundEnabled ? '알림음 끄기' : '알림음 켜기';
  soundToggle.setAttribute('aria-pressed', String(soundEnabled));
  soundToggle.title = soundEnabled && (!audioContext || audioContext.state !== 'running') ?
    '화면을 클릭하면 알림음이 활성화됩니다.' : '';
}
function unlockAudio() {
  if (!soundEnabled) return;
  try {
    const Audio = window.AudioContext || window.webkitAudioContext;
    if (!Audio) return;
    if (!audioContext) audioContext = new Audio();
    audioContext.resume().then(updateSoundToggle).catch(() => {});
  } catch (_) {}
}
soundToggle.addEventListener('click', () => {
  soundEnabled = !soundEnabled;
  try { localStorage.setItem('amr-event-sound', soundEnabled ? 'on' : 'off'); } catch (_) {}
  if (soundEnabled) unlockAudio();
  else {
    clearInterval(titleTimer);
    document.title = normalTitle;
    if (audioContext) audioContext.suspend().catch(() => {});
  }
  updateSoundToggle();
});
document.addEventListener('pointerdown', unlockAudio);
document.addEventListener('keydown', unlockAudio);
updateSoundToggle();
function alertEvent() {
  if (!soundEnabled) return;
  clearInterval(titleTimer);
  let flashes = 0;
  titleTimer = setInterval(() => {
    document.title = ++flashes % 2 ? '감지 알림 · AMR 중앙 관제' : normalTitle;
    if (flashes >= 10) { clearInterval(titleTimer); document.title = normalTitle; }
  }, 500);
  if (!audioContext || audioContext.state !== 'running') return;
  for (let n = 0; n < 3; n++) {
    const oscillator = audioContext.createOscillator();
    const gain = audioContext.createGain();
    const start = audioContext.currentTime + n * 0.25;
    oscillator.frequency.value = 880;
    gain.gain.setValueAtTime(0, start);
    gain.gain.linearRampToValueAtTime(0.12, start + 0.01);
    gain.gain.linearRampToValueAtTime(0, start + 0.13);
    oscillator.connect(gain);
    gain.connect(audioContext.destination);
    oscillator.start(start);
    oscillator.stop(start + 0.15);
    oscillator.onended = () => { oscillator.disconnect(); gain.disconnect(); };
  }
}
function renderEvents() {
  const list = document.querySelector('#events');
  const rows = [...eventRows.values()].sort(eventOrder).slice(0, 50);
  document.querySelector('#event-status').textContent = rows.length ? '최근 기록 · 2초마다 확인' : '감지 기록이 없습니다.';
  // seq is a change cursor, not visible content or event chronology.
  const signature = JSON.stringify(rows.map(e => [e.id, e.wall_time, e.robot_id, e.floor,
    e.text, e.severity, e.has_image]));
  if (signature === visibleEvents) return;
  visibleEvents = signature;
  list.replaceChildren(...rows.map(e => {
    const row = element('li', undefined, 'event-row');
    const copy = element('div', undefined, 'event-copy');
    const date = new Date(e.wall_time * 1000);
    const time = date.toLocaleTimeString('ko-KR', {hour12: false, hour: '2-digit', minute: '2-digit', second: '2-digit'});
    copy.append(element('p', `${time} · ${e.robot_id} · ${e.floor || '—'}`, 'event-meta'),
      element('p', e.text, 'event-text'));
    const severity = {info: '정보', warn: '주의', alert: '경보'}[e.severity] || '정보';
    row.append(copy, element('span', severity, `badge ${e.severity === 'alert' ? 'danger' : e.severity === 'info' ? 'safe' : ''}`));
    if (e.has_image && /^[0-9a-f]{32}$/.test(e.id)) {
      const link = element('a', undefined, 'event-image');
      link.href = `/api/events/${e.id}/image.jpg`;
      link.target = '_blank';
      link.rel = 'noopener noreferrer';
      link.setAttribute('aria-label', `${e.robot_id} 감지 이미지 크게 보기`);
      const image = element('img');
      image.src = link.href;
      image.alt = '감지 스냅숏';
      image.loading = 'lazy';
      link.append(image);
      row.append(link);
    }
    return row;
  }));
}
for (const [id, only] of [['events-all', false], ['events-security', true]]) {
  document.querySelector('#' + id).addEventListener('click', () => {
    if (securityOnly === only) return;
    securityOnly = only;
    eventFilterVersion++;
    document.querySelector('#events-all').setAttribute('aria-pressed', String(!only));
    document.querySelector('#events-security').setAttribute('aria-pressed', String(only));
    document.querySelector('#event-status').textContent = '기록을 불러오는 중…';
    pollEvents();
  });
}
async function pollEvents() {
  if (eventPolling) return;
  eventPolling = true;
  const filterVersion = eventFilterVersion;
  const kindQuery = securityOnly ? '&kind_prefix=security.' : '';
  try {
    const latestResponse = await fetch('/api/events/latest_seq', {cache: 'no-store', signal: timeoutSignal(2000)});
    if (!latestResponse.ok) throw new Error('events');
    const latest = await latestResponse.json();
    if (filterVersion !== eventFilterVersion) return;
    const reload = filterVersion !== loadedFilterVersion;
    if (eventsInitialized && !reload && latest.latest_seq === eventCursor) {
      renderEvents();
      return;
    }
    // A replaced database starts a new cursor history.
    if (latest.latest_seq < eventCursor) {
      eventCursor = 0; eventsInitialized = false; eventRows.clear(); seenEvents.clear();
    }
    const response = await fetch(`/api/events?after=${reload ? 0 : eventCursor}&limit=200${kindQuery}`, {cache: 'no-store', signal: timeoutSignal(2000)});
    if (!response.ok) throw new Error('events');
    const body = await response.json();
    if (filterVersion !== eventFilterVersion) return;
    if (reload) eventRows.clear();
    let alert = false;
    for (const e of body.events) {
      if (eventsInitialized && !reload && !seenEvents.has(e.id) && e.kind.startsWith('security.')) alert = true;
      seenEvents.add(e.id);
      eventRows.set(e.id, e);
    }
    // Keep only the latest 50; IDs remain remembered to avoid alerts on delayed images.
    const ordered = [...eventRows.values()].sort(eventOrder);
    for (const e of ordered.slice(50)) eventRows.delete(e.id);
    eventCursor = body.latest_seq;
    loadedFilterVersion = filterVersion;
    eventsInitialized = true;
    renderEvents();
    if (alert) alertEvent();
  } catch (_) {
    if (filterVersion === eventFilterVersion) {
      document.querySelector('#event-status').textContent = '감지 기록 연결 끊김 · 다시 시도 중';
    }
  } finally {
    eventPolling = false;
    if (filterVersion !== eventFilterVersion) pollEvents();
  }
}
pollEvents();
setInterval(pollEvents, 2000);
