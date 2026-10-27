/* On-robot delivery kiosk. No external dependencies; commands are never retried. */
(() => {
  'use strict';
const ICON = {
  arrow:'<path d="m5 19 14-14M5 5h14v14"/>',
  pin:'<path d="M12 21s-7-6.2-7-11.5A7 7 0 0 1 19 9.5C19 14.8 12 21 12 21Z"/><circle cx="12" cy="9.5" r="2.5"/>',
  route:'<circle cx="6" cy="18" r="2"/><circle cx="18" cy="6" r="2"/><path d="M8 18h7a3 3 0 0 0 0-6H9a3 3 0 0 1 0-6h7"/>',
  stop:'<rect x="6" y="6" width="12" height="12" rx="2"/>',
  check:'<path d="m5 12.5 4.5 4.5L19 7.5"/>',
  alert:'<path d="M12 3 2 20h20L12 3Z"/><path d="M12 10v4m0 3v.01"/>',
  hand:'<path d="M8 12V6a2 2 0 0 1 4 0v6-8a2 2 0 0 1 4 0v8-5a2 2 0 0 1 4 0v8c0 4-2 6-6 6h-2c-2 0-3-1-5-3l-4-5a2 2 0 0 1 3-2l2 1Z"/>',
  gear:'<circle cx="12" cy="12" r="3"/><path d="M19.4 15a1.7 1.7 0 0 0 .3 1.8l.1.1a2 2 0 1 1-2.8 2.8l-.1-.1a1.7 1.7 0 0 0-1.8-.3 1.7 1.7 0 0 0-1 1.5V21a2 2 0 1 1-4 0v-.1a1.7 1.7 0 0 0-1.1-1.5 1.7 1.7 0 0 0-1.8.3l-.1.1a2 2 0 1 1-2.8-2.8l.1-.1a1.7 1.7 0 0 0 .3-1.8 1.7 1.7 0 0 0-1.5-1H3a2 2 0 1 1 0-4h.1a1.7 1.7 0 0 0 1.5-1.1 1.7 1.7 0 0 0-.3-1.8l-.1-.1a2 2 0 1 1 2.8-2.8l.1.1a1.7 1.7 0 0 0 1.8.3H9a1.7 1.7 0 0 0 1-1.5V3a2 2 0 1 1 4 0v.1a1.7 1.7 0 0 0 1 1.5 1.7 1.7 0 0 0 1.8-.3l.1-.1a2 2 0 1 1 2.8 2.8l-.1.1a1.7 1.7 0 0 0-.3 1.8V9a1.7 1.7 0 0 0 1.5 1H21a2 2 0 1 1 0 4h-.1a1.7 1.7 0 0 0-1.5 1Z"/>',
  locate:'<circle cx="12" cy="12" r="7"/><circle cx="12" cy="12" r="2"/><path d="M12 2v3m0 14v3M2 12h3m14 0h3"/>',
  lift:'<rect x="4" y="3" width="16" height="18" rx="2"/><path d="M12 3v18M8 9l-1.5 2h3L8 9Zm8 0-1.5 2h3L16 9Z"/>',
  clock:'<circle cx="12" cy="12" r="9"/><path d="M12 7v5l3 2"/>',
  history:'<path d="M3 12a9 9 0 1 0 3-6.7L3 8"/><path d="M3 3v5h5M12 7v5l3 2"/>',
};
const icon = (n, cls = 'i') => `<svg class="${cls}" viewBox="0 0 24 24">${ICON[n]}</svg>`;

  const ACTIVE = new Set(['SENDING', 'ACCEPTED', 'CANCELING']);
  const TERMINAL = new Set(['SUCCEEDED', 'FAILED', 'REJECTED', 'CANCELED']);
  const PHASE_STEPS = Object.freeze({NAV_TO_HALL:1, CALL_ELEVATOR:2, WAIT_DOOR_OPEN:2,
    BOARDING:3, BACKING_OUT:3, IN_CAR_ALIGN:3, SELECT_FLOOR:3, RIDING:4, SWITCH_MAP:4,
    ALIGHTING:5, RETURN_TO_CAR:5, RELOCALIZE:5, NAV_TO_GOAL:6,
    IDLE:0, SUCCEEDED:6, FAILED_SAFE:0, CANCELED:0});
  const MISSION_ACTIVE = new Set(Object.keys(PHASE_STEPS).filter(p => !['IDLE','SUCCEEDED','FAILED_SAFE','CANCELED'].includes(p)));
  const phaseToStep = phase => PHASE_STEPS[phase] ?? 0;
  const splitRoomId = id => {
    const [number, ...suffix] = String(id || '').split('-');
    return {number, suffix:suffix.join('-')};
  };
  const roomLabel = id => {
    const {number, suffix} = splitRoomId(id);
    return number ? `${number}호${suffix ? ' '+suffix : ''}` : '';
  };
  // Same-floor goals only need the loaded map; a startup floor stays ASSUMED until the hall beacon is heard.
  const floorConfirmed = s => ['BEACON','CONFIRMED'].includes(s?.floor?.confidence);
  function readyForGoal(s) {
    return Boolean(!s?.hold?.held && s?.mode === 'AUTO' && !(s.estop?.required && s.estop.error) && s.estop?.state !== 'ESTOP' &&
      !MISSION_ACTIVE.has(s.floor_mission?.phase) && s?.localization?.ready && (s.localization.age == null || s.localization.age < 2) &&
      s.action_ready && s.floor?.floor && (floorConfirmed(s) || s.floor.confidence === 'ASSUMED') && s.floor.zone !== 2 &&
      !s.tf_error && !s.localization_reset_pending && !s.pending_request);
  }
  function estopDeliveryRoom(s, ownedRequestIds, pendingRoom='') {
    if (pendingRoom) return pendingRoom;
    const goal = s?.goal;
    return ACTIVE.has(goal?.status) && ownedRequestIds.includes(goal.request_id) &&
      goal.request_id !== s.acknowledged_request ? goal.room || '' : '';
  }
  // Pure selection: connection is an overlay; the caller retains the last underlying screen.
  function selectScreen(s, {fetchFailed=false, stateAgeMs=0, startPending=false, ownedRequestIds=[]}={}) {
    if (!s || fetchFailed || stateAgeMs > 3000) return 'connection';
    if ((s.estop?.required && s.estop.error) || s.estop?.state === 'ESTOP') return 'estop';
    if (s.mode === 'MANUAL') return 'manual';
    if (s.hold?.held) return 'held';
    const active = ACTIVE.has(s.goal?.status);
    if (active && (s.goal.kind === 'floor_goal' || MISSION_ACTIVE.has(s.floor_mission?.phase))) return 'transit';
    if (active || startPending) return 'driving';
    if (s.goal?.origin !== 'return_home' && TERMINAL.has(s.goal?.status) && s.goal.room && ownedRequestIds.includes(s.goal.request_id) &&
        s.goal.request_id !== s.acknowledged_request) return 'arrived';
    return readyForGoal(s) ? 'idle' : 'localizing';
  }
  function formatETA(seconds) {
    if (!Number.isFinite(seconds) || seconds < 0) return '';
    const rounded = Math.ceil(seconds);
    return rounded < 60 ? `약 ${rounded}초` : `약 ${Math.ceil(rounded/60)}분`;
  }
  // Immutable tracker, reset per request. Only the latest ten seconds affect observed speed.
  function updateProgress(previous, requestId, remaining, nowMs) {
    let p = previous?.requestId === requestId ? previous : {requestId, initial:null, percent:0, samples:[]};
    if (!Number.isFinite(remaining) || remaining < 0 || (p.initial === null && remaining <= 0.05)) return {...p, remaining:null, eta:''};
    const feedbackStartedAt = p.feedbackStartedAt ?? nowMs;
    const calibrating = nowMs-feedbackStartedAt <= 3000;
    const initial = p.initial === null ? remaining : calibrating ? Math.max(p.initial, remaining) : p.initial;
    const samples = p.samples.filter(v => nowMs-v.t <= 10000 && v.t < nowMs).concat({t:nowMs, r:remaining});
    const first = samples[0], duration = (nowMs-first.t)/1000;
    const observed = duration > 0 ? (first.r-remaining)/duration : 0;
    const speed = observed > 0.01 ? observed : 0.3;
    // While initial is still calibrating, an early percent may be against a too-small initial; recompute it.
    const computed = Math.min(100, Math.max(0, initial > 0 ? (1-remaining/initial)*100 : remaining === 0 ? 100 : 0));
    const percent = calibrating ? computed : Math.max(p.percent, computed);
    return {requestId, initial, feedbackStartedAt, percent, samples, remaining, eta:formatETA(remaining/speed)};
  }
  const requestId = () => globalThis.crypto?.randomUUID ? crypto.randomUUID() :
    `kiosk_${Date.now()}_${Math.random().toString(36).slice(2)}`;

  const START_BLOCKED = '출발할 수 없습니다. 상태를 확인한 뒤 다시 선택하세요.';

  // The delayed action can be tested without a DOM or a robot. Stop cancels any active goal, whoever sent it:
  // ownership lives in sessionStorage and is lost when Chromium restarts mid-delivery.
  function createDeliveryFlow(deps) {
    let pending = null, timer = null, stopping = false, stopTarget = null, cancelAwaiting = null;
    const changed = () => deps.changed();
    const drop = () => { if (timer !== null) deps.clearTimer(timer); timer = null; pending = null; changed(); };
    const rejectStart = () => { drop(); deps.error(START_BLOCKED); };
    function cancelBusy(s = deps.state()) {
      if (cancelAwaiting && (deps.now() >= cancelAwaiting.until ||
          (s?.goal?.request_id === cancelAwaiting.id && (s.goal.status === 'CANCELING' || TERMINAL.has(s.goal.status))))) cancelAwaiting = null;
      return Boolean(cancelAwaiting);
    }
    async function sendCancel(goal) {
      if (stopping || cancelBusy() || goal?.status !== 'ACCEPTED') return;
      stopTarget = null; stopping = true; changed();
      try {
        await deps.post('cancel', {request_id:requestId()});
        cancelAwaiting = {id:goal.request_id, until:deps.now()+5000};
      }
      catch (e) { deps.error(e.message); }
      finally { stopping = false; changed(); }
    }
    return {
      get pending() { return pending; },
      get stopping() { return stopping || cancelBusy() || Boolean(stopTarget); },
      start(payload) {
        if (pending || !deps.ready()) return;
        const action = {payload:{...payload, request_id:requestId()}, phase:'delay', stopRequested:false, startedAt:deps.now()};
        stopTarget = null; pending = action;
        timer = deps.setTimer(async () => {
          timer = null;
          if (pending !== action) return;
          if (!deps.ready()) { rejectStart(); return; }
          action.phase = 'sending'; deps.remember(action.payload.request_id); changed();
          try {
            await deps.post('goal', action.payload);
            if (pending === action) { action.phase = 'awaiting'; changed(); }
            // A stop tapped during POST waits until this request is visible in state.
            this.reconcile(deps.state());
          } catch (e) {
            if (pending === action) drop();
            // Preserve ownership: a timed-out POST might already have reached the server.
            deps.error(e.message);
          }
        }, 3000);
        changed();
      },
      stop() {
        if (stopping || cancelBusy()) return;
        if (pending?.phase === 'delay') { drop(); return; }
        if (pending) { pending.stopRequested = true; stopTarget = pending.payload.request_id; }
        else if (ACTIVE.has(deps.state()?.goal?.status)) stopTarget = deps.state().goal.request_id;
        changed(); this.reconcile(deps.state());
      },
      reconcile(s) {
        cancelBusy(s);
        if (stopTarget && s?.goal?.request_id === stopTarget) {
          if (s.goal.status === 'ACCEPTED') void sendCancel(s.goal);
          else if (TERMINAL.has(s.goal.status)) { stopTarget = null; changed(); }
        }
        if (!pending) return;
        if (pending.phase === 'delay') { if (!deps.ready()) rejectStart(); return; }
        const goal = s?.goal;
        if (goal?.request_id === pending.payload.request_id && pending.phase !== 'sending') {
          drop();
        } else if (s?.notice?.request_id === pending.payload.request_id && ['FAILED','REJECTED','PREVIEW'].includes(s.notice.status)) {
          const detail = s.notice.detail; drop(); if (detail) deps.error(detail);
        } else if (pending.phase === 'awaiting' && deps.now()-pending.startedAt > 13000) {
          drop(); deps.error('연결 대기');
        }
      },
      cancelDelay(notify=false) { if (pending?.phase === 'delay') { if (notify) rejectStart(); else drop(); } },
    };
  }
  window.AMRKiosk = {selectScreen, phaseToStep, PHASE_STEPS, readyForGoal, floorConfirmed, splitRoomId,
    roomLabel, updateProgress, formatETA, createDeliveryFlow, estopDeliveryRoom};
  if (typeof document === 'undefined') return;

  const $ = id => document.getElementById(id), G = window.AMRGeometry;
  const esc = value => String(value ?? '').replace(/[&<>"']/g, c => ({'&':'&amp;','<':'&lt;','>':'&gt;','"':'&quot;',"'":'&#39;'}[c]));
  const setText = (id, value) => { const e = $(id); if (e && e.textContent !== String(value)) e.textContent = value; };
  const htmlValues = new WeakMap();
  const setHTML = (id, value) => { const e = $(id); if (e && htmlValues.get(e) !== value) { e.innerHTML = value; htmlValues.set(e, value); } };
  const setHidden = (id, value) => { const e = $(id); if (e && e.hidden !== value) e.hidden = value; };
  const setDisabled = (id, value) => { const e = $(id); if (e && e.disabled !== Boolean(value)) e.disabled = Boolean(value); };
  const setClass = (id, value) => { const e = $(id); if (e && e.className !== value) e.className = value; };
  const setWidth = (id, value) => { const e = $(id); if (e && e.style.width !== value) e.style.width = value; };
  let state = null, lastStateAt = -Infinity, fetchFailed = true, floors = [], floorsAt = -Infinity;
  let viewFloor = null, previousFloor = null, selection = null, screen = '', bodyKey = '';
  let admin = false, token = null, tokenUntil = 0, lastAdminInput = 0, pin = null, busy = false;
  let adminPose = null, adminFloor = null, gesture = null, adminRequest = null, adminRequestAt = 0;
  const adminRequests = new Set();
  let pollTimer = null, polling = false, pollAbort = null, floorLoading = false, progress = null;
  let estopStartRoom = '';
  let toastUntil = 0, drawFrame = null, mapSignature = '', mapCamera = null, lastNotice = '';
  const maps = new Map(), mapLoading = new Set(), started = new Map();
  const STORAGE = 'amr-kiosk-goals-v1';
  let owned = [];
  try { const ids = JSON.parse(sessionStorage.getItem(STORAGE) || '[]'); if (Array.isArray(ids)) owned = ids.filter(x => typeof x === 'string'); } catch (_) { /* memory works when storage is disabled */ }
  function remember(id) {
    owned = [...new Set([...owned, id])].slice(-128);
    try { sessionStorage.setItem(STORAGE, JSON.stringify(owned)); } catch (_) { /* no token is persisted */ }
  }
  const now = () => performance.now();
  const online = () => Boolean(state && !fetchFailed && now()-lastStateAt <= 3000);
  const currentFloor = () => state?.floor?.floor || state?.floor?.transit_from || null;
  const floorInfo = name => floors.find(f => f.name === name);
  function floorLabel(name) {
    const level = floorInfo(name)?.level;
    if (Number.isFinite(level)) return level < 0 ? `지하 ${-level}층` : `${level}층`;
    return /^\d+F$/.test(name || '') ? `${parseInt(name,10)}층` : name || '층 확인 중';
  }
  const commandMapVersion = floor => floor === currentFloor() ? state?.map_version : floorInfo(floor)?.map?.version;
  function canStart() {
    return online() && selectScreen(state, {ownedRequestIds:owned}) === 'idle' && !busy &&
      !admin && !document.hidden && Boolean(floorInfo(viewFloor)?.map) &&
      floorInfo(viewFloor)?.selectable !== false && commandMapVersion(viewFloor) != null &&
      (viewFloor === currentFloor() || (state.floor_action_ready && floorConfirmed(state)));
  }
  function toast(message) { setText('toast', message); toastUntil = now()+6500; setHidden('toast', false); }
  function leaveAdmin() { admin = false; token = null; tokenUntil = 0; pin = null; adminPose = null; gesture = null; render(); }
  async function post(kind, body) {
    if (kind === 'goal' || kind === 'initialpose') {
      const version = commandMapVersion(kind === 'goal' ? body.floor : currentFloor());
      if (version == null) throw new Error(START_BLOCKED);
      body = {...body, map_version:version};
    }
    const headers = {'Content-Type':'application/json', 'X-AMR-Request':'1', 'X-AMR-Client':'kiosk'};
    const privileged = ['initialpose','relocalize','set_floor','kiosk/exit'].includes(kind);
    if (privileged && state?.kiosk?.pin_required && token) headers['X-AMR-Kiosk-Token'] = token;
    const response = await fetch(`/api/${kind}`, {method:'POST', headers, body:JSON.stringify(body), signal:AbortSignal.timeout(5000)});
    const result = await response.json();
    if (!response.ok) {
      if (privileged && response.status === 403) leaveAdmin();
      throw new Error(result.error || '요청 처리 실패');
    }
    return result;
  }
  const flow = createDeliveryFlow({post, changed:() => render(), error:toast, remember,
    ready:canStart, now, state:() => state,
    setTimer:(fn, ms) => setTimeout(fn, ms), clearTimer:id => clearTimeout(id)});

  $('app').innerHTML = `<header class="top">
    <div class="brand"><span class="mark">${icon('route')}</span>배송 로봇</div>
    <div class="where">${icon('pin')}<span>G동 <b id="where"></b></span></div>
    <span id="admin-tag" class="admin-tag" hidden>관리 모드</span>
    <div class="right"><span id="status-pill" class="pill"><span class="dot"></span><span id="status-label"></span></span>
      <span id="battery" class="batt" hidden><span class="cell"><span id="battery-fill"></span></span><span id="battery-percent"></span></span>
      <span id="clock" class="clock"></span><span class="rid">AMR-01</span>
      <button id="admin-button" class="gear" data-action="admin" aria-label="관리 모드">${icon('gear')}</button></div></header>
    <main class="body"><div id="screen"></div><div id="sheet" class="scrim" hidden></div>
      <div id="connection" class="scrim connection-overlay" role="status" hidden><div class="sheet"><h2>연결 대기</h2></div></div>
      <div id="toast" role="status" hidden></div></main>`;
  const stopButton = () => `<div id="stop-wrap" class="stopwrap"><button id="stop" data-action="stop" class="btn stop">${icon('stop')}정지</button><p>정지하면 배송이 취소됩니다</p></div>`;
  const mapStage = () => '<div class="stage"><canvas id="map" aria-label="현재 층 지도"></canvas></div>';
  const mapLegend = '<div class="legend"><span><i style="background:#3264f5"></i>로봇</span><span><i style="background:#3264f5;border-radius:3px;height:5px"></i>남은 경로</span><span><i style="background:#1f8f73"></i>배송지</span></div>';
  function floorButtons(adminMode=false) {
    return floors.map(f => `<button data-${adminMode ? 'set-floor' : 'floor'}="${esc(f.name)}" class="${f.name === (adminMode ? currentFloor() : viewFloor) ? 'on' : ''}" ${!adminMode && f.selectable === false ? 'disabled' : ''}>
      ${esc(floorLabel(f.name))}${!adminMode ? `<small>${f.selectable === false ? '운행 안 함' : f.name === currentFloor() ? '현재 층' : ''}</small>` : ''}</button>`).join('');
  }
  function idleTemplate(locked) {
    return `<div class="idle"><section class="card status">
      <span class="badge ${locked ? 'warning' : ''}">${icon(locked ? 'locate' : 'check')}</span>
      <div><div class="label">상태</div><h2>${locked ? '위치 확인 중' : '대기 중'}</h2></div>
      <p>${locked ? '잠시 기다려 주세요. 1분 이상 계속되면 관리자에게 문의하세요.' : '물건을 받을 호실을 선택하세요.'}</p>
      ${locked ? '' : `<figure class="minimap"><div class="mini-stage"><canvas id="map" aria-label="현재 층 지도"></canvas></div><figcaption><span style="color:var(--accent)">● 현재 위치</span><span id="mini-floor"></span></figcaption></figure>`}
      </section><section class="card picker"><h1>배송지 선택</h1><div id="floors" class="floors"></div><div id="rooms" class="rooms"></div></section></div>`;
  }
  const templates = {
    idle:() => idleTemplate(false), localizing:() => idleTemplate(true),
    driving:() => `<div class="drive"><section class="card mapcard"><div class="head"><b id="map-floor"></b></div>${mapStage()}${mapLegend}</section>
      <aside class="card side"><div><div id="drive-label" class="label accent">배송 중</div><h1><span id="drive-room"></span><span id="drive-wording">로<br>배송 중</span></h1></div>
      <div class="progress"><div class="row"><span class="muted">진행률</span><strong><span id="progress-value">0</span><small>%</small></strong></div><div class="bar"><span id="progress-fill"></span></div></div>
      <div class="kv"><div><span>남은 거리</span><b><span id="remaining">—</span><small>m</small></b></div><div id="eta-box" hidden><span>예상 도착</span><b id="eta"></b></div></div>
      <div class="notice">${icon('alert')}통로를 비워 주세요</div>${stopButton()}</aside></div>`,
    transit:() => `<div class="transit"><section id="steps" class="card steps"></section><div class="transit-main">
      <section class="card now-card"><div class="lift"><div id="lift-floor" class="disp"></div><div class="doors"><span></span><span></span></div><div class="bot"></div></div>
      <div><div id="step-number" class="label"></div><h2 id="phase-title"></h2><p id="phase-detail"></p><div class="notice">${icon('lift')}호출 버튼은 로봇이 자동으로 누릅니다</div></div></section>
      <aside class="card side"><div><div id="transit-label" class="label accent">배송지</div><h1 id="transit-destination"></h1></div><div class="kv"><div><span>경과</span><b id="elapsed"></b></div></div>${stopButton()}</aside></div></div>`,
    arrived:() => `<div id="result" class="full ok"><div class="inner"><span id="result-icon" class="big"></span><div><div id="result-label" class="label"></div><h1 id="result-title"></h1>
      <p id="result-detail" class="one-line"></p><button id="ack" data-action="ack" class="btn ok"></button><div id="result-foot" class="foot"></div></div></div>`,
    estop:() => `<div class="full bad"><div class="inner"><span class="big">${icon('stop')}</span><div><div class="label">비상 정지</div><h1>비상 정지 상태</h1><p id="estop-detail"></p>
      <ol><li>주변 안전 확인</li><li>비상 정지 버튼을 돌려서 해제</li><li>배송지 다시 선택</li></ol></div></div></div>`,
    held:() => `<div class="full warn held"><div class="inner"><span class="big">${icon('hand')}</span><div><div class="label">관리자 정지</div><h1>관리자 정지 중</h1>
      <p>관리자가 로봇을 잠시 정지했습니다. 해제될 때까지 기다려 주세요.</p></div></div></div>`,
    manual:() => `<div class="full warn"><div class="inner"><span class="big">${icon('hand')}</span><div><div class="label">수동 조작</div><h1>수동 조작 중</h1>
      <p>관리자가 조이스틱으로 조작하고 있습니다.<br>진행 중이던 배송은 취소되었습니다.</p><div class="foot">수동 조작이 끝나면 배송지를 선택할 수 있습니다</div></div></div></div>`,
    admin:() => `<div class="admin-grid"><section class="card mapcard"><div class="head"><span><b>위치 지정</b> · 누른 곳이 위치, 끈 방향이 로봇 방향</span><span id="map-floor"></span></div>${mapStage()}
      <div class="legend"><span><i style="background:#f4a548"></i>지정 위치·방향</span></div></section><aside class="card side">
      <div id="pin-warning" class="warnline">${icon('alert')}PIN 잠금 해제됨 (시험 운영)</div>
      <div class="block"><h3>지정 위치 적용</h3><p>지도에서 지정한 위치·방향으로 위치를 다시 잡습니다.</p><button id="apply-pose" data-action="apply-pose" class="btn primary">${icon('locate')}적용</button></div>
      <div class="block"><h3>마지막 위치로 시작</h3><p id="last-pose"></p><button id="apply-last" data-action="apply-last" class="btn ghost">${icon('history')}적용</button></div>
      <div class="block"><h3>현재 층 변경</h3><p>층 정보가 실제와 다를 때만 변경하세요.</p><div id="admin-floors" class="seg"></div></div>
      <div class="exits"><button id="close-kiosk" data-action="close-kiosk" class="exit">키오스크 닫기</button><button data-action="exit" class="exit">관리 모드 종료</button></div></aside></div>`,
  };
  function phaseHeadline(phase) {
    return {NAV_TO_HALL:'승강장 이동 중', CALL_ELEVATOR:'엘리베이터 호출 중', WAIT_DOOR_OPEN:'문 열림 대기',
      BOARDING:'엘리베이터 탑승 중', BACKING_OUT:'승강장 복귀 중', IN_CAR_ALIGN:'엘리베이터 안 정렬 중',
      SELECT_FLOOR:'층 버튼 선택 중', RIDING:`${floorLabel(state.goal?.floor || state.floor_mission?.goal_floor)} 이동 중`,
      SWITCH_MAP:'지도 전환 중', ALIGHTING:'엘리베이터 하차 중', RETURN_TO_CAR:'엘리베이터 안 복귀 중',
      RELOCALIZE:'위치 확인 중', NAV_TO_GOAL:`${roomLabel(state.goal?.room)}로 배송 중`}[phase] || '승강장 이동 중';
  }
  function renderTop(base) {
    const labels = {idle:'정상', arrived:'정상', driving:'배송 중', transit:'층간 이동', localizing:'준비 중', estop:'비상 정지', manual:'수동 조작', held:'관리자 정지'};
    setText('status-label', ['driving','transit'].includes(base) && state.goal?.origin === 'return_home' ? '관리자 호출로 복귀 중' : labels[base] || '준비 중');
    setClass('status-pill', `pill ${base === 'estop' ? 'bad' : ['manual','held','localizing'].includes(base) ? 'warn' : ''}`);
    const location = base === 'arrived' && state.goal?.status === 'SUCCEEDED' ? `${roomLabel(state.goal.room)} 앞` :
      base === 'transit' ? '엘리베이터 앞' : base === 'localizing' ? '위치 확인 중' : base === 'driving' ? '복도' : '현재 위치';
    setText('where', `${floorLabel(currentFloor())} · ${location}`);
    setText('clock', new Date().toLocaleTimeString('ko-KR', {hour:'2-digit', minute:'2-digit', hour12:false}));
    const telemetry = state?.telemetry || {};
    const fresh = item => online() && item && item.valid !== false && item.fresh !== false &&
      Number.isFinite(item.age) && item.age >= 0 && item.age+(now()-lastStateAt)/1000 < (item.timeout ?? 2);
    const b = fresh(telemetry.battery) ? telemetry.battery : null;
    const left = fresh(telemetry.motor_left_power) ? telemetry.motor_left_power : null;
    const right = fresh(telemetry.motor_right_power) ? telemetry.motor_right_power : null;
    const voltage = Number.isFinite(b?.voltage) ? b.voltage : Number.isFinite(left?.voltage) && Number.isFinite(right?.voltage) ? (left.voltage+right.voltage)/2 : null;
    const percent = Number.isFinite(b?.soc) && b.soc >= 0 && b.soc <= 100 ? Math.round(b.soc) :
      typeof estimateOperationalBattery === 'function' ? estimateOperationalBattery(voltage) : null;
    setHidden('battery', percent === null);
    if (percent !== null) { setText('battery-percent', `${percent}%`); setWidth('battery-fill', `${percent}%`); }
    setHidden('admin-tag', !admin); setClass('app', `app${admin ? ' admin' : ''}`);
  }
  function render() {
    if (document.hidden) return;
    if (!state?.hold?.held && ACTIVE.has(state?.goal?.status) && (admin || pin !== null)) {
      admin = false; token = null; tokenUntil = 0; pin = null; adminPose = null; gesture = null;
    }
    const base = selectScreen(state, {fetchFailed, stateAgeMs:now()-lastStateAt, startPending:Boolean(flow.pending), ownedRequestIds:owned});
    setHidden('connection', base !== 'connection');
    renderTop(base === 'connection' ? (screen || 'localizing') : base);
    if (base === 'connection' && screen) { updateControls(); return; }
    const next = base === 'connection' ? 'localizing' : admin && !['estop','manual'].includes(base) ? 'admin' : base;
    if (next !== 'idle') selection = null;
    if (next !== screen) {
      screen = next; bodyKey = ''; mapSignature = ''; gesture = null; mapObserver.disconnect();
      $('screen').innerHTML = templates[screen]();
      if ($('map')) { mapObserver.observe($('map')); bindMap(); }
    }
    const goal = flow.pending?.payload || state?.goal || {};
    const returning = goal.origin === 'return_home';
    setHidden('stop-wrap', returning);
    const cf = currentFloor();
    if (cf !== adminFloor) { adminFloor = cf; adminPose = null; gesture = null; mapCamera = null; }
    setText('map-floor', floorLabel(cf)); setText('mini-floor', floorLabel(cf));
    if (['idle','localizing'].includes(screen)) {
      setHTML('floors', floorButtons());
      const floorLocked = screen === 'idle' && viewFloor !== cf && !floorConfirmed(state);
      const locked = screen === 'localizing' || floorLocked, rooms = floorInfo(viewFloor)?.rooms || [];
      const key = JSON.stringify([viewFloor, rooms, locked, floorLocked, selection?.room]);
      if (key !== bodyKey) {
        bodyKey = key;
        setClass('rooms', `rooms${locked ? ' locked' : ''}`);
        setHTML('rooms', rooms.map(r => { const parts = splitRoomId(r.id); return `<button class="room${selection?.room === r.id ? ' sel' : ''}" data-room="${esc(r.id)}"><b>${esc(parts.number)}</b><small>${esc(parts.suffix || '호')}</small></button>`; }).join('') +
          (floorLocked ? '<div class="lock-note"><div>현재 층 확인 후 다른 층을 선택할 수 있습니다<small>엘리베이터 홀로 이동하거나 관리자에게 문의하세요</small></div></div>' :
            locked ? '<div class="lock-note"><div>위치 확인 후 선택할 수 있습니다</div></div>' : ''));
      }
    } else if (screen === 'driving') {
      const p = flow.pending ? {percent:0, remaining:null, eta:''} : progress;
      setText('drive-label', returning ? '관리자 호출로 복귀 중' : '배송 중');
      setHidden('drive-wording', returning);
      setText('drive-room', roomLabel(goal.room) || '배송지');
      const percent = Math.floor(p?.percent || 0);
      setText('progress-value', percent); setWidth('progress-fill', `${percent}%`);
      setText('remaining', Number.isFinite(p?.remaining) ? p.remaining.toFixed(1) : '—');
      setHidden('eta-box', !p?.eta); setText('eta', p?.eta || '');
    } else if (screen === 'transit') {
      const phase = state.floor_mission?.phase || goal.phase, step = phaseToStep(phase) || 1;
      const destination = `${floorLabel(goal.floor || state.floor_mission?.goal_floor)} ${roomLabel(goal.room || state.floor_mission?.goal_room)}`;
      const steps = ['승강장 이동','호출','탑승',`${floorLabel(goal.floor || state.floor_mission?.goal_floor)} 이동`,'하차',`${roomLabel(goal.room || state.floor_mission?.goal_room)} 도착`];
      setHTML('steps', steps.map((t, i) => `<div class="step ${i+1 < step ? 'done' : i+1 === step ? 'now' : ''}"><span class="n">${i+1 < step ? '✓' : i+1}</span>${esc(t)}</div>`).join(''));
      setText('step-number', `${step} / 6`); setText('phase-title', phaseHeadline(phase));
      setText('phase-detail', step === 3 ? '탑승이 끝날 때까지 출입문 앞을 비워 주세요.' : '통로를 비워 주세요');
      setText('transit-label', state.hold?.pending ? '하차 후 정지 예정' : returning ? '관리자 호출로 복귀 중' : '배송지');
      setText('transit-destination', destination); setText('lift-floor', floorLabel(cf));
      const elapsed = Math.max(0, Math.floor(Number.isFinite(state.floor_mission?.elapsed) ? state.floor_mission.elapsed : (now()-(started.get(goal.request_id) ?? now()))/1000));
      setText('elapsed', `${Math.floor(elapsed/60)}:${String(elapsed%60).padStart(2,'0')}`);
    } else if (screen === 'arrived') {
      const success = goal.status === 'SUCCEEDED', title = success ? '도착' : goal.status === 'CANCELED' ? '배송 취소' : '배송 실패';
      setClass('result', `full ${success ? 'ok' : 'warn'}`); setHTML('result-icon', icon(success ? 'check' : 'alert'));
      setText('result-label', title); setText('result-title', success ? `${roomLabel(goal.room)} 도착` : title);
      setText('result-detail', success ? '물건을 꺼낸 뒤 수령 확인을 눌러 주세요.' : goal.error_message ||
        (state.notice?.request_id === goal.request_id ? state.notice.detail : '') || title);
      setClass('ack', `btn ${success ? 'ok' : 'primary'}`); setText('ack', success ? '수령 확인' : '확인');
      setText('result-foot', success ? '수령 확인 전까지 이 화면이 유지됩니다' : '');
    } else if (screen === 'estop') {
      const room = estopDeliveryRoom(state, owned, flow.pending?.payload.room || estopStartRoom);
      setText('estop-detail', `비상 정지 버튼이 눌려 있습니다.${room ? ' '+roomLabel(room)+' 배송은 취소되었습니다.' : ''}`);
    } else if (screen === 'admin') {
      setHidden('pin-warning', Boolean(state?.kiosk?.pin_required));
      const last = state?.localization?.last_pose;
      const saved = last?.saved_at ? new Date(last.saved_at*1000) : null;
      const time = saved && Number.isFinite(saved.getTime()) ? `${String(saved.getMonth()+1).padStart(2,'0')}-${String(saved.getDate()).padStart(2,'0')} ${String(saved.getHours()).padStart(2,'0')}:${String(saved.getMinutes()).padStart(2,'0')} 저장` : '';
      setText('last-pose', last ? [floorLabel(cf), time].filter(Boolean).join(' · ') : '—');
      setHTML('admin-floors', floorButtons(true));
    }
    renderSheet(); updateControls(); requestDraw();
  }
  function renderSheet() {
    if (pin !== null) {
      const html = `<div class="sheet pin-sheet" role="dialog" aria-modal="true" aria-label="관리 모드"><div class="label">관리 모드</div><h2>PIN</h2><output id="pin-value" aria-label="PIN 입력"></output>
        <div class="pin-pad">${['1','2','3','4','5','6','7','8','9','지우기','0','←'].map(k => `<button data-pin="${k}">${k}</button>`).join('')}</div>
        <div class="actions"><button class="btn ghost" data-action="pin-close">취소</button><button id="unlock" class="btn primary" data-action="unlock">확인</button></div></div>`;
      if (!$('pin-value')) setHTML('sheet', html);
      setText('pin-value', '●'.repeat(pin.length));
      setDisabled('unlock', busy || !pin.length || !online()); setHidden('sheet', false); return;
    }
    if (!selection || screen !== 'idle') { setHidden('sheet', true); setHTML('sheet', ''); return; }
    const cross = selection.floor !== currentFloor();
    setHTML('sheet', `<div class="sheet" role="dialog" aria-modal="true" aria-label="배송지 확인"><div class="label">배송지 확인</div><h2>${esc(floorLabel(selection.floor))} ${esc(roomLabel(selection.room))}</h2>
      ${cross ? '<p>엘리베이터로 이동합니다.</p>' : ''}<div class="facts">${cross ? '<span>층간 이동</span>' : ''}</div>
      <div class="actions"><button class="btn ghost" data-action="dismiss">취소</button><button id="depart" class="btn primary" data-action="depart">${icon('arrow')}출발</button></div></div>`);
    setHidden('sheet', false);
  }
  function adminBlocked() {
    return !online() || busy || Boolean(state?.pending_request || adminRequest || flow.pending) ||
      ACTIVE.has(state?.goal?.status) || state?.mode === 'MANUAL' || state?.localization_reset_pending ||
      ['RUNNING','WAITING_NAV2','STOPPING','MANEUVER'].includes(state?.navigation);
  }
  function updateControls() {
    document.querySelectorAll('[data-room]').forEach(b => { const disabled = !canStart(); if (b.disabled !== disabled) b.disabled = disabled; });
    setDisabled('depart', !canStart());
    setDisabled('admin-button', !online() || busy || (!state?.hold?.held && ACTIVE.has(state?.goal?.status)) || Boolean(flow.pending));
    setDisabled('stop', !online() || flow.stopping || flow.pending?.stopRequested || (!flow.pending &&
      (!ACTIVE.has(state?.goal?.status) || state?.goal?.status === 'CANCELING')));
    setDisabled('ack', !online() || busy);
    setDisabled('close-kiosk', !online() || busy);
    const info = floorInfo(currentFloor())?.map, bitmap = maps.get(currentFloor());
    const mapReady = info && bitmap?.version === info.version && state?.map_version != null;
    setDisabled('apply-pose', adminBlocked() || !adminPose || !mapReady || adminPose.map_version !== info?.version);
    const last = state?.localization?.last_pose;
    setDisabled('apply-last', adminBlocked() || !mapReady || !last);
    document.querySelectorAll('[data-set-floor]').forEach(b => { const disabled = adminBlocked(); if (b.disabled !== disabled) b.disabled = disabled; });
    document.querySelectorAll('[data-pin]').forEach(b => { if (b.disabled !== busy) b.disabled = busy; });
  }
  async function adminCommand(kind, values) {
    if (adminBlocked() || !admin) return;
    const id = requestId(); adminRequest = id; adminRequestAt = now(); adminRequests.add(id); busy = true; render();
    try { await post(kind, {request_id:id, ...values}); }
    catch (e) { adminRequest = null; toast(e.message); }
    finally { busy = false; render(); void poll(); }
  }
  $('app').addEventListener('click', async event => {
    const button = event.target.closest('button');
    if (!button || button.disabled) return;
    if (admin || pin !== null) lastAdminInput = now();
    if (button.dataset.floor) { if (!online()) return; viewFloor = button.dataset.floor; selection = null; render(); return; }
    if (button.dataset.room) {
      if (!canStart()) return;
      const floor = floorInfo(viewFloor), room = floor?.rooms.find(r => r.id === button.dataset.room);
      if (room) selection = {floor:floor.name, room:room.id, x:room.x, y:room.y, yaw:room.yaw, map_version:commandMapVersion(floor.name)};
      render(); return;
    }
    if (button.dataset.setFloor) { await adminCommand('set_floor', {floor:button.dataset.setFloor}); return; }
    if (button.dataset.pin !== undefined && pin !== null) {
      const key = button.dataset.pin;
      pin = key === '지우기' ? '' : key === '←' ? pin.slice(0,-1) : pin.length < 32 ? pin+key : pin;
      renderSheet(); return;
    }
    switch (button.dataset.action) {
      case 'dismiss': selection = null; render(); break;
      case 'depart': {
        if (!selection || !canStart()) break;
        const payload = selection; selection = null; flow.start(payload); break;
      }
      case 'stop': flow.stop(); break;
      case 'ack': {
        if (!online() || busy || !owned.includes(state?.goal?.request_id)) break;
        const id = state.goal.request_id; busy = true; render();
        try {
          await post('acknowledge', {request_id:id});
          // Only a successful acknowledgement releases the result screen.
          if (state?.goal?.request_id === id) state.acknowledged_request = id;
        } catch (e) { toast(e.message); }
        finally { busy = false; render(); void poll(); }
        break;
      }
      case 'admin':
        if (!online() || busy || (!state?.hold?.held && ACTIVE.has(state?.goal?.status)) || flow.pending) break;
        selection = null; lastAdminInput = now();
        if (state.kiosk?.pin_required && !token) pin = '';
        else admin = true;
        render(); break;
      case 'pin-close': pin = null; render(); break;
      case 'unlock': {
        if (!pin || busy || !online()) break;
        busy = true; renderSheet(); updateControls();
        try {
          const result = await post('kiosk/unlock', {pin});
          if (pin !== null && typeof result.token === 'string') {
            token = result.token; tokenUntil = now()+Math.max(0, Number(result.expires_in) || 0)*1000;
            pin = null; admin = true; lastAdminInput = now();
          }
        } catch (e) { if (pin !== null) pin = ''; toast(e.message); }
        finally { busy = false; render(); }
        break;
      }
      case 'exit': leaveAdmin(); break;
      case 'close-kiosk':
        if (!online() || busy) break;
        busy = true; render();
        // The server signals scripts/kiosk/launch.sh, which closes this browser.
        try { await post('kiosk/exit', {}); }
        catch (e) { toast(e.message); }
        finally { busy = false; render(); }
        break;
      case 'apply-pose':
        if (adminPose && !$('apply-pose').disabled) {
          const {x,y,yaw} = adminPose;
          await adminCommand('initialpose', {x,y,yaw,map_version:state.map_version});
        }
        break;
      case 'apply-last': {
        const last = state?.localization?.last_pose, info = floorInfo(currentFloor())?.map;
        if (last && info && !$('apply-last').disabled) {
          await adminCommand('initialpose', {x:last.x, y:last.y, yaw:last.yaw, map_version:state.map_version});
        }
        break;
      }
    }
  });
  // Timeouts are measured from actual input, never extended by polling or rendering.
  for (const name of ['pointerdown','pointermove','keydown']) document.addEventListener(name, () => {
    if (admin || pin !== null) lastAdminInput = now();
  }, {passive:true});

  async function fetchFloors() {
    if (floorLoading || document.hidden) return;
    floorLoading = true;
    try {
      const response = await fetch('/api/floors', {signal:AbortSignal.timeout(2500)});
      if (!response.ok) throw new Error('층 정보 조회 실패');
      const value = await response.json(); floors = value.floors || []; floorsAt = now();
      if (!floorInfo(viewFloor)) viewFloor = currentFloor() || value.current_floor || floors[0]?.name;
      if (selection) {
        const f = floorInfo(selection.floor), r = f?.rooms.find(x => x.id === selection.room);
        if (!r || commandMapVersion(selection.floor) !== selection.map_version || r.x !== selection.x || r.y !== selection.y || r.yaw !== selection.yaw) selection = null;
      }
      render(); void loadMap();
    } catch (_) { /* Existing rooms stay visible; POSTs still carry their original map version. */ }
    finally { floorLoading = false; }
  }
  async function loadMap() {
    const name = currentFloor(), info = floorInfo(name)?.map;
    if (!info || document.hidden || maps.get(name)?.version === info.version || mapLoading.has(name)) return;
    mapLoading.add(name);
    try {
      const response = await fetch(`/api/floors/${encodeURIComponent(name)}/map.png?v=${encodeURIComponent(info.version)}`, {signal:AbortSignal.timeout(2500)});
      if (!response.ok) return;
      const header = response.headers.get('X-Map-Version');
      if (header === null || Number(header) !== info.version) { floorsAt = -Infinity; return; }
      const bitmap = await createImageBitmap(await response.blob());
      if (floorInfo(name)?.map?.version !== info.version) { bitmap.close(); return; }
      maps.get(name)?.bitmap.close(); maps.set(name, {bitmap, version:info.version});
      requestDraw(); updateControls();
    } catch (_) { /* Failed downloads are retried by a later state poll, never used for pose input. */ }
    finally { mapLoading.delete(name); }
  }
  async function poll() {
    if (document.hidden || polling) return;
    clearTimeout(pollTimer); polling = true;
    pollAbort = new AbortController();
    const timeout = setTimeout(() => pollAbort?.abort(), 2500);
    try {
      const response = await fetch('/api/kiosk/state', {signal:pollAbort.signal});
      if (!response.ok) throw new Error('연결 대기');
      const next = await response.json();
      const epochChanged = state && next.connection_epoch !== state.connection_epoch;
      state = next; lastStateAt = now(); fetchFailed = false;
      if ((state.estop?.required && state.estop.error) || state.estop?.state === 'ESTOP') {
        if (flow.pending?.payload.room) estopStartRoom = flow.pending.payload.room;
      } else estopStartRoom = '';
      if (epochChanged) { flow.cancelDelay(true); selection = null; adminPose = null; progress = null; }
      const cf = currentFloor();
      if (cf !== previousFloor) {
        if (!selection) viewFloor = cf; previousFloor = cf; floorsAt = -Infinity;
      }
      if (state.goal?.request_id) {
        if (!started.has(state.goal.request_id)) started.set(state.goal.request_id, now());
        progress = updateProgress(progress, state.goal.request_id, state.goal.distance_remaining, now());
      }
      if (adminRequest && (state.notice?.request_id === adminRequest || state.pending_request?.request_id === adminRequest)) adminRequest = null;
      if (state.notice?.detail && adminRequests.has(state.notice.request_id)) {
        const key = JSON.stringify(state.notice);
        // Only surface administrative responses issued by this page.
        if (key !== lastNotice && ['FAILED','REJECTED','PUBLISHED'].includes(state.notice.status)) toast(state.notice.detail);
        lastNotice = key;
      }
      flow.reconcile(state);
      if (now()-floorsAt > 10000) void fetchFloors();
      void loadMap(); render();
    } catch (_) {
      fetchFailed = true; flow.cancelDelay(true); selection = null; render();
    } finally {
      clearTimeout(timeout); pollAbort = null; polling = false;
      if (!document.hidden) pollTimer = setTimeout(poll, 500);
    }
  }
  function requestDraw() {
    if (drawFrame === null && !document.hidden && $('map')) drawFrame = requestAnimationFrame(drawMap);
  }
  const mapObserver = new ResizeObserver(() => requestDraw());
  function drawMap() {
    drawFrame = null;
    const canvas = $('map'); if (!canvas || document.hidden) return;
    const rect = canvas.getBoundingClientRect(), w = Math.round(rect.width), h = Math.round(rect.height);
    const name = currentFloor(), info = floorInfo(name)?.map, entry = maps.get(name);
    const robot = state?.robot && state.robot.age < 2 && !state.tf_error ? state.robot : null;
    const path = screen === 'driving' && state?.global_path?.age < 2 ? state.global_path.points : [];
    const goal = screen === 'driving' ? flow.pending?.payload || state?.goal : null;
    const target = goal?.floor === name ? goal : null;
    const signature = JSON.stringify([screen, name, info, entry?.version, w,h,
      robot && [robot.x,robot.y,robot.yaw], path, target && [target.x,target.y,target.room], adminPose]);
    if (signature === mapSignature || w <= 0 || h <= 0) return;
    mapSignature = signature;
    // CSS pixels are sufficient on this CPU-limited display; Chromium applies its 1.5 scale.
    if (canvas.width !== w) canvas.width = w;
    if (canvas.height !== h) canvas.height = h;
    const ctx = canvas.getContext('2d'); ctx.clearRect(0,0,w,h); mapCamera = null;
    const origin = info && G.renderOrigin(info);
    if (!origin || !entry || entry.version !== info.version) return;
    const corners = [[0,0],[info.width,0],[0,info.height],[info.width,info.height]].map(p => G.gridToWorld(...p,info));
    const xs = corners.map(p => p.x), ys = corners.map(p => p.y);
    const left = Math.min(...xs), right = Math.max(...xs), bottom = Math.min(...ys), top = Math.max(...ys);
    const camera = {x:(left+right)/2, y:(bottom+top)/2, scale:Math.min((w-24)/(right-left),(h-24)/(top-bottom))};
    if (screen === 'idle' && robot) { camera.x = robot.x; camera.y = robot.y; camera.scale = w/24; }
    mapCamera = {camera,w,h,info,floor:name};
    const point = p => G.worldToScreen(p,camera,w,h), o = point({x:origin[0],y:origin[1]});
    ctx.save(); ctx.translate(o.x,o.y); ctx.rotate(-origin[2]);
    ctx.scale(camera.scale*info.resolution,camera.scale*info.resolution); ctx.imageSmoothingEnabled = false;
    ctx.drawImage(entry.bitmap,0,-info.height,info.width,info.height); ctx.restore();
    if (path?.length) {
      ctx.beginPath(); ctx.strokeStyle = '#3264f5'; ctx.lineWidth = 4; ctx.lineJoin = 'round'; ctx.lineCap = 'round';
      path.forEach((p,i) => { const q = point({x:p[0],y:p[1]}); if (i) ctx.lineTo(q.x,q.y); else ctx.moveTo(q.x,q.y); }); ctx.stroke();
    }
    function marker(pose, color, heading=true) {
      if (!Number.isFinite(pose?.x) || !Number.isFinite(pose?.y)) return;
      const p = point(pose); ctx.save(); ctx.translate(p.x,p.y);
      ctx.beginPath(); ctx.arc(0,0,18,0,Math.PI*2); ctx.fillStyle = color+'26'; ctx.fill();
      ctx.beginPath(); ctx.arc(0,0,9,0,Math.PI*2); ctx.fillStyle = color; ctx.fill(); ctx.strokeStyle = '#fff'; ctx.lineWidth = 3; ctx.stroke();
      if (heading) { ctx.rotate(-(pose.yaw || 0)); ctx.beginPath(); ctx.moveTo(14,-6);ctx.lineTo(23,0);ctx.lineTo(14,6);ctx.closePath();ctx.fill(); }
      ctx.restore();
    }
    if (target) {
      marker(target,'#1f8f73',false);
      const p = point(target), label = splitRoomId(target.room).number;
      if (label) {
        ctx.font = 'bold 16px sans-serif'; ctx.textAlign = 'center';
        const labelWidth = ctx.measureText(label).width+20;
        ctx.fillStyle = '#1f8f73'; ctx.fillRect(p.x-labelWidth/2,p.y-44,labelWidth,26);
        ctx.fillStyle = '#fff'; ctx.fillText(label,p.x,p.y-25);
      }
    }
    if (robot) marker(robot,'#3264f5');
    if (screen === 'admin' && adminPose) {
      const p = point(adminPose); ctx.save(); ctx.translate(p.x,p.y); ctx.rotate(-adminPose.yaw);
      ctx.beginPath(); ctx.arc(0,0,28,0,Math.PI*2); ctx.fillStyle = '#f4a54833'; ctx.fill();
      ctx.strokeStyle = '#f4a548'; ctx.lineWidth = 2; ctx.setLineDash([5,4]); ctx.stroke(); ctx.setLineDash([]);
      ctx.strokeStyle = '#e08a12'; ctx.lineWidth = 4; ctx.lineCap = 'round';
      ctx.beginPath(); ctx.moveTo(0,0); ctx.lineTo(44,0); ctx.moveTo(34,-9); ctx.lineTo(44,0); ctx.lineTo(34,9); ctx.stroke(); ctx.restore();
    }
  }
  function bindMap() {
    const canvas = $('map');
    function world(event) {
      const rect = canvas.getBoundingClientRect(), m = mapCamera;
      return G.screenToWorld({x:event.clientX-rect.left,y:event.clientY-rect.top},m.camera,m.w,m.h);
    }
    canvas.addEventListener('pointerdown', e => {
      if (screen !== 'admin' || adminBlocked() || !mapCamera || e.button !== 0 || !e.isPrimary ||
          mapCamera.floor !== currentFloor() || mapCamera.info.version !== floorInfo(currentFloor())?.map?.version) return;
      const p = world(e), info = mapCamera.info, grid = G.worldToGrid(p.x,p.y,info);
      if (grid.x < 0 || grid.y < 0 || grid.x >= info.width || grid.y >= info.height) return;
      gesture = {id:e.pointerId, start:p}; canvas.setPointerCapture(e.pointerId);
      adminPose = {...p, yaw:state?.robot?.yaw || 0, map_version:info.version}; updateControls(); requestDraw();
    });
    canvas.addEventListener('pointermove', e => {
      if (!gesture || e.pointerId !== gesture.id || !mapCamera || !adminPose || adminBlocked() ||
          adminPose.map_version !== mapCamera.info.version || mapCamera.floor !== currentFloor()) return;
      const p = world(e);
      if (Math.hypot(p.x-gesture.start.x,p.y-gesture.start.y)*mapCamera.camera.scale > 5) {
        adminPose.yaw = G.heading(gesture.start,p); requestDraw();
      }
    });
    canvas.addEventListener('pointerup', e => { if (e.pointerId === gesture?.id) gesture = null; });
    canvas.addEventListener('pointercancel', () => { gesture = null; });
  }
  document.addEventListener('visibilitychange', () => {
    clearTimeout(pollTimer);
    if (document.hidden) {
      pollAbort?.abort(); flow.cancelDelay();
      if (drawFrame !== null) { cancelAnimationFrame(drawFrame); drawFrame = null; }
    } else { render(); void poll(); }
  });
  setInterval(() => {
    if ((admin || pin !== null) && (now()-lastAdminInput >= 300000 || (token && now() >= tokenUntil))) leaveAdmin();
    if (adminRequest && now()-adminRequestAt > 10000 && state?.pending_request?.request_id !== adminRequest) { adminRequest = null; toast('연결 대기'); }
    if (toastUntil && now() >= toastUntil) { toastUntil = 0; setHidden('toast', true); }
    if (!document.hidden) render();
  }, 500);
  render(); void poll();
})();
