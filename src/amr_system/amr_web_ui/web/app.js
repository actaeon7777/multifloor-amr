/* No CDN dependencies. All commands are explicit user gestures; polling never retries them. */
(() => {
  'use strict';
  const $ = id => document.getElementById(id), G = window.AMRGeometry;
  const canvas = $('map'), stage = $('stage'), ctx = canvas.getContext('2d');
  const camera = {x:0, y:0, scale:16};
  const layers = Object.fromEntries([...document.querySelectorAll('[data-layer]')].map(e => [e.dataset.layer, e.checked]));
  const images = {}, loading = {}, pointers = new Map();
  let state = null, connected = false, connectionIssue = '', mode = 'goal', selected = null, gesture = null;
  let multiTouch = false, pinch = null, follow = false, fitted = false, busy = false;
  let width = 1, height = 1, drawFrame = null, lastNotice = '', lastRequest = null, headingOverride = false;
  let localizationRequest = null;
  let lastStateAt = 0, pollTimer = null, polling = false;
  const scanColors = Array.from({length:32}, (_, i) => `hsl(${(1-i/31)*240},78%,48%)`).concat('#009bb9');
  function requestDraw() {
    if (drawFrame === null && !document.hidden) drawFrame = requestAnimationFrame(draw);
  }
  const rfCommandLabels = {RF_TIMEOUT:'버튼 무선 전달 지연 · 재시도', NO_RESULT:'버튼 응답 없음 · 재시도',
    DEVICE_REJECTED:'비콘이 버튼 명령을 거절', NO_ROUTE:'대상 비콘 없음', BAD_FRAME:'버튼 명령 형식 오류'};
  // Multi-floor: floorsInfo from /api/floors; viewFloor is the floor whose map is shown.
  let floorsInfo = null, viewFloor = null, shownCurrentFloor = null, floorPolls = 0;
  const floorImages = {}, floorLoading = {};
  const missionLabels = window.AMRMissionLabels || {};  // transit.js
  const terminal = new Set(['SUCCEEDED', 'CANCELED', 'FAILED', 'REJECTED']);
  const navigationLabels = {IDLE:'목적지 선택', WAITING_NAV2:'주행 준비 중',
    RUNNING:'이동 중', STOPPING:'주행 종료 처리 중', MANUAL:'수동 조작 중', OVERRIDDEN:'수동 조작으로 중단됨', ESTOP:'E-stop으로 중단됨',
    SUCCEEDED:'도착했습니다', CANCELED:'주행 취소됨', FAILED:'이동하지 못했습니다', UNKNOWN:'로봇 연결 대기'};
  const details = {IDLE:'지도에서 이동할 위치를 터치하세요.',
    WAITING_NAV2:'목표까지의 주행을 준비하고 있습니다.', RUNNING:'선택한 목적지로 이동하고 있습니다.',
    STOPPING:'진행 중인 주행을 종료하고 있습니다.',
    MANUAL:'조이스틱 LB로 수동 조작 중입니다. LB를 놓으면 목적지를 지정할 수 있습니다.',
    OVERRIDDEN:'수동 조작(LB)으로 주행을 중단했습니다. 목적지를 다시 지정하세요.',
    ESTOP:'E-stop으로 주행을 중단했습니다. 버튼을 원위치로 돌린 뒤 목적지를 다시 지정하세요.',
    SUCCEEDED:'새 목적지를 선택할 수 있습니다.', CANCELED:'새 목적지를 선택할 수 있습니다.',
    FAILED:'위치와 경로를 확인한 후 다시 선택해 주세요.', UNKNOWN:'내비게이션 데이터를 기다리고 있습니다.'};
  const localizationLabels = {WAITING_DATA:'센서 수신 대기', SEARCHING:'자동 위치 찾는 중',
    VERIFYING:'위치 확인 중', WAITING_INITIAL_POSE:'위치 확인 필요',
    STARTING_NAVIGATION:'주행 준비 중', READY:'주행 준비 완료', FAILED:'위치 확인 필요', SUSPENDED:'엘리베이터 구간'};
  navigationLabels.MANEUVER = '엘리베이터 출입 중';
  details.MANEUVER = '엘리베이터 문을 통과하고 있습니다.';
  // Floors need the elevator monitor's /floor/state; without it the UI stays single-floor.
  function floorsEnabled() {
    const f = state?.floor;
    return Boolean(floorsInfo?.floors?.length > 1 && f && (f.floor || f.transit_from) && viewFloor);
  }
  function inTransit() { return Boolean(state?.floor && !state.floor.floor && state.floor.transit_to); }
  // While the car travels the live map is still the departure floor's.
  function currentFloor() { return state?.floor?.floor || state?.floor?.transit_from || floorsInfo?.current_floor || null; }
  function viewingOther() { return floorsEnabled() && viewFloor !== currentFloor(); }
  // "413-뒷문" → "413호 뒷문" (a room with more than one door).
  function roomLabel(id) { const [room, door] = String(id).split('-', 2); return door ? `${room}호 ${door}` : `${room}호`; }
  function floorInfo(name) { return floorsInfo?.floors?.find(f => f.name === name) || null; }
  function activeMap() {
    if (viewingOther()) {
      const info = floorInfo(viewFloor)?.map, image = floorImages[viewFloor];
      return {info, image: image && info && image.version === info.version ? image : null};
    }
    const info = state?.layers?.map, image = images.map;
    return {info, image: image && info && image.version === info.version ? image : null};
  }

  function stateAge(value) { return value ? value.age + (performance.now()-lastStateAt)/1000 : Infinity; }
  function localized() {
    return !localizationRequest && !state?.localization_reset_pending &&
      state?.localization?.ready === true && stateAge(state.localization) < 2;
  }
  function canResetLocation() {
    const v = state?.velocity;
    return !['WAITING_NAV2','RUNNING','STOPPING','MANEUVER'].includes(state?.navigation) &&
      !['SENDING','ACCEPTED','CANCELING'].includes(state?.goal?.status) &&
      stateAge(v) < 2 && Math.abs(v.linear) <= .02 && Math.abs(v.angular) <= .03;
  }
  // The bridge lease (age + time since this poll) must be live; never trust a stale view.
  function estopLive() {
    const e = state?.estop;
    return Boolean(e?.fresh && stateAge(e) < (e.timeout ?? .6));
  }
  function estopError() {
    const e = state?.estop;
    if (!e?.required || (e.healthy && estopLive())) return null;
    return e.error || 'E-stop 상태를 받지 못했습니다. CAN 브릿지 연결을 확인하세요.';
  }
  function commandError(kind) {
    if (!connected || busy || state?.pending_request) return '이전 요청을 처리 중이거나 연결을 기다리고 있습니다.';
    if (state?.preview || kind === 'cancel') return null;
    if (kind === 'goal' && estopError()) return estopError();
    if (kind === 'goal' && state?.mode === 'MANUAL')
      return '수동 조작(LB) 중이라 자율주행 명령을 받을 수 없습니다. LB를 놓은 뒤 목적지를 다시 지정하세요.';
    if (kind === 'goal' && state?.mode !== 'AUTO') return '제어 모드 수신을 기다리고 있습니다.';
    if (kind === 'goal' && (!localized() || !state.action_ready || stateAge(state.robot) >= 2 || state.tf_error))
      return '위치 확인과 주행 준비가 완료될 때까지 기다려 주세요.';
    if (kind === 'goal' && viewingOther() && !state.floor_action_ready)
      return '층간 이동 관리자 연결을 기다리고 있습니다.';
    if (kind === 'goal' && viewingOther() && !floorInfo(viewFloor)?.selectable)
      return `${viewFloor}은 실차 층간 이동이 꺼져 있습니다.`;
    if (kind === 'initialpose' && viewingOther()) return '초기 위치는 현재 층 지도에서만 지정할 수 있습니다.';
    if (kind === 'goal' && ['SENDING','CANCELING'].includes(state?.goal?.status))
      return '이전 목표 요청을 처리 중입니다. 잠시 기다려 주세요.';
    if (kind === 'set_floor' && ['NAV_TO_HALL','CALL_ELEVATOR','WAIT_DOOR_OPEN','BOARDING','BACKING_OUT','IN_CAR_ALIGN',
      'SELECT_FLOOR','RIDING','SWITCH_MAP','ALIGHTING','RETURN_TO_CAR','RELOCALIZE','NAV_TO_GOAL'].includes(state?.floor_mission?.phase))
      return '진행 중인 층간 이동이 있습니다.';
    if (['initialpose','relocalize','set_floor'].includes(kind) && !canResetLocation())
      return '목표를 취소하고 로봇이 정지한 뒤 위치를 다시 설정해 주세요. 속도 데이터도 수신되어야 합니다.';
    return null;
  }

  function activity(text, error=false) {
    $('activity').textContent = text;
    document.querySelector('.activity-bar').classList.toggle('error', error);
  }
  function screen(p) { return G.worldToScreen(p, camera, width, height); }
  function world(p) { return G.screenToWorld(p, camera, width, height); }
  function local(event) {
    const rect = canvas.getBoundingClientRect();
    return {x:event.clientX-rect.left, y:event.clientY-rect.top};
  }
  function setMode(value) {
    requestDraw();
    mode = value; gesture = null;
    document.querySelectorAll('[data-mode]').forEach(e => {
      e.classList.toggle('active', e.dataset.mode === value);
      e.setAttribute('aria-pressed', String(e.dataset.mode === value));
    });
    stage.classList.toggle('pan', value === 'pan');
    $('hint-text').textContent = {goal:'목적지를 터치하세요 · 드래그로 도착 방향 지정',
      pan:'드래그로 지도 이동 · 휠 또는 두 손가락으로 확대',
      initialpose:'현재 로봇 위치와 방향을 지정하세요'}[value];
    updateControls();
  }
  function setSelected(pose) {
    requestDraw();
    selected = pose;
    $('goal-x').textContent = pose.x.toFixed(2);
    $('goal-y').textContent = pose.y.toFixed(2);
    $('heading').value = Math.round(pose.yaw*180/Math.PI);
    $('goal-label').textContent = '선택됨';
    if ($('destination-name')) $('destination-name').textContent = (pose.floor ? `${pose.floor} ` : '') +
      (pose.room ? roomLabel(pose.room) : `X ${pose.x.toFixed(2)} m · Y ${pose.y.toFixed(2)} m`);
    updateControls();
  }
  function updateControls() {
    const goal = state?.goal;
    const pending = busy || state?.pending_request || (goal && ['SENDING', 'CANCELING'].includes(goal.status));
    const ready = connected && activeMap().info && !commandError(mode === 'initialpose' ? 'initialpose' : 'goal');
    $('send').disabled = !selected || !ready || pending || mode === 'pan';
    $('send').querySelector('span').textContent = state?.preview ? '선택한 위치 미리보기' :
      (mode === 'initialpose' ? '초기 위치 적용' : '선택한 위치로 이동');
    $('cancel').disabled = !connected || busy || !goal ||
      !(state.preview ? goal.status === 'PREVIEW' : goal.status === 'ACCEPTED');
    $('heading').disabled = busy;
    $('relocalize').disabled = Boolean(commandError('relocalize')) || pending ||
      (!state?.preview && (localizationRequest || state?.localization_reset_pending ||
        (state?.localization?.age < 2 && ['SEARCHING','VERIFYING','STARTING_NAVIGATION'].includes(state.localization.state))));
    const last = state?.localization?.last_pose;
    $('last-pose').hidden = !connected || Boolean(state?.preview) || !last || localized() || viewingOther();
    $('last-pose').disabled = Boolean(commandError('initialpose')) || pending || !activeMap().info;
    $('set-floor').hidden = !connected || Boolean(state?.preview) || !floorsEnabled();
    $('set-floor').disabled = Boolean(commandError('set_floor')) || pending;
  }
  function updateSidebar() {
    const s = state, goal = s.goal;
    $('connection').className = 'connection ' + (connected ? (s.preview ? 'preview' : 'live') : '');
    $('connection-label').textContent = connected ? (s.preview ? '미리보기' : '웹 연결됨') : connectionIssue || '연결 끊김';
    $('preview-banner').hidden = !s.preview;
    let status = s.navigation;
    if (goal?.status === 'SENDING') status = 'SENDING';
    else if (goal?.status === 'CANCELING') status = 'STOPPING';
    // A manual (LB) override or E-stop outranks the aborted goal's FAILED result.
    else if (terminal.has(goal?.status) && !['MANUAL','OVERRIDDEN','ESTOP'].includes(s.navigation)) status = goal.status;
    if (s.preview) {
      $('nav-status').textContent = '목표를 선택하세요';
      $('nav-detail').textContent = '실제 저장 지도로 터치 조작을 확인합니다.';
    } else if (!localized() && connected) {
      const loc = s.localization;
      $('nav-status').textContent = localizationRequest || s.localization_reset_pending ? '위치 확인 중' :
        (!loc || loc.age >= 2 ? '위치 상태 수신 대기' : localizationLabels[loc.state] || '위치 확인 필요');
      $('nav-detail').textContent = localizationRequest || s.localization_reset_pending ?
        '새 위치 확인 결과를 기다리고 있습니다.' : (!loc || loc.age >= 2 ?
          '위치 확인 연결이 복구될 때까지 목표를 보낼 수 없습니다.' : loc.detail);
    } else {
      $('nav-status').textContent = !connected ? connectionIssue || '연결 끊김' :
        status === 'SENDING' ? '목표 전송 중' : status === 'REJECTED' ? '목표가 거절됨' :
        (status === 'IDLE' ? '주행 준비 완료' : navigationLabels[status] || '주행 상태 확인 중');
      $('nav-detail').textContent = !connected ? '연결이 복구되면 상태를 다시 확인합니다.' :
        status === 'SENDING' ? '로봇의 목표 접수 응답을 기다리고 있습니다.' :
        status === 'REJECTED' ? '다른 위치를 선택해 주세요.' :
        (status === 'UNKNOWN' && s.tf_error ? s.tf_error : details[status] || '');
      if (connected && localized() && s.localization.tracking_warning) {
        $('nav-detail').textContent += ` 위치 추적 경고: ${s.localization.tracking_warning}`;
      }
    }
    $('nav-status-icon').className = 'nav-status-icon ' +
      (status === 'SUCCEEDED' ? 'success' : ['FAILED','REJECTED'].includes(status) ? 'error' : '');
    const robot = s.robot, poseValid = connected && robot && robot.age < 2 && !s.tf_error && localized();
    $('pose-state').textContent = poseValid ? '확인됨' : robot ? '확인 중' : '대기';
    $('robot-position').textContent = robot ? `${robot.x.toFixed(2)} / ${robot.y.toFixed(2)} m` : '— / — m';
    $('robot-heading').textContent = robot ? `${Math.round(robot.yaw*180/Math.PI)} °` : '— °';
    $('robot-speed').textContent = connected && s.velocity?.age < 2 ? `${s.velocity.linear.toFixed(2)} m/s` : '— m/s';
    $('remaining').textContent = connected && goal?.status === 'ACCEPTED' && Number.isFinite(goal.distance_remaining) ?
      `${goal.distance_remaining.toFixed(1)} m` : '— m';
    $('layer-scan').textContent = connected && s.scan?.age < 1 ? '수신 중' : s.scan ? '지연' : '대기';
    if (goal && selected && Math.hypot(selected.x-goal.x,selected.y-goal.y)<.01) {
      $('goal-label').textContent = {SENDING:'전송 중', ACCEPTED:'접수됨', SUCCEEDED:'도착', CANCELED:'취소됨',
        CANCELING:'취소 중', FAILED:'실패', REJECTED:'거절됨', PREVIEW:'미리보기'}[goal.status] || '선택됨';
    }
    updateFloorUi();
    updateEstop();
    window.AMRDashboard?.update(s, connected, connectionIssue);
    updateControls();
  }
  function updateEstop() {
    const e = state?.estop, banner = $('estop-banner');
    // Shown only when the bridge interlock is required and not READY (or bypassed).
    const visible = Boolean(connected && !state?.preview && e?.required && !(e.healthy && estopLive()));
    banner.hidden = !visible;
    if (!visible) return;
    const live = estopLive(), lost = ['status_timeout', 'invalid_status'].includes(e.reason);
    const [tone, title, detail] = !live ? ['warn', 'E-stop 상태 수신 없음',
        '로봇의 E-stop 상태를 받지 못했습니다. 상태가 복구될 때까지 주행할 수 없습니다.'] :
      e.state === 'ESTOP' ? ['danger', lost ? 'E-stop 상태 수신 끊김 · 정지' : 'E-stop 눌림',
        lost ? 'STM32 상태가 끊겨 정지했습니다. CAN 연결을 확인하세요.' : '버튼을 원위치로 돌리면 자동으로 주행 준비를 합니다.'] :
      e.state === 'ARMING' ? ['info', '주행 준비 중', 'E-stop이 해제되었습니다. STM32 주행 준비를 기다리고 있습니다. 새 목적지나 조작으로만 움직입니다.'] :
      ['warn', 'E-stop 상태 확인 중', e.detail || 'STM32 E-stop 상태를 기다리고 있습니다.'];
    banner.className = `estop-banner ${tone}`;
    $('estop-title').textContent = title;
    $('estop-detail').textContent = detail;
  }
  function updateFloorUi() {
    const current = currentFloor();
    if (current && current !== shownCurrentFloor) { viewFloor = current; shownCurrentFloor = current; renderFloorTabs(); }
    $('floor-tabs').hidden = $('room-select').hidden = !floorsEnabled();
    if (floorsEnabled()) {
      $('floor-eyebrow').textContent = inTransit() ? `G-BUILDING · ${state.floor.transit_from} → ${state.floor.transit_to} 이동 중` :
        `G-BUILDING · ${current || '층 확인 중'}`;
      $('map-floor').textContent = viewFloor ? `${viewFloor}${viewingOther() ? ' (다른 층 · 목적지 선택용)' : ''}` : '층 확인 중';
      document.querySelectorAll('#floor-tabs button').forEach(b => {
        b.classList.toggle('active', b.dataset.floor === viewFloor);
        b.classList.toggle('current', b.dataset.floor === current);
        b.setAttribute('aria-pressed', String(b.dataset.floor === viewFloor));
      });
    }
    const mission = state?.floor_mission, line = $('floor-mission');
    const visible = mission && mission.phase && mission.phase !== 'IDLE';
    line.hidden = $('floor-mission-link').hidden = !visible;
    if (visible) {
      const route = mission.start_floor && mission.goal_floor ? `${mission.start_floor} → ${mission.goal_floor}${mission.goal_room ? ' ' + roomLabel(mission.goal_room) : ''} · ` : '';
      const press = state.elevator?.last_command;
      const pressNote = press && press.age < 10 && rfCommandLabels[press.status] &&
        !['SUCCEEDED', 'FAILED_SAFE', 'CANCELED'].includes(mission.phase) ? ` · ${rfCommandLabels[press.status]}` : '';
      line.textContent = `층간 이동: ${route}${missionLabels[mission.phase] || mission.phase}` +
        (state.floor?.zone === 2 ? ' · 엘리베이터 안' : '') + pressNote +
        (mission.detail && mission.phase === 'FAILED_SAFE' ? ` (${mission.detail})` : '');
    }
  }
  function renderFloorTabs() {
    const tabs = $('floor-tabs'), select = $('room-select');
    const floors = floorsInfo?.floors || [];
    tabs.hidden = select.hidden = !floorsEnabled();
    tabs.replaceChildren(...floors.map(f => {
      const b = document.createElement('button');
      b.type = 'button'; b.dataset.floor = f.name; b.textContent = f.name;
      b.title = f.selectable ? (f.provisional ? '임시 지도(4층 복사본)' : '') : '실차 층간 이동 비활성';
      b.disabled = !f.selectable && f.name !== currentFloor();
      b.addEventListener('click', () => { viewFloor = f.name; renderFloorTabs(); fitted = false; fit(); updateSidebar(); });
      return b;
    }));
    const rooms = floorInfo(viewFloor)?.rooms || [];
    select.replaceChildren(new Option('호실 선택', ''), ...rooms.map(r => new Option(`${viewFloor} ${roomLabel(r.id)}`, r.id)));
    $('set-floor').replaceChildren(new Option('현재 층 변경', ''), ...floors.map(f => new Option(f.name, f.name)));
    if (state) updateFloorUi();
  }
  async function fetchFloors() {
    if (document.hidden) return;
    try {
      const response = await fetch('/api/floors', {signal:AbortSignal.timeout(4000)});
      if (response.ok) {
        floorsInfo = await response.json();
        if (!viewFloor) viewFloor = floorsInfo.current_floor || floorsInfo.default_floor;
        renderFloorTabs();
        for (const f of floorsInfo.floors) if (f.map) fetchFloorLayer(f.name, f.map);
      }
    } catch (_error) { /* floors are optional */ }
  }
  async function fetchFloorLayer(name, info) {
    if (document.hidden) return;
    if (floorImages[name]?.version === info.version || floorLoading[name]) return;
    floorLoading[name] = true;
    try {
      const response = await fetch(`/api/floors/${encodeURIComponent(name)}/map.png?v=${info.version}`, {signal:AbortSignal.timeout(4000)});
      if (!response.ok) return;
      const bitmap = await createImageBitmap(await response.blob());
      floorImages[name]?.bitmap.close();
      floorImages[name] = {bitmap, version:Number(response.headers.get('X-Map-Version'))};
      requestDraw();
    } catch (_error) { /* keep the previous image */ }
    finally { floorLoading[name] = false; }
  }
  async function fetchLayer(name, info) {
    if (document.hidden) return;
    if (images[name]?.version === info.version || loading[name]) return;
    loading[name] = true;
    try {
      const response = await fetch(`/api/layers/${name}.png?v=${info.version}`, {signal:AbortSignal.timeout(4000)});
      if (!response.ok) return;
      const version = Number(response.headers.get('X-Map-Version'));
      const bitmap = await createImageBitmap(await response.blob());
      if (state?.layers[name]?.version !== version) { bitmap.close(); return; }
      images[name]?.bitmap.close();
      images[name] = {bitmap, version};
      requestDraw();
      if (name === 'map' && !fitted && canvas.clientWidth && canvas.clientHeight) { fit(); fitted = true; }
    } catch (_error) {
      // Retain the prior image; no coordinate commands use an outdated map image.
    } finally { loading[name] = false; }
  }
  async function poll() {
    if (document.hidden || polling) return;
    clearTimeout(pollTimer);
    polling = true;
    try {
      const response = await fetch('/api/state', {signal:AbortSignal.timeout(2500)});
      if (!response.ok) {
        const error = new Error('상태 조회 실패'); error.status = response.status; throw error;
      }
      const previous = connected;
      state = await response.json(); connected = true; connectionIssue = ''; lastStateAt = performance.now();
      // Floor list and other floors' maps change rarely: refresh about every 10 s with the state poll.
      if (floorPolls++ % 50 === 0) fetchFloors();
      if (localizationRequest && (state.pending_request?.request_id === localizationRequest ||
          state.notice?.request_id === localizationRequest)) localizationRequest = null;
      for (const [name, info] of Object.entries(state.layers)) {
        if (name === 'map' || layers[name]) fetchLayer(name, info);
      }
      $('empty').hidden = Boolean(images.map);
      const info = state.layers.map;
      if (info) $('map-meta').textContent = `${info.width} × ${info.height} · ${info.resolution} m/cell`;
      if (state.scan) {
        const intensities = state.scan.points.map(p=>p[2]).filter(Number.isFinite);
        const bounds = intensities.length ? [Math.min(...intensities), Math.max(...intensities)] : null;
        state.scan.buckets = Array.from({length:33}, () => []);
        for (const point of state.scan.points) {
          const value = bounds && point[2] !== null ? (point[2]-bounds[0])/(bounds[1]-bounds[0] || 1) : null;
          const bucket = value === null ? 32 : Math.max(0, Math.min(31, Math.round(value*31)));
          state.scan.buckets[bucket].push(point);
        }
      }
      const notice = state.notice;
      const key = notice ? JSON.stringify(notice) : '';
      if (notice && key !== lastNotice && notice.request_id === lastRequest) {
        lastNotice = key;
        activity(notice.detail || {SUCCEEDED:'목적지에 도착했습니다.', CANCELED:'주행 목표가 취소되었습니다.',
          FAILED:'이동하지 못했습니다. 상태를 확인해 주세요.'}[notice.status] || '상태가 갱신되었습니다.',
          ['FAILED','REJECTED'].includes(notice.status));
      } else if (!previous) {
        activity(state.preview ? '미리보기입니다. 지도 터치는 차량에 명령을 보내지 않습니다.' :
          '웹이 연결되었습니다. 지도에서 목적지를 선택하세요.');
        if (state.goal && !selected) { setSelected({...state.goal, map_version:info?.version}); lastRequest = state.goal.request_id; }
      }
      updateSidebar();
    } catch (error) {
      connectionIssue = error.status === 503 ? 'AMR 상태 지연' : error.status === 401 ? '인증 필요' :
        lastStateAt && performance.now()-lastStateAt < 5000 ? '웹 응답 지연' : '연결 끊김';
      if (connected) activity(error.status === 503 ? '서버는 응답하고 있지만 AMR 상태가 지연되었습니다.' :
        '상태 응답을 기다립니다. 목표를 자동으로 다시 전송하지 않습니다.', true);
      connected = false;
      if (state) updateSidebar(); else {
        $('connection-label').textContent = connectionIssue || '연결 대기';
        $('nav-status').textContent = connectionIssue || '연결 대기';
        window.AMRDashboard?.update(null, false, connectionIssue);
      }
      updateControls();
    } finally {
      polling = false;
      requestDraw();
      if (!document.hidden) pollTimer = setTimeout(poll, 200);
    }
  }
  function requestId() {
    return crypto.randomUUID ? crypto.randomUUID() : `web_${Date.now()}_${Math.random().toString(36).slice(2)}`;
  }
  async function command(kind, pose) {
    const blocked = commandError(kind);
    if (blocked) { activity(blocked, true); return; }
    if (['goal','initialpose'].includes(kind) && (!activeMap().image || activeMap().info.version !== pose?.map_version)) {
      activity('지도를 갱신하고 있습니다. 잠시 후 다시 선택해 주세요.', true); return;
    }
    const request_id = requestId(); lastRequest = request_id;
    if (!state.preview && ['initialpose','relocalize'].includes(kind)) localizationRequest = request_id;
    busy = true; updateControls();
    activity(kind === 'cancel' ? '목표 취소를 요청하고 있습니다.' : kind === 'relocalize' ?
      '자동 위치 찾기를 요청하고 있습니다.' : kind === 'set_floor' ? '현재 층 변경을 요청하고 있습니다.' :
      '선택한 위치를 전송하고 있습니다.');
    try {
      const response = await fetch(`/api/${kind}`, {method:'POST',
        headers:{'Content-Type':'application/json', 'X-AMR-Request':'1'},
        body:JSON.stringify({request_id, ...(kind === 'relocalize' ? {map_version:state.layers.map?.version} :
          kind === 'goal' && floorsEnabled() ? {...pose, floor:viewFloor, room:pose.room || ''} :
          // The server accepts floor/room only on goals; a tapped pose carries the viewed floor.
          ['goal','initialpose'].includes(kind) ? {x:pose.x, y:pose.y, yaw:pose.yaw, map_version:pose.map_version} : pose || {})}),
        signal:AbortSignal.timeout(5000)});
      const result = await response.json();
      if (!response.ok) {
        if (localizationRequest === request_id) localizationRequest = null;
        throw new Error(result.error || '요청 처리 실패');
      }
      activity(result.status === 'PREVIEW' ? '목표 미리보기입니다. 실제 주행 명령은 전송하지 않았습니다.' :
        kind === 'initialpose' ? '초기 위치 전송을 요청했습니다.' :
        kind === 'relocalize' ? '자동 위치 찾기를 요청했습니다. 위치 확인 완료를 기다려 주세요.' :
        kind === 'cancel' ? '취소 요청을 접수했습니다. 완료 응답을 기다립니다.' :
        kind === 'set_floor' ? '현재 층 변경을 요청했습니다. 층 표시가 바뀌는지 확인하세요.' : '목표를 전송했습니다. 로봇의 접수 응답을 기다립니다.');
      if (kind === 'initialpose' || (kind === 'cancel' && state.preview)) {
        selected=null; $('goal-x').textContent='—'; $('goal-y').textContent='—';
        $('goal-label').textContent='선택 전'; setMode('goal');
      }
    } catch (error) {
      activity(error.name === 'TimeoutError' ? '응답이 지연되고 있습니다. 현재 상태를 확인해 주세요.' : error.message, true);
    } finally { busy = false; updateControls(); requestDraw(); }
  }
  function fit() {
    requestDraw();
    const info = activeMap().info;
    if (!info) return;
    const corners = [[0,0],[info.width,0],[0,info.height],[info.width,info.height]]
      .map(p => G.gridToWorld(...p, info));
    const xs = corners.map(p=>p.x), ys = corners.map(p=>p.y);
    const minX=Math.min(...xs), maxX=Math.max(...xs), minY=Math.min(...ys), maxY=Math.max(...ys);
    camera.x = (minX+maxX)/2; camera.y = (minY+maxY)/2;
    camera.scale = Math.max(1, Math.min((width-64)/(maxX-minX), (height-100)/(maxY-minY)));
    follow = false; $('follow').classList.remove('active'); $('follow').setAttribute('aria-pressed','false');
  }
  function zoom(factor, anchor={x:width/2,y:height/2}) {
    requestDraw();
    const before = world(anchor);
    camera.scale = Math.max(2, Math.min(350, camera.scale*factor));
    const after = world(anchor); camera.x += before.x-after.x; camera.y += before.y-after.y;
  }
  function drawImage(name, opacity=1) {
    const info = state?.layers[name], image = images[name];
    if (!info || !image || image.version !== info.version) return;
    const origin = G.renderOrigin(info);
    if (!origin) return;
    const p = screen({x:origin[0],y:origin[1]});
    ctx.save(); ctx.translate(p.x,p.y); ctx.rotate(-origin[2]);
    const scale = camera.scale*info.resolution;
    ctx.scale(scale,scale); ctx.globalAlpha=opacity; ctx.imageSmoothingEnabled=false;
    ctx.drawImage(image.bitmap,0,-info.height,info.width,info.height); ctx.restore();
  }
  function line(points, color, thickness=2, close=false, alpha=1) {
    if (!points?.length) return;
    ctx.save(); ctx.strokeStyle=color; ctx.lineWidth=thickness; ctx.globalAlpha=alpha; ctx.lineJoin='round';
    ctx.beginPath(); points.forEach((point,i) => { const p=screen({x:point[0],y:point[1]}); i?ctx.lineTo(p.x,p.y):ctx.moveTo(p.x,p.y); });
    if (close) ctx.closePath(); ctx.stroke(); ctx.restore();
  }
  function arrow(pose, color, radius=11, halo=false) {
    const p=screen(pose); ctx.save(); ctx.translate(p.x,p.y); ctx.rotate(-pose.yaw);
    if (halo) { ctx.beginPath(); ctx.arc(0,0,radius*2.2,0,Math.PI*2); ctx.fillStyle='#2563eb15';ctx.fill(); }
    ctx.beginPath();ctx.moveTo(radius*1.4,0);ctx.lineTo(-radius,-radius*.8);ctx.lineTo(-radius*.55,0);ctx.lineTo(-radius,radius*.8);ctx.closePath();
    ctx.fillStyle=color;ctx.fill();ctx.strokeStyle='white';ctx.lineWidth=2;ctx.stroke();ctx.restore();
  }
  function goalMarker(pose, color, draft=false) {
    const p=screen(pose);ctx.save();ctx.translate(p.x,p.y);ctx.strokeStyle=color;ctx.lineWidth=2;
    ctx.setLineDash(draft?[4,3]:[]);ctx.beginPath();ctx.arc(0,0,12,0,Math.PI*2);ctx.stroke();ctx.setLineDash([]);
    ctx.beginPath();ctx.arc(0,0,3,0,Math.PI*2);ctx.fillStyle=color;ctx.fill();
    ctx.rotate(-pose.yaw);ctx.beginPath();ctx.moveTo(14,0);ctx.lineTo(33,0);ctx.lineTo(27,-4);ctx.moveTo(33,0);ctx.lineTo(27,4);ctx.stroke();ctx.restore();
  }
  function uncertainty(pose) {
    const [a,b,c,d]=pose.covariance, cross=(b+c)/2;
    const delta=Math.sqrt((a-d)*(a-d)+4*cross*cross);
    const major=Math.sqrt(Math.max(0,(a+d+delta)/2)), minor=Math.sqrt(Math.max(0,(a+d-delta)/2));
    const p=screen(pose), angle=.5*Math.atan2(2*cross,a-d);
    ctx.save();ctx.translate(p.x,p.y);ctx.rotate(-angle);ctx.beginPath();
    ctx.ellipse(0,0,major*camera.scale,minor*camera.scale,0,0,Math.PI*2);
    ctx.fillStyle='#ffb40040';ctx.fill();ctx.strokeStyle='#ffb40090';ctx.lineWidth=1;ctx.stroke();ctx.restore();
  }
  function draw() {
    drawFrame = null;
    if (document.hidden) return;
    const ratio=Math.min(devicePixelRatio||1,2), rect=canvas.getBoundingClientRect();
    if (!rect.width || !rect.height) return;
    const resized = Math.abs(width-rect.width) > 1;
    width=rect.width;height=rect.height;
    if(canvas.width!==Math.round(width*ratio)||canvas.height!==Math.round(height*ratio)) {
      canvas.width=Math.round(width*ratio);canvas.height=Math.round(height*ratio);
      if(!fitted&&state?.layers.map&&images.map){fit();fitted=true;}
    }
    if(resized&&fitted&&!follow)fit();
    ctx.setTransform(ratio,0,0,ratio,0,0);ctx.clearRect(0,0,width,height);
    if(follow&&connected&&state?.robot?.age<2){camera.x=state.robot.x;camera.y=state.robot.y;}
    const gridStep=camera.scale>=8?1:5;
    ctx.strokeStyle='#dfe3e945';ctx.lineWidth=1;ctx.beginPath();
    for(let x=Math.floor((camera.x-width/2/camera.scale)/gridStep)*gridStep;x<camera.x+width/2/camera.scale;x+=gridStep){let p=screen({x,y:0});ctx.moveTo(p.x,0);ctx.lineTo(p.x,height);}
    for(let y=Math.floor((camera.y-height/2/camera.scale)/gridStep)*gridStep;y<camera.y+height/2/camera.scale;y+=gridStep){let p=screen({x:0,y});ctx.moveTo(0,p.y);ctx.lineTo(width,p.y);}
    ctx.stroke();
    if(viewingOther()){const {info,image}=activeMap();if(info&&image){const p=screen({x:info.origin[0],y:info.origin[1]});
      ctx.save();ctx.translate(p.x,p.y);ctx.rotate(-info.origin[2]);const sc=camera.scale*info.resolution;ctx.scale(sc,sc);
      ctx.imageSmoothingEnabled=false;ctx.drawImage(image.bitmap,0,-info.height,info.width,info.height);ctx.restore();}}
    else drawImage('map');
    if(!viewingOther()&&layers.global_costmap)drawImage('global_costmap',.3);
    if(!viewingOther()&&layers.local_costmap)drawImage('local_costmap',.35);
    if(state&&!viewingOther()){
      const alpha=connected?1:.25;
      if(layers.global_path)line(state.global_path?.points,'#ff3737',2.3,false,alpha);
      if(layers.local_path)line(state.local_path?.points,'#263cff',2.5,false,alpha);
      if(layers.scan&&connected&&state.scan?.age<1){
        state.scan.buckets.forEach((points, bucket) => {
          if (!points.length) return;
          ctx.fillStyle=scanColors[bucket];ctx.beginPath();
          for(const [x,y] of points){const p=screen({x,y});
            if(p.x<0||p.x>width||p.y<0||p.y>height)continue;
            ctx.rect(p.x-1.5,p.y-1.5,3,3);
          }
          ctx.fill();
        });
      }
      if(layers.footprint)line(state.footprint?.points,'#29dc13',2,true,alpha);
      if(layers.amcl&&state.amcl){ctx.globalAlpha=alpha;uncertainty(state.amcl);arrow(state.amcl,'#ffb400',9);ctx.globalAlpha=1;}
      if(state.robot){ctx.globalAlpha=connected&&state.robot.age<2?1:.3;arrow(state.robot,'#2563eb',11,true);ctx.globalAlpha=1;}
    }
    if(state?.goal&&state.goal.kind!=='initialpose'&&!['CANCELED','FAILED','REJECTED'].includes(state.goal.status)&&
       (!state.goal.floor||state.goal.floor===(viewingOther()?viewFloor:currentFloor())))goalMarker(state.goal,'#2563eb');
    if(gesture?.kind==='pose')goalMarker(gesture.pose,mode==='initialpose'?'#ffb400':'#2563eb',true);
    else if(selected&&(!state?.goal||Math.hypot(selected.x-state.goal.x,selected.y-state.goal.y)>.01))goalMarker(selected,'#2563eb',true);
    const wanted=90/camera.scale, magnitude=10**Math.floor(Math.log10(wanted));
    const length=[1,2,5,10].map(v=>v*magnitude).find(v=>v>=wanted)||10*magnitude;
    $('scale-line').style.width=`${length*camera.scale}px`;$('scale-label').textContent=`${length} m`;
  }

  canvas.addEventListener('pointerdown', event=>{
    if(event.button!==0)return;
    requestDraw();
    const p=local(event);pointers.set(event.pointerId,p);canvas.setPointerCapture(event.pointerId);
    if(pointers.size>1){multiTouch=true;gesture=null;const [a,b]=[...pointers.values()];
      pinch={distance:Math.hypot(a.x-b.x,a.y-b.y),center:{x:(a.x+b.x)/2,y:(a.y+b.y)/2}};return;}
    if(multiTouch)return;
    if(mode==='pan'){gesture={kind:'pan',start:p,camera:{...camera}};stage.classList.add('dragging');return;}
    const active=activeMap();
    if(!connected||busy||!active.info||!active.image)return;
    const point=world(p), robot=viewingOther()?null:state.robot;
    const heading=headingOverride?Number($('heading').value)*Math.PI/180:
      mode==='goal'&&robot?G.heading(robot,point):Number($('heading').value)*Math.PI/180;
    gesture={kind:'pose',start:p,pose:{...point,yaw:heading,map_version:active.info.version,floor:viewFloor||undefined}};
  });
  canvas.addEventListener('pointermove',event=>{
    const p=local(event), position=world(p);$('cursor-coordinates').textContent=`X ${position.x.toFixed(2)} · Y ${position.y.toFixed(2)}`;
    if(!pointers.has(event.pointerId))return;pointers.set(event.pointerId,p);
    requestDraw();
    if(multiTouch&&pointers.size>=2){const [a,b]=[...pointers.values()],center={x:(a.x+b.x)/2,y:(a.y+b.y)/2},distance=Math.hypot(a.x-b.x,a.y-b.y);
      if(pinch?.distance>0){zoom(distance/pinch.distance,pinch.center);camera.x-=(center.x-pinch.center.x)/camera.scale;camera.y+=(center.y-pinch.center.y)/camera.scale;}
      pinch={distance,center};return;}
    if(!gesture)return;
    if(gesture.kind==='pan'){camera.x=gesture.camera.x-(p.x-gesture.start.x)/camera.scale;camera.y=gesture.camera.y+(p.y-gesture.start.y)/camera.scale;
      follow=false;$('follow').classList.remove('active');$('follow').setAttribute('aria-pressed','false');}
    else if(Math.hypot(p.x-gesture.start.x,p.y-gesture.start.y)>8)gesture.pose.yaw=G.heading(gesture.pose,world(p));
  });
  canvas.addEventListener('pointerup',event=>{
    requestDraw();
    pointers.delete(event.pointerId);stage.classList.remove('dragging');
    if(multiTouch){if(!pointers.size){multiTouch=false;pinch=null;}gesture=null;return;}
    const current=gesture;gesture=null;
    if(current?.kind==='pose'){setSelected(current.pose);command(mode==='initialpose'?'initialpose':'goal',current.pose);}
  });
  canvas.addEventListener('pointercancel',event=>{requestDraw();pointers.delete(event.pointerId);gesture=null;pinch=null;multiTouch=pointers.size>0;stage.classList.remove('dragging');});
  canvas.addEventListener('wheel',event=>{event.preventDefault();zoom(Math.exp(-event.deltaY*.001),local(event));},{passive:false});
  canvas.addEventListener('keydown',event=>{
    if(event.key==='Escape'){gesture=null;setMode('pan');return;}
    if(event.key==='Enter'&&selected&&mode!=='pan'&&!$('send').disabled){event.preventDefault();command(mode==='initialpose'?'initialpose':'goal',selected);return;}
    const movements={ArrowLeft:[-1,0],ArrowRight:[1,0],ArrowUp:[0,1],ArrowDown:[0,-1]};
    if(movements[event.key]&&activeMap().info){event.preventDefault();const p=selected||{x:camera.x,y:camera.y,yaw:0};const [dx,dy]=movements[event.key];
      setSelected({...p,x:p.x+dx*.25,y:p.y+dy*.25,map_version:activeMap().info.version,room:undefined});}
  });
  document.querySelectorAll('[data-mode]').forEach(e=>e.addEventListener('click',()=>setMode(e.dataset.mode)));
  document.querySelectorAll('[data-layer]').forEach(e=>e.addEventListener('change',()=>{layers[e.dataset.layer]=e.checked;requestDraw();}));
  $('heading').addEventListener('input',()=>{requestDraw();headingOverride=true;if(selected)selected.yaw=Number($('heading').value)*Math.PI/180;});
  $('send').addEventListener('click',()=>{if(selected)command(mode==='initialpose'?'initialpose':'goal',selected);});
  $('cancel').addEventListener('click',()=>command('cancel'));
  $('relocalize').addEventListener('click',()=>command('relocalize'));
  // Operator override when the floor is wrong (e.g. after a mission ended inside the car).
  $('set-floor').addEventListener('change',event=>{const floor=event.target.value;event.target.value='';
    if(floor&&confirm(`현재 층을 ${floor}(으)로 바꿀까요? 실제 층과 다를 때만 바꾸세요.`))command('set_floor',{floor});});
  // Only previews the saved pose; the operator still confirms with '초기 위치 적용'.
  $('last-pose').addEventListener('click',()=>{const last=state?.localization?.last_pose,info=activeMap().info;
    if(!last||!info)return;setMode('initialpose');setSelected({x:last.x,y:last.y,yaw:last.yaw,map_version:info.version});
    activity('마지막으로 확인된 위치를 표시했습니다. 지도에서 로봇 위치와 맞으면 ‘초기 위치 적용’을 누르세요.');});
  $('fit').addEventListener('click',fit);$('zoom-in').addEventListener('click',()=>zoom(1.3));$('zoom-out').addEventListener('click',()=>zoom(1/1.3));
  $('follow').addEventListener('click',()=>{requestDraw();follow=!follow;$('follow').classList.toggle('active',follow);$('follow').setAttribute('aria-pressed',String(follow));});
  $('select-goal')?.addEventListener('click',()=>{setMode('goal');canvas.focus();});
  $('room-select').addEventListener('change',event=>{
    const room=(floorInfo(viewFloor)?.rooms||[]).find(r=>r.id===event.target.value);event.target.value='';
    const info=activeMap().info;if(!room||!info)return;
    setMode('goal');const pose={x:room.x,y:room.y,yaw:room.yaw,map_version:info.version,floor:viewFloor,room:room.id};
    setSelected(pose);command('goal',pose);
  });
  new ResizeObserver(requestDraw).observe(canvas);
  window.addEventListener('resize', requestDraw);
  document.addEventListener('visibilitychange', () => {
    clearTimeout(pollTimer);
    if (document.hidden) {
      if (drawFrame !== null) cancelAnimationFrame(drawFrame);
      drawFrame = null;
    } else {
      requestDraw();
      poll();
    }
  });
  requestDraw();poll();
})();
