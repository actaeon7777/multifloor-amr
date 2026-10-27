/* Floor transit view: elevator mission steps, door, beacon signals and event log. Read-only. */
(() => {
  'use strict';
  const $ = id => document.getElementById(id);
  const text = (id, value) => { const e = $(id); if (e && e.textContent !== String(value)) e.textContent = value; };
  const badge = (id, value, tone='') => { text(id, value); const name = `health-badge ${tone}`; if ($(id).className !== name) $(id).className = name; };
  const seconds = value => !Number.isFinite(value) ? '—' : value < 60 ? `${Math.floor(value)}초` :
    `${Math.floor(value/60)}분 ${String(Math.floor(value%60)).padStart(2, '0')}초`;
  const el = (tag, className, content) => {
    const e = document.createElement(tag);
    if (className) e.className = className;
    if (content != null) e.textContent = content;
    return e;
  };

  // Shared with app.js (main view mission line).
  const labels = window.AMRMissionLabels = {NAV_TO_HALL:'엘리베이터 홀로 이동', CALL_ELEVATOR:'엘리베이터 호출', WAIT_DOOR_OPEN:'문 열림 대기',
    BOARDING:'탑승 중', BACKING_OUT:'탑승 중단 · 홀로 복귀', IN_CAR_ALIGN:'엘리베이터 안 정렬', SELECT_FLOOR:'층 버튼 선택',
    RIDING:'층 이동 중', SWITCH_MAP:'지도 전환', ALIGHTING:'하차 중', RETURN_TO_CAR:'하차 중단 · 카 안 복귀',
    RELOCALIZE:'위치 재확인', NAV_TO_GOAL:'목적지로 이동', SUCCEEDED:'층간 이동 완료', FAILED_SAFE:'안전 정지',
    CANCELED:'층간 이동 취소', IDLE:'대기'};
  // Mission order (amr_elevator mission_node ACTIVE); branch phases belong to their parent step.
  const steps = [
    ['NAV_TO_HALL', '홀 대기 위치로 이동', '엘리베이터 문 앞 대기 위치까지 주행하고 정렬합니다.'],
    ['CALL_ELEVATOR', '호출 버튼 누르기', '홀 비콘이 올라가기·내려가기 버튼을 누릅니다.'],
    ['WAIT_DOOR_OPEN', '도착 · 문 열림 감지', 'LiDAR로 엘리베이터 문이 열렸는지 판단합니다.'],
    ['BOARDING', '탑승', '열린 문을 통과해 카 안으로 들어갑니다.', 'BACKING_OUT'],
    ['IN_CAR_ALIGN', '카 안 정렬', '카 안에서 문 쪽을 향하도록 회전합니다.'],
    ['SELECT_FLOOR', '목적층 버튼 누르기', '카 비콘이 목적층 버튼을 누릅니다.'],
    ['RIDING', '층 이동', '목적층 도착과 문 열림을 기다립니다.'],
    ['SWITCH_MAP', '도착층 지도 전환', '목적층 지도를 불러옵니다.'],
    ['ALIGHTING', '하차', '문 열림을 유지하며 홀로 나갑니다.', 'RETURN_TO_CAR'],
    ['RELOCALIZE', '위치 재확인', '도착층에서 위치추정을 다시 검증합니다.'],
    ['NAV_TO_GOAL', '목적지로 이동', '목적지까지 주행합니다.'],
  ];
  const stepOf = phase => steps.findIndex(([name, , , branch]) => name === phase || branch === phase);
  const terminal = new Set(['SUCCEEDED', 'FAILED_SAFE', 'CANCELED']);
  const doorLabels = {0:'판단 불가', 1:'닫힘', 2:'열리는 중', 3:'열림', 4:'닫히는 중'};
  const zoneLabels = {0:'구역 미확인', 1:'엘리베이터 밖', 2:'엘리베이터 안'};
  const confidenceLabels = {CONFIRMED:'확정', BEACON:'비콘 확인', TRANSIT:'이동 중', ASSUMED:'추정', UNKNOWN:'미확인'};
  const localizationLabels = {SUSPENDED:'엘리베이터 구간 · 위치추정 중지', WAITING_DATA:'센서 대기',
    WAITING_INITIAL_POSE:'초기 위치 전달 대기', VERIFYING:'위치 검증 중', SEARCHING:'자동 위치 찾는 중',
    STARTING_NAVIGATION:'주행 준비 중', READY:'위치 확인 완료', FAILED:'위치 확인 실패'};
  const statusLabels = {DELIVERED:'RF 전달됨', COMPLETED:'눌림 결과 수신', DEVICE_REJECTED:'비콘이 거절',
    RF_TIMEOUT:'무선 전달 실패', NO_RESULT:'결과 없음', NO_ROUTE:'대상 비콘 없음', BAD_FRAME:'명령 형식 오류',
    REPLACED:'새 명령으로 대체'};
  const statusTone = {DELIVERED:'', COMPLETED:'good', DEVICE_REJECTED:'bad', NO_ROUTE:'bad', BAD_FRAME:'bad',
    RF_TIMEOUT:'warn', NO_RESULT:'warn', REPLACED:''};
  const floorEvents = {ENTERED_ELEVATOR:'엘리베이터 진입 감지', EXITED_ELEVATOR:'엘리베이터 밖으로 나옴',
    DEPARTED:'카 출발 감지', ARRIVED:'층 도착 감지', FLOOR_CONFIRMED:'현재 층 확정', RULE_VIOLATION:'층 판단 규칙 위반'};
  const rfFlags = {BATT_LOW:['배터리 부족', 'warn'], BUTTON_FAULT:['버튼 고장', 'bad'], CONFIG_ERR:['설정 오류', 'bad'],
    BOOT:['재부팅 직후', '']};

  let beacons = [], stepsSignature = '', beaconSignature = '', eventSignature = '';
  // Hall beacon floor_code -> floors.yaml name (the beacon rows carry both).
  const floorName = code => beacons.find(b => b.role === 'hall' && b.floor_code === code)?.label ||
    (code > 0 ? `${code}F` : `B${-code}`);
  const commandName = c => c.command === 1 ? `${floorName(c.floor_code)} 홀 ${c.direction === 1 ? '▲ 올라가기' : c.direction === 2 ? '▼ 내려가기' : ''} 버튼` :
    c.command === 2 ? `카 안 ${floorName(c.floor_code)} 버튼` : c.command === 3 ? `문열림 버튼${c.repeat > 1 ? ` ${c.repeat}회 반복` : ''}` : `명령 ${c.command}`;
  // RSSI: the real relay only grades -60 (strong) / -80 (weak); simulation sends dBm; 0 is unknown.
  const grade = rssi => rssi == null ? 'none' : rssi === 0 ? 'heard' : rssi >= -70 ? 'strong' : 'weak';
  const rssiLabel = rssi => rssi == null ? '미수신' : rssi === 0 ? '세기 미상' : rssi === -60 ? '강' : rssi === -80 ? '약' : `${rssi} dBm`;

  function buttonProgress(command, kind) {
    if (!command || command.command !== kind) return null;
    const delivered = command.ack_age != null || ['DELIVERED', 'COMPLETED'].includes(command.status);
    const pressed = command.ack_age != null;
    const failed = ['DEVICE_REJECTED', 'NO_ROUTE', 'BAD_FRAME'].includes(command.status);
    return {
      title: commandName(command) + (command.attempt > 1 ? ` · ${command.attempt}번째 시도` : ''),
      items: [[`전송 #${command.sequence}`, 'done'], ['RF 전달', delivered ? 'done' : failed ? 'bad' : 'wait'],
        ['눌림 확인', pressed ? 'done' : failed ? 'bad' : 'wait']],
      note: command.status && !pressed ? statusLabels[command.status] || command.status : '',
    };
  }

  const waiting = {title: '버튼 명령 전송 대기', items: [], note: ''};
  function stepDetail(phase, t, s) {
    const m = t.mission, c = m.counters || {}, floor = s?.floor, door = t.door;
    const doorText = door ? `${doorLabels[door.state] ?? door.state}${Number.isFinite(door.open_ratio) ? ` · 열림 ${Math.round(door.open_ratio*100)}%` : ''}` : '문 상태 미수신';
    switch (phase) {
      case 'NAV_TO_HALL': return {text: `${m.start_floor || '현재 층'} 엘리베이터 홀 대기 위치로 주행 중`, extra: c.localization_waits ? `위치 회복 대기 ${c.localization_waits}회` : ''};
      case 'CALL_ELEVATOR': return {button: buttonProgress(t.command, 1) || waiting, extra: c.button_retries ? `재전송 ${c.button_retries}회` : ''};
      case 'WAIT_DOOR_OPEN': return {door: true, text: `LiDAR 문 상태: ${doorText}`, extra: c.recalls ? `재호출 ${c.recalls}회` : ''};
      case 'BOARDING': return {door: true, text: `카 안으로 진입 중 · 문 ${doorText}`};
      case 'BACKING_OUT': return {door: true, tone: 'warn', text: `문이 닫혀 홀로 후진 중 · 시도 ${(c.boarding_attempts || 0) + 1}`};
      case 'IN_CAR_ALIGN': return {text: '카 안에서 문 쪽으로 회전 정렬 중'};
      case 'SELECT_FLOOR': return {button: buttonProgress(t.command, 2) || waiting};
      case 'RIDING': {
        const hall = beacons.filter(b => b.role === 'hall' && Number.isFinite(b.presence_age) && b.presence_age < 3)
          .sort((a, b) => a.presence_age - b.presence_age)[0];
        const where = floor?.transit_to ? `${floor.transit_from || '?'} → ${floor.transit_to} 이동 중` :
          floor?.floor ? `${floor.floor} (${confidenceLabels[floor.confidence] || floor.confidence})` : '층 확인 중';
        return {text: `${where} · 목적층 ${m.goal_floor}`, extra: [hall ? `수신 중인 홀 비콘 ${hall.label}` : '홀 비콘 미수신',
          c.car_repress ? `층 버튼 재입력 ${c.car_repress}회` : ''].filter(Boolean).join(' · ')};
      }
      case 'SWITCH_MAP': return {text: `${m.goal_floor} 지도를 불러오는 중`};
      case 'ALIGHTING':
      case 'RETURN_TO_CAR': {
        const hold = t.command?.command === 3 ? `${commandName(t.command)} ${t.command.ack_age != null ? '시작됨' : '응답 대기'}` : '';
        return {door: true, tone: phase === 'RETURN_TO_CAR' ? 'warn' : '', text: phase === 'RETURN_TO_CAR' ?
          '문이 닫혀 카 안으로 복귀 중' : `홀로 나가는 중 · 문 ${doorText}`, extra: hold};
      }
      case 'RELOCALIZE': {
        const state = s?.localization?.state;
        return {text: `위치추정: ${localizationLabels[state] || state || '상태 대기'}`,
          extra: m.phases?.at(-2) === 'BACKING_OUT' ? '홀로 돌아와 위치 확인 후 다시 호출합니다.' : ''};
      }
      case 'NAV_TO_GOAL': return {text: `${m.goal_floor}${m.goal_room ? ` ${m.goal_room}호` : ''} 목적지로 주행 중`};
      default: return {text: ''};
    }
  }

  function renderSteps(t, s) {
    const m = t?.mission, list = $('transit-steps');
    const phase = m?.phase || 'IDLE';
    const visited = new Set(m?.phases || []);
    const active = !terminal.has(phase) && phase !== 'IDLE';
    // After BACKING_OUT the robot relocalizes in the hall: still the boarding step.
    const recovering = phase === 'RELOCALIZE' && m.phases?.at(-2) === 'BACKING_OUT';
    const last = terminal.has(phase) ? [...(m.phases || [])].reverse().find(p => stepOf(p) >= 0) : phase;
    const current = recovering ? stepOf('BOARDING') : stepOf(last);
    const rows = steps.map(([name, title, description, branch], index) => {
      const seen = visited.has(name) || (branch && visited.has(branch));
      let state = 'pending';
      if (phase === 'SUCCEEDED') state = seen ? 'done' : 'skipped';
      else if (index < current) state = seen ? 'done' : 'skipped';
      else if (index === current) state = active ? 'current' : phase === 'FAILED_SAFE' ? 'failed' : phase === 'CANCELED' ? 'canceled' : 'pending';
      const took = (m?.timings?.[name] || 0) + (branch ? m?.timings?.[branch] || 0 : 0);
      const detail = state === 'current' ? stepDetail(recovering ? 'RELOCALIZE' : phase, t, s) : null;
      return {name, title, description, state, took, detail};
    });
    const signature = JSON.stringify([rows.map(r => [r.state, Math.round(r.took), r.detail]), Math.floor(m?.phase_age ?? -1)]);
    if (signature === stepsSignature) return;
    stepsSignature = signature;
    const stateText = {done:'완료', skipped:'건너뜀', current:'진행 중', failed:'여기서 정지', canceled:'여기서 취소', pending:''};
    list.replaceChildren(...rows.map((row, index) => {
      const li = el('li', `transit-step ${row.state}`);
      if (row.state === 'current') li.setAttribute('aria-current', 'step');
      li.append(el('span', 'transit-step-marker', row.state === 'done' ? '✓' : row.state === 'failed' ? '!' : String(index + 1)));
      const body = el('div', 'transit-step-body');
      const head = el('div', 'transit-step-head');
      head.append(el('strong', '', row.title));
      const time = row.state === 'current' ? seconds(m.phase_age) : row.took > 0 ? seconds(row.took) : '';
      head.append(el('span', 'transit-step-state', [stateText[row.state], time].filter(Boolean).join(' · ')));
      body.append(head);
      const d = row.detail;
      if (!d) body.append(el('p', '', row.description));
      else {
        if (d.text) body.append(el('p', `transit-step-text ${d.tone || ''}`, d.text));
        if (d.button) {
          body.append(el('p', 'transit-button-title', d.button.title));
          if (d.button.items.length) {
            const track = el('div', 'transit-button-track');
            for (const [label, state] of d.button.items) track.append(el('span', state, label));
            body.append(track);
          }
          if (d.button.note) body.append(el('p', 'transit-step-text warn', d.button.note));
        }
        if (d.door) {
          const bar = el('div', 'transit-ratio');
          const fill = el('span');
          fill.style.width = `${Math.round((t.door?.open_ratio ?? 0)*100)}%`;
          bar.append(fill);
          body.append(bar);
        }
        if (d.extra) body.append(el('small', '', d.extra));
      }
      li.append(body);
      return li;
    }));
  }

  function renderHeader(t, s, online) {
    const m = t?.mission, phase = m?.phase || 'IDLE';
    const running = !terminal.has(phase) && phase !== 'IDLE';
    text('transit-route', m && phase !== 'IDLE' && m.goal_floor ? `${m.start_floor || '?'} → ${m.goal_floor}${m.goal_room ? ` · ${m.goal_room}호` : ''}` : '진행 중인 층간 이동이 없습니다');
    badge('transit-phase', !online ? '연결 끊김' : labels[phase] || phase,
      phase === 'SUCCEEDED' ? 'good' : phase === 'FAILED_SAFE' ? 'bad' : ['BACKING_OUT', 'RETURN_TO_CAR', 'CANCELED'].includes(phase) ? 'warn' : '');
    text('transit-elapsed', m && phase !== 'IDLE' ? `${running ? '경과' : '소요'} ${seconds(m.elapsed)}` : '—');
    text('transit-detail', phase === 'FAILED_SAFE' ? `오류: ${m.detail || '상세 없음'} · 운영자 확인이 필요합니다.` :
      phase === 'IDLE' ? '다른 층 목적지를 지정하면 이 화면에 진행 단계가 표시됩니다. 비콘과 문 상태는 항상 표시합니다.' :
      running ? `현재 단계: ${labels[phase] || phase}` : m?.detail || '');
    const f = s?.floor;
    text('transit-floor', f ? f.floor || (f.transit_to ? `${f.transit_from || '?'} → ${f.transit_to}` : '확인 중') : '미수신');
    text('transit-zone', f ? zoneLabels[f.zone] ?? '—' : '—');
    text('transit-confidence', f ? confidenceLabels[f.confidence] || f.confidence || '—' : '—');
  }

  function renderDoor(t) {
    const d = t?.door, live = d && d.age < (t.door_timeout ?? 1);
    badge('transit-door-state', !d ? '미수신' : !live ? '수신 지연' : doorLabels[d.state] ?? String(d.state),
      !live ? (d ? 'bad' : '') : d.state === 3 ? 'good' : [2, 4].includes(d.state) ? 'warn' : '');
    const ratio = live && Number.isFinite(d.open_ratio) ? d.open_ratio : null;
    text('transit-door-ratio', ratio == null ? '—' : `${Math.round(ratio*100)}%`);
    $('transit-door-fill').style.width = `${Math.round((ratio ?? 0)*100)}%`;
    text('transit-door-beams', live ? `${d.valid_beams}개` : '—');
    const c = t?.command;
    text('transit-command', !c ? '—' : `${commandName(c)} #${c.sequence} · ${c.ack_age != null ? '눌림 확인' :
      statusLabels[c.status] || '응답 대기'} · ${seconds(c.age)} 전`);
  }

  function renderBeacons(t, s) {
    beacons = t?.beacons || [];
    const here = s?.floor?.floor || s?.floor?.transit_from, target = t?.mission && !terminal.has(t.mission.phase) ? t.mission.goal_floor : '';
    const rows = beacons.map(b => {
      const flags = (b.flags || []).filter(f => rfFlags[f]);
      const heard = b.bins.filter(v => v != null).length;
      return {
        label: b.label, role: b.role === 'car' ? '카' : b.mapped ? '홀' : '홀 · 통과층',
        tags: [b.role === 'car' && s?.floor?.zone === 2 ? '탑승 중' : '', b.role === 'hall' && b.label === here ? '현재 층' : '',
          b.role === 'hall' && b.label === target ? '목적층' : ''].filter(Boolean),
        grade: Number.isFinite(b.presence_age) && b.presence_age < 3 ? grade(b.rssi) : 'none',
        signal: Number.isFinite(b.presence_age) ? `${rssiLabel(b.rssi)} · ${Math.floor(b.presence_age)}초 전` : `${t.window}초간 미수신`,
        bins: b.bins.map(grade), count: `수신 ${heard}/${t.window}초`,
        battery: Number.isInteger(b.battery_mv) && b.battery_mv > 0 ? `${(b.battery_mv/1000).toFixed(2)} V` : '',
        flags: flags.map(f => rfFlags[f]),
      };
    });
    const signature = JSON.stringify(rows);
    if (signature === beaconSignature) return;
    beaconSignature = signature;
    $('transit-beacons').replaceChildren(...rows.map(row => {
      const card = el('article', `beacon-card ${row.grade === 'none' ? 'silent' : ''} ${row.tags.length ? 'highlight' : ''}`);
      const head = el('header');
      const name = el('div');
      name.append(el('strong', '', row.label), el('small', '', row.role));
      const bars = el('span', `signal-bars ${row.grade}`);
      bars.setAttribute('aria-hidden', 'true');
      bars.append(el('i'), el('i'), el('i'));
      head.append(name, bars);
      card.append(head);
      if (row.tags.length) {
        const tags = el('div', 'beacon-tags');
        for (const tag of row.tags) tags.append(el('span', 'health-badge good', tag));
        card.append(tags);
      }
      card.append(el('p', 'beacon-signal', row.signal));
      const timeline = el('div', 'beacon-timeline');
      timeline.setAttribute('role', 'img');
      timeline.setAttribute('aria-label', `최근 ${t.window}초 ${row.count}`);
      for (const value of row.bins) timeline.append(el('i', value));
      card.append(timeline);
      const foot = el('footer');
      foot.append(el('span', '', row.count));
      const health = el('span');
      if (row.battery) health.append(el('span', '', row.battery));
      for (const [label, tone] of row.flags) health.append(el('span', `health-badge ${tone}`, label));
      foot.append(health);
      card.append(foot);
      return card;
    }));
    text('transit-beacon-note', !beacons.length ? '층간 이동 설정이 없습니다.' :
      beacons.some(b => Number.isFinite(b.presence_age)) ? '홀 비콘은 그 층 가까이에서만 수신됩니다. 실차 세기는 강/약 2단계입니다.' :
      '비콘 신호(/elevator/beacon)를 기다리고 있습니다. 실차는 start_elevator_link:=true가 필요합니다.');
  }

  function eventText(e) {
    switch (e.kind) {
      case 'phase': return [`단계: ${labels[e.phase] || e.phase}${e.detail ? ` (${e.detail})` : ''}`,
        e.phase === 'SUCCEEDED' ? 'good' : e.phase === 'FAILED_SAFE' ? 'bad' : ['BACKING_OUT', 'RETURN_TO_CAR'].includes(e.phase) ? 'warn' : 'phase'];
      case 'command': return [`${commandName(e)} 명령 전송 #${e.sequence}${e.attempt > 1 ? ` · 재시도 ${e.attempt - 1}` : ''}`, ''];
      case 'status': return [`버튼 명령 #${e.cmd_seq}: ${statusLabels[e.status] || e.status}` +
        (e.device_result && !['NONE', 'PRESSED'].includes(e.device_result) ? ` (${e.device_result})` : ''), statusTone[e.status] || ''];
      case 'ack': return [`${e.location === 1 ? '카' : `${floorName(e.floor_code)} 홀`} 비콘 버튼 눌림 확인 #${e.ack_sequence}`, 'good'];
      case 'floor': return [`${floorEvents[e.event] || e.event}${e.from_floor || e.to_floor ? ` · ${e.from_floor ? `${e.from_floor} → ` : ''}${e.to_floor || '?'}` : ''}` +
        (e.detail ? ` (${e.detail})` : ''), e.event === 'RULE_VIOLATION' ? 'bad' : ''];
      case 'door': return [`문 ${doorLabels[e.state] ?? e.state} 감지 (LiDAR)`, e.state === 3 ? 'good' : ''];
      default: return [e.kind, ''];
    }
  }

  function renderEvents(t) {
    const events = t?.events || [];
    const signature = `${events.length}:${events.at(-1)?.t ?? ''}:${events[0]?.t ?? ''}`;
    if (signature === eventSignature) return;
    eventSignature = signature;
    $('transit-events').replaceChildren(...events.slice().reverse().map(e => {
      const [message, tone] = eventText(e);
      const li = el('li', tone);
      const time = el('time', 'mono', new Date(e.t*1000).toLocaleTimeString('ko-KR', {hour12:false, timeZone:'Asia/Seoul'}));
      time.dateTime = new Date(e.t*1000).toISOString();
      li.append(time, el('span', '', message));
      return li;
    }));
    $('transit-events-empty').hidden = events.length > 0;
  }

  // Camera snapshot: one <img> refreshed at most every 500 ms while this view is
  // visible and the AMR reports a fresh frame. No timer: leaving the view stops it.
  let cameraRequestedAt = 0;
  let cameraLoading = false;
  function renderCamera(s, online) {
    const img = $('transit-camera');
    const camera = online ? s?.camera : null;
    const fresh = camera && Number(camera.age) < 3;
    if (!fresh) {
      img.hidden = true;
      $('transit-camera-empty').hidden = false;
      text('transit-camera-note', s?.preview ? '미리보기에서는 영상이 없습니다.' : !online ? '연결 끊김' :
        camera ? `마지막 영상 ${Number(camera.age).toFixed(1)}초 전 · 수신 중단` : 'bringup에서 start_camera:=true로 켭니다.');
      return;
    }
    text('transit-camera-note', `${camera.width}×${camera.height} · ${Math.round((camera.bytes || 0)/1024)} KB · ${Number(camera.age).toFixed(1)}초 전`);
    const now = performance.now();
    if (cameraLoading || now - cameraRequestedAt < 500) return;
    cameraRequestedAt = now;
    cameraLoading = true;
    const next = new Image();
    next.onload = () => { img.src = next.src; img.hidden = false; $('transit-camera-empty').hidden = true; cameraLoading = false; };
    next.onerror = () => { cameraLoading = false; };
    next.src = `/api/camera.jpg?t=${Date.now()}`;
  }

  window.AMRTransit = {
    update(s, online) {
      if ($('transit-view').hidden) return;
      const t = online ? s?.transit : null;
      // Beacon rows first: step and log texts name floors by their beacon code.
      renderBeacons(t || {beacons: [], window: 60}, online ? s : null);
      renderHeader(t, online ? s : null, online);
      renderSteps(t || {}, online ? s : null);
      renderDoor(t);
      renderCamera(s, online);
      renderEvents(t);
      text('transit-source', s?.preview ? '미리보기 · ROS 미연결' : !online ? '연결 끊김' :
        !t ? '층 설정 없음' : s?.remote ? `중계 ${Number(s.remote.age ?? 0).toFixed(1)}초` : '실시간 ROS');
    },
  };
})();
