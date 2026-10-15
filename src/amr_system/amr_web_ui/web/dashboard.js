/* Clarity shell and read-only health display. Commands remain in app.js. */
(() => {
  'use strict';
  const $ = id => document.getElementById(id);
  const text = (id, value) => { const e = $(id); if (e && e.textContent !== String(value)) e.textContent = value; };
  const number = (value, digits=1) => Number.isFinite(value) ? value.toFixed(digits) : '—';
  const fresh = (item, limit=2) => item && item.valid !== false && item.fresh !== false &&
    Number.isFinite(item.age) && item.age < limit;
  const badge = (id, value, tone='') => { text(id, value); const name = `health-badge ${tone}`; if ($(id).className !== name) $(id).className = name; };
  const labels = {WAITING_AUTO:'출발 대기', WAITING_NAV2:'주행 준비 중', RUNNING:'주행 중', PAUSED:'일시정지',
    STOPPING:'종료 처리 중', IDLE:'대기', UNKNOWN:'상태 대기', SUCCEEDED:'도착', CANCELED:'취소됨',
    FAILED:'주행 실패', REJECTED:'목표 거절', SENDING:'목표 전송 중', CANCELING:'취소 중'};

  let latestState = null, latestOnline = false, latestConnectionIssue = '';

  function menu(open) {
    $('main-menu').classList.toggle('open', open);
    $('menu-toggle').setAttribute('aria-expanded', String(open));
    $('menu-backdrop').hidden = !open;
    if (!open && window.matchMedia('(max-width:900px)').matches && $('main-menu').contains(document.activeElement)) $('menu-toggle').focus();
  }
  function view() {
    const vehicle = location.hash === '#vehicle-status';
    $('main-view').hidden = vehicle;
    $('vehicle-view').hidden = !vehicle;
    text('current-view', vehicle ? '차량 상태' : '메인');
    document.querySelectorAll('[data-view]').forEach(e => {
      const active = e.dataset.view === (vehicle ? 'vehicle-status' : 'main');
      e.classList.toggle('active', active);
      if (active) e.setAttribute('aria-current', 'page'); else e.removeAttribute('aria-current');
    });
    menu(false);
    window.AMRDashboard?.update(latestState, latestOnline, latestConnectionIssue);
  }
  $('menu-toggle').addEventListener('click', () => menu($('menu-backdrop').hidden));
  $('menu-backdrop').addEventListener('click', () => menu(false));
  window.addEventListener('keydown', e => { if (e.key === 'Escape') menu(false); });
  window.addEventListener('hashchange', view);
  window.matchMedia('(min-width:901px)').addEventListener('change', e => { if (e.matches) menu(false); });
  text('server-address', location.hostname || '—');
  function clock() {
    const now = new Date();
    text('clock', now.toLocaleTimeString('ko-KR', {hour12:false, timeZone:'Asia/Seoul'})+' KST');
    $('clock').dateTime = now.toISOString();
  }
  clock(); setInterval(clock, 1000); view();

  window.AMRDashboard = {
    update(s, online, connectionIssue='') {
      latestState = s; latestOnline = online; latestConnectionIssue = connectionIssue;
      $('offline-banner').hidden = online;
      text('offline-banner', connectionIssue === 'AMR 상태 지연' ?
        '서버는 응답하고 있지만 AMR 상태가 지연되었습니다. 최신 상태를 기다립니다.' :
        '최신 상태를 기다리고 있습니다. 데이터가 갱신되면 조작할 수 있습니다.');
      text('side-connection', online ? s?.preview ? '미리보기' : fresh(s?.robot) ? 'ROS 수신 중' : 'ROS 대기' : connectionIssue || '연결 끊김');
      text('map-source', s?.preview ? '저장 지도 · 미리보기' : '실시간 지도');
      const relay = s?.remote;
      const relayFresh = !relay || (Number.isFinite(relay.age) && relay.age >= 0 &&
        relay.age < (relay.timeout ?? 2));
      const t = online && relayFresh && !s?.preview ? s?.telemetry || {} : {};
      // Display the AMR's latest source verdict while the relay snapshot is live.
      // Never use source_age for command eligibility; app.js uses end-to-end age.
      const sampleFresh = (item, limit=2) => {
        const age = relay && Number.isFinite(item?.source_age) ? item.source_age : item?.age;
        return relayFresh && item && item.valid !== false && item.fresh !== false &&
          Number.isFinite(age) && age >= 0 && age < limit;
      };
      const vehicle = sampleFresh(t.vehicle, .5) ? t.vehicle : null;
      text('health-source', s?.preview ? '미리보기 · ROS 미연결' : vehicle ? `/vehicle/status${relay ? ' · '+number(relay.age,1)+'초 전' : ''}` : '차량 상태 수신 대기');
      const loc = sampleFresh(t.localization) ? t.localization : null;
      const ready = loc?.ready === true;
      let status = s?.goal && ['SENDING','CANCELING','SUCCEEDED','CANCELED','FAILED','REJECTED'].includes(s.goal.status)
        ? s.goal.status : s?.navigation;
      if (!online) status = 'UNKNOWN';
      badge('drive-status', s?.preview && online ? '미리보기' : labels[status] || '상태 대기',
        online && status === 'RUNNING' ? 'good' : ['FAILED','REJECTED'].includes(status) ? 'bad' : '');
      const goal = s?.goal;
      const remaining = online && goal?.status === 'ACCEPTED' &&
        (goal.feedback_age == null || goal.feedback_age < 2) && Number.isFinite(goal.distance_remaining)
          ? Math.max(0, goal.distance_remaining) : null;
      const journey = t.journey;
      const travelled = journey?.request_id === goal?.request_id && journey?.complete && sampleFresh(t.wheel, 1)
        ? journey.travelled : null;
      let progress = null;
      if (online && goal?.status === 'SUCCEEDED') progress = 100;
      else if (Number.isFinite(travelled) && Number.isFinite(remaining) && travelled+remaining > 0)
        progress = Math.min(99, Math.round(100*travelled/(travelled+remaining)));
      text('travelled', number(travelled));
      text('remaining', `${number(remaining)} m`);
      text('drive-progress', Number.isFinite(progress) ? progress : '—');
      $('drive-progress-fill').style.width = `${progress ?? 0}%`;
      if (progress === null) {
        $('progressbar').removeAttribute('aria-valuenow');
        $('progressbar').setAttribute('aria-valuetext', '거리 데이터 대기');
      } else {
        $('progressbar').setAttribute('aria-valuenow', progress);
        $('progressbar').removeAttribute('aria-valuetext');
      }
      const x = $('goal-x').textContent, y = $('goal-y').textContent;
      text('destination-name', online && x !== '—' ? `X ${x} m · Y ${y} m` : '목적지를 선택하세요');

      let received = 0;
      for (const name of ['scan','wheel','imu','ekf']) {
        const item = t[name], valid = sampleFresh(item, name === 'scan' ? 1 : .75);
        if (valid) received++;
        text(`sensor-${name}-hz`, valid ? number(item.hz) : '—');
        text(`sensor-${name}-age`, item ? Math.round(item.age*1000).toLocaleString('ko-KR') : '—');
        badge(`sensor-${name}-state`, valid ? '수신 중' : item?.valid === false ? '유효하지 않음' : item ? '지연' : '미수신', valid ? 'good' : item ? 'bad' : '');
      }
      const time = sampleFresh(t.time, t.time?.timeout ?? .6) ? t.time : null;
      badge('health-mode', vehicle ? vehicle.mode : '미수신');
      for (const id of ['health-location', 'health-location-detail'])
        badge(id, ready ? '검증 완료' : loc ? '검증 대기' : '미수신', ready ? 'good' : loc ? 'warn' : '');
      badge('health-sensors', received ? `${received}/4종 수신 중` : '미수신', received === 4 ? 'good' : received ? 'warn' : '');
      badge('health-time', time ? time.healthy ? '유효' : '확인 필요' : '미수신', time ? time.healthy ? 'good' : 'bad' : '');
      const pose = sampleFresh(t.pose) ? t.pose : null;
      const healthy = ready && received === 4 && time?.healthy && pose;
      const summaryClass = `health-summary ${s?.preview || !online ? 'neutral' : healthy ? 'good' : 'warn'}`;
      if ($('health-summary').className !== summaryClass) $('health-summary').className = summaryClass;
      text('health-title', !online ? '차량 상태를 확인할 수 없습니다' : s?.preview ? '미리보기 · 차량 데이터 미연결' :
        healthy ? '자동주행 조건 충족' : !vehicle ? '차량 상태 수신 대기' : !loc ? '위치 상태 수신 대기' : !ready ? '위치 확인 필요' : '주행 상태 확인 필요');
      text('health-detail', !online ? '최신 데이터를 다시 기다리고 있습니다.' : s?.preview ? '저장 지도로 화면과 조작을 확인할 수 있습니다.' :
        healthy ? '위치 검증 완료 · 필수 센서 수신 · 시간 데이터 유효' : loc?.tracking_warning || loc?.detail || '공용 차량 상태와 필수 센서 데이터를 기다리고 있습니다.');
      for (const side of ['left','right']) {
        text(`motor-${side}-rpm`, sampleFresh(t.joints, .75) ? `${number(t.joints[side+'_rpm'])} rpm` : '—');
        const heartbeat = t[`motor_${side}_heartbeat`], temperature = t[`motor_${side}_temperature`];
        const heartFresh = sampleFresh(heartbeat, heartbeat?.timeout ?? 2);
        text(`motor-${side}-state`, heartFresh ? `상태 코드 · ${heartbeat.axis_state}` : heartbeat ? '수신 지연' : '미연동');
        text(`motor-${side}-error`, heartFresh ? `0x${heartbeat.error_code.toString(16).padStart(8,'0')}` : heartbeat ? '수신 지연' : '미연동');
        text(`motor-${side}-heartbeat`, heartFresh ? `${number(heartbeat.hz)} Hz` : heartbeat ? '수신 지연' : '미연동');
        text(`motor-${side}-temperature`, sampleFresh(temperature,temperature?.timeout ?? 2) ? `${number(temperature.temperature)} ℃` : temperature ? '수신 지연' : '미연동');
      }
      const diagnostics = sampleFresh(t.diagnostics, 3) ? t.diagnostics : null;
      for (const [id, key] of [['invalid','invalid'],['stale','stale_frames'],['rebases','wheel_rebases']])
        text(`counter-${id}`, diagnostics?.counters?.[key] ?? '—');
      text('diagnostics-detail', diagnostics?.detail || 'CAN 진단 데이터 수신 대기 · 노드 시작 이후 누적 카운터');
      text('health-localization-state', loc?.state || '—');
      text('health-tf', pose ? '연결됨' : vehicle ? '차량 위치 대기' : '—');
      text('health-navigation', vehicle ? labels[vehicle.navigation] || '상태 대기' : '—');
      text('robot-position', pose ? `${number(pose.x,2)} / ${number(pose.y,2)} m` : '— / — m');
      text('robot-heading', pose ? `${Math.round(pose.yaw*180/Math.PI)} °` : '— °');
      text('pose-state', pose ? ready ? '확인됨' : '확인 중' : '대기');
      text('health-tracking', loc?.tracking_warning || loc?.detail || '위치 감독기의 최신 검증 결과를 기다리고 있습니다.');
      for (const name of ['command','wheel','ekf']) text(`motion-${name}`, sampleFresh(t[name], .75) ? number(t[name].linear, 2) : '—');
      const power = {};
      for (const side of ['left','right']) {
        const input = t[`motor_${side}_power`];
        power[side] = sampleFresh(input,input?.timeout ?? 2) ? input : null;
        text(`power-${side}-voltage`, number(power[side]?.voltage,2));
        text(`power-${side}-current`, number(power[side]?.current,2));
      }
      const battery = sampleFresh(t.battery,t.battery?.timeout ?? 2) ? t.battery : null;
      const voltage = Number.isFinite(battery?.voltage) ? battery.voltage :
        Number.isFinite(power.left?.voltage) && Number.isFinite(power.right?.voltage)
          ? (power.left.voltage+power.right.voltage)/2 : null;
      const bms = Number.isFinite(battery?.soc) && battery.soc >= 0 && battery.soc <= 100;
      const percent = bms ? Math.round(battery.soc) : estimateOperationalBattery(voltage);
      const batteryNote = bms ? 'BMS 잔량' : percent === null ? '전압 미연동' : `전압 추정 · ${voltage.toFixed(2)} V`;
      text('side-battery-percent', percent ?? '—');
      $('side-battery-fill').style.width = `${percent ?? 0}%`;
      text('side-battery-note', batteryNote);
      text('health-battery-percent', percent ?? '—');
      $('health-battery-fill').style.width = `${percent ?? 0}%`;
      text('health-battery-note', batteryNote);
      text('power-source', bms ? 'BMS' : Number.isFinite(voltage) ? 'CAN 전압' : '전압 연동 대기');
      badge('battery-charge-state', battery ? ({1:'방전 중',2:'충전 중',3:'충전 완료'}[battery.charge_state] || '미확인') : '미연동');
      badge('battery-temperature', Number.isFinite(battery?.temperature) ? `${number(battery.temperature)} ℃` : '미연동');
    },
  };
})();
