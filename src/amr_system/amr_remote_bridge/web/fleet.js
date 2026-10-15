/* Use textContent for agent-provided values; never retry movement commands. */
(() => {
  'use strict';
  async function poll() {
    const status = document.getElementById('status');
    try {
      const response = await fetch('/api/robots', {signal: AbortSignal.timeout(2500)});
      if (!response.ok) throw new Error('조회 실패');
      const {robots} = await response.json();
      const list = document.getElementById('robots');
      list.replaceChildren();
      for (const robot of robots) {
        const card = document.createElement('a');
        card.className = 'robot' + (robot.online ? ' online' : '');
        card.href = '/robots/' + encodeURIComponent(robot.robot_id) + '/';
        const name = document.createElement('strong');
        name.textContent = robot.robot_id;
        const detail = document.createElement('span');
        detail.textContent = robot.online ? '연결됨 · ' + (robot.navigation || '상태 대기') : '연결 지연 · 제어 불가';
        card.append(name, detail);
        list.append(card);
      }
      status.textContent = robots.length ? robots.length + '대 등록됨' : 'AMR 클라이언트의 연결을 기다리고 있습니다.';
    } catch (_error) {
      status.textContent = '관제 서버 연결을 확인해 주세요.';
    } finally { setTimeout(poll, 1000); }
  }
  poll();
})();
