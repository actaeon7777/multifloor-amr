"""Pure patrol decisions. The adapter executes actions and cancels superseded goals.

``goal_serial`` changes whenever a goal is replaced; late action results must be
ignored by the adapter. Events are drained separately from the tick action queue.
All deadlines use the supplied ROS clock, so a simulation pause pauses a dwell.
"""
from collections import deque
import math
import re


REQUEST_ID = re.compile(r'[A-Za-z0-9_-]{8,80}')


def valid_request_id(value):
    return isinstance(value, str) and REQUEST_ID.fullmatch(value) is not None


def validate_settings(settings):
    for name in ('tick_period', 'nav_timeout_s', 'goal_xy_tolerance', 'cancel_result_timeout_s'):
        value = settings.get(name, 10.0)
        if type(value) not in (int, float) or not math.isfinite(value) or value <= 0:
            raise ValueError(f'{name} must be positive and finite')
    for name, default in (('min_battery_voltage', 26.1), ('retry_delay_s', 3.0), ('battery_low_s', 10.0)):
        value = settings.get(name, default)
        if type(value) not in (int, float) or not math.isfinite(value) or value < 0:
            raise ValueError(f'{name} must be non-negative and finite')
    if type(settings.get('require_estop', True)) is not bool:
        raise ValueError('require_estop must be a bool')
    for name, minimum in (('max_retries', 0), ('max_consecutive_failures', 1)):
        if type(settings[name]) is not int or settings[name] < minimum:
            raise ValueError(f'{name} must be an integer >= {minimum}')


class Patrol:
    def __init__(self, config, now, *, nav_timeout_s=180.0, max_retries=1,
                 max_consecutive_failures=3, min_battery_voltage=26.1,
                 retry_delay_s=3.0, battery_low_s=10.0):
        validate_settings(dict(tick_period=.2, goal_xy_tolerance=.3,
                               nav_timeout_s=nav_timeout_s, max_retries=max_retries,
                               max_consecutive_failures=max_consecutive_failures,
                               min_battery_voltage=min_battery_voltage,
                               retry_delay_s=retry_delay_s, battery_low_s=battery_low_s))
        self.config = config
        self.clock = now if callable(now) else None
        self.now = float(now() if callable(now) else now)
        self.nav_timeout_s = nav_timeout_s
        self.max_retries = max_retries
        self.max_consecutive_failures = max_consecutive_failures
        self.min_battery_voltage = min_battery_voltage
        self.retry_delay_s, self.battery_low_s = retry_delay_s, battery_low_s
        self.retry_at, self.retry_action = None, None
        self.battery_low_since = None
        self.manual_wait = False
        self.wait_detail = ''
        self.phase, self.route, self.index = 'IDLE', None, 0
        self.detail, self.started_at, self.ended_at = '', None, None
        self.last_command = None
        self.skipped = []
        self.actions, self.events = deque(), deque()
        self.goal_serial = 0
        self.pose = None
        self.awaiting = False
        self.deadline = None
        self.retries = self.failures = 0
        self.early_return = False

    @property
    def active(self):
        return self.phase in ('NAV', 'DWELL', 'RETURN')

    def _refresh(self):
        if self.clock is not None:
            now = float(self.clock())
            backwards = now < self.now
            self.now = now
            if backwards and self.active:
                self._finish(False, 'ROS 시각 역행으로 순찰 중단')

    def ack(self, request_id, accepted, detail):
        self.last_command = dict(request_id=request_id, accepted=accepted, detail=detail)
        return accepted

    def start(self, route, request_id, floor, battery_v, nav_busy, *, held=False,
              estop_healthy=True, confidence='CONFIRMED', in_elevator=False):
        self._refresh()
        reason = None
        selected = self.config.routes.get(route) if isinstance(route, str) else None
        if not valid_request_id(request_id):
            reason = '잘못된 요청 ID'
        elif self.active:
            reason = '이미 순찰 중입니다'
        elif selected is None:
            reason = '알 수 없는 순찰 경로'
        elif selected.floor != floor or confidence not in ('CONFIRMED', 'BEACON', 'ASSUMED'):
            reason = '순찰 경로의 층과 현재 확인된 층이 다릅니다'
        elif in_elevator:
            reason = '엘리베이터 안에서는 순찰을 시작할 수 없습니다'
        elif held:
            reason = '관리자 정지'
        elif not estop_healthy:
            reason = 'E-stop 상태가 정상이어야 합니다'
        elif nav_busy:
            reason = '다른 주행이 진행 중입니다'
        elif self._low_battery(battery_v):
            reason = '배터리 부족으로 순찰을 시작할 수 없습니다'
        elif selected.end == 'home' and not self._home_available(selected):
            reason = '현재 층의 복귀 위치가 없습니다'
        if reason is not None:
            return self.ack(request_id, False, reason)
        self.route, self.index = selected, 1
        self.started_at, self.ended_at = self.now, None
        self.skipped = []
        self.retries = self.failures = 0
        self.early_return = False
        self.actions.clear()
        self.battery_low_since = None
        self._waypoint()
        self._event('patrol.started', '순찰 시작')
        return self.ack(request_id, True, '순찰 시작')

    def _home_available(self, route):
        return self.config.home_pose is not None and self.config.home_floor == route.floor

    def _low_battery(self, value):
        return (self.min_battery_voltage > 0 and type(value) in (int, float)
                and math.isfinite(value) and value < self.min_battery_voltage)

    def battery(self, voltage):
        self._refresh()
        if not self.active or self.phase == 'RETURN' or not self._low_battery(voltage):
            self.battery_low_since = None
            return
        if self.battery_low_since is None:
            self.battery_low_since = self.now
        if self.now - self.battery_low_since >= self.battery_low_s:
            self._close_dwell()
            self.early_return = True
            self._return_home()

    def navigation_state(self, state, held=False, other_navigation=False):
        """Pause dispatch during manual control; yield ownership after failure."""
        waiting = state == 'MANUAL' or held
        if not self.active:
            self.manual_wait = waiting
            return  # Keep DONE/ABORTED reasons unchanged after manual release.
        if waiting and not self.manual_wait:
            self.wait_detail = self.detail
        elif not waiting and self.manual_wait:
            self.detail = self.wait_detail
            if self.awaiting:
                self.navigation_dispatched(self.now)
        self.manual_wait = waiting
        if self.active and waiting:
            self.detail = '수동 조작 중: 순찰 대기'
        elif self.active and other_navigation:
            self.stop('다른 주행 명령으로 순찰 중단')

    def stop(self, reason='순찰 정지 요청'):
        self._refresh()
        if self.active:
            self._finish(False, reason)

    def _event(self, kind, text):
        self.events.append(dict(kind=kind, severity='warn' if kind in (
            'patrol.aborted', 'patrol.waypoint_skipped') else 'info', text=text,
            floor=self.route.floor if self.route else None, room='',
            pose=list(self.pose) if self.pose else None, image_path=None, stamp=self.now))

    def _discard_motion(self):
        self.actions = deque(a for a in self.actions if a[0] == 'publish_observation')
        self.awaiting = False
        self.deadline = None
        self.retry_at, self.retry_action = None, None
        self.goal_serial += 1

    def _navigate(self, pose):
        self._discard_motion()
        if self.manual_wait:
            self.wait_detail = self.detail
        self.pose = tuple(pose)
        self.actions.append(('navigate', self.pose))

    def _waypoint(self):
        self.phase = 'NAV'
        point = self.route.waypoints[self.index - 1]
        yaw = point.yaw
        if yaw is None:
            previous = self.route.waypoints[self.index - 2] if self.index > 1 else None
            yaw = math.atan2(point.y - previous.y, point.x - previous.x) if previous else 0.0
        self.detail = point.note
        self._navigate((point.x, point.y, yaw))

    def _observation(self, event):
        point = self.route.waypoints[self.index - 1]
        return dict(event=event, route=self.route.name, index=self.index,
                    x=self.pose[0], y=self.pose[1], yaw=self.pose[2],
                    dwell_s=point.dwell_s, stamp=self.now)

    def _close_dwell(self):
        if self.phase == 'DWELL':
            self.actions.append(('publish_observation', self._observation('end')))
            self.phase = 'NAV'

    def _advance(self):
        self.retries = 0
        if self.index < len(self.route.waypoints):
            self.index += 1
            self._waypoint()
        elif self.route.end == 'home':
            self._return_home()
        else:
            self._finish(True, '순찰 완료')

    def _return_home(self):
        if not self._home_available(self.route):
            self._finish(False, '현재 층의 복귀 위치가 없습니다')
            return
        self.phase = 'RETURN'
        self.index = len(self.route.waypoints) + 1
        self.retries = 0
        self.detail = '배터리 부족으로 조기 복귀' if self.early_return else 'home으로 복귀'
        self._navigate(self.config.home_pose)

    def on_nav_result(self, ok, *, nav_state='', held=False, other_navigation=False):
        self._refresh()
        if not self.awaiting or self.phase not in ('NAV', 'RETURN'):
            return
        self.awaiting = False
        self.deadline = None
        if not ok:
            if nav_state == 'MANUAL' or held or self.manual_wait:
                self._schedule_retry('resume')
                self.navigation_state('MANUAL' if nav_state == 'MANUAL' or self.manual_wait else '', held)
            elif other_navigation:
                self._finish(False, '다른 주행 명령으로 순찰 중단')
            elif self.retries < self.max_retries:
                self.retries += 1
                self.detail = f'주행 재시도 {self.retries}/{self.max_retries}'
                self._schedule_retry('retry')
            elif self.phase == 'RETURN':
                self._finish(False, 'home 복귀 실패')
            else:
                self.skipped.append(self.index)
                self.failures += 1
                self._event('patrol.waypoint_skipped', f'{self.index}번 지점 주행 실패: 건너뜀')
                if self.failures >= self.max_consecutive_failures:
                    self._finish(False, '연속 주행 실패로 순찰 중단')
                else:
                    self.detail = '다음 지점 주행 대기'
                    self._schedule_retry('advance')
            return
        self.failures = 0
        if self.phase == 'RETURN':
            self._finish(True, '배터리 부족으로 조기 복귀' if self.early_return else '순찰 완료')
            return
        point = self.route.waypoints[self.index - 1]
        if point.dwell_s > 0:
            self.phase, self.detail = 'DWELL', point.note
            self.deadline = self.now + point.dwell_s
            self.actions.append(('publish_observation', self._observation('begin')))
        else:
            self._advance()

    def _schedule_retry(self, action):
        self._discard_motion()
        self.retry_action, self.retry_at = action, self.now + self.retry_delay_s

    def _resume_retry(self):
        action = self.retry_action
        self.retry_at, self.retry_action = None, None
        if action == 'advance':
            self._advance()
        else:
            self._navigate(self.pose)

    def _finish(self, ok, detail):
        self._close_dwell()
        self._discard_motion()
        self.phase = 'DONE' if ok else 'ABORTED'
        self.detail, self.ended_at = detail, self.now
        self._event('patrol.finished' if ok else 'patrol.aborted', detail)
        self.actions.append(('finish', dict(phase=self.phase, detail=detail)))

    def navigation_dispatched(self, now):
        """Start the timeout when the adapter actually dispatches a queued goal."""
        self.deadline = now + self.nav_timeout_s

    def _next_action(self):
        if not self.actions:
            return None
        action = self.actions.popleft()
        if action[0] == 'navigate':
            self.awaiting = True
            self.navigation_dispatched(self.now)
        return action

    def tick(self, now):
        now = float(now)
        if now < self.now and self.active:
            self.now = now
            self.stop('ROS 시각 역행으로 순찰 중단')
        self.now = now
        if self.actions and self.actions[0][0] != 'navigate':
            return self._next_action()
        if self.active and self.manual_wait and self.phase in ('NAV', 'RETURN'):
            self.detail = '수동 조작 중: 순찰 대기'
            return ('wait', .2)
        if self.retry_at is not None:
            if now < self.retry_at:
                return ('wait', self.retry_at - now)
            self._resume_retry()
        if self.actions:
            return self._next_action()
        if self.phase == 'DWELL':
            if now < self.deadline:
                return ('wait', self.deadline - now)
            self._close_dwell()
            self._advance()
        elif self.awaiting and now >= self.deadline:
            self.on_nav_result(False)
            if self.retry_at is not None:
                if self.now < self.retry_at:
                    return ('wait', self.retry_at - self.now)
                self._resume_retry()
        # on_nav_result may sample a slightly newer clock. Do not recursively
        # tick with the old sample and mistake that for a ROS clock reversal.
        return self._next_action()

    def status(self, now):
        end = self.ended_at if self.ended_at is not None else now
        return dict(active=self.active, route=self.route.name if self.route else None,
                    floor=self.route.floor if self.route else None, phase=self.phase,
                    index=self.index, total=len(self.route.waypoints) if self.route else 0,
                    detail=self.detail, started_at=self.started_at,
                    elapsed_s=max(0.0, end - self.started_at) if self.started_at is not None else 0.0,
                    skipped=list(self.skipped), routes=list(self.config.routes),
                    last_command=self.last_command, stamp=now)
