"""One-shot, bounded straight-line test policy; no hardware or ROS calls."""
from math import atan2, cos, isfinite, sin


class StraightTest:
    def __init__(self):
        self.state = 'WAITING'
        self.reason = 'waiting_for_manual_and_fresh_sensors'
        self.origin = None
        self.started = None
        self.manual_seen = False
        self.ready_since = None
        self.distance = self.lateral = self.heading = 0.

    def stop(self, reason, complete=False):
        self.state = 'COMPLETE' if complete else 'ABORTED'
        self.reason = reason
        return 0.

    def tick(self, now, mode, pose, measured_v, fresh, clear, exclusive=True, imu_heading=0.):
        if self.state in ('COMPLETE', 'ABORTED'):
            return 0.
        if not exclusive:
            return self.stop('competing_command_publisher')
        if pose is None or not all(isfinite(v) for v in (*pose, measured_v)):
            fresh = False
        if self.state == 'WAITING':
            if mode == 'MANUAL' and fresh and clear and abs(measured_v) < .02:
                if self.ready_since is None:
                    self.ready_since = now
                if now-self.ready_since >= 1.:
                    self.manual_seen = True
                    self.reason = 'ready_release_lb_to_start'
            elif mode != 'AUTO':
                self.ready_since = None
                self.manual_seen = False
                self.reason = 'waiting_for_manual_and_fresh_sensors'
            if mode != 'AUTO' or not self.manual_seen:
                return 0.
            if not fresh or not clear or abs(measured_v) >= .02:
                return self.stop('start_conditions_not_met')
            self.origin = pose
            self.started = now
            self.state, self.reason = 'RUNNING', 'straight_1m'
        if mode != 'AUTO':
            return self.stop('operator_left_AUTO')
        if not fresh or not isfinite(imu_heading):
            return self.stop('sensor_or_joy_timeout')
        if abs(imu_heading) > .0872664626:
            return self.stop('imu_heading_exceeded')
        if not clear:
            return self.stop('obstacle_in_remaining_path')
        dx, dy = pose[0]-self.origin[0], pose[1]-self.origin[1]
        self.distance = dx*cos(self.origin[2])+dy*sin(self.origin[2])
        self.lateral = -dx*sin(self.origin[2])+dy*cos(self.origin[2])
        self.heading = atan2(sin(pose[2]-self.origin[2]), cos(pose[2]-self.origin[2]))
        if self.distance < -.025:
            return self.stop('reverse_odometry')
        if abs(self.lateral) > .06 or abs(self.heading) > .0872664626:
            return self.stop('deviation_exceeded')
        if measured_v < -.03 or abs(measured_v) > .20:
            return self.stop('unexpected_measured_speed')
        if now-self.started > 20.:
            return self.stop('20_second_timeout')
        if now-self.started > 4. and self.distance < .02:
            return self.stop('no_forward_progress')
        if self.distance >= .995:
            return self.stop('target_reached', complete=True)
        # Slow approach, then stop once; never reverse to correct overshoot.
        return min(.10, max(.025, (1.-self.distance)*.5))
