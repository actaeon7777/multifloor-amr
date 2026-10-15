"""One-shot left 90-degree verification, using gyro angle with wheel cross-checks."""
from math import atan2, hypot, isfinite, pi, sin, cos


class RotationTest:
    def __init__(self):
        self.state = 'WAITING'
        self.reason = 'waiting_for_manual_and_fresh_sensors'
        self.origin = None
        self.started = None
        self.ready_since = None
        self.manual_seen = False
        self.wheel_angle = self.imu_angle = self.translation = 0.

    def stop(self, reason, complete=False):
        self.state = 'COMPLETE' if complete else 'ABORTED'
        self.reason = reason
        return 0.

    def tick(self, now, mode, pose, v, w, gyro_z, imu_angle, fresh, clear, exclusive=True):
        if self.state in ('COMPLETE', 'ABORTED'):
            return 0.
        if not exclusive:
            return self.stop('competing_command_publisher')
        if pose is None or not all(isfinite(x) for x in (*pose,v,w,gyro_z,imu_angle)):
            fresh=False
        stationary=abs(v)<.02 and abs(w)<.04 and abs(gyro_z)<.02
        if self.state=='WAITING':
            if mode=='MANUAL' and fresh and clear and stationary:
                if self.ready_since is None:self.ready_since=now
                if now-self.ready_since>=1.:
                    self.manual_seen=True
                    self.reason='ready_press_RB_once_to_start_left_turn'
            elif mode!='AUTO':
                self.ready_since=None;self.manual_seen=False
                self.reason='waiting_for_manual_and_fresh_sensors'
            if mode!='AUTO' or not self.manual_seen:return 0.
            if not fresh or not clear or not stationary:return self.stop('start_conditions_not_met')
            self.origin=pose;self.started=now
            self.state='RUNNING';self.reason='left_90_degrees'
        if mode!='AUTO':return self.stop('operator_left_AUTO')
        if not fresh:return self.stop('sensor_or_joy_timeout')
        if not clear:return self.stop('obstacle_in_rotation_sweep')
        self.wheel_angle=atan2(sin(pose[2]-self.origin[2]),cos(pose[2]-self.origin[2]))
        self.imu_angle=imu_angle
        self.translation=hypot(pose[0]-self.origin[0],pose[1]-self.origin[1])
        if self.translation>.12 or abs(v)>.10:return self.stop('unexpected_translation')
        if abs(w)>.40 or abs(gyro_z)>.40:return self.stop('unexpected_rotation_speed')
        if self.wheel_angle < -5*pi/180 or imu_angle < -5*pi/180:return self.stop('wrong_rotation_direction')
        if abs(self.wheel_angle-imu_angle)>25*pi/180:return self.stop('wheel_imu_disagreement')
        if self.wheel_angle>120*pi/180 or imu_angle>105*pi/180:return self.stop('angle_limit_exceeded')
        if now-self.started>30.:return self.stop('30_second_timeout')
        if now-self.started>4. and imu_angle<3*pi/180:return self.stop('no_rotation_progress')
        if imu_angle>=89.5*pi/180:return self.stop('gyro_target_reached',complete=True)
        return min(.15,max(.04,(pi/2-imu_angle)*.5))
