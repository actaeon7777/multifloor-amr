"""Automated joystick input through the real joy_teleop -> control_mux pipeline."""
import json
import math
import time
import numpy as np


class ManualTrial:
    def __init__(self, bridge, root):
        from geometry_msgs.msg import Twist, PoseWithCovarianceStamped
        from sensor_msgs.msg import Joy
        self.Joy, self.bridge, self.root = Joy, bridge, root
        self.publisher = bridge.create_publisher(Joy, '/joy', 10)
        self.pose_publisher = bridge.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)
        self.Pose = PoseWithCovarianceStamped
        self.spawn = json.loads((root/'sim/config/floor4.json').read_text())['spawn']
        self.commands, self.samples = [], []
        self.sub = bridge.create_subscription(Twist, '/cmd_vel', self.command, 10)
        self.schedule = [('release', 0., 0., 3.), ('forward', .3, 0., 3.),
                         ('reverse', -.3, 0., 3.), ('release_stop', 0., 0., 1.),
                         ('left', 0., .8, 3.), ('right', 0., -.8, 3.), ('release_turn', 0., 0., 1.),
                         ('max_forward', 1., 0., 2.), ('max_reverse', -1., 0., 2.), ('release_max', 0., 0., 2.),
                         ('arc_left', .4, .8, 2.), ('arc_back', -.4, -.8, 2.),
                         ('arc_right', .4, -.8, 2.), ('arc_return', -.4, .8, 2.)]
        self.schedule += [('full_diagonal', 1., .8, 2.), ('full_diagonal_back', -1., -.8, 2.)]
        self.schedule += [(f'reversal_{i}', .35 if i%2 else -.35, .8 if i%2 else -.8, .5) for i in range(12)]
        self.schedule += [('release_final', 0., 0., 2.), ('timeout_drive', .2, 0., 1.), ('joy_loss', 0., 0., 2.)]
        self.duration = sum(p[3] for p in self.schedule)
        self.last_publish, self.phase = 0., ''

    def command(self, msg):
        self.commands.append([self.bridge.sim_time, msg.linear.x, msg.angular.z])

    def tick(self, elapsed):
        remaining = elapsed
        for name, v, w, duration in self.schedule:
            if remaining < duration: break
            remaining -= duration
        if name != self.phase:
            print('MANUAL_PHASE '+name, flush=True)
            self.phase = name
        now = time.monotonic()
        if now-self.last_publish < .04 or name == 'joy_loss': return
        self.last_publish = now
        if 1. < elapsed < 1.3:
            from amr_sim_bridge.bridge import stamp
            pose = self.Pose()
            pose.header.frame_id, pose.header.stamp = 'map', stamp(elapsed)
            pose.pose.pose.position.x, pose.pose.pose.position.y = self.spawn[:2]
            pose.pose.pose.orientation.w = math.cos(self.spawn[2]/2)
            pose.pose.pose.orientation.z = math.sin(self.spawn[2]/2)
            pose.pose.covariance[0] = pose.pose.covariance[7] = pose.pose.covariance[35] = .01
            self.pose_publisher.publish(pose)
        joy = self.Joy()
        joy.axes = [0., v, w/.8, 0.]
        joy.buttons = [0]*8
        joy.buttons[6] = int(v != 0. or w != 0.)
        self.publisher.publish(joy)

    def sample(self, elapsed, p, q, lv, av):
        assert all(np.isfinite(v).all() for v in (p, q, lv, av)), 'Non-finite physics state'
        qw, qx, qy, qz = q
        roll = math.atan2(2*(qw*qx+qy*qz), 1-2*(qx*qx+qy*qy))
        pitch = math.asin(float(np.clip(2*(qw*qy-qz*qx), -1, 1)))
        self.samples.append([elapsed, *p, roll, pitch, *lv, *av])

    def finish(self):
        data = np.asarray(self.samples)
        moving = data[data[:,0]>=3.]
        acceleration = np.diff(moving[:,8])/np.diff(moving[:,0])
        commands = np.asarray(self.commands)
        np.savez_compressed(self.root/'logs/floor4_manual_trace.npz', samples=data, commands=commands)
        metrics = {'max_tilt_deg': float(np.rad2deg(np.max(np.abs(moving[:,4:6])))),
                   'height_range_mm': float(np.ptp(moving[:,3])*1000),
                   'vertical_accel_rms_m_s2': float(np.sqrt(np.mean(acceleration**2))),
                   'vertical_accel_peak_m_s2': float(np.max(np.abs(acceleration))),
                   'final_speed_m_s': float(np.linalg.norm(data[-1,6:9])),
                   'final_yaw_rate_rad_s': float(abs(data[-1,11])),
                   'obstacle_contacts': self.bridge.contacts,
                   'travel_extent_m': float(np.linalg.norm(np.ptp(data[:,1:3], axis=0))),
                   'command_count': len(commands), 'real_time_factor': self.bridge.sim_time/(time.monotonic()-self.bridge.started)}
        checks = {'tilt': metrics['max_tilt_deg']<1., 'height': metrics['height_range_mm']<3.,
                  'vertical_rms': metrics['vertical_accel_rms_m_s2']<.5,
                  'vertical_peak': metrics['vertical_accel_peak_m_s2']<5.,
                  'stopped': metrics['final_speed_m_s']<.02 and metrics['final_yaw_rate_rad_s']<.02,
                  'moved': metrics['travel_extent_m']>1., 'no_collisions': self.bridge.contacts==0,
                  'commands_received': len(commands)>300,
                  'both_directions': len(commands)>0 and commands[:,1].max()>.9 and commands[:,1].min()<-.9,
                  'both_turns': len(commands)>0 and commands[:,2].max()>.7 and commands[:,2].min()<-.7,
                  'joy_loss_stop': len(commands[commands[:,0]>self.duration-1.])>0 and np.max(np.abs(commands[commands[:,0]>self.duration-1.,1:]))<1e-6}
        checks = {k: bool(v) for k,v in checks.items()}
        report = {'passed': all(checks.values()), 'coverage': 'Synthetic Joy -> production joy_teleop -> control_mux -> DDS cmd_vel -> PhysX, real sensor publication, LB release and Joy loss',
                  'scene': 'sim/capstone_amr_4_floor.usda', 'metrics': metrics, 'checks': checks,
                  'body_pose_or_velocity_overrides': False}
        (self.root/'reports/floor4_manual.json').write_text(json.dumps(report, indent=2)+'\n')
        print('AMR_MANUAL_RESULT '+json.dumps(report), flush=True)
        assert report['passed'], report
