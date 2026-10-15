#!/usr/bin/env python3
"""One straight 1 m test via existing /cmd_vel_auto; physical RB starts/cancels.

Requires the stack, actual Joy, LiDAR, wheel odometry and IMU. Publishes no CAN,
no /joy and no final /cmd_vel. Never run beside another autonomous publisher.
"""
import json
import math
from pathlib import Path
import signal
from threading import Event
from time import monotonic

import rclpy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from rclpy.signals import SignalHandlerOptions
from sensor_msgs.msg import Imu, Joy, LaserScan
from std_msgs.msg import String
from tf2_ros import Buffer, TransformListener

from amr_control.straight_test import StraightTest


class Probe(Node):
    def __init__(self, folder, node_name='straight_1m_test'):
        super().__init__(node_name)
        self.folder = folder
        self.policy = StraightTest()
        self.mode = 'UNKNOWN'
        self.imu_yaw = 0.
        self.previous_gyro = None
        self.pose, self.v = None, 0.
        self.received = {}
        self.obstacles = None
        self.health = False
        self.data = []
        self.start_time = monotonic()
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)
        self.pub = self.create_publisher(Twist, '/cmd_vel_auto', 1)
        self.create_subscription(String, '/control_mux/mode', self.on_mode,
                                 QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.create_subscription(Odometry, '/wheel/odometry', self.on_wheel, 20)
        self.create_subscription(Imu, '/imu/data_raw', self.on_imu, qos_profile_sensor_data)
        self.create_subscription(Joy, '/joy', self.on_joy, qos_profile_sensor_data)
        self.create_subscription(LaserScan, '/scan', self.on_scan, qos_profile_sensor_data)
        self.create_subscription(DiagnosticArray, '/diagnostics', self.on_health, 10)
        self.create_subscription(Twist, '/cmd_vel', self.on_output, 10)
        self.last_log = None
        self.finish_at = None
        self.final = None

    def on_mode(self, msg):
        self.mode = msg.data

    def sensor(self, name, msg):
        age = (self.get_clock().now().nanoseconds/1e9 - msg.header.stamp.sec - msg.header.stamp.nanosec/1e9)
        if not -.02 <= age < .25:
            return False
        self.received[name] = monotonic()
        return True

    def on_wheel(self, msg):
        if msg.header.frame_id != 'odom' or msg.child_frame_id != 'base_footprint' or not self.sensor('wheel',msg):
            return
        q=msg.pose.pose.orientation
        yaw=math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
        self.pose=(msg.pose.pose.position.x,msg.pose.pose.position.y,yaw)
        self.v=msg.twist.twist.linear.x
        self.data.append({'t':monotonic()-self.start_time,'type':'wheel','pose':self.pose,'v':self.v,'w':msg.twist.twist.angular.z})

    def on_imu(self, msg):
        if self.sensor('imu',msg):
            stamp=msg.header.stamp.sec+msg.header.stamp.nanosec/1e9
            if self.policy.state=='RUNNING':
                if self.previous_gyro is not None:
                    previous_stamp,previous_rate=self.previous_gyro
                    dt=stamp-previous_stamp
                    if 0.<dt<.35:
                        self.imu_yaw+=dt*(previous_rate+msg.angular_velocity.z)/2
                self.previous_gyro=(stamp,msg.angular_velocity.z)
            else:
                self.previous_gyro=None
            self.data.append({'t':monotonic()-self.start_time,'type':'imu','gyro_z':msg.angular_velocity.z})

    def on_joy(self, msg):
        # Existing mux validates all axes/buttons. Here require real fresh input.
        if len(msg.buttons)>7 and len(msg.axes)>2:
            self.received['joy']=monotonic()

    def on_health(self, msg):
        statuses=[s for s in msg.status if s.name=='amr/can_telemetry']
        if statuses:
            self.health=all(s.level==DiagnosticStatus.OK for s in statuses)
            self.received['health']=monotonic()

    def on_output(self, msg):
        self.data.append({'t':monotonic()-self.start_time,'type':'cmd_vel','v':msg.linear.x,'w':msg.angular.z})

    def on_scan(self, msg):
        try:
            if not self.sensor('scan', msg):
                return
            tf=self.buffer.lookup_transform('base_link',msg.header.frame_id,rclpy.time.Time()).transform
            q=tf.rotation
            if abs(q.x)>.001 or abs(q.y)>.001:
                raise ValueError('nonplanar scan mounting')
            yaw=math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
            points=[]
            finite=0
            for index, distance in enumerate(msg.ranges):
                if not math.isfinite(distance) or not msg.range_min <= distance <= msg.range_max:
                    continue
                finite+=1
                angle=msg.angle_min+index*msg.angle_increment+yaw
                x=tf.translation.x+distance*math.cos(angle)
                y=tf.translation.y+distance*math.sin(angle)
                if x>0. and abs(y)<.38:
                    points.append(x)
            if finite<20:
                raise ValueError('insufficient scan returns')
            self.obstacles=min(points,default=float('inf'))
        except Exception:
            self.obstacles=None

    def tick(self):
        now=monotonic()
        fresh=self.health and all(now-self.received.get(k,-1e20)<limit for k,limit in
                                  [('wheel',.15),('imu',.35),('scan',.25),('joy',.25),('health',2.)])
        clear=self.obstacles is not None and self.obstacles>max(0.,1.-self.policy.distance)+.55
        # Wait for discovery before judging publisher counts.
        auto_publishers=self.count_publishers('/cmd_vel_auto')
        final_publishers=self.count_publishers('/cmd_vel')
        exclusive=(auto_publishers<=1 and final_publishers<=1)
        fresh=fresh and auto_publishers==1 and final_publishers==1
        if now-self.start_time<2.:
            self.pub.publish(Twist())
            return False
        command=self.policy.tick(now,self.mode,self.pose,self.v,fresh,clear,exclusive,self.imu_yaw)
        if now-self.start_time>120 and self.policy.state=='WAITING':
            command=self.policy.stop('start_wait_timeout')
        msg=Twist();msg.linear.x=command;self.pub.publish(msg)
        key=(self.policy.state,self.policy.reason)
        if key!=self.last_log:
            print(json.dumps({'state':key[0],'reason':key[1],'fresh':fresh,'path_clear':clear,
                              'nearest_front_x':self.obstacles,'mode':self.mode}),flush=True)
            self.last_log=key
        if self.policy.state in ('COMPLETE','ABORTED'):
            if self.finish_at is None:
                self.finish_at=now
            if now-self.finish_at>=2.:
                if not fresh or abs(self.v)>.02:
                    self.policy.stop('stop_not_confirmed_by_feedback')
                return True
            return False
        return False

    def save(self):
        p=self.policy
        result={'state':p.state,'reason':p.reason,'target_m':1.,'max_command_mps':.1,
                'origin':p.origin,'final_pose':self.pose,'progress_at_stop_m':p.distance,
                'lateral_at_stop_m':p.lateral,'yaw_at_stop_rad':p.heading,
                'integrated_imu_yaw_rad':self.imu_yaw,
                'physical_distance_m':None,'note':'Independent tape measurement is required; odometry is not ground truth.'}
        (self.folder/'result.json').write_text(json.dumps(result,indent=2)+'\n')
        with (self.folder/'samples.jsonl').open('w') as f:
            for row in self.data:f.write(json.dumps(row)+'\n')
        print(json.dumps(result),flush=True)


def main():
    import argparse
    parser=argparse.ArgumentParser()
    parser.add_argument('--output',required=True)
    args=parser.parse_args()
    folder=Path(args.output);folder.mkdir(parents=True,exist_ok=False)
    stop=Event();previous={}
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node=None
    try:
        for sig in (signal.SIGINT,signal.SIGTERM):previous[sig]=signal.signal(sig,lambda *_:stop.set())
        node=Probe(folder)
        next_tick=monotonic()
        while rclpy.ok() and not stop.is_set():
            rclpy.spin_once(node,timeout_sec=.01)
            if monotonic()>=next_tick:
                if node.tick():break
                next_tick=monotonic()+.05
    finally:
        if node is not None:
            if node.policy.state not in ('COMPLETE','ABORTED'):node.policy.stop('interrupted')
            deadline=monotonic()+.5
            while rclpy.ok() and monotonic()<deadline:
                node.pub.publish(Twist());rclpy.spin_once(node,timeout_sec=.02)
            node.save();node.destroy_node()
        if rclpy.ok():rclpy.shutdown()
        for sig,handler in previous.items():signal.signal(sig,handler)


if __name__=='__main__':main()
