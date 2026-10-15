#!/usr/bin/env python3
"""One supervised left 90-degree turn, all commands through the existing mux.

The gyro controls the stopping point; floor markings provide independent truth.
Wheel disagreement, reverse rotation, translation, stale inputs and obstacles abort.
"""
import argparse
import json
import math
from pathlib import Path
import signal
from threading import Event
from time import monotonic

import rclpy
from geometry_msgs.msg import Twist
from rclpy.signals import SignalHandlerOptions

from straight_1m_test import Probe
from amr_control.rotation_test import RotationTest


class RotationProbe(Probe):
    def __init__(self,folder):
        super().__init__(folder,node_name='rotation_90_test')
        self.policy=RotationTest()
        self.angular=0.
        self.gyro_z=0.
        self.last_progress_log=0.

    def on_wheel(self,msg):
        if msg.header.frame_id!='odom' or msg.child_frame_id!='base_footprint':return
        if not self.sensor('wheel',msg):return
        super().on_wheel(msg)
        self.angular=msg.twist.twist.angular.z

    def on_imu(self,msg):
        if msg.header.frame_id!='imu_link' or not math.isfinite(msg.angular_velocity.z):return
        if not self.sensor('imu',msg):return
        stamp=msg.header.stamp.sec+msg.header.stamp.nanosec/1e9
        self.gyro_z=msg.angular_velocity.z
        # Continue integration through stopping and settling for the final comparison.
        if self.policy.started is not None:
            if self.previous_gyro is not None:
                previous_stamp,previous_rate=self.previous_gyro
                dt=stamp-previous_stamp
                if not 0.<dt<.35:
                    if self.policy.state=='RUNNING':self.policy.stop('invalid_imu_sample_interval')
                else:self.imu_yaw+=dt*(previous_rate+self.gyro_z)/2
            self.previous_gyro=(stamp,self.gyro_z)
        else:self.previous_gyro=None
        self.data.append({'t':monotonic()-self.start_time,'stamp':stamp,'type':'imu',
                          'gyro_z':self.gyro_z,'integrated_yaw':self.imu_yaw})

    def on_scan(self,msg):
        try:
            if not self.sensor('scan',msg):return
            tf=self.buffer.lookup_transform('base_link',msg.header.frame_id,rclpy.time.Time()).transform
            q=tf.rotation
            if abs(q.x)>.001 or abs(q.y)>.001:raise ValueError('nonplanar mounting')
            yaw=math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
            distances=[]
            for index,distance in enumerate(msg.ranges):
                if not math.isfinite(distance) or not msg.range_min<=distance<=msg.range_max:continue
                angle=msg.angle_min+index*msg.angle_increment+yaw
                x=tf.translation.x+distance*math.cos(angle)
                y=tf.translation.y+distance*math.sin(angle)
                distances.append(math.hypot(x,y))
            if len(distances)<20:raise ValueError('insufficient scan returns')
            self.obstacles=min(distances)
        except Exception:self.obstacles=None

    def tick(self):
        now=monotonic()
        fresh=self.health and all(now-self.received.get(k,-1e20)<limit for k,limit in
                                  [('wheel',.15),('imu',.35),('scan',.25),('joy',.25),('health',2.)])
        clear=self.obstacles is not None and self.obstacles>.60
        auto_count=self.count_publishers('/cmd_vel_auto');final_count=self.count_publishers('/cmd_vel')
        exclusive=auto_count<=1 and final_count<=1
        fresh=fresh and auto_count==1 and final_count==1
        if now-self.start_time<2.:
            self.pub.publish(Twist());return False
        command=self.policy.tick(now,self.mode,self.pose,self.v,self.angular,self.gyro_z,
                                 self.imu_yaw,fresh,clear,exclusive)
        if now-self.start_time>150 and self.policy.state=='WAITING':command=self.policy.stop('start_wait_timeout')
        msg=Twist();msg.angular.z=command;self.pub.publish(msg)
        key=(self.policy.state,self.policy.reason)
        if key!=self.last_log or (self.policy.state=='RUNNING' and now-self.last_progress_log>2.):
            print(json.dumps({'state':key[0],'reason':key[1],'fresh':fresh,'sweep_clear':clear,
                'nearest_obstacle_m':self.obstacles,'mode':self.mode,'command_w':command,
                'wheel_deg':math.degrees(self.policy.wheel_angle),'imu_deg':math.degrees(self.imu_yaw)}),flush=True)
            self.last_log=key;self.last_progress_log=now
        if self.policy.state in ('COMPLETE','ABORTED'):
            if self.finish_at is None:self.finish_at=now
            if now-self.finish_at>=2.:
                if not fresh or abs(self.v)>.02 or abs(self.angular)>.04 or abs(self.gyro_z)>.02:
                    self.policy.stop('stop_not_confirmed_by_feedback')
                return True
        return False

    def save(self):
        p=self.policy
        wheel_angle=None
        if p.origin is not None and self.pose is not None:
            wheel_angle=math.atan2(math.sin(self.pose[2]-p.origin[2]),math.cos(self.pose[2]-p.origin[2]))
        result={'state':p.state,'reason':p.reason,'target_deg':90.,'max_command_radps':.15,
                'origin':p.origin,'final_pose':self.pose,'final_wheel_yaw_deg':math.degrees(wheel_angle) if wheel_angle is not None else None,
                'final_imu_yaw_deg':math.degrees(self.imu_yaw),'translation_m':p.translation,
                'final_measured_v':self.v,'final_measured_w':self.angular,'final_gyro_z':self.gyro_z,
                'physical_angle_deg':None,'note':'Gyro stopping target is not independent verification; compare to floor reference.'}
        (self.folder/'result.json').write_text(json.dumps(result,indent=2)+'\n')
        with (self.folder/'samples.jsonl').open('w') as f:
            for row in self.data:f.write(json.dumps(row)+'\n')
        print(json.dumps(result),flush=True)


def main():
    parser=argparse.ArgumentParser();parser.add_argument('--output',required=True);args=parser.parse_args()
    folder=Path(args.output);folder.mkdir(parents=True,exist_ok=False)
    stop=Event();previous={};node=None
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    try:
        for sig in (signal.SIGINT,signal.SIGTERM):previous[sig]=signal.signal(sig,lambda *_:stop.set())
        node=RotationProbe(folder);next_tick=monotonic()
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
