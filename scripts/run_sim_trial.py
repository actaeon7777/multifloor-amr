#!/usr/bin/env python3
"""One isolated physical Nav2 trial. Synthetic Joy is strictly limited to domain 107."""
import argparse
import json
import hashlib
import yaml
import math
import os
from pathlib import Path
import signal
import subprocess
import time

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import QoSProfile, DurabilityPolicy, qos_profile_sensor_data
from geometry_msgs.msg import PoseWithCovarianceStamped, Twist
from nav_msgs.msg import Odometry, Path as NavPath
from nav2_msgs.action import NavigateToPose
from sensor_msgs.msg import Joy, LaserScan
from std_msgs.msg import String
from tf2_ros import Buffer, TransformListener
from rclpy.time import Time
from action_msgs.msg import GoalStatus
from lifecycle_msgs.srv import GetState
from rcl_interfaces.srv import ListParameters, GetParameters
from rclpy.parameter import parameter_value_to_python

ROOT=Path(__file__).resolve().parents[1]


class Trial(Node):
    def __init__(self, spawn=(0., 0., 0.)):
        super().__init__('sim_trial', parameter_overrides=[Parameter('use_sim_time', value=True)])
        self.spawn = spawn
        self.joy_pub=self.create_publisher(Joy,'/joy',10)
        self.pose_pub=self.create_publisher(PoseWithCovarianceStamped,'/initialpose',10)
        self.action=ActionClient(self,NavigateToPose,'/navigate_to_pose')
        self.truth=None; self.odom=None; self.scan=None; self.mode=''; self.sim_status={}; self.recoveries=0
        self.localization_status = {}
        self.cmd=(0.,0.); self.path=[]; self.samples=[]; self.command_times=[]; self.raw_command_times=[]
        self.tf=Buffer(); self.listener=TransformListener(self.tf,self)
        self.subs=[
            self.create_subscription(Odometry,'/sim/ground_truth',lambda m:setattr(self,'truth',m),10),
            self.create_subscription(Odometry,'/odometry/filtered',lambda m:setattr(self,'odom',m),10),
            self.create_subscription(LaserScan,'/scan',lambda m:setattr(self,'scan',m),qos_profile_sensor_data),
            self.create_subscription(Twist,'/cmd_vel',self.command,100),
            self.create_subscription(Twist,'/cmd_vel_navigation',lambda m:self.raw_command_times.append(time.monotonic()),100),
            self.create_subscription(NavPath,'/plan',lambda m:setattr(self,'path',[[p.pose.position.x,p.pose.position.y] for p in m.poses]),10),
            self.create_subscription(String,'/control_mux/mode',lambda m:setattr(self,'mode',m.data),QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL)),
            self.create_subscription(String,'/sim/status',lambda m:setattr(self,'sim_status',json.loads(m.data)),QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL)),
            self.create_subscription(String,'/localization/status',lambda m:setattr(self,'localization_status',json.loads(m.data)),QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL)),
        ]
        self.rb=False; self.previous_joy=0.; self.previous_sample=0.

    def command(self,msg):
        self.cmd=(msg.linear.x,msg.angular.z)
        self.command_times.append(time.monotonic())

    def spin(self,seconds=.05):
        end=time.monotonic()+seconds
        while time.monotonic()<end:
            rclpy.spin_once(self,timeout_sec=.01)
            now=time.monotonic()
            if now-self.previous_joy>=.05:
                self.previous_joy=now
                joy=Joy();joy.axes=[0.]*4;joy.buttons=[0]*8;joy.buttons[7]=int(self.rb)
                self.joy_pub.publish(joy)
            if self.truth and now-self.previous_sample>=.05:
                self.previous_sample=now
                estimate=None
                try:
                    transform=self.tf.lookup_transform('map','base_footprint',Time())
                    p=transform.transform.translation; q=transform.transform.rotation
                    estimate=[p.x,p.y,math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))]
                except Exception: pass
                p=self.truth.pose.pose.position;q=self.truth.pose.pose.orientation
                self.samples.append({'t':self.truth.header.stamp.sec+self.truth.header.stamp.nanosec/1e9,
                                     'truth':[p.x,p.y,math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))],
                                     'estimate':estimate,'cmd':self.cmd,'path':self.path,
                                     'rtf':self.sim_status.get('real_time_factor'),
                                     'scan_age_s':self.truth.header.stamp.sec+self.truth.header.stamp.nanosec/1e9-(self.scan.header.stamp.sec+self.scan.header.stamp.nanosec/1e9) if self.scan else None,
                                     'odom_age_s':self.truth.header.stamp.sec+self.truth.header.stamp.nanosec/1e9-(self.odom.header.stamp.sec+self.odom.header.stamp.nanosec/1e9) if self.odom else None})

    def until(self,predicate,timeout):
        deadline=time.monotonic()+timeout
        while not predicate():
            if time.monotonic()>deadline: raise TimeoutError('Trial timed out')
            self.spin()

    def initialize(self, automatic=False):
        self.until(lambda:self.truth is not None and self.odom is not None and self.scan is not None,75)
        if automatic:
            self.until(lambda: self.localization_status.get('ready') is True, 90)
        # Initial pose is the declared test setup, not feedback from ground truth.
        for _ in range(0 if automatic else 5):
            pose=PoseWithCovarianceStamped();pose.header.frame_id='map';pose.header.stamp=self.get_clock().now().to_msg()
            pose.pose.pose.position.x, pose.pose.pose.position.y = self.spawn[:2]
            pose.pose.pose.orientation.z=math.sin(self.spawn[2]/2);pose.pose.pose.orientation.w=math.cos(self.spawn[2]/2)
            pose.pose.covariance[0]=.01;pose.pose.covariance[7]=.01;pose.pose.covariance[35]=.01
            self.pose_pub.publish(pose);self.spin(.3)
        self.until(lambda:self.tf.can_transform('map','base_footprint',Time()),20)
        self.spin(.3);self.rb=True;self.spin(.15);self.rb=False
        self.until(lambda:self.mode=='AUTO',3)
        self.until(lambda:self.action.server_is_ready(),30)
        for name in ('amcl','planner_server','controller_server','behavior_server','bt_navigator'):
            client=self.create_client(GetState,f'/{name}/get_state')
            self.until(client.service_is_ready,30)
            deadline=time.monotonic()+30
            while True:
                future=client.call_async(GetState.Request())
                self.until(future.done,5)
                if future.result().current_state.id==3:break
                if time.monotonic()>deadline:raise TimeoutError(f'{name} did not activate')
                self.spin(.3)
            self.destroy_client(client)
        # Public Action existence and ACTIVE lifecycle states can precede the
        # supervisor's final readiness heartbeat. Use the same gate as the UI.
        self.until(lambda: self.localization_status.get('ready') is True
                   and self.localization_status.get('state') == 'READY'
                   and 0 <= time.time()-self.localization_status.get('stamp', 0) < 2, 30)

    def goal(self,x,y,yaw,timeout):
        goal=NavigateToPose.Goal();goal.pose.header.frame_id='map';goal.pose.header.stamp=self.get_clock().now().to_msg()
        goal.pose.pose.position.x=float(x);goal.pose.pose.position.y=float(y)
        goal.pose.pose.orientation.z=math.sin(yaw/2);goal.pose.pose.orientation.w=math.cos(yaw/2)
        sent=self.action.send_goal_async(goal,feedback_callback=lambda msg:setattr(self,'recoveries',max(self.recoveries,msg.feedback.number_of_recoveries)))
        self.until(sent.done,10)
        handle=sent.result()
        if not handle.accepted: raise RuntimeError('Navigation manager rejected goal')
        result=handle.get_result_async()
        try: self.until(result.done,timeout)
        except TimeoutError:
            handle.cancel_goal_async();self.spin(1.);raise
        self.spin(1.)
        return result.result().status


def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('--goal',nargs=3,type=float,default=[2.,0.,0.])
    parser.add_argument('--world',default='arena',choices=['arena','corner_left','corner_right','s_curve','narrow','obstacle','floor4'])
    parser.add_argument('--seed',type=int,default=0)
    parser.add_argument('--timeout',type=float,default=100.,help='Goal execution wall-clock limit in seconds')
    parser.add_argument('--navigation-config',type=Path,default=ROOT/'src/amr_system/amr_stack/config/navigation.yaml')
    parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args()
    if not math.isfinite(args.timeout) or args.timeout <= 0:
        parser.error('--timeout must be positive and finite')
    if os.environ.get('ROS_DOMAIN_ID')!='107':raise SystemExit('Only domain 107 is allowed. Source env, then export ROS_DOMAIN_ID=107.')
    if args.output.exists():raise SystemExit('Output already exists; each trial requires a fresh directory')
    args.output.mkdir(parents=True)
    log=(args.output/'launch.log').open('w')
    process=subprocess.Popen([str(ROOT/'scripts/experiment.sh'),'sim','headless:=true','start_joy:=false',f'world:={args.world}',f'seed:={args.seed}',
                              f'navigation_config:={args.navigation_config}'],stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
    bag_log=(args.output/'bag.log').open('w')
    recorder=subprocess.Popen(['ros2','bag','record','--use-sim-time','--include-hidden-topics','-s','sqlite3','-o',str(args.output/'bag'),
        '/clock','/cmd_vel','/cmd_vel_navigation','/cmd_vel_auto','/wheel/odometry','/joint_states',
        '/imu/data_raw','/scan','/odometry/filtered','/amcl_pose','/tf','/tf_static','/plan',
        '/map','/local_costmap/costmap','/local_costmap/costmap_updates',
        '/global_costmap/costmap','/global_costmap/costmap_updates',
        '/local_costmap/published_footprint','/local_plan','/transformed_global_plan',
        '/rosout','/diagnostics','/parameter_events','/initialpose','/goal_pose','/localization/status',
        '/navigate_to_pose/_action/status','/navigation/navigate_to_pose/_action/status',
        '/navigate_to_pose/_action/feedback','/navigation/navigate_to_pose/_action/feedback',
        '/control_mux/mode','/control_mux/status','/navigation/status','/sim/status','/sim/ground_truth'],
        stdout=bag_log,stderr=subprocess.STDOUT,start_new_session=True)
    spawn = json.loads((ROOT/f'sim/config/{args.world}.json').read_text()).get('spawn', [0., 0., 0.])
    rclpy.init(args=[]);node=Trial(spawn)
    navigation=yaml.safe_load(args.navigation_config.read_text())
    frozen_navigation=json.loads(json.dumps(navigation))
    frozen_navigation['controller_server']['ros__parameters'].pop('FollowPath',None)
    frozen={'navigation_without_dwb':frozen_navigation}
    for directory in ('sim/scripts','src/amr_system/amr_sim_bridge','src/amr_system/amr_can_bridge',
                      'src/amr_system/amr_stack/amr_stack'):
        for path in (ROOT/directory).rglob('*.py'):
            frozen[str(path.relative_to(ROOT))]=hashlib.sha256(path.read_bytes()).hexdigest()
    for path in [ROOT/'sim/config/physics.json',ROOT/'sim/config/sensors.json',ROOT/f'sim/config/{args.world}.json',
                 ROOT/'src/amr_system/amr_stack/config/ekf.yaml',ROOT/'src/amr_system/amr_stack/config/localization.yaml',
                 ROOT/'src/amr_system/amr_stack/config/localization_startup.yaml',
                 ROOT/'src/amr_system/amr_stack/config/control_mux.yaml']:
        frozen[path.name]=hashlib.sha256(path.read_bytes()).hexdigest()
    (args.output/'navigation.yaml').write_text(args.navigation_config.read_text())
    report={'fixed_inputs_sha256':hashlib.sha256(json.dumps(frozen,sort_keys=True).encode()).hexdigest(),'passed':False,'calibration_status':'uncalibrated','seed':args.seed,'goal':args.goal,'world':args.world,'goal_timeout_sec':args.timeout}
    try:
        from capture_manifest import manifest
        (args.output/'manifest.json').write_text(json.dumps(manifest('sim',args.output.name,[args.navigation_config]),indent=2)+'\n')
        node.initialize()
        params=args.output/'params';params.mkdir()
        for name in ('control_mux','controller_server','planner_server','ekf_filter_node','amcl',
                     'navigation_startup','navigation_manager',
                     'local_costmap/local_costmap','global_costmap/global_costmap'):
            list_client=node.create_client(ListParameters,f'/{name}/list_parameters')
            get_client=node.create_client(GetParameters,f'/{name}/get_parameters')
            node.until(lambda:list_client.service_is_ready() and get_client.service_is_ready(),10)
            listed=list_client.call_async(ListParameters.Request())
            node.until(listed.done,10)
            names=listed.result().result.names
            values=get_client.call_async(GetParameters.Request(names=names))
            node.until(values.done,10)
            if len(values.result().values)==len(names):
                actual=dict(zip(names,map(parameter_value_to_python,values.result().values)))
            else:
                # robot_localization declares unused imu1/odom1 slots without values;
                # querying one such slot makes a bulk get return an empty response.
                actual={}
                for parameter_name in names:
                    value=get_client.call_async(GetParameters.Request(names=[parameter_name]))
                    node.until(value.done,5)
                    if value.result().values:
                        actual[parameter_name]=parameter_value_to_python(value.result().values[0])
            if actual.get('use_sim_time') is not True:raise RuntimeError(f'{name}: use_sim_time must be true')
            (params/(name.replace('/','__')+'.yaml')).write_text(yaml.safe_dump({'/'+name:{'ros__parameters':actual}}))
            node.destroy_client(list_client);node.destroy_client(get_client)
        node.spin(.5)
        node.samples.clear();node.raw_command_times.clear()
        status=node.goal(*args.goal,timeout=args.timeout)
        report['action_status']=status
        report['goal_succeeded']=status==GoalStatus.STATUS_SUCCEEDED
        report['stopped']=bool(node.odom and abs(node.odom.twist.twist.linear.x)<.03 and abs(node.odom.twist.twist.angular.z)<.05)
        report['passed']=report['goal_succeeded'] and report['stopped'] and node.sim_status.get('obstacle_contact_events')==0
        report['recoveries']=node.recoveries
        report['simulation']=node.sim_status
        gaps=[b-a for a,b in zip(node.raw_command_times,node.raw_command_times[1:])]
        report['command_timing']={'samples':len(gaps),'mean_period_s':sum(gaps)/len(gaps) if gaps else None,
                                  'max_period_s':max(gaps) if gaps else None,'over_75ms':sum(dt>.075 for dt in gaps)}
        if not report['passed']:raise RuntimeError('Goal did not succeed and stop')
    except Exception as exc:
        report['error']=str(exc)
    finally:
        # Stop synthetic Joy first. No real transport can be running in this mode.
        node.destroy_node();rclpy.shutdown()
        if process.poll() is None:
            os.killpg(process.pid,signal.SIGINT)
            try:process.wait(timeout=20)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid,signal.SIGTERM)
                try:process.wait(timeout=10)
                except subprocess.TimeoutExpired:os.killpg(process.pid,signal.SIGKILL);process.wait()
        if recorder.poll() is None:
            os.killpg(recorder.pid,signal.SIGINT)
            try:recorder.wait(timeout=15)
            except subprocess.TimeoutExpired:os.killpg(recorder.pid,signal.SIGTERM);recorder.wait(timeout=5)
        report['recorder_exit_code'] = recorder.returncode
        report['recording_complete'] = (recorder.returncode in (0, -signal.SIGINT, 130)
                                        and (args.output/'bag/metadata.yaml').is_file())
        if not report['recording_complete']:
            report['passed'] = False
            report.setdefault('error', 'Bag recording did not finish successfully; see bag.log')
        bag_log.close()
        log.close()
        (args.output/'samples.json').write_text(json.dumps(node.samples)+'\n')
        (args.output/'result.json').write_text(json.dumps(report,indent=2)+'\n')
        from evaluate_trial import evaluate
        metrics=evaluate(node.samples,report)
        metrics['world']=args.world;metrics['seed']=args.seed
        (args.output/'metrics.json').write_text(json.dumps(metrics,indent=2)+'\n')
    print(json.dumps(report,indent=2))
    return 0 if report['passed'] else 1

if __name__=='__main__':raise SystemExit(main())
