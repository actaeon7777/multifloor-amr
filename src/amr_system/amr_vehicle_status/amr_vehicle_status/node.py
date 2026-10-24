"""Publish one reusable vehicle status topic; subscribe only, never command hardware."""
import json
import math
import time
import uuid

import rclpy
from amr_control.time_guard import ClockMonitor, HealthLease, SessionOrder
from amr_vehicle_interfaces.msg import DataStatus, MotorStatus, VehicleStatus
from diagnostic_msgs.msg import DiagnosticArray
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import Imu, JointState, LaserScan
from std_msgs.msg import String
from tf2_ros import Buffer, TransformException, TransformListener

from .model import StatusModel, TIMEOUTS

ESTOP_STATES = {'UNKNOWN','ESTOP','DISARMED','ARMING','READY','BYPASS'}
LOCALIZATION_STATES = {'WAITING_DATA','SEARCHING','VERIFYING','WAITING_INITIAL_POSE',
                       'STARTING_NAVIGATION','READY','FAILED','SUSPENDED'}


class VehicleStatusNode(Node):
    def __init__(self, **kwargs):
        super().__init__('vehicle_status', **kwargs)
        rate = self.declare_parameter('publish_rate', 10.).value
        if not math.isfinite(rate) or not 1 <= rate <= 50:
            raise ValueError('publish_rate must be finite and between 1 and 50 Hz')
        self.model = StatusModel()
        self.session, self.sequence, self.epoch = str(uuid.uuid4()), 0, 0
        self.clock_monitor = ClockMonitor()
        self.health = HealthLease()
        self.estop = HealthLease()
        self.localization_order = SessionOrder()
        self.control_mode = self.navigation = 'UNKNOWN'
        self.mode_received = self.navigation_received = False
        self.tf = Buffer(node=self)
        self.listener = TransformListener(self.tf, self, spin_thread=False)
        self.publisher = self.create_publisher(VehicleStatus, '/vehicle/status', QoSProfile(depth=1))
        self.sources = []
        for name, typ, topic in [('scan',LaserScan,'/scan'),('wheel',Odometry,'/wheel/odometry'),
                                 ('imu',Imu,'/imu/data_raw'),('ekf',Odometry,'/odometry/filtered'),
                                 ('joints',JointState,'/joint_states'),('command',Twist,'/cmd_vel'),
                                 ('diagnostics',DiagnosticArray,'/diagnostics')]:
            self.sources.append(self.create_subscription(typ,topic,
                lambda m,n=name:self.on_sample(n,m),qos_profile_sensor_data))
        for side in ('left', 'right'):
            self.sources.append(self.create_subscription(MotorStatus, '/motor/'+side+'/telemetry',
                lambda msg,s=side:self.on_motor(s,msg),qos_profile_sensor_data))
        retained = QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL)
        for topic, callback in [('/control_mux/mode',self.on_mode),('/navigation/status',self.on_navigation),
                                 ('/localization/status',self.on_localization),('/system/time_health',self.on_time),
                                 ('/estop/status',self.on_estop)]:
            self.sources.append(self.create_subscription(String,topic,callback,retained))
        self.create_timer(1/rate,self.tick,clock=Clock(clock_type=ClockType.STEADY_TIME))

    def ros_now(self):
        return self.get_clock().now().nanoseconds/1e9

    @staticmethod
    def stamp(value):
        return Time(nanoseconds=round(value*1e9)).to_msg()

    def on_mode(self, msg):
        self.control_mode = msg.data if msg.data in ('AUTO','MANUAL') else 'UNKNOWN'
        self.mode_received = self.control_mode != 'UNKNOWN'

    def on_navigation(self, msg):
        self.navigation = msg.data[:80]
        self.navigation_received = True

    def on_localization(self, msg):
        now, wall = time.monotonic(), time.time()
        try:
            value = json.loads(msg.data)
            if (not isinstance(value,dict) or value.get('state') not in LOCALIZATION_STATES
                    or type(value.get('ready')) is not bool or not isinstance(value.get('detail'),str)
                    or type(value.get('stamp')) not in (int,float) or not math.isfinite(value['stamp'])
                    or not -0.1 <= wall-value['stamp'] < 2
                    or not isinstance(value.get('session'),str)
                    or type(value.get('generation')) is not int or value['generation'] < 0
                    or not isinstance(value.get('tracking_warning',''),str)):
                raise ValueError('Invalid localization state')
            if not self.localization_order.accept(value['session'],value['generation'],value['stamp']):
                return
            self.model.observe('localization',dict(state=value['state'],
                ready=value['ready'] and value['state']=='READY',detail=value['detail'][:1000],
                warning=value.get('tracking_warning','')[:1000]),now,base_age=max(0.,wall-value['stamp']))
        except (ValueError,TypeError,KeyError):
            self.model.reject('localization',now)

    def on_time(self, msg):
        now = time.monotonic()
        if self.health.receive(msg.data,now=now):
            self.model.observe('time',dict(healthy=self.health.status['healthy'],
                latched=self.health.status['latched'],detail=str(self.health.status.get('detail',''))[:1000]),
                now,base_age=max(0.,time.time()-self.health.status['stamp']))
        elif not self.health.status:
            self.model.reject('time',now)

    def on_estop(self, msg):
        now = time.monotonic()
        if self.estop.receive(msg.data,now=now) and self.estop.status.get('state') in ESTOP_STATES:
            status = self.estop.status
            stm32 = status.get('stm32') if isinstance(status.get('stm32'),dict) else {}
            trips = stm32.get('trip_count',0)
            self.model.observe('estop',dict(state=status['state'],
                approval_required=status.get('approval_required') is True,
                pressed=stm32.get('raw') is True,stopped=stm32.get('stopped') is True,
                drive_enabled=stm32.get('drive_enabled') is True,
                trip_count=trips if type(trips) is int and 0 <= trips < 2**32 else 0,
                detail=str(status.get('detail',''))[:1000]),
                now,base_age=max(0.,time.time()-status['stamp']))
        elif not self.estop.status or self.estop.status.get('state') not in ESTOP_STATES:
            self.model.reject('estop',now)

    def on_sample(self, name, msg):
        now = time.monotonic()
        stamp = None
        if hasattr(msg,'header'):
            stamp = msg.header.stamp.sec+msg.header.stamp.nanosec/1e9
            if not -.1 <= self.ros_now()-stamp <= 2:
                self.model.reject(name,now)
                return
        values = {}
        if name in ('wheel','ekf','command'):
            twist = msg if name=='command' else msg.twist.twist
            values = dict(linear=twist.linear.x,angular=twist.angular.z)
            if not all(math.isfinite(v) for v in values.values()):
                self.model.reject(name,now)
                return
            if name=='wheel':
                p = msg.pose.pose.position
                if not all(math.isfinite(v) for v in (p.x,p.y)):
                    self.model.reject(name,now)
                    return
                values.update(x=p.x,y=p.y)
        elif name=='joints':
            for side in ('left','right'):
                joint = side+'_drive_wheel_joint'
                if joint in msg.name:
                    index = msg.name.index(joint)
                    if index<len(msg.velocity) and math.isfinite(msg.velocity[index]):
                        values[side+'_rpm'] = msg.velocity[index]*60/(2*math.pi)
            if len(values)!=2:
                self.model.reject(name,now)
                return
        elif name=='imu':
            if not all(math.isfinite(getattr(vector,axis)) for vector in
                       (msg.angular_velocity,msg.linear_acceleration) for axis in ('x','y','z')):
                self.model.reject(name,now)
                return
        elif name=='diagnostics':
            status = next((s for s in msg.status if s.name=='amr/can_telemetry'),None)
            if status is None:
                return
            counters = {v.key:v.value for v in status.values}
            try:
                values = {key:int(counters[key]) for key in
                          ('received','invalid','stale_frames','wheel_rebases','clock_resets')}
                if any(not 0<=v<2**64 for v in values.values()):
                    raise ValueError('Invalid counter')
            except (ValueError,TypeError,KeyError):
                self.model.reject(name,now)
                return
            values.update(level=status.level[0] if isinstance(status.level,bytes) else int(status.level),
                          detail=status.message[:1000])
        accepted = self.model.observe(name,values,now,stamp=stamp)
        if accepted and name=='wheel':
            self.model.wheel(values['x'],values['y'],stamp)

    def on_motor(self, side, msg):
        now, ros = time.monotonic(), self.ros_now()
        for source in ('power', 'heartbeat'):
            meta = getattr(msg, source)
            if not meta.received:
                continue  # Each CAN frame updates only its own source lease.
            name = side+'_'+source
            stamp = meta.source_stamp.sec+meta.source_stamp.nanosec/1e9
            if (msg.node_id != (1 if side == 'left' else 0) or not meta.valid
                    or not meta.stamp_valid or not -.1 <= ros-stamp < TIMEOUTS[name]):
                self.model.reject(name,now)
                continue
            if source == 'power':
                values = dict(voltage=msg.input_voltage_v,current=msg.bus_current_a)
                if not all(math.isfinite(v) for v in values.values()) or values['voltage'] <= 0:
                    self.model.reject(name,now)
                    continue
            else:
                values = dict(error_code=msg.error_code,axis_state=msg.axis_state)
            self.model.observe(name,values,now,stamp=stamp)

    def update_pose(self):
        try:
            transform = self.tf.lookup_transform('map','base_footprint',Time())
            stamp = transform.header.stamp.sec+transform.header.stamp.nanosec/1e9
            if not -.1<=self.ros_now()-stamp<=2:
                return
            p,q = transform.transform.translation,transform.transform.rotation
            values = dict(x=p.x,y=p.y,z=p.z,qx=q.x,qy=q.y,qz=q.z,qw=q.w)
            if all(math.isfinite(v) for v in values.values()):
                self.model.observe('pose',values,time.monotonic(),stamp=stamp)
        except TransformException:
            pass

    def message(self, now=None, ros=None):
        now = time.monotonic() if now is None else now
        ros = self.ros_now() if ros is None else ros
        samples = {name:self.model.snapshot(name,now,ros) for name in TIMEOUTS}
        msg = VehicleStatus()
        msg.header.stamp, msg.header.frame_id = self.stamp(ros),'map'
        msg.published_wall_stamp = self.stamp(time.time())
        self.sequence += 1
        msg.publisher_session,msg.sequence,msg.epoch = self.session,self.sequence,self.epoch
        for name in ('scan','wheel','imu','ekf','joints','command','diagnostics'):
            setattr(msg,name,self.metadata(samples[name]))
        for name in ('command','wheel','ekf'):
            sample = samples[name]
            if sample['valid']:
                twist = getattr(msg,name+'_velocity')
                twist.linear.x,twist.angular.z = sample['values']['linear'],sample['values']['angular']
        for side,node_id in [('left',1),('right',0)]:
            motor = getattr(msg,side+'_motor')
            motor.node_id,motor.encoder = node_id,self.metadata(samples['joints'])
            motor.wheel_rpm = samples['joints']['values'].get(side+'_rpm',0.)
            power, heartbeat = samples[side+'_power'], samples[side+'_heartbeat']
            motor.power, motor.heartbeat = self.metadata(power), self.metadata(heartbeat)
            motor.input_voltage_v = power['values'].get('voltage',0.)
            motor.bus_current_a = power['values'].get('current',0.)
            motor.error_code = heartbeat['values'].get('error_code',0)
            motor.axis_state = heartbeat['values'].get('axis_state',0)
        left, right = samples['left_power'], samples['right_power']
        if left['received'] and right['received']:
            # Derived voltage expires with the older of the two actual CAN samples.
            older = left if left['age_sec'] >= right['age_sec'] else right
            msg.battery_status = self.metadata(older)
            msg.battery_status.valid = left['valid'] and right['valid']
            msg.battery_status.fresh = left['fresh'] and right['fresh']
            msg.battery_status.rate_valid = False
            msg.battery_status.rate_hz = 0.
            msg.battery_voltage_available = msg.battery_status.fresh
            if msg.battery_voltage_available:
                msg.battery_voltage_v = (left['values']['voltage']+right['values']['voltage'])/2
            # This is driver input voltage, not a BMS SOC/current/temperature reading.
        msg.wheel_distance_m,msg.wheel_distance_epoch = self.model.distance,self.model.distance_epoch
        msg.wheel_distance_valid = samples['wheel']['fresh'] and self.model.last_wheel is not None
        msg.pose_status = self.metadata(samples['pose'])
        p = samples['pose']['values']
        if p:
            msg.map_pose.position.x,msg.map_pose.position.y,msg.map_pose.position.z = p['x'],p['y'],p['z']
            q = msg.map_pose.orientation
            q.x,q.y,q.z,q.w = p['qx'],p['qy'],p['qz'],p['qw']
        msg.control_mode_received,msg.control_mode = self.mode_received,self.control_mode
        msg.navigation_state_received,msg.navigation_state = self.navigation_received,self.navigation
        loc = samples['localization']
        msg.localization_status = self.metadata(loc)
        msg.localization_state = loc['values'].get('state','UNKNOWN')
        msg.localization_ready = loc['fresh'] and loc['values'].get('ready',False)
        msg.localization_detail = loc['values'].get('detail','')
        msg.tracking_warning = loc['values'].get('warning','')
        health = samples['time']
        msg.time_status = self.metadata(health)
        msg.time_healthy = health['fresh'] and self.health.healthy(now)
        msg.time_fault_latched = health['values'].get('latched',False)
        msg.time_detail = health['values'].get('detail','')
        estop = samples['estop']
        msg.estop_status = self.metadata(estop)
        values = estop['values'] if estop['fresh'] else {}
        msg.estop_state = values.get('state','UNKNOWN')
        msg.estop_approval_required = values.get('approval_required',False)
        msg.estop_button_pressed = values.get('pressed',False)
        msg.estop_stm32_stopped = values.get('stopped',False)
        msg.estop_drive_enabled = values.get('drive_enabled',False)
        msg.estop_trip_count = estop['values'].get('trip_count',0)
        msg.estop_detail = values.get('detail','E-stop 상태 수신 시간이 초과되었습니다.'
                                      if estop['received'] else '')
        diag = samples['diagnostics']['values']
        msg.diagnostic_level,msg.diagnostic_detail = diag.get('level',3),diag.get('detail','')
        for field,key in [('received_frames','received'),('invalid_frames','invalid'),('stale_frames','stale_frames'),
                          ('wheel_rebases','wheel_rebases'),('clock_resets','clock_resets')]:
            setattr(msg,field,diag.get(key,0))
        return msg

    @classmethod
    def metadata(cls, sample):
        meta = DataStatus()
        meta.received,meta.valid,meta.fresh = sample['received'],sample['valid'],sample['fresh']
        meta.stamp_valid = sample['stamp'] is not None
        if meta.stamp_valid:
            meta.source_stamp = cls.stamp(sample['stamp'])
        meta.age_sec,meta.timeout_sec = sample['age_sec'],sample['timeout_sec']
        meta.rate_valid = sample['rate_hz'] is not None
        meta.rate_hz = sample['rate_hz'] or 0.
        return meta

    def tick(self):
        ros,now = self.ros_now(),time.monotonic()
        if self.clock_monitor.observe(ros,now,self.get_parameter('use_sim_time').value):
            self.epoch += 1
            self.model.reset()
            self.tf.clear()
        self.update_pose()
        self.publisher.publish(self.message(now,ros))


def main(args=None):
    rclpy.init(args=args)
    node = VehicleStatusNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
