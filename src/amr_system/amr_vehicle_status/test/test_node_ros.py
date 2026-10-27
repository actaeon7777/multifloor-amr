"""Typed contract and leases in isolated ROS domain 195, without vehicle IO."""
import json
import math
import os
import time

import pytest

if os.environ.get('AMR_RUN_VEHICLE_STATUS_TESTS')!='1':
    pytest.skip('Set AMR_RUN_VEHICLE_STATUS_TESTS=1 in isolated ROS domain 195',allow_module_level=True)

import rclpy
from sensor_msgs.msg import JointState
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from std_msgs.msg import String
from amr_vehicle_status.node import VehicleStatusNode


@pytest.fixture
def node():
    rclpy.init()
    node = VehicleStatusNode()
    try:
        yield node
    finally:
        node.destroy_node()
        rclpy.shutdown()


def test_unknown_fields_and_only_state_publisher(node):
    msg = node.message()
    assert msg.header.frame_id=='map' and msg.publisher_session
    assert msg.left_motor.node_id==1 and msg.right_motor.node_id==0
    assert not msg.left_motor.encoder.received and not msg.left_motor.heartbeat.received
    assert not msg.battery_status.received and not msg.battery_soc_available
    assert not msg.time_healthy and not msg.localization_ready
    assert not any(p.node_name==node.get_name() for p in node.get_publishers_info_by_topic('/cmd_vel'))
    assert node.message().sequence==msg.sequence+1


def test_invalid_and_expired_encoder_samples(node):
    msg = JointState()
    msg.header.stamp = node.get_clock().now().to_msg()
    msg.name = ['right_drive_wheel_joint','left_drive_wheel_joint']
    msg.velocity = [math.pi,2*math.pi]
    node.on_sample('joints',msg)
    status = node.message()
    assert status.left_motor.encoder.fresh
    assert status.left_motor.wheel_rpm==pytest.approx(60.)
    assert status.right_motor.wheel_rpm==pytest.approx(30.)
    later = node.message(now=time.monotonic()+1)
    assert later.joints.valid and not later.joints.fresh
    msg.header.stamp = node.get_clock().now().to_msg()
    msg.velocity = [float('nan'),2*math.pi]
    node.on_sample('joints',msg)
    assert node.message().joints.received and not node.message().joints.valid
    assert not node.message().left_motor.encoder.fresh


def localization(session='first',generation=1,**extra):
    return String(data=json.dumps(dict(state='READY',ready=True,detail='verified',
        stamp=time.time(),session=session,generation=generation,**extra)))


def test_localization_retired_sessions_and_expiry(node):
    node.on_localization(localization(generation=999))
    assert node.message().localization_ready
    node.on_localization(localization(session='second',generation=0))
    assert node.message().localization_ready
    node.on_localization(localization(generation=1000))
    assert node.localization_order.session=='second'
    assert node.message().localization_ready
    assert not node.message(now=time.monotonic()+2.1).localization_ready
    node.on_localization(String(data='{"state":"READY","ready":true}'))
    assert not node.message().localization_ready
    assert not node.message().localization_status.valid


def test_time_lease_expiry_even_when_ros_clock_stops(node):
    health = dict(session='time-test',sequence=1,epoch=0,stamp=time.time(),
                  healthy=True,latched=False,detail='valid')
    node.on_time(String(data=json.dumps(health)))
    assert node.message().time_healthy
    assert not node.message(now=time.monotonic()+.7,ros=node.ros_now()).time_healthy
    node.on_time(String(data=json.dumps({**health,'sequence':2,'stamp':time.time(),'healthy':False,'latched':True})))
    assert node.message().time_fault_latched and not node.message().time_healthy


def test_estop_fields_and_lease_expiry(node):
    msg = node.message()
    assert msg.estop_state=='UNKNOWN' and not msg.estop_status.received and msg.estop_detail==''
    status = dict(session='estop-test',sequence=1,epoch=1,stamp=time.time(),state='ARMING',
                  healthy=False,latched=False,detail='주행 준비 중',
                  stm32=dict(raw=False,stopped=False,drive_enabled=False,trip_count=3))
    node.on_estop(String(data=json.dumps(status)))
    msg = node.message()
    assert msg.estop_status.fresh and msg.estop_state=='ARMING'
    assert not msg.estop_button_pressed and not msg.estop_stm32_stopped and msg.estop_trip_count==3
    node.on_estop(String(data=json.dumps({**status,'sequence':2,'stamp':time.time(),'state':'ESTOP',
        'latched':True,'stm32':dict(raw=True,stopped=True,trip_count=4)})))
    msg = node.message()
    assert msg.estop_state=='ESTOP' and msg.estop_button_pressed and msg.estop_stm32_stopped
    stale = node.message(now=time.monotonic()+.7,ros=node.ros_now())
    assert stale.estop_state=='UNKNOWN' and not stale.estop_status.fresh and stale.estop_detail
    node.on_estop(String(data=json.dumps({**status,'sequence':3,'stamp':time.time(),'state':'BOGUS'})))
    assert not node.message().estop_status.valid


def test_clock_reset_clears_measurements_and_advances_epoch(node):
    node.model.observe('wheel',dict(linear=.3,angular=0.),time.monotonic(),stamp=node.ros_now())
    node.model.wheel(0.,0.,node.ros_now())
    node.clock_monitor.previous = (node.ros_now()+1.,time.monotonic())
    node.tick()
    msg = node.message()
    assert msg.epoch==1 and not msg.wheel.received and not msg.wheel_distance_valid


def test_missing_diagnostic_counters_do_not_become_known_zero(node):
    msg = DiagnosticArray()
    msg.header.stamp = node.get_clock().now().to_msg()
    msg.status = [DiagnosticStatus(name='amr/can_telemetry',values=[KeyValue(key='invalid',value='7')])]
    node.on_sample('diagnostics',msg)
    assert node.message().diagnostics.received
    assert not node.message().diagnostics.valid


def motor_sample(side, voltage=26., stamp=None):
    from amr_vehicle_interfaces.msg import MotorStatus
    msg = MotorStatus(node_id=1 if side=='left' else 0)
    msg.power.received = msg.power.valid = msg.power.fresh = msg.power.stamp_valid = True
    msg.power.source_stamp = VehicleStatusNode.stamp(time.time() if stamp is None else stamp)
    msg.input_voltage_v, msg.bus_current_a = voltage, -.5
    return msg


def test_voltage_average_requires_both_fresh_drivers(node):
    now = node.ros_now()
    node.on_motor('left',motor_sample('left',26.2,now-1.))
    assert not node.message().battery_voltage_available
    node.on_motor('right',motor_sample('right',26.4,now))
    msg = node.message()
    assert msg.left_motor.power.fresh and msg.right_motor.power.fresh
    assert msg.left_motor.bus_current_a == -.5
    assert msg.battery_voltage_available and msg.battery_voltage_v == pytest.approx(26.3)
    assert msg.battery_status.age_sec >= 1.
    assert not msg.battery_soc_available and not msg.battery_current_available
    later = node.message(now=time.monotonic()+1.6,ros=node.ros_now()+1.6)
    assert later.right_motor.power.fresh and not later.left_motor.power.fresh
    assert not later.battery_status.fresh and not later.battery_voltage_available


def test_heartbeat_does_not_renew_voltage_and_bad_power_is_unknown(node):
    from amr_vehicle_interfaces.msg import MotorStatus
    now = node.ros_now()
    node.on_motor('left',motor_sample('left',26.2,now-1.))
    heartbeat = MotorStatus(node_id=1,error_code=32,axis_state=8)
    heartbeat.heartbeat.received = heartbeat.heartbeat.valid = heartbeat.heartbeat.stamp_valid = True
    heartbeat.heartbeat.source_stamp = node.stamp(now)
    node.on_motor('left',heartbeat)
    result = node.message()
    assert result.left_motor.power.age_sec >= 1.
    assert result.left_motor.heartbeat.fresh and result.left_motor.error_code == 32
    node.on_motor('right',motor_sample('right',26.4,now))
    node.on_motor('left',motor_sample('left',float('nan'),now+.001))
    assert not node.message().left_motor.power.valid
    assert not node.message().battery_voltage_available


def test_motor_source_stamp_order_mapping_and_expiry(node):
    now = node.ros_now()
    node.on_motor('left',motor_sample('left',26.2,now))
    node.on_motor('left',motor_sample('left',25.,now-.1))
    assert node.message().left_motor.input_voltage_v == 26.2
    node.on_motor('right',motor_sample('left',26.4,now))
    assert not node.message().right_motor.power.valid
    node.on_motor('left',motor_sample('left',26.4,now-3.))
    assert not node.message().left_motor.power.valid


def test_can_to_vehicle_to_web_power_over_ros(node):
    import struct
    from types import SimpleNamespace
    from rclpy.executors import SingleThreadedExecutor
    from amr_can_bridge.can_telemetry import CanTelemetry
    from amr_web_ui.vehicle_status import display_snapshot
    class Bus:
        def __init__(self,**kwargs): pass
        def recv(self,timeout): return None
        def send(self,*args,**kwargs): pytest.fail('No CAN transmission permitted')
        def shutdown(self): pass
    can_node = CanTelemetry(bus_factory=Bus)
    executor = SingleThreadedExecutor()
    executor.add_node(node); executor.add_node(can_node)
    def spin(duration):
        end=time.monotonic()+duration
        while time.monotonic()<end: executor.spin_once(timeout_sec=.01)
    try:
        spin(.3)
        now=time.time()
        for cid,payload in ((0x17,struct.pack('<ff',26.4,.8)),(0x37,struct.pack('<ff',26.2,.9)),
                            (0x21,struct.pack('<IBBBB',16,8,0,0,0))):
            can_node.process_message(SimpleNamespace(arbitration_id=cid,data=payload,timestamp=now,
                is_extended_id=False,is_remote_frame=False,is_error_frame=False,is_fd=False),now)
        spin(.3)
        msg=node.message()
        assert msg.battery_voltage_v == pytest.approx(26.3)
        assert msg.left_motor.power.fresh and msg.left_motor.heartbeat.fresh
        web,_=display_snapshot(msg,0.)
        assert web['motor_left_power']['voltage'] == pytest.approx(26.2)
        assert web['motor_right_power']['current'] == pytest.approx(.8)
        assert web['battery']['voltage'] == pytest.approx(26.3)
        assert web['battery']['soc'] is None
        assert web['motor_left_heartbeat']['error_code'] == 16
    finally:
        executor.shutdown(); can_node.close(); can_node.destroy_node()
