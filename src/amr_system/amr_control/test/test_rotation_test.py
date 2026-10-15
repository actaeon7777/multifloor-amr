from math import pi
import pytest
from amr_control.rotation_test import RotationTest


def call(p,now=2.,mode='AUTO',pose=(0.,0.,.1),v=0.,w=.15,gyro_z=.15,imu_angle=.1,fresh=True,clear=True,exclusive=True):
    return p.tick(now,mode,pose,v,w,gyro_z,imu_angle,fresh,clear,exclusive)


def armed():
    p=RotationTest()
    for t in (0.,1.1):assert call(p,t,'MANUAL',(0.,0.,0.),0.,0.,0.,0.)==0.
    assert call(p,1.2,'AUTO',(0.,0.,0.),0.,0.,0.,0.)==.15
    return p


def test_left_target_and_no_restart():
    p=armed()
    assert call(p,pose=(0.,0.,1.4),imu_angle=1.4)<.15
    assert call(p,pose=(0.,0.,pi/2),imu_angle=89.6*pi/180)==0.
    assert p.state=='COMPLETE'
    assert call(p,imu_angle=0.)==0.


@pytest.mark.parametrize('overrides,reason',[
    ({'mode':'MANUAL'},'operator_left_AUTO'),
    ({'fresh':False},'sensor_or_joy_timeout'),
    ({'clear':False},'obstacle_in_rotation_sweep'),
    ({'exclusive':False},'competing_command_publisher'),
    ({'pose':(.13,0.,.1)},'unexpected_translation'),
    ({'v':.11},'unexpected_translation'),
    ({'w':.41},'unexpected_rotation_speed'),
    ({'gyro_z':-.41},'unexpected_rotation_speed'),
    ({'imu_angle':-.10,'pose':(0.,0.,0.)},'wrong_rotation_direction'),
    ({'pose':(0.,0.,-.10),'imu_angle':0.},'wrong_rotation_direction'),
    ({'pose':(0.,0.,.6),'imu_angle':.1},'wheel_imu_disagreement'),
    ({'now':32.,'pose':(0.,0.,.1)},'30_second_timeout'),
    ({'now':6.,'pose':(0.,0.,0.),'imu_angle':0.},'no_rotation_progress'),
    ({'imu_angle':float('nan')},'sensor_or_joy_timeout'),
    ({'pose':(0.,0.,1.9),'imu_angle':1.9},'angle_limit_exceeded'),
])
def test_failures_latch_stop(overrides,reason):
    p=armed();assert call(p,**overrides)==0.
    assert p.state=='ABORTED' and p.reason==reason
    assert call(p)==0.


def test_requires_real_manual_preparation_and_handles_yaw_wrap():
    p=RotationTest();assert call(p)==0. and p.state=='WAITING'
    for t in (0.,1.1):assert call(p,t,'MANUAL',(0.,0.,3.),0.,0.,0.,0.)==0.
    assert call(p,1.2,'AUTO',(0.,0.,3.),0.,0.,0.,0.)==.15
    assert call(p,pose=(0.,0.,-3.),imu_angle=2*pi-6.)==.15
