from math import pi
import pytest
from amr_control.straight_test import StraightTest


def ready(p):
    assert p.tick(0.,'MANUAL',(0.,0.,0.),0.,True,True)==0.
    assert p.tick(1.1,'MANUAL',(0.,0.,0.),0.,True,True)==0.
    assert p.manual_seen


def test_one_run_ramps_down_and_never_restarts():
    p=StraightTest()
    assert p.tick(0.,'AUTO',(0.,0.,0.),0.,True,True)==0.
    ready(p)
    assert p.tick(1.2,'AUTO',(2.,3.,pi/2),0.,True,True)==.1
    assert p.tick(10.,'AUTO',(2.,3.9,pi/2),.1,True,True)==pytest.approx(.05)
    assert p.tick(12.,'AUTO',(2.,3.996,pi/2),.025,True,True)==0.
    assert p.state=='COMPLETE'
    assert p.tick(13.,'MANUAL',(0.,0.,0.),0.,True,True)==0.
    assert p.tick(14.,'AUTO',(0.,0.,0.),0.,True,True)==0.


@pytest.mark.parametrize('changes,reason',[
    ({'mode':'MANUAL'},'operator_left_AUTO'),
    ({'fresh':False},'sensor_or_joy_timeout'),
    ({'clear':False},'obstacle_in_remaining_path'),
    ({'imu_heading':-.09},'imu_heading_exceeded'),
    ({'imu_heading':.09},'imu_heading_exceeded'),
    ({'imu_heading':float('nan')},'sensor_or_joy_timeout'),
    ({'exclusive':False},'competing_command_publisher'),
    ({'pose':(-.03,0.,0.)},'reverse_odometry'),
    ({'pose':(.1,.07,0.)},'deviation_exceeded'),
    ({'pose':(.1,0.,.09)},'deviation_exceeded'),
    ({'measured_v':.21},'unexpected_measured_speed'),
    ({'now':22.,'pose':(.9,0.,0.)},'20_second_timeout'),
    ({'now':6.},'no_forward_progress'),
    ({'pose':(float('nan'),0.,0.)},'sensor_or_joy_timeout'),
])
def test_abort_is_latched(changes,reason):
    p=StraightTest();ready(p);p.tick(1.2,'AUTO',(0.,0.,0.),0.,True,True)
    args=dict(now=2.,mode='AUTO',pose=(0.,0.,0.),measured_v=.1,fresh=True,clear=True,exclusive=True)
    args.update(changes)
    assert p.tick(**args)==0.
    assert p.state=='ABORTED' and p.reason==reason
    assert p.tick(2.1,'AUTO',(.1,0.,0.),.1,True,True)==0.


def test_no_start_without_fresh_clear_stationary_manual_window():
    p=StraightTest()
    for t in (0.,1.,2.):
        assert p.tick(t,'MANUAL',(0.,0.,0.),0.,False,True)==0.
    assert p.tick(3.,'AUTO',(0.,0.,0.),0.,True,True)==0.
    assert p.state=='WAITING'
