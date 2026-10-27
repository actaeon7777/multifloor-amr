"""Annex C.1 keys and floor substitution, without ROS."""
from amr_elevator.announcements import ANNOUNCEMENTS


def test_complete_announcement_table():
    assert set(ANNOUNCEMENTS) == {
        'call', 'waiting', 'board_start', 'board_space', 'board_cancel', 'board_done',
        'alight_floor', 'arrival', 'alight_start', 'alight_cancel', 'alight_done',
        'elevator_error', 'robot_error', 'obstacle',
    }
    for text in ANNOUNCEMENTS.values():
        assert isinstance(text, str) and text.strip()
        assert '{' not in text.format(floor='3층')
    assert ANNOUNCEMENTS['alight_floor'].format(floor='3층') == '3층에서 하차하겠습니다'


def test_json_announcement_rate_limit_and_target_floor_without_ros_node():
    import json
    from types import SimpleNamespace
    from unittest.mock import Mock
    from amr_elevator.mission_node import FloorMissionManager

    clock = [10.]
    node = SimpleNamespace(now=lambda: clock[0], _announced={}, p={'announce_period': 5.},
                           current_floor=lambda: '4F', announcement_pub=Mock(),
                           mission=SimpleNamespace(phase='SELECT_FLOOR', floor='3F'))
    for key in ('board_done', 'alight_floor', 'alight_floor'):
        FloorMissionManager.announce(node, key)
    assert node.announcement_pub.publish.call_count == 2
    payload = json.loads(node.announcement_pub.publish.call_args.args[0].data)
    assert payload == {'key': 'alight_floor', 'text': '3F에서 하차하겠습니다',
                       'floor': '3F', 'phase': 'SELECT_FLOOR', 'stamp': 10.}
    clock[0] = 14.99
    FloorMissionManager.announce(node, 'alight_floor')
    assert node.announcement_pub.publish.call_count == 2
    clock[0] = 15.
    FloorMissionManager.announce(node, 'alight_floor')
    assert node.announcement_pub.publish.call_count == 3


def test_failed_announcement_cannot_prevent_stop_finish_or_hall_resume():
    from types import SimpleNamespace
    from unittest.mock import Mock
    from amr_elevator.mission_node import FloorMissionManager
    from amr_elevator_interfaces.msg import FloorState

    for inside in (False, True):
        events = []
        mission = SimpleNamespace(finished=False, counters={'_suspended': True})
        def finish(*args):
            mission.finished = True
            events.append('finish')
        def call(name, client, request):
            assert name == 'suspend' and request.data is False
            events.append('resume')
            return True
        def announce(key):
            assert key == 'robot_error' and mission.finished
            assert events[0] == 'stop'
            events.append('announce')
            raise RuntimeError('publisher unavailable')
        node = SimpleNamespace(mission=mission, seed_after_resume=False, suspend_client=object(),
                               floor_state=SimpleNamespace(zone=FloorState.ZONE_IN_ELEVATOR if inside
                                                           else FloorState.ZONE_OUTSIDE),
                               get_logger=lambda: Mock(), release_lease=lambda: events.append('stop'),
                               call=call, finish=finish, announce=announce)
        FloorMissionManager.fail(node, 'TEST_FAILURE')
        assert mission.finished and events[-2:] == ['finish', 'announce']
        assert node.seed_after_resume == (not inside)
        assert ('resume' in events) == (not inside)


def test_late_suspend_success_after_failure_requests_resume_again_without_ros_node():
    from types import MethodType, SimpleNamespace
    from unittest.mock import Mock
    from rclpy.task import Future
    from std_srvs.srv import SetBool
    from amr_elevator.mission_node import FloorMissionManager
    from amr_elevator_interfaces.msg import FloorState

    late_suspend, first_resume, final_resume = Future(), Future(), Future()
    client = Mock()
    client.service_is_ready.return_value = True
    client.call_async.side_effect = [late_suspend, first_resume, final_resume]
    mission = SimpleNamespace(counters={}, finished=False)
    def finish(*args):
        mission.finished = True
    node = SimpleNamespace(calls={}, mission=mission, suspend_client=client, seed_after_resume=False,
                           floor_state=SimpleNamespace(zone=FloorState.ZONE_OUTSIDE),
                           get_logger=lambda: Mock(), release_lease=Mock(), finish=finish, announce=Mock())
    node.call = MethodType(FloorMissionManager.call, node)
    assert node.call('suspend', client, SetBool.Request(data=True))
    assert not late_suspend.done()
    FloorMissionManager.fail(node, 'PREPARE_FAILED')
    assert mission.finished and node.seed_after_resume
    first_resume.set_result(SetBool.Response(success=True))
    late_suspend.set_result(SetBool.Response(success=True))
    assert [c.args[0].data for c in client.call_async.call_args_list] == [True, False, False]
    assert node.calls['suspend'] is final_resume


def test_obstacle_is_announced_only_after_the_hold_delay():
    from types import SimpleNamespace
    from unittest.mock import Mock
    from amr_elevator.mission_node import FloorMissionManager

    node = SimpleNamespace(p={'obstacle_announce_delay': 1.5}, announce=Mock())
    FloorMissionManager.announce_blocked(node, 1.49)
    node.announce.assert_not_called()
    FloorMissionManager.announce_blocked(node, 1.5)
    node.announce.assert_called_once_with('obstacle')
