"""racecar_neo_ros2_driver.ros_cli: one-job nodes and service calls with timeouts."""

import pytest

rclpy = pytest.importorskip('rclpy')


def test_node_starts_and_stops_rclpy():
    from racecar_neo_ros2_driver.ros_cli import ros_node

    with ros_node('ros_cli_test') as node:
        assert rclpy.ok() and node.get_name() == 'ros_cli_test'
    assert not rclpy.ok()


def test_missing_service_is_unavailable():
    from racecar_neo_ros2_driver.ros_cli import call_service, ros_node
    from std_srvs.srv import Trigger

    with ros_node('ros_cli_test') as node:
        status, result = call_service(
            node, Trigger, '/ros_cli_test/nobody', Trigger.Request(), wait_sec=0.2, timeout_sec=0.2
        )
    assert (status, result) == ('unavailable', None)


def test_answered_call_is_ok_and_slow_one_times_out():
    from racecar_neo_ros2_driver.ros_cli import call_service, ros_node
    from std_srvs.srv import Trigger

    def answer(_req, resp):
        resp.success, resp.message = True, 'saved'
        return resp

    with ros_node('ros_cli_test') as node:
        node.create_service(Trigger, '/ros_cli_test/save', answer)
        status, result = call_service(
            node, Trigger, '/ros_cli_test/save', Trigger.Request(), wait_sec=2.0, timeout_sec=2.0
        )
        assert status == 'ok' and result.message == 'saved'
        status, _ = call_service(
            node, Trigger, '/ros_cli_test/save', Trigger.Request(), wait_sec=2.0, timeout_sec=0.0
        )
        assert status == 'timeout'
