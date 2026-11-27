"""racecar_neo_ros2_driver.qos, params, spin.run_node: the pieces every node shares."""

import pytest

rclpy = pytest.importorskip('rclpy')


def test_profiles_keep_their_settings():
    from racecar_neo_ros2_driver.qos import LATCHED, sensor, SENSOR_LATEST
    from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSReliabilityPolicy

    assert LATCHED.depth == 1
    assert LATCHED.reliability == QoSReliabilityPolicy.RELIABLE
    assert LATCHED.durability == QoSDurabilityPolicy.TRANSIENT_LOCAL
    for profile, depth in ((SENSOR_LATEST, 1), (sensor(10), 10)):
        assert profile.depth == depth
        assert profile.history == QoSHistoryPolicy.KEEP_LAST
        assert profile.reliability == QoSReliabilityPolicy.BEST_EFFORT
        assert profile.durability == QoSDurabilityPolicy.VOLATILE


def test_declare_params_returns_overrides_and_defaults():
    from racecar_neo_ros2_driver.params import declare_params
    from rclpy.parameter import Parameter

    rclpy.init()
    try:
        node = rclpy.create_node(
            'node_kit_test', parameter_overrides=[Parameter('rate', value=20.0)]
        )
        got = declare_params(node, {'rate': 50.0, 'topic': '/x', 'axes': [0, 1, 2]})
        assert got == {'rate': 20.0, 'topic': '/x', 'axes': [0, 1, 2]}
        assert node.get_parameter('topic').value == '/x'
        node.destroy_node()
    finally:
        rclpy.try_shutdown()


@pytest.mark.parametrize('stop', [KeyboardInterrupt, None])
def test_run_node_cleans_up(stop):
    from racecar_neo_ros2_driver.spin import run_node

    calls = []

    def factory():
        node = rclpy.create_node('node_kit_run')
        calls.append('built')
        return node

    def spinner(node):
        calls.append('spin')
        if stop:
            raise stop()

    run_node(factory, [], spinner=spinner, before_destroy=lambda node: calls.append('before'))
    assert calls == ['built', 'spin', 'before']
    assert not rclpy.ok()
