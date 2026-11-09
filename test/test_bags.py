"""racecar_neo_ros2_driver.bags: readers for mcap and sqlite3 bags alike."""

import pytest

rosbag2_py = pytest.importorskip('rosbag2_py')


def write_bag(path, storage):
    from rclpy.serialization import serialize_message
    from std_msgs.msg import String

    writer = rosbag2_py.SequentialWriter()
    writer.open(
        rosbag2_py.StorageOptions(uri=str(path), storage_id=storage),
        rosbag2_py.ConverterOptions('cdr', 'cdr'),
    )
    for topic in ('/a', '/b'):
        writer.create_topic(
            rosbag2_py.TopicMetadata(
                id=0, name=topic, type='std_msgs/msg/String', serialization_format='cdr'
            )
        )
    for i, topic in enumerate(('/a', '/b', '/a')):
        writer.write(topic, serialize_message(String(data=str(i))), i)
    del writer


@pytest.mark.parametrize('storage', ['mcap', 'sqlite3'])
def test_reads_either_storage_and_filters(tmp_path, storage):
    from racecar_neo_ros2_driver.bags import open_reader

    bag = tmp_path / storage
    try:
        write_bag(bag, storage)
    except RuntimeError as exc:
        pytest.skip(f'{storage} storage plugin not installed: {exc}')
    reader, types = open_reader(bag, ['/a', '/missing'])
    assert types == {'/a': 'std_msgs/msg/String', '/b': 'std_msgs/msg/String'}
    topics = []
    while reader.has_next():
        topics.append(reader.read_next()[0])
    assert topics == ['/a', '/a']
