"""Unit tests for the frame tree: frames.yaml loading, xacro render, transforms."""

import importlib.util
import math
from pathlib import Path
import xml.etree.ElementTree as ET

from launch_ros.actions import Node
import numpy as np
import pytest

from racecar_neo_ros2_driver import frames
from racecar_neo_ros2_driver.frames import load_frames, xacro_mappings

xacro = pytest.importorskip('xacro')

ROOT = Path(__file__).resolve().parent.parent
CONFIG = ROOT / 'config'
URDF = ROOT / 'urdf' / 'racecar_neo.urdf.xacro'


def _load_launch(name: str):
    spec = importlib.util.spec_from_file_location(name, ROOT / 'launch' / f'{name}.launch.py')
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def _render(values: dict[str, float]) -> ET.Element:
    return ET.fromstring(xacro.process_file(str(URDF), mappings=xacro_mappings(values)).toxml())


def _rot(rpy: list[float]) -> np.ndarray:
    """URDF rpy: fixed-axis roll, then pitch, then yaw (R = Rz Ry Rx)."""
    r, p, y = rpy
    rx = np.array([[1, 0, 0], [0, math.cos(r), -math.sin(r)], [0, math.sin(r), math.cos(r)]])
    ry = np.array([[math.cos(p), 0, math.sin(p)], [0, 1, 0], [-math.sin(p), 0, math.cos(p)]])
    rz = np.array([[math.cos(y), -math.sin(y), 0], [math.sin(y), math.cos(y), 0], [0, 0, 1]])
    return rz @ ry @ rx


def _transforms(robot: ET.Element) -> dict[str, np.ndarray]:
    """Return each link's 4x4 pose in base_link, following the fixed joints."""
    joints = {}
    for j in robot.iter('joint'):
        o = j.find('origin')
        xyz = [float(v) for v in o.get('xyz', '0 0 0').split()]
        rpy = [float(v) for v in o.get('rpy', '0 0 0').split()]
        tf = np.eye(4)
        tf[:3, :3] = _rot(rpy)
        tf[:3, 3] = xyz
        joints[j.find('child').get('link')] = (j.find('parent').get('link'), tf)
    poses = {'base_link': np.eye(4)}

    def pose(link: str) -> np.ndarray:
        if link not in poses:
            parent, tf = joints[link]
            poses[link] = pose(parent) @ tf
        return poses[link]

    for link in joints:
        pose(link)
    return poses


@pytest.fixture(scope='module')
def values(tmp_path_factory) -> dict[str, float]:
    # The committed fleet values only: a car's gitignored frames.local.yaml
    # (a calibrated lidar yaw, say) must not change what the tests check.
    fleet = tmp_path_factory.mktemp('fleet')
    (fleet / 'frames.yaml').write_text((CONFIG / 'frames.yaml').read_text())
    return load_frames(fleet)


@pytest.fixture(scope='module')
def poses(values) -> dict[str, np.ndarray]:
    return _transforms(_render(values))


class TestLoadFrames:
    def test_repo_config_loads(self, values):
        assert values['laser_yaw_deg'] == 180.0

    def test_local_keys_win(self, tmp_path, car_profile_dir):
        (tmp_path / 'frames.yaml').write_text('laser_yaw_deg: 180.0\nlaser_x_mm: 1.0\n')
        (car_profile_dir / 'params').mkdir(parents=True)
        (car_profile_dir / 'params' / 'frames.yaml').write_text('laser_yaw_deg: 179.4\n')
        assert load_frames(tmp_path) == {'laser_yaw_deg': 179.4, 'laser_x_mm': 1.0}

    def test_driver_folder_override_is_ignored(self, tmp_path):
        (tmp_path / 'frames.yaml').write_text('laser_yaw_deg: 180.0\n')
        (tmp_path / 'frames.local.yaml').write_text('laser_yaw_deg: 179.4\n')
        assert load_frames(tmp_path) == {'laser_yaw_deg': 180.0}

    def test_unknown_local_key_is_an_error(self, tmp_path, car_profile_dir):
        (tmp_path / 'frames.yaml').write_text('laser_yaw_deg: 180.0\n')
        (car_profile_dir / 'params').mkdir(parents=True)
        (car_profile_dir / 'params' / 'frames.yaml').write_text('laser_yaw_dg: 179.4\n')
        with pytest.raises(ValueError, match='laser_yaw_dg'):
            load_frames(tmp_path)

    @pytest.mark.parametrize(
        'text', ['laser_x: 1.0\n', 'laser_x_mm: "near"\n', 'laser_x_mm: true\n']
    )
    def test_bad_entries_are_errors(self, tmp_path, text):
        (tmp_path / 'frames.yaml').write_text(text)
        with pytest.raises(ValueError):
            load_frames(tmp_path)

    def test_local_file_is_gitignored(self):
        lines = (ROOT / '.gitignore').read_text().splitlines()
        assert 'config/*.local.yaml' in lines


class TestXacroMappings:
    def test_units_converted_and_suffix_dropped(self):
        got = xacro_mappings({'laser_x_mm': 238.3, 'laser_yaw_deg': 180.0})
        assert float(got['laser_x']) == pytest.approx(0.2383)
        assert float(got['laser_yaw']) == pytest.approx(math.pi)

    def test_every_key_is_used_by_the_urdf(self, values):
        text = URDF.read_text()
        unused = [k for k in xacro_mappings(values) if f'$(arg {k})' not in text]
        assert unused == []

    def test_missing_key_fails_the_render(self, values):
        partial = dict(values)
        del partial['laser_x_mm']
        with pytest.raises(xacro.XacroException, match='laser_x'):
            _render(partial)


class TestFrameTree:
    def test_links(self, poses):
        # Nothing below camera_link: the RealSense driver publishes that subtree.
        assert set(poses) == {
            'base_link',
            'laser',
            'imu_link',
            'camera_bottom_screw_frame',
            'camera_link',
        }

    def test_laser_position(self, poses, values):
        mm = poses['laser'][:3, 3] * 1e3
        expected = [values['laser_x_mm'], values['laser_y_mm'], values['laser_z_mm']]
        assert mm == pytest.approx(expected)

    def test_laser_faces_aft(self, poses):
        # Scan zero (laser +x) points along base -x for the 180 deg mount.
        assert poses['laser'][:3, 0] == pytest.approx([-1.0, 0.0, 0.0], abs=1e-9)
        assert poses['laser'][:3, 2] == pytest.approx([0.0, 0.0, 1.0], abs=1e-9)

    def test_imu_link_has_base_link_axes(self, poses):
        # pit.yaml and imu_fusion.yaml remap both IMUs into REP-103 body axes.
        assert poses['imu_link'][:3, :3] == pytest.approx(np.eye(3), abs=1e-9)

    def test_camera_link_from_case_geometry(self, poses, values):
        # realsense2_description: screw 14.9 mm behind the front face; camera_link
        # 10.6 ahead of it, 17.5 left, 12.5 up.
        expected = [
            values['camera_front_x_mm'] - 14.9 + 10.6,
            values['camera_y_mm'] + 17.5,
            values['camera_bottom_z_mm'] + 12.5,
        ]
        assert poses['camera_link'][:3, 3] * 1e3 == pytest.approx(expected)

    def test_camera_pitch_turns_about_the_screw(self, values):
        tilted = dict(values, camera_pitch_deg=10.0)
        p = _transforms(_render(tilted))
        screw = p['camera_bottom_screw_frame'][:3, 3]
        assert screw * 1e3 == pytest.approx(
            [
                values['camera_front_x_mm'] - 14.9,
                values['camera_y_mm'],
                values['camera_bottom_z_mm'],
            ]
        )
        # Positive pitch tips the lens down.
        assert p['camera_link'][2, 0] == pytest.approx(-math.sin(math.radians(10.0)))


class TestLaunch:
    def test_robot_description_renders_from_package_layout(self):
        module = _load_launch('frames')
        description = module.robot_description(str(ROOT))
        assert '<robot name="racecar_neo"' in description

    def test_node_respawns_with_a_cap(self, monkeypatch):
        module = _load_launch('frames')
        monkeypatch.setattr(module, 'get_package_share_directory', lambda _: str(ROOT))
        (node,) = [e for e in module.generate_launch_description().entities if isinstance(e, Node)]
        assert node._ExecuteLocal__respawn is True
        assert node._ExecuteLocal__respawn_max_retries == module.RESPAWN_MAX_RETRIES > 0

    def test_teleop_includes_frames_behind_an_enable_arg(self):
        assert 'frames' in _load_launch('teleop')._SUBSYSTEMS

    def test_module_names_frames_yaml(self):
        assert frames.FRAMES_YAML == 'frames.yaml'
        assert (CONFIG / frames.FRAMES_YAML).is_file()
