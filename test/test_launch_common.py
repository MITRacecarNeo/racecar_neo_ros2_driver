"""Unit tests for launch_common: parameter-file order and per-car overrides."""

from pathlib import Path

from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
import pytest

from racecar_neo_ros2_driver import launch_common
from racecar_neo_ros2_driver.launch_common import local_overrides, single_node_launch


@pytest.fixture
def share(tmp_path: Path, monkeypatch: pytest.MonkeyPatch) -> Path:
    (tmp_path / 'config').mkdir()
    monkeypatch.setattr(launch_common, 'get_package_share_directory', lambda _: str(tmp_path))
    return tmp_path / 'config'


def _params(ld):
    node = next(e for e in ld.entities if isinstance(e, Node))
    return node._Node__parameters


def _args(ld):
    return [e.name for e in ld.entities if isinstance(e, DeclareLaunchArgument)]


class TestLocalOverrides:
    def test_only_existing_files_in_input_order(self, tmp_path):
        (tmp_path / 'b.local.yaml').write_text('')
        (tmp_path / 'a.local.yaml').write_text('')
        got = local_overrides(str(tmp_path), ['a.yaml', 'missing.yaml', 'b.yaml'])
        assert got == [str(tmp_path / 'a.local.yaml'), str(tmp_path / 'b.local.yaml')]

    def test_none_present(self, tmp_path):
        assert local_overrides(str(tmp_path), ['a.yaml']) == []


class TestSingleNodeLaunch:
    def test_declares_one_arg_per_yaml(self, share):
        ld = single_node_launch(
            'pit_config',
            'pit.yaml',
            'racecar_neo_ros2_driver',
            'pit_node',
            extra_yamls=(('cal_config', 'cal.yaml'),),
        )
        assert _args(ld) == ['pit_config', 'cal_config']

    def test_locals_load_after_all_shipped_files(self, share):
        (share / 'pit.local.yaml').write_text('')
        (share / 'cal.local.yaml').write_text('')
        ld = single_node_launch(
            'pit_config',
            'pit.yaml',
            'racecar_neo_ros2_driver',
            'pit_node',
            extra_yamls=(('cal_config', 'cal.yaml'),),
        )
        params = _params(ld)
        assert len(params) == 4
        locals_ = [p.param_file[0].text for p in params[2:]]
        assert locals_ == [str(share / 'pit.local.yaml'), str(share / 'cal.local.yaml')]

    def test_no_locals_means_shipped_files_only(self, share):
        ld = single_node_launch('mux_config', 'mux.yaml', 'racecar_neo_ros2_driver', 'mux_node')
        assert len(_params(ld)) == 1


class TestShippedLaunchFiles:
    """pit and imu_fusion expose their config args (teleop passes pit_config through)."""

    @staticmethod
    def _load(name):
        import importlib.util

        path = Path(__file__).parent.parent / 'launch' / f'{name}.launch.py'
        spec = importlib.util.spec_from_file_location(name, path)
        mod = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(mod)
        return mod.generate_launch_description()

    def test_pit_args(self, share):
        assert _args(self._load('pit')) == [
            'pit_config',
            'lsm9ds1_cal_config',
            'lsm9ds1_mag_cal_config',
        ]

    def test_imu_fusion_args(self, share):
        assert _args(self._load('imu_fusion')) == ['imu_fusion_config', 'realsense_cal_config']
