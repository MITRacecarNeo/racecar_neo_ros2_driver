"""Tests for scripts/features.py: per-boot feature choice and teleop launch arguments."""

from conftest import load_script
import pytest


@pytest.fixture(scope='module')
def features():
    return load_script('features')


@pytest.fixture
def conf(tmp_path):
    return tmp_path / 'features.conf'


class TestReadWrite:
    def test_defaults_without_a_file(self, features, conf):
        assert features.read(conf, 'boot1') == {'camera': True, 'detections': False}

    def test_round_trip_in_the_same_boot(self, features, conf):
        features.write({'camera': True, 'detections': True}, conf, 'boot1')
        assert features.read(conf, 'boot1') == {'camera': True, 'detections': True}

    def test_a_reboot_returns_to_the_defaults(self, features, conf):
        features.write({'camera': False, 'detections': False}, conf, 'boot1')
        assert features.read(conf, 'boot2') == features.DEFAULTS

    def test_unknown_boot_reads_defaults(self, features, conf):
        features.write({'camera': False, 'detections': False}, conf, '')
        assert features.read(conf, '') == features.DEFAULTS

    def test_detections_without_camera_reads_off(self, features, conf):
        conf.write_text('BOOT_ID=b\nCAMERA=0\nDETECTIONS=1\n')
        assert features.read(conf, 'b') == {'camera': False, 'detections': False}


class TestChange:
    def test_detections_on_turns_the_camera_on(self, features):
        out = features.change({'camera': False, 'detections': False}, 'detections', True)
        assert out == {'camera': True, 'detections': True}

    def test_camera_off_turns_detections_off(self, features):
        out = features.change({'camera': True, 'detections': True}, 'camera', False)
        assert out == {'camera': False, 'detections': False}

    def test_unknown_feature(self, features):
        with pytest.raises(ValueError):
            features.change(features.DEFAULTS, 'lidar', False)


class TestLaunchArgs:
    def test_one_argument_per_subsystem(self, features):
        assert features.launch_args({'camera': True, 'detections': False}) == [
            'realsense_enable:=true',
            'edgetpu_enable:=false',
        ]

    def test_given_arguments_win(self, features):
        out = features.launch_args(features.DEFAULTS, ['edgetpu_enable:=true'])
        assert out == ['realsense_enable:=true']

    def test_running_reads_the_launch_args_file(self, features, tmp_path):
        (tmp_path / 'launch_args').write_text('realsense_enable:=false\nedgetpu_enable:=true\n')
        assert features.running(tmp_path) == {'camera': False, 'detections': True}

    def test_running_without_arguments_means_on(self, features, tmp_path):
        assert features.running(tmp_path) == {'camera': True, 'detections': True}

    def test_launch_script_passes_the_features(self):
        from pathlib import Path

        script = Path(__file__).resolve().parent.parent / 'scripts' / 'launch_teleop.sh'
        text = script.read_text()
        assert 'features.py" args "$@"' in text
        assert text.index('features.py') < text.index('launch_args"')
