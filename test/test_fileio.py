"""Tests for racecar_neo_ros2_driver/fileio.py: atomic writes, KEY=VALUE files, the log folder."""

from pathlib import Path

from racecar_neo_ros2_driver import fileio


class TestWriteAtomic:
    def test_creates_the_folder_and_leaves_no_temporary(self, tmp_path):
        path = tmp_path / 'a' / 'b' / 'file.conf'
        assert fileio.write_atomic(path, 'X=1\n') == path
        assert path.read_text() == 'X=1\n'
        assert [p.name for p in path.parent.iterdir()] == ['file.conf']

    def test_replaces_the_old_file(self, tmp_path):
        path = tmp_path / 'file.conf'
        path.write_text('old')
        fileio.write_atomic(path, 'new')
        assert path.read_text() == 'new'


class TestReadKeyvals:
    def test_reads_lines_and_skips_the_rest(self, tmp_path):
        path = tmp_path / 'f.conf'
        path.write_text('# comment\n\nA=1\n  B = two  \nno equals sign\nC=x=y\n')
        assert fileio.read_keyvals(path) == {'A': '1', 'B': 'two', 'C': 'x=y'}

    def test_removes_one_pair_of_matching_quotes(self, tmp_path):
        path = tmp_path / 'networking.conf'
        path.write_text('A="static"\nB=\'dhcp\'\nC="unclosed\nD=""\n')
        assert fileio.read_keyvals(path) == {'A': 'static', 'B': 'dhcp', 'C': '"unclosed', 'D': ''}

    def test_missing_file_is_empty(self, tmp_path):
        assert fileio.read_keyvals(tmp_path / 'absent') == {}


def test_latest_log_dir():
    assert fileio.latest_log_dir() == Path.home() / 'logs' / 'latest'
