"""Unit tests for scripts/dmatrix_patterns.py (pattern-generation helpers)."""

import importlib.util
from pathlib import Path

import pytest

SCRIPT = Path(__file__).parent.parent / 'scripts' / 'dmatrix_patterns.py'


def _load_patterns_module():
    # The script is at scripts/dmatrix_patterns.py (not on the import path).
    # Load it by file path so we can hit its pure helpers without rclpy.init().
    spec = importlib.util.spec_from_file_location('dmatrix_patterns', SCRIPT)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


@pytest.fixture(scope='module')
def patterns():
    return _load_patterns_module()


class TestCheckerboard:
    def test_alternates_per_pixel(self, patterns):
        data = patterns._checkerboard(24)
        assert len(data) == 8 * 24
        # row 0 starts at 0, row 1 starts at 1 (because (r+c)&1)
        assert data[0] == 0
        assert data[1] == 1
        assert data[24] == 1  # row 1, col 0
        assert data[25] == 0

    def test_works_for_arbitrary_width(self, patterns):
        for w in (8, 16, 24, 32, 40):
            data = patterns._checkerboard(w)
            assert len(data) == 8 * w


class TestAllOn:
    def test_every_pixel_is_one(self, patterns):
        data = patterns._all_on(24)
        assert data == [1] * (8 * 24)


class TestSweepFrame:
    def test_only_one_column_lit_per_frame(self, patterns):
        for col in (0, 5, 23):
            frame = patterns._sweep_frame(24, col)
            assert sum(frame) == 8  # 8 rows × 1 lit col
            # That lit column is `col` in every row
            for r in range(8):
                assert frame[r * 24 + col] == 1

    def test_out_of_range_column_blanks_frame(self, patterns):
        frame = patterns._sweep_frame(24, 99)
        assert sum(frame) == 0


class TestModuleId:
    def test_each_module_lights_a_unique_row(self, patterns):
        # 3-module display, 24 px wide. Module 0 → row 0 lit on cols 0..7;
        # module 1 → row 1 lit on cols 8..15; module 2 → row 2 lit on cols 16..23.
        data = patterns._module_id(24, 3)
        assert data[0 * 24 + 0] == 1  # module 0 row 0 col 0
        assert data[0 * 24 + 7] == 1  # module 0 row 0 col 7
        assert data[0 * 24 + 8] == 0  # module 1 columns row 0 are off
        assert data[1 * 24 + 8] == 1  # module 1 row 1 col 8
        assert data[1 * 24 + 15] == 1
        assert data[2 * 24 + 16] == 1  # module 2 row 2 col 16
        assert data[2 * 24 + 23] == 1

    def test_more_modules_than_rows_clamps_to_last_row(self, patterns):
        # 9 modules but only 8 rows → modules 8+ pile onto row 7.
        data = patterns._module_id(72, 9)
        # module 8 spans cols 64..71; rows 7 cols 64..71 lit.
        for c in range(64, 72):
            assert data[7 * 72 + c] == 1


class TestPatternRegistry:
    def test_registry_lists_expected_patterns(self, patterns):
        assert set(patterns.PATTERNS) == {
            'checkerboard', 'all-on', 'sweep', 'module-id', 'font'
        }
