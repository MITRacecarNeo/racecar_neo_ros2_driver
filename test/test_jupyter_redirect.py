"""Tests for scripts/jupyter/racecar_jupyter_redirect.py and its unit wiring."""

import importlib.util
from pathlib import Path
import re

import pytest

ROOT = Path(__file__).resolve().parent.parent
MODULE = ROOT / 'scripts' / 'jupyter' / 'racecar_jupyter_redirect.py'
UNIT = ROOT / 'scripts' / 'racecar-jupyter.service'


@pytest.fixture(scope='module')
def ext():
    spec = importlib.util.spec_from_file_location('racecar_jupyter_redirect', MODULE)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


@pytest.mark.parametrize(
    'path, rest',
    [('/', None), ('/lab', 'lab'), ('/lab/tree/labs/a.ipynb', 'lab/tree/labs/a.ipynb'),
     ('/tree', 'tree'), ('/notebooks/a.ipynb', 'notebooks/a.ipynb')],
)  # fmt: skip
def test_old_addresses_redirect(ext, path, rest):
    m = re.fullmatch(ext.OLD_PATHS, path)
    assert m is not None and m.group(1) == rest


@pytest.mark.parametrize('path', ['/jupyter/lab', '/api/status', '/laboratory', '/static/x.js'])
def test_other_paths_are_left_alone(ext, path):
    assert re.fullmatch(ext.OLD_PATHS, path) is None


def test_only_under_the_jupyter_base(ext):
    class App:
        def __init__(self, base):
            self.base_url = base
            self.added = []
            self.web_app = self

        def add_handlers(self, host, handlers):
            self.added.append(handlers)

    on, off = App('/jupyter/'), App('/')
    ext._load_jupyter_server_extension(on)
    ext._load_jupyter_server_extension(off)
    assert len(on.added) == 1 and off.added == []


def test_unit_enables_it():
    unit = UNIT.read_text()
    assert '--ServerApp.base_url=/jupyter/' in unit
    assert '--ServerApp.jpserver_extensions=racecar_jupyter_redirect=True' in unit
    assert 'racecar_neo_ros2_driver/scripts/jupyter\n' in unit
