"""Shared test helpers: load scripts/*.py by path (scripts/ is not a package)."""

import importlib.util
from pathlib import Path
import sys
from types import ModuleType

SCRIPTS = Path(__file__).resolve().parent.parent / 'scripts'


def load_script(name: str) -> ModuleType:
    """
    Import scripts/<name>.py and register it in sys.modules under `name`.

    Registration is required: dataclasses resolve annotations through
    sys.modules, and dashboard.py and watchdog.py import sysinfo by that name.
    A module already loaded from the same file is returned as is, so every
    test file sees one module object to monkeypatch.
    """
    path = SCRIPTS / f'{name}.py'
    existing = sys.modules.get(name)
    if existing is not None and Path(getattr(existing, '__file__', '') or '').resolve() == path:
        return existing
    spec = importlib.util.spec_from_file_location(name, path)
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    sys.modules[name] = module
    spec.loader.exec_module(module)
    return module
