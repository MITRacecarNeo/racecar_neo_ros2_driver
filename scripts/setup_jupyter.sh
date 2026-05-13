#!/bin/bash
# JupyterLab — install + workspace setup for racecar student notebooks.
# Idempotent: re-runs skip work already done.
set -eo pipefail

USER_NAME="${SUDO_USER:-$USER}"
USER_HOME="$(getent passwd "$USER_NAME" | cut -d: -f6)"
JUPYTER_WS="$USER_HOME/jupyter_ws"

# PEP 668 → per-user install. jupyterlab pulls jupyter_server, ipykernel,
# tornado, et al; ~100 MB total on disk.
if ! command -v "$USER_HOME/.local/bin/jupyter" >/dev/null 2>&1; then
    pip3 install --user --break-system-packages jupyterlab
fi

# Student-library runtime deps. nptyping is imported by camera/lidar/physics
# for return-type annotations; pandas backs telemetry.visualize(); ipywidgets
# powers the live FPS / joystick / detection widgets in
# labs/tests/test_async_core_real.ipynb. JupyterLab 4.x renders ipywidgets >= 8
# natively (no labextension install needed).
#
# nptyping is pinned <2 because the v2 release ships an incompatible generic
# syntax (Shape["..."]) and the student library uses the v1 form
# NDArray[(480, 640, 3), np.uint8]; on Python 3.12 (Pi 5 default) pip resolves
# to 2.5.0 and every annotation in camera.py / lidar.py / physics.py raises
# InvalidArgumentsError at import. Pinning here keeps the v1 library readable
# until a future library bump rewrites the annotations.
LIB_DEPS=(ipywidgets pandas 'nptyping<2')
MISSING_DEPS=()
for dep in "${LIB_DEPS[@]}"; do
    # Strip any version specifier (e.g. 'nptyping<2' -> 'nptyping') for the
    # import probe; pip handles the version check on install.
    mod="${dep%%[<>=!~ ]*}"
    if ! sudo -u "$USER_NAME" python3 -c "
import importlib, sys
try:
    importlib.import_module('${mod//-/_}')
except Exception:
    sys.exit(1)
# nptyping 2.x installs cleanly but breaks the library's NDArray[(...)]
# syntax; treat it as 'missing' so pip downgrades to <2.
if '${mod}' == 'nptyping':
    import nptyping
    major = int(nptyping.__version__.split('.')[0])
    sys.exit(0 if major < 2 else 1)
" >/dev/null 2>&1; then
        MISSING_DEPS+=("$dep")
    fi
done
if [ ${#MISSING_DEPS[@]} -gt 0 ]; then
    pip3 install --user --break-system-packages "${MISSING_DEPS[@]}"
fi

# Notebook root. Empty unless we ship example notebooks later.
if [ ! -d "$JUPYTER_WS" ]; then
    mkdir -p "$JUPYTER_WS"
    cat > "$JUPYTER_WS/README.md" <<'EOF'
# RACECAR Neo Jupyter Workspace

JupyterLab serves this directory at http://<robot>:8888 when
racecar-jupyter.service is running.

Start a notebook and `import rclpy` — the systemd unit pre-sets
PYTHONPATH/AMENT_PREFIX_PATH/LD_LIBRARY_PATH so ROS2 messages and the
racecar driver are importable.
EOF
    echo "Created $JUPYTER_WS"
fi
