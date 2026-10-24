#!/bin/bash
# JupyterLab: install + workspace setup for racecar student notebooks.
# Idempotent: re-runs skip work already done.
set -eo pipefail

USER_NAME="${SUDO_USER:-$USER}"
USER_HOME="$(getent passwd "$USER_NAME" | cut -d: -f6)"
JUPYTER_WS="$USER_HOME/jupyter_ws"

# PEP 668 -> per-user install. jupyterlab pulls jupyter_server, ipykernel,
# tornado, et al; ~100 MB total on disk.
if ! command -v "$USER_HOME/.local/bin/jupyter" >/dev/null 2>&1; then
    pip3 install --user --break-system-packages jupyterlab==4.6.1
fi

# Student-library runtime deps for the v2 racecar-neo library / labs:
#   - ipywidgets: live FPS / joystick / detection widgets in the lab notebooks
#   - pandas: backs telemetry_real.visualize() reading the recorded CSV
#   - matplotlib-inline<0.2: 0.2.x breaks plt.subplots() on the apt matplotlib
#     these images ship
# nptyping is deliberately absent. Both pins and the omission are explained in
# docs/troubleshooting.md, "Jupyter dependency pins".
LIB_DEPS=(ipywidgets==8.1.8 pandas==3.0.3 'matplotlib-inline<0.2')
MISSING_DEPS=()
for dep in "${LIB_DEPS[@]}"; do
    # Strip any version specifier (e.g. 'matplotlib-inline<0.2' -> 'matplotlib-inline')
    # for the import probe; pip handles the constraint on install.
    mod="${dep%%[<>=!~ ]*}"
    if ! sudo -u "$USER_NAME" python3 -c "
import importlib, sys
try:
    m = importlib.import_module('${mod//-/_}')
except Exception:
    sys.exit(1)
# 0.2.x needs matplotlib 3.10 or newer; treat it as missing so pip installs <0.2.
if '${mod}' == 'matplotlib-inline':
    parts = m.__version__.split('.')
    major, minor = int(parts[0]), int(parts[1]) if len(parts) > 1 else 0
    sys.exit(0 if (major, minor) < (0, 2) else 1)
" >/dev/null 2>&1; then
        MISSING_DEPS+=("$dep")
    fi
done
if [ ${#MISSING_DEPS[@]} -gt 0 ]; then
    pip3 install --user --break-system-packages "${MISSING_DEPS[@]}"
fi

# Notebook root.
if [ ! -d "$JUPYTER_WS" ]; then
    mkdir -p "$JUPYTER_WS"
    cat > "$JUPYTER_WS/README.md" <<'EOF'
# RACECAR Neo Jupyter Workspace

JupyterLab serves this directory at http://<robot>:8888 when
racecar-jupyter.service is running.

Start a notebook and `import rclpy`; the systemd unit pre-sets
PYTHONPATH/AMENT_PREFIX_PATH/LD_LIBRARY_PATH so ROS2 messages and the
racecar driver are importable.
EOF
    echo "Created $JUPYTER_WS"
fi

# The data root (racecar_neo_ros2_driver/data_root.py) and its saved maps,
# linked into the JupyterLab file browser, which is rooted at ~/jupyter_ws.
# `racecar disk setup` points the links again when the root moves.
if [ -r /run/racecar/data_root ]; then
    DATA_ROOT="$(cat /run/racecar/data_root)"
else
    DATA_ROOT="$USER_HOME/data"
fi
MAP_ROOT="$DATA_ROOT/maps"
if [ ! -d "$MAP_ROOT" ]; then
    sudo -u "$USER_NAME" mkdir -p "$MAP_ROOT"
fi
for link in data maps; do
    target="$DATA_ROOT"
    [ "$link" = maps ] && target="$MAP_ROOT"
    if [ -L "$JUPYTER_WS/$link" ] || [ ! -e "$JUPYTER_WS/$link" ]; then
        sudo -u "$USER_NAME" ln -sfn "$target" "$JUPYTER_WS/$link"
        echo "Linked $JUPYTER_WS/$link -> $target"
    fi
done
