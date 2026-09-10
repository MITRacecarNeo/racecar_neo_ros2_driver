#!/bin/bash
# setup_coral.sh - Install Coral EdgeTPU dependencies (M.2/PCIe and USB).
#
# Common to both form factors:
#   - libedgetpu runtime (.deb from depend/)
#   - tflite_runtime + pycoral Python wheels (from depend/)
#
# M.2 / PCIe (Coral Apex, PCI 1ac1:089a), auto-selected when the card is on the
# bus:
#   - gasket/apex kernel driver via DKMS (depend/gasket-dkms_*.deb), patched by
#     scripts/gasket-msi-fallback.patch
#   - coral-msi device-tree overlay (scripts/coral-msi.dts)
#   - apex access group for non-root /dev/apex_0
#   Requires a reboot (overlay + auto-load take effect at boot).
#   What the patch and overlay are for: docs/troubleshooting.md,
#   "Coral M.2 interrupts".
#
# USB accelerator (1a6e:089a / 18d1:9302):
#   - non-root access via the racecar udev rules (scripts/udev/99-racecar.rules,
#     installed by setup_udev.sh). No reboot.
#
# Idempotent: re-runs skip completed work.

set -eo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
DEPEND_DIR="$SCRIPT_DIR/../depend"

echo "=== Coral EdgeTPU Setup ==="

# --- 1. libedgetpu runtime ---
echo "[1/5] Installing libedgetpu runtime..."
if dpkg -l libedgetpu1-std 2>/dev/null | grep -q "^ii"; then
    echo "  libedgetpu1-std already installed. Skipping."
else
    sudo dpkg -i "$DEPEND_DIR"/libedgetpu1-std_*.deb
fi

# --- 2. tflite_runtime ---
echo "[2/5] Installing tflite_runtime..."
if python3 -c "import tflite_runtime" 2>/dev/null; then
    echo "  tflite_runtime already installed. Skipping."
else
    pip3 install --user --break-system-packages "$DEPEND_DIR"/tflite_runtime-*.whl
fi

# --- 3. pycoral ---
echo "[3/5] Installing pycoral..."
if python3 -c "import pycoral" 2>/dev/null; then
    echo "  pycoral already installed. Skipping."
else
    pip3 install --user --break-system-packages "$DEPEND_DIR"/pycoral-*.whl
fi

# --- 4. Driver + access rule (form-factor dependent) ---
NEEDS_REBOOT=0
if lspci -nn 2>/dev/null | grep -qi '1ac1:089a'; then
    echo "[4/5] M.2/PCIe Coral (Apex) detected. Installing gasket driver + overlay..."

    # Build prerequisites for the DKMS module. linux-headers-raspi (meta) keeps
    # headers tracking the kernel so DKMS auto-rebuilds apex after a kernel
    # update; the explicit headers cover the current kernel immediately (needed
    # when running from a cloned image whose kernel is already installed).
    KREL="$(uname -r)"
    sudo apt-get install -y dkms linux-headers-raspi "linux-headers-$KREL" \
        device-tree-compiler

    # gasket/apex DKMS driver. DKMS builds it for the running kernel and
    # rebuilds on kernel updates; ships its own apex udev rule (GROUP=apex).
    if dpkg -l gasket-dkms 2>/dev/null | grep -q "^ii"; then
        echo "  gasket-dkms already installed. Skipping."
    else
        sudo dpkg -i "$DEPEND_DIR"/gasket-dkms_*.deb || sudo apt-get -f install -y
    fi

    # Non-root /dev/apex_0 access.
    sudo groupadd -f apex
    sudo usermod -aG apex "$USER"

    # coral-msi device-tree overlay -> /boot/firmware/overlays + config.txt.
    dtc -@ -I dts -O dtb -o /tmp/coral-msi.dtbo "$SCRIPT_DIR/coral-msi.dts"
    sudo install -m 0644 /tmp/coral-msi.dtbo /boot/firmware/overlays/coral-msi.dtbo
    if grep -q "^dtoverlay=coral-msi" /boot/firmware/config.txt; then
        echo "  dtoverlay=coral-msi already in config.txt."
    else
        sudo tee -a /boot/firmware/config.txt >/dev/null <<'__CFG__'

# Coral M.2 (Apex) MSI routing: send external PCIe MSIs through pcie1's own
# controller instead of the small mip1 peripheral, so apex gets its 13 vectors.
dtoverlay=coral-msi
__CFG__
        echo "  added dtoverlay=coral-msi to config.txt."
    fi
    NEEDS_REBOOT=1
else
    echo "[4/5] No M.2 Apex on the bus. Coral USB access comes from the racecar"
    echo "      udev rules (scripts/udev/99-racecar.rules); run 'racecar setup udev'"
    echo "      or scripts/setup_udev.sh to install them. No reboot for USB."
fi

# --- 5. Verify ---
echo "[5/5] Verifying Coral EdgeTPU..."
if [ "$NEEDS_REBOOT" = "1" ]; then
    if dkms status 2>/dev/null | grep -q "gasket.*installed"; then
        echo "  gasket DKMS module installed."
    else
        echo "  WARNING: gasket DKMS module not reported installed. Check the build log."
    fi
    if [ -e /dev/apex_0 ]; then
        echo "  /dev/apex_0 present."
    else
        echo "  /dev/apex_0 not present yet (expected before reboot)."
    fi
else
    if lsusb | grep -qi "google\|1a6e:089a\|18d1:9302"; then
        echo "  Coral USB device detected on bus."
    else
        echo "  WARNING: Coral USB device not found. Is it plugged in?"
    fi
    if python3 -c "from pycoral.utils.edgetpu import list_edge_tpus; assert list_edge_tpus()" 2>/dev/null; then
        echo "  pycoral can see the EdgeTPU. OK."
    else
        echo "  WARNING: pycoral cannot detect the EdgeTPU. Check the connection and libedgetpu install."
    fi
fi

echo ""
echo "=== Coral EdgeTPU setup complete! ==="
if [ "$NEEDS_REBOOT" = "1" ]; then
    echo "REBOOT required: the overlay and apex auto-load take effect at boot."
    echo "After reboot, verify: python3 -c 'from pycoral.utils.edgetpu import list_edge_tpus; print(list_edge_tpus())'"
else
    echo "No reboot required."
fi
