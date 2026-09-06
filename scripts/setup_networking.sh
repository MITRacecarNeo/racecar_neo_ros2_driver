#!/bin/bash
# setup_networking.sh — configure eth0 addressing and an isolated AP on the ALFA
# dongle for racecar.
#
# This script:
#   1. Installs a NetworkManager dispatcher that blocks FORWARD on the AP
#      interface so AP clients can reach the Pi's services (dashboard, jupyter,
#      SSH) but cannot use the Pi as an internet gateway.
#   2. Creates the racecar AP NetworkManager connection on the AP interface
#      (WPA2 / 2.4 GHz / channel 6 / 10.42.0.1/24). The AP interface is the ALFA
#      MT7612U dongle, pinned to wlan1 by the 99-racecar.rules udev rule.
#   3. Puts eth0 in a single IPv4 addressing mode by calling setup_eth.sh
#      (static by default, at 192.168.52.200/24). Earlier versions rendered a
#      dual-IP block here, carrying a static address and a DHCP lease at once,
#      which is what made the static drop periodically.
#   4. Resets the Pi's built-in wlan0 to default (client) mode, removing any AP
#      connection a pre-v0.7.0 setup left bound to it.
#
# WARNING: this script reconfigures the AP interface. If you're SSH'd in over
# the AP, the connection will drop when the AP cycles. Run from a wired (eth0)
# session or directly on the console.
#
# Reset mode: RACECAR_AP_RESET=1 disables the AP only (downs and deletes the
# AP connection on the AP interface), leaves eth0 untouched, and exits. Each car
# needs a distinct SSID, so the golden image ships with no active AP; `racecar
# setup networking --reset` uses this before imaging.
#
# Parameters (override via environment variables before running):
#   RACECAR_AP_IFACE      (default: wlan1 — the ALFA dongle)
#   RACECAR_AP_ID         (default: 1 — the number appended to the SSID base)
#   RACECAR_AP_SSID       (default: racecar-neo-<id> — full SSID override)
#   RACECAR_AP_PSK        (default: racecar@mit)
#   RACECAR_AP_CHANNEL    (default: 6)
#   RACECAR_AP_ADDR       (default: 10.42.0.1/24)
#   RACECAR_ETH_STATIC    (default: 192.168.52.200/24)
#   RACECAR_ETH_MODE      (default: static; or dynamic)
#
# All steps are idempotent — re-running is safe.

set -eo pipefail

# Persisted overrides — `racecar setup networking --ssid=...` writes here.
# Precedence: env vars in the current shell > persisted file > defaults.
USER_HOME="$(getent passwd "${SUDO_USER:-$USER}" | cut -d: -f6)"
PERSIST_FILE="${USER_HOME}/.config/racecar/networking.env"
if [ -f "$PERSIST_FILE" ]; then
    # Only load keys not already set in the environment so the caller can
    # always override with `KEY=value racecar setup networking`.
    while IFS='=' read -r key val; do
        [ -z "$key" ] && continue
        [[ "$key" =~ ^[A-Z_][A-Z0-9_]*$ ]] || continue
        if [ -z "${!key:-}" ]; then
            # Strip leading/trailing quotes from val (sh writes "value").
            val="${val%\"}"
            val="${val#\"}"
            export "$key=$val"
        fi
    done < "$PERSIST_FILE"
fi

echo "=== RACECAR Neo Networking Setup ==="

AP_IFACE="${RACECAR_AP_IFACE:-wlan1}"
# SSID is a fixed base plus a per-car ID so multiple cars don't clash. A full
# RACECAR_AP_SSID still overrides the composed value.
AP_SSID_BASE="${RACECAR_AP_SSID_BASE:-racecar-neo}"
AP_SSID="${RACECAR_AP_SSID:-${AP_SSID_BASE}-${RACECAR_AP_ID:-1}}"
AP_PSK="${RACECAR_AP_PSK:-racecar@mit}"
AP_CON_NAME="racecar-neo-ap"
AP_BAND="bg"
AP_CHANNEL="${RACECAR_AP_CHANNEL:-6}"
AP_ADDR="${RACECAR_AP_ADDR:-10.42.0.1/24}"

ETH_STATIC_ADDR="${RACECAR_ETH_STATIC:-192.168.52.200/24}"

DISPATCHER_PATH="/etc/NetworkManager/dispatcher.d/99-racecar-ap-isolate"

CHANGES_MADE=false

# --- Reset mode: disable the AP only, leave eth0 alone -----------------------
# Deletes the AP connection so a cloned/golden image ships with no active AP and
# no baked-in SSID; each car sets its own ID on the next `racecar setup
# networking`. Connection delete is by name, so it works even if the dongle is
# absent; bringing the interface down is best-effort.
if [ "${RACECAR_AP_RESET:-0}" = "1" ]; then
    echo "Disabling the WiFi AP on $AP_IFACE (reset)..."
    sudo nmcli connection down "$AP_CON_NAME" >/dev/null 2>&1 || true
    if nmcli -t -f NAME con show 2>/dev/null | grep -qx "$AP_CON_NAME"; then
        sudo nmcli connection delete "$AP_CON_NAME" || true
        echo "  Deleted AP connection '$AP_CON_NAME'."
    else
        echo "  No AP connection '$AP_CON_NAME' present."
    fi
    sudo ip link set "$AP_IFACE" down 2>/dev/null || true
    echo "  eth0 configuration left unchanged."
    echo "=== AP disabled ==="
    exit 0
fi

# --- Resolve the AP interface ------------------------------------------------
# Default wlan1 is the udev-assigned stable name for the ALFA MT7612U
# (0e8d:7612, driver mt76x2u). If the rename hasn't taken effect yet (e.g. no
# reboot since setup_udev.sh), fall back to whatever wireless netdev the
# mt76x2u driver bound.
if [ ! -d "/sys/class/net/$AP_IFACE/wireless" ]; then
    for cand in /sys/class/net/*/wireless; do
        [ -e "$cand" ] || continue
        i="$(basename "$(dirname "$cand")")"
        drv="$(basename "$(readlink -f "/sys/class/net/$i/device/driver" 2>/dev/null)" 2>/dev/null)"
        if [ "$drv" = "mt76x2u" ]; then
            echo "  AP interface '$AP_IFACE' not present; using detected ALFA interface '$i'."
            echo "  (Reboot after setup_udev.sh so the udev rule renames it to wlan1.)"
            AP_IFACE="$i"
            break
        fi
    done
fi
if [ ! -d "/sys/class/net/$AP_IFACE/wireless" ]; then
    echo "ERROR: no AP wireless interface found (looked for '$AP_IFACE' and an mt76x2u adapter)." >&2
    echo "       Plug in the ALFA dongle; run scripts/setup_udev.sh and reboot so it becomes wlan1." >&2
    exit 1
fi
echo "AP interface: $AP_IFACE"

# --- 1. AP-isolation dispatcher ----------------------------------------------
echo "[1/4] Installing AP isolation dispatcher at $DISPATCHER_PATH..."
TMP_DISPATCHER=$(mktemp)
cat >"$TMP_DISPATCHER" <<SCRIPT
#!/bin/sh
# RACECAR Neo hotspot isolation — NM's ipv4.method=shared enables IP forwarding
# and sets up NAT, which would let AP clients route out through eth0. Block
# FORWARD in/out of the AP interface so clients can reach the Pi's own services
# (dashboard, jupyter, SSH) but cannot use the Pi as an internet gateway.

iface="\$1"
action="\$2"

[ "\$iface" = "$AP_IFACE" ] || exit 0
[ "\$CONNECTION_ID" = "$AP_CON_NAME" ] || exit 0

case "\$action" in
    up)
        iptables -D FORWARD -i $AP_IFACE -j REJECT 2>/dev/null
        iptables -D FORWARD -o $AP_IFACE -j REJECT 2>/dev/null
        iptables -I FORWARD -i $AP_IFACE -j REJECT
        iptables -I FORWARD -o $AP_IFACE -j REJECT
        ;;
    down|pre-down)
        iptables -D FORWARD -i $AP_IFACE -j REJECT 2>/dev/null
        iptables -D FORWARD -o $AP_IFACE -j REJECT 2>/dev/null
        ;;
esac
exit 0
SCRIPT
if sudo cmp -s "$TMP_DISPATCHER" "$DISPATCHER_PATH" 2>/dev/null; then
    echo "  $DISPATCHER_PATH already up to date."
else
    sudo install -m 755 -o root -g root "$TMP_DISPATCHER" "$DISPATCHER_PATH"
    echo "  Dispatcher installed."
    CHANGES_MADE=true
fi
rm -f "$TMP_DISPATCHER"

# The dispatcher is socket-activated by NetworkManager-dispatcher.service.
# That service is shipped enabled by default on Ubuntu Server but is often
# disabled on Desktop / Raspberry Pi OS images. Without it the dispatcher
# script is never invoked and the iptables isolation rules silently never
# apply (exactly the failure mode v0.0.6 hit on first install).
#
# The service is Type=simple with no RemainAfterExit, so it shows "inactive"
# whenever no script is currently running — `is-active` is the wrong probe.
# `is-enabled` is the property we actually care about: will systemd start it
# the next time NM emits a connection event?
if ! systemctl is-enabled --quiet NetworkManager-dispatcher.service; then
    echo "  Enabling NetworkManager-dispatcher.service..."
    sudo systemctl enable --now NetworkManager-dispatcher.service
    CHANGES_MADE=true
fi

# --- 2. Create or update the AP connection -----------------------------------
# Order matters: the AP must be up BEFORE we reset wlan0 below. If any step from
# here through netplan-apply fails under `set -e`, the prior config is still
# intact for recovery.
echo "[2/4] Configuring AP connection '$AP_CON_NAME' on $AP_IFACE (SSID: $AP_SSID)..."
if nmcli -t -f NAME con show | grep -qx "$AP_CON_NAME"; then
    # Diff each user-tunable setting against what nmcli reports; only call
    # `nmcli connection modify` when at least one field differs. (modify
    # always returns 0 even on no-op, so we can't rely on its exit code
    # to detect change.) PSK is hidden by default — use `--show-secrets`.
    # connection.interface-name is included so a pre-v0.7.0 AP pinned to wlan0
    # migrates onto the ALFA interface.
    nmcli_get() { sudo nmcli --show-secrets -g "$1" con show "$AP_CON_NAME" 2>/dev/null; }
    diff_ap=false
    for spec in \
        "connection.interface-name=$AP_IFACE" \
        "802-11-wireless.ssid=$AP_SSID" \
        "802-11-wireless.mode=ap" \
        "802-11-wireless.band=$AP_BAND" \
        "802-11-wireless.channel=$AP_CHANNEL" \
        "802-11-wireless-security.key-mgmt=wpa-psk" \
        "802-11-wireless-security.psk=$AP_PSK" \
        "ipv4.method=shared" \
        "ipv4.addresses=$AP_ADDR" \
        "connection.autoconnect=yes"; do
        key="${spec%%=*}"; want="${spec#*=}"
        have=$(nmcli_get "$key" || true)
        if [ "$have" != "$want" ]; then
            diff_ap=true
            break
        fi
    done
    if [ "$diff_ap" = "true" ]; then
        echo "  Settings differ — applying."
        sudo nmcli connection modify "$AP_CON_NAME" \
            connection.interface-name "$AP_IFACE" \
            802-11-wireless.ssid "$AP_SSID" \
            802-11-wireless.mode ap \
            802-11-wireless.band "$AP_BAND" \
            802-11-wireless.channel "$AP_CHANNEL" \
            802-11-wireless-security.key-mgmt wpa-psk \
            802-11-wireless-security.psk "$AP_PSK" \
            ipv4.method shared \
            ipv4.addresses "$AP_ADDR" \
            connection.autoconnect yes
        CHANGES_MADE=true
    else
        echo "  Connection already matches desired settings."
    fi
else
    echo "  Creating new AP connection..."
    sudo nmcli connection add \
        type wifi \
        ifname "$AP_IFACE" \
        con-name "$AP_CON_NAME" \
        autoconnect yes \
        ssid "$AP_SSID" \
        802-11-wireless.mode ap \
        802-11-wireless.band "$AP_BAND" \
        802-11-wireless.channel "$AP_CHANNEL" \
        802-11-wireless-security.key-mgmt wpa-psk \
        802-11-wireless-security.psk "$AP_PSK" \
        ipv4.method shared \
        ipv4.addresses "$AP_ADDR"
    CHANGES_MADE=true
fi

# Bring the AP up only if it isn't already, OR if settings just changed
# (changes require a cycle to take effect, including a migration from wlan0 to
# the ALFA interface). Avoids momentarily dropping AP clients on no-op re-runs.
ap_state=$(nmcli -t -f GENERAL.STATE con show "$AP_CON_NAME" 2>/dev/null | head -1)
if [ "$CHANGES_MADE" = "true" ] || [ "$ap_state" != "activated" ]; then
    sudo nmcli connection down "$AP_CON_NAME" >/dev/null 2>&1 || true
    sudo nmcli connection up "$AP_CON_NAME" >/dev/null 2>&1 || true
fi

# --- 3. eth0 addressing mode -------------------------------------------------
# Delegated to setup_eth.sh, which is the only writer of the eth0 netplan file.
# This step used to render its own dual-IP block here (static AND DHCP at the
# same time), which is what made the static address drop periodically. Keeping
# one writer means this script and `racecar eth` cannot disagree.
ETH_MODE="${RACECAR_ETH_MODE:-static}"
echo "[3/4] Configuring eth0 ($ETH_MODE)..."
SETUP_ETH="$(dirname "$0")/setup_eth.sh"
if [ ! -x "$SETUP_ETH" ]; then
    echo "ERROR: $SETUP_ETH not found or not executable." >&2
    exit 1
fi
# setup_eth.sh applies and verifies on its own, so its work is not folded into
# CHANGES_MADE; the netplan apply below is for the AP/dispatcher changes only.
RACECAR_ETH_STATIC="$ETH_STATIC_ADDR" bash "$SETUP_ETH" "$ETH_MODE" --force


# Only `netplan apply` when something actually changed — it triggers a
# NetworkManager reconfigure that briefly bounces eth0 (and on some systems
# logs noisy systemd-networkd warnings even when we render via NM).
if [ "$CHANGES_MADE" = "true" ]; then
    echo
    echo "Applying netplan..."
    sudo netplan apply
    # `netplan apply` briefly bounces NetworkManager's D-Bus service. Wait for
    # it to come back so the wlan0 reset below runs against a live NM instead
    # of failing with "NetworkManager is not running" and silently no-op'ing.
    for _ in $(seq 1 15); do
        nmcli general status >/dev/null 2>&1 && break
        sleep 1
    done
fi

# --- 4. Reset the Pi's built-in wlan0 to default (client) mode ---------------
# Done LAST: by this point the AP is up on the ALFA interface, so removing an
# AP connection left on wlan0 by a pre-v0.7.0 setup can't strand the box. If
# anything above failed under `set -e`, we never get here.
echo "[4/4] Resetting the Pi's built-in wlan0 to default (client) mode..."
mapfile -t wlan0_aps < <(
    nmcli -t -f NAME,TYPE con show |
    awk -F: '$2 == "802-11-wireless" { print $1 }'
)
reset_any=false
for con in "${wlan0_aps[@]}"; do
    [ -z "$con" ] && continue
    ifn=$(sudo nmcli -g connection.interface-name con show "$con" 2>/dev/null || true)
    mode=$(sudo nmcli -g 802-11-wireless.mode con show "$con" 2>/dev/null || true)
    if [ "$ifn" = "wlan0" ] && [ "$mode" = "ap" ]; then
        echo "  Removing stale AP connection '$con' pinned to wlan0."
        sudo nmcli connection delete "$con"
        CHANGES_MADE=true
        reset_any=true
    fi
done
# Ensure wlan0 is NetworkManager-managed (available as a normal client).
sudo nmcli device set wlan0 managed yes >/dev/null 2>&1 || true
if [ "$reset_any" = "false" ]; then
    echo "  No AP connection bound to wlan0; left as a managed client interface."
fi

echo
echo "=== Done ==="
echo
echo "Verify with:"
echo "  racecar eth status                 # one IPv4 address, no conflict"
echo "  iw dev $AP_IFACE info              # ssid $AP_SSID, type AP, ch $AP_CHANNEL"
echo "  iw dev wlan0 info                 # type managed (client/default)"
echo "  sudo iptables-nft -L FORWARD -nv  # two REJECT rules with $AP_IFACE in in/out columns"
echo "                                    # (use -nv; plain -n hides the iface columns)"
echo
echo "Join the AP from a client:"
echo "  SSID: $AP_SSID"
echo "  Password: $AP_PSK"
echo "  Pi reachable at $AP_ADDR (or http://racecar-neo.local)"
if [ "$CHANGES_MADE" = "false" ]; then
    echo
    echo "(No configuration changes were necessary — system already matched.)"
fi
