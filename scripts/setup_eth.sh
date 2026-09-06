#!/bin/bash
# setup_eth.sh - put eth0 in exactly one IPv4 addressing mode.
#
# Usage: setup_eth.sh <static|dynamic|status> [--addr=CIDR] [--force]
#
# eth0 previously carried a static address and a DHCP lease at the same time.
# NetworkManager reconciles the whole IPv4 config for an interface on every
# lease event, so the static was repeatedly torn down and re-added; in the
# field the link drops periodically and comes back only after the cable is
# physically reseated. The two modes are now mutually exclusive, and this
# script is the only writer of the netplan file, so setup_networking.sh and
# `racecar eth` cannot disagree about what eth0 should look like.
#
# Static is the default because a known address is what makes a car
# debuggable on a bare switch. It carries no gateway or DNS in either address
# family, so a static car has no route out over ethernet and reaches the
# internet over wlan0 or not at all.
#
# IPv6 keeps its addresses but never a default route in static mode. Router
# advertisements would otherwise hand eth0 a v6 default route even with no v4
# gateway configured, and since most large destinations are dual-stack a
# static car would send most of its traffic out an interface the design
# treats as inert. ipv6.never-default suppresses that route and nothing else.
# The kernel accept_ra sysctls are not the lever: they read 0 on eth0 while
# the routes are still proto ra, because NetworkManager handles RA itself.
#
# Idempotent: the netplan file is written only when the rendered content
# differs, and `netplan apply` runs only when something changed.
#
# Test hooks:
#   RACECAR_ETH_NETPLAN   netplan file path (default /etc/netplan/99-racecar-eth0.yaml)
#   RACECAR_ETH_CONFIG    persisted settings file
#   RACECAR_ETH_DRY_RUN=1 render and write without sudo; never runs netplan apply

set -eo pipefail

IFACE="${RACECAR_ETH_IFACE:-eth0}"
NETPLAN_PATH="${RACECAR_ETH_NETPLAN:-/etc/netplan/99-racecar-eth0.yaml}"
DRY_RUN="${RACECAR_ETH_DRY_RUN:-0}"

SUDO="sudo"
[ "$DRY_RUN" = "1" ] && SUDO=""

USER_HOME="$(getent passwd "${SUDO_USER:-$USER}" | cut -d: -f6)"
CONFIG_FILE="${RACECAR_ETH_CONFIG:-${USER_HOME}/.config/racecar/networking.env}"

# Persisted settings feed the defaults; a variable already in the environment
# always wins, matching setup_networking.sh.
if [ -f "$CONFIG_FILE" ]; then
    while IFS='=' read -r key val; do
        [ -z "$key" ] && continue
        case "$key" in \#*) continue ;; esac
        [[ "$key" =~ ^[A-Z_][A-Z0-9_]*$ ]] || continue
        if [ -z "${!key:-}" ]; then
            val="${val%\"}"; val="${val#\"}"
            export "$key=$val"
        fi
    done < "$CONFIG_FILE"
fi

STATIC_ADDR="${RACECAR_ETH_STATIC:-192.168.52.200/24}"

usage() {
    cat <<'USAGE'
usage: setup_eth.sh <static|dynamic|status> [--addr=CIDR] [--force]

  static    one fixed address (default 192.168.52.200/24), no gateway,
            no IPv6 default route. The shipped default.
  dynamic   address and default route from DHCP.
  status    configured mode, live addresses, routes, and conflict checks.

  --addr=CIDR  static address to use, persisted for later runs
  --force      skip the confirmation when the calling SSH session is on eth0
USAGE
}

# --- argument parsing ---------------------------------------------------------

MODE=""
FORCE=0
for arg in "$@"; do
    case "$arg" in
        static|dynamic|status) MODE="$arg" ;;
        --addr=*)   STATIC_ADDR="${arg#*=}" ;;
        --force|-y) FORCE=1 ;;
        --help|-h)  usage; exit 0 ;;
        *)
            echo "setup_eth.sh: unknown argument '$arg'" >&2
            usage >&2
            exit 2
            ;;
    esac
done

[ -z "$MODE" ] && MODE="status"

# --- inspection helpers -------------------------------------------------------

# Global IPv4 addresses on the interface, one CIDR per line.
v4_addrs() {
    ip -4 -o addr show "$IFACE" scope global 2>/dev/null | awk '{print $4}'
}

v6_default_route() { ip -6 route show default dev "$IFACE" 2>/dev/null; }
v4_default_route() { ip -4 route show default dev "$IFACE" 2>/dev/null; }

# Read a root-owned file without ever prompting. netplan files are mode 600
# root:root, so a plain user cannot read them; `racecar eth status` is
# read-only and must not stop to ask for a password, so this degrades to an
# empty result and the caller reports that it could not look.
read_priv() {
    if [ -r "$1" ]; then
        cat "$1" 2>/dev/null
    elif [ "$DRY_RUN" != "1" ]; then
        sudo -n cat "$1" 2>/dev/null
    fi
}

# The mode the netplan file declares, independent of what is currently live.
# "unreadable" and "unknown" are different answers: the first means we could
# not look, the second means we looked and the file declares neither.
configured_mode() {
    local content
    if [ ! -e "$NETPLAN_PATH" ] && ! sudo -n test -e "$NETPLAN_PATH" 2>/dev/null; then
        echo "absent"
        return
    fi
    content="$(read_priv "$NETPLAN_PATH")"
    if [ -z "$content" ]; then
        echo "unreadable"
    elif grep -qE '^[[:space:]]*dhcp4:[[:space:]]*true' <<<"$content"; then
        echo "dynamic"
    elif grep -qE '^[[:space:]]*dhcp4:[[:space:]]*false' <<<"$content"; then
        echo "static"
    else
        echo "unknown"
    fi
}

# Other netplan files that also configure this interface. Nothing trips this
# today (the 90-NM-*.yaml files are wifi connections and carry no ethernet
# stanza), but a competing address from another source is a second route back
# to the same conflict, so the check stays as a guard.
# Sets COMPETING to the matching files and COMPETING_SCANNED to 0 when none of
# the files could be read, so status can distinguish "nothing competes" from
# "could not look".
competing_netplan() {
    local f base dir content
    base="$(basename "$NETPLAN_PATH")"
    dir="$(dirname "$NETPLAN_PATH")"
    COMPETING=""
    COMPETING_SCANNED=0
    for f in "$dir"/*.yaml; do
        [ -e "$f" ] || continue
        [ "$(basename "$f")" = "$base" ] && continue
        content="$(read_priv "$f")"
        [ -z "$content" ] && continue
        COMPETING_SCANNED=$((COMPETING_SCANNED + 1))
        if grep -qE "^[[:space:]]*(ethernets:|${IFACE}:)" <<<"$content"; then
            COMPETING="${COMPETING}${f}"$'\n'
        fi
    done
}

# True when the invoking SSH session arrives on this interface, in which case
# applying a mode change cuts the caller off.
session_is_on_iface() {
    [ -n "${SSH_CONNECTION:-}" ] || return 1
    local server_ip
    server_ip="$(awk '{print $3}' <<<"$SSH_CONNECTION")"
    [ -n "$server_ip" ] || return 1
    v4_addrs | cut -d/ -f1 | grep -qx "$server_ip"
}

# --- renderers ----------------------------------------------------------------

render_static() {
    cat <<YAML
network:
  version: 2
  ethernets:
    ${IFACE}:
      renderer: NetworkManager
      dhcp4: false
      dhcp6: true
      optional: true
      addresses:
      - "$STATIC_ADDR"
      networkmanager:
        passthrough:
          ipv6.never-default: "true"
YAML
}

render_dynamic() {
    cat <<YAML
network:
  version: 2
  ethernets:
    ${IFACE}:
      renderer: NetworkManager
      dhcp4: true
      dhcp6: true
      optional: true
      dhcp4-overrides:
        route-metric: 100
      networkmanager:
        passthrough:
          ipv4.dhcp-timeout: "15"
          ipv4.may-fail: "true"
YAML
}

# --- status -------------------------------------------------------------------

print_status() {
    local cfg live_v4 count v4def v6def rc=0
    cfg="$(configured_mode)"

    echo "=== $IFACE addressing ==="
    case "$cfg" in
        unreadable)
            echo "  configured mode:  unreadable (netplan is root-only; re-run with sudo to check)"
            ;;
        absent)
            echo "  configured mode:  no racecar netplan file yet"
            ;;
        *)
            echo "  configured mode:  $cfg"
            ;;
    esac
    echo "  persisted mode:   ${RACECAR_ETH_MODE:-static (default)}"
    echo "  static address:   $STATIC_ADDR"
    echo

    live_v4="$(v4_addrs)"
    if [ -z "$live_v4" ]; then
        echo "  IPv4:             (none)"
    else
        while read -r a; do
            [ -n "$a" ] && echo "  IPv4:             $a"
        done <<<"$live_v4"
    fi

    v4def="$(v4_default_route)"
    v6def="$(v6_default_route)"
    if [ -n "$v4def" ]; then
        echo "  IPv4 default:     via $(awk '{print $3}' <<<"$v4def")"
    else
        echo "  IPv4 default:     (none)"
    fi
    if [ -n "$v6def" ]; then
        echo "  IPv6 default:     via $(awk '{print $3}' <<<"$v6def")"
    else
        echo "  IPv6 default:     (none)"
    fi
    echo

    # The conflict check is IPv4-scoped on purpose. The link-local fe80::
    # address is always present and SLAAC may add more, so counting every
    # address on the interface would report a conflict on a healthy car.
    count="$(grep -c . <<<"$live_v4" || true)"
    [ -z "$live_v4" ] && count=0
    if [ "$count" -gt 1 ]; then
        echo "  [FAIL] $count global IPv4 addresses on $IFACE; expected exactly one."
        echo "         This is the dual-address state that drops the static."
        echo "         Fix with: racecar eth static   (or: racecar eth dynamic)"
        rc=1
    elif [ "$count" -eq 0 ]; then
        echo "  [WARN] no global IPv4 address on $IFACE (cable unplugged?)"
    else
        echo "  [ OK ] exactly one global IPv4 address"
    fi

    if [ "$cfg" = "static" ] && [ -n "$v6def" ]; then
        echo "  [FAIL] static mode, but $IFACE holds an IPv6 default route."
        echo "         ipv6.never-default did not take. Re-apply: racecar eth static"
        rc=1
    fi

    competing_netplan
    if [ -n "$COMPETING" ]; then
        echo "  [WARN] other netplan files also configure $IFACE:"
        grep . <<<"$COMPETING" | sed 's/^/           /'
        rc=1
    elif [ "$COMPETING_SCANNED" -eq 0 ]; then
        echo "  [INFO] could not read other netplan files; competing-config scan skipped"
    else
        echo "  [ OK ] no other netplan file configures $IFACE"
    fi

    return $rc
}

if [ "$MODE" = "status" ]; then
    print_status
    exit $?
fi

# --- apply --------------------------------------------------------------------

echo "=== Setting $IFACE to $MODE ==="
if [ "$MODE" = "static" ]; then
    echo "  address: $STATIC_ADDR (no gateway, no DNS, no IPv6 default)"
else
    echo "  address: from DHCP"
fi

if [ "$DRY_RUN" != "1" ] && session_is_on_iface && [ "$FORCE" -ne 1 ]; then
    echo
    echo "WARNING: this SSH session arrives on $IFACE, so applying the change"
    echo "         will drop your connection. Reconnect over the AP (wlan1),"
    echo "         wlan0, or an HDMI console, or re-run with --force."
    if [ -t 0 ]; then
        read -r -p "Continue anyway? [y/N] " reply
        case "$reply" in
            [yY]*) ;;
            *) echo "Aborted; nothing changed."; exit 0 ;;
        esac
    else
        echo "Aborted; nothing changed (no TTY to confirm on)."
        exit 3
    fi
fi

TMP_NETPLAN="$(mktemp)"
cleanup() { [ -n "${TMP_NETPLAN:-}" ] && [ -e "$TMP_NETPLAN" ] && unlink "$TMP_NETPLAN"; }
trap cleanup EXIT

if [ "$MODE" = "static" ]; then
    render_static >"$TMP_NETPLAN"
else
    render_dynamic >"$TMP_NETPLAN"
fi

CHANGED=false
if $SUDO cmp -s "$TMP_NETPLAN" "$NETPLAN_PATH" 2>/dev/null; then
    echo "  $NETPLAN_PATH already matches $MODE mode."
else
    if [ -n "$SUDO" ]; then
        $SUDO install -m 600 -o root -g root "$TMP_NETPLAN" "$NETPLAN_PATH"
    else
        mkdir -p "$(dirname "$NETPLAN_PATH")"
        install -m 600 "$TMP_NETPLAN" "$NETPLAN_PATH"
    fi
    echo "  Wrote $NETPLAN_PATH"
    CHANGED=true
fi

# Persist the mode so a later bare `racecar eth` reports what was intended,
# not only what happens to be live.
mkdir -p "$(dirname "$CONFIG_FILE")"
touch "$CONFIG_FILE"
chmod 600 "$CONFIG_FILE"
if grep -q '^RACECAR_ETH_MODE=' "$CONFIG_FILE" 2>/dev/null; then
    sed -i "s|^RACECAR_ETH_MODE=.*|RACECAR_ETH_MODE=\"$MODE\"|" "$CONFIG_FILE"
else
    printf 'RACECAR_ETH_MODE="%s"\n' "$MODE" >> "$CONFIG_FILE"
fi
if [ "$MODE" = "static" ]; then
    if grep -q '^RACECAR_ETH_STATIC=' "$CONFIG_FILE" 2>/dev/null; then
        sed -i "s|^RACECAR_ETH_STATIC=.*|RACECAR_ETH_STATIC=\"$STATIC_ADDR\"|" "$CONFIG_FILE"
    else
        printf 'RACECAR_ETH_STATIC="%s"\n' "$STATIC_ADDR" >> "$CONFIG_FILE"
    fi
fi

if [ "$DRY_RUN" = "1" ]; then
    echo "=== Dry run; not applying ==="
    exit 0
fi

if [ "$CHANGED" = "false" ]; then
    echo
    echo "=== No change needed ==="
    exit 0
fi

echo
echo "Applying netplan..."
sudo netplan apply

# `netplan apply` bounces NetworkManager's D-Bus service. Wait for it to come
# back before verifying, otherwise the address check races the reconfigure.
for _ in $(seq 1 15); do
    nmcli general status >/dev/null 2>&1 && break
    sleep 1
done
sleep 2

# --- verify -------------------------------------------------------------------

echo
echo "=== Verifying ==="
LIVE="$(v4_addrs)"
COUNT="$(grep -c . <<<"$LIVE" || true)"
[ -z "$LIVE" ] && COUNT=0
RC=0

if [ "$COUNT" -ne 1 ]; then
    echo "  [FAIL] expected exactly one global IPv4 address, found $COUNT"
    [ -n "$LIVE" ] && sed 's/^/           /' <<<"$LIVE"
    RC=1
elif [ "$MODE" = "static" ]; then
    if [ "$LIVE" = "$STATIC_ADDR" ]; then
        echo "  [ OK ] $IFACE holds $LIVE"
    else
        echo "  [FAIL] expected $STATIC_ADDR, found $LIVE"
        RC=1
    fi
else
    if [ "$LIVE" = "$STATIC_ADDR" ]; then
        echo "  [FAIL] dynamic mode, but $IFACE still holds the static $LIVE"
        RC=1
    else
        echo "  [ OK ] $IFACE holds $LIVE from DHCP"
    fi
fi

if [ "$MODE" = "static" ]; then
    if [ -n "$(v6_default_route)" ]; then
        echo "  [FAIL] $IFACE still has an IPv6 default route"
        RC=1
    else
        echo "  [ OK ] no IPv6 default route via $IFACE"
    fi
fi

echo
if [ "$RC" -eq 0 ]; then
    echo "=== $IFACE is in $MODE mode ==="
else
    echo "=== $IFACE did not reach a clean $MODE state ==="
fi
exit $RC
