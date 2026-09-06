# RACECAR Neo shell tool — `racecar <subcommand>`.
# Sourced from ~/.bashrc by setup_user_env.sh.
# Not executed directly: it defines a `racecar` shell function so the build /
# test / source subcommands can mutate the current shell (PWD, env).

racecar() {
    local pkg="racecar_neo_ros2_driver"
    local ws="$HOME/ros2_ws"
    local pkg_dir="$ws/src/$pkg"
    local cmd="${1:-help}"
    shift || true

    case "$cmd" in
        build)
            ( cd "$ws" && colcon build --packages-select "$pkg" --symlink-install "$@" ) \
                && source "$ws/install/setup.bash"
            ;;

        test)
            ( cd "$ws" \
                && colcon test --packages-select "$pkg" --event-handlers console_direct+ "$@" \
                && colcon test-result --verbose )
            ;;

        source)
            # shellcheck disable=SC1091
            source "$ws/install/setup.bash"
            ;;

        teleop)
            # Use the launch wrapper so we get a timestamped log dir at
            # ~/logs/<ts>/ and a fresh FastRTPS SHM sweep. Extra args (e.g.
            # `lidar_enable:=false`) forward through to ros2 launch.
            bash "$pkg_dir/scripts/launch_teleop.sh" "$@"
            ;;

        cd)
            # Hop to the package source dir. Has to be a shell function (not
            # subprocess) so the cd sticks in the user's interactive shell.
            cd "$pkg_dir" || return 1
            ;;

        launch)
            local name="$1"
            if [[ -z "$name" ]]; then
                echo "usage: racecar launch <name>   # e.g. racecar launch dotmatrix" >&2
                return 2
            fi
            shift
            ros2 launch "$pkg" "${name}.launch.py" "$@"
            ;;

        clear)
            local target=""
            for arg in "$@"; do
                case "$arg" in
                    --dmatrix|--dotmatrix) target="dmatrix" ;;
                    *) echo "racecar clear: unknown flag '$arg'" >&2; return 2 ;;
                esac
            done
            case "$target" in
                dmatrix)
                    python3 "$pkg_dir/scripts/clear_dotmatrix.py"
                    ;;
                "")
                    echo "usage: racecar clear --dmatrix" >&2
                    return 2
                    ;;
            esac
            ;;

        udev)
            bash "$pkg_dir/scripts/setup_udev.sh"
            ;;

        watchdog)
            # Run the watchdog in the foreground. Restarts dead nodes via
            # their individual launch files; logs to ~/logs/latest/watchdog.log.
            # When `racecar-watchdog.service` is installed, prefer
            # `racecar service start watchdog`.
            python3 "$pkg_dir/scripts/watchdog.py" "$@"
            ;;

        service)
            local action="${1:-status}"
            shift || true
            local -a units=("racecar-teleop" "racecar-watchdog"
                            "racecar-dashboard" "racecar-jupyter")
            case "$action" in
                install)
                    bash "$pkg_dir/scripts/setup_services.sh"
                    ;;
                start)
                    if [[ -n "$1" ]]; then
                        sudo systemctl start "racecar-$1"
                    else
                        # Start teleop; BindsTo pulls watchdog with it.
                        sudo systemctl start racecar-teleop
                    fi
                    ;;
                stop)
                    if [[ -n "$1" ]]; then
                        sudo systemctl stop "racecar-$1"
                    else
                        sudo systemctl stop racecar-teleop
                    fi
                    ;;
                restart)
                    if [[ -n "$1" ]]; then
                        sudo systemctl restart "racecar-$1"
                    else
                        sudo systemctl restart racecar-teleop
                    fi
                    ;;
                enable|disable)
                    for u in "${units[@]}"; do
                        sudo systemctl "$action" "$u"
                    done
                    ;;
                logs)
                    local unit="${1:-teleop}"
                    sudo journalctl -u "racecar-$unit" -f
                    ;;
                status|"")
                    for u in "${units[@]}"; do
                        local state
                        state=$(systemctl is-active "$u" 2>&1 || true)
                        local enabled
                        enabled=$(systemctl is-enabled "$u" 2>&1 || true)
                        printf "  %-22s  active=%-12s enabled=%s\n" \
                            "$u" "$state" "$enabled"
                    done
                    ;;
                -h|--help|help)
                    cat <<'__RC_SVC_HELP__'
usage: racecar service <action> [unit]
actions:
  install        Drop unit files in /etc/systemd/system/ + daemon-reload + enable
  start [name]   Start racecar-<name>; default = teleop (watchdog follows via BindsTo)
  stop [name]    Stop racecar-<name>; default = teleop
  restart [name] Restart racecar-<name>; default = teleop
  enable         Enable all racecar-* units (auto-start on boot)
  disable        Disable all racecar-* units
  logs [name]    journalctl -u racecar-<name> -f; default = teleop
  status         active/enabled snapshot for all units (default)
units: teleop, watchdog, dashboard, jupyter
__RC_SVC_HELP__
                    ;;
                *)
                    echo "racecar service: unknown action '$action'" >&2
                    return 2
                    ;;
            esac
            ;;

        setup)
            local phase="${1:-}"
            shift || true
            case "$phase" in
                "")
                    echo "usage: racecar setup <phase>" >&2
                    echo "  phases: all, networking, realsense" >&2
                    return 2
                    ;;
                all)
                    bash "$pkg_dir/scripts/setup_all.sh" "$@"
                    ;;
                realsense)
                    # Offline per-machine firmware flash for airgapped fleet units.
                    # Reads the staged .bin from /opt/racecar/firmware (no network);
                    # idempotent, skips a camera already at the target version.
                    bash "$pkg_dir/scripts/flash_realsense_offline.sh" "$@"
                    ;;
                networking)
                    # Persist any --flag values to ~/.config/racecar/networking.env
                    # so subsequent runs (and reboots) keep the same settings.
                    local cfg_dir="$HOME/.config/racecar"
                    local cfg_file="$cfg_dir/networking.env"
                    mkdir -p "$cfg_dir"
                    # Overridable so tests can stub the destructive script.
                    local net_script="${RACECAR_NETWORKING_SCRIPT:-$pkg_dir/scripts/setup_networking.sh}"

                    # Load existing persisted values into a local assoc array.
                    local -A vals=()
                    if [[ -f "$cfg_file" ]]; then
                        while IFS='=' read -r k v; do
                            [[ -z "$k" || "$k" =~ ^# ]] && continue
                            v="${v%\"}"; v="${v#\"}"
                            vals["$k"]="$v"
                        done < "$cfg_file"
                    fi

                    # Pass 1: parse every flag into either vals[] (overrides)
                    # or an action variable. Don't act yet — that way `--ssid=foo
                    # --show` persists foo before showing, and `--ssid=foo --reset`
                    # is rejected as nonsense (would be lost immediately).
                    local action="apply"   # default: persist + run setup_networking.sh
                    local vals_changed=0
                    while [[ "$1" == --* ]]; do
                        local arg="$1"; shift
                        local key val
                        if [[ "$arg" == *=* ]]; then
                            key="${arg%%=*}"; val="${arg#*=}"
                        else
                            key="$arg"; val="${1:-}"; shift || true
                        fi
                        case "$key" in
                            --ssid)        vals[RACECAR_AP_SSID]="$val";    vals_changed=1 ;;
                            --psk)         vals[RACECAR_AP_PSK]="$val";     vals_changed=1 ;;
                            --channel)     vals[RACECAR_AP_CHANNEL]="$val"; vals_changed=1 ;;
                            --ap-addr)     vals[RACECAR_AP_ADDR]="$val";    vals_changed=1 ;;
                            --ap-iface)    vals[RACECAR_AP_IFACE]="$val";   vals_changed=1 ;;
                            --eth-static)  vals[RACECAR_ETH_STATIC]="$val"; vals_changed=1 ;;
                            --eth-mode)    vals[RACECAR_ETH_MODE]="$val";   vals_changed=1 ;;
                            --show)        action="show" ;;
                            --reset)       action="reset" ;;
                            --help|-h)     action="help" ;;
                            *)
                                echo "racecar setup networking: unknown flag '$key'" >&2
                                return 2
                                ;;
                        esac
                    done

                    if [[ "$action" == "help" ]]; then
                        cat <<'__RC_NET_HELP__'
usage: racecar setup networking [--ssid=NAME] [--psk=PASS]
                                [--channel=N] [--ap-addr=CIDR]
                                [--ap-iface=NAME] [--eth-static=CIDR]
                                [--eth-mode=static|dynamic]
                                [--show] [--reset]
Persists any --flag values to ~/.config/racecar/networking.env and then
runs scripts/setup_networking.sh (eth0 addressing + ALFA-dongle isolated AP).
  --ap-iface=NAME  AP interface (default wlan1, the ALFA dongle)
  --eth-mode=      eth0 addressing: static (default) or dynamic. Same setting
                   as `racecar eth`; eth0 never holds both at once.
  --show   print current persisted overrides and exit
  --reset  disable the wlan1 AP (down + delete the connection) and clear the
           saved car ID/overrides; eth0 is left unchanged. Use before imaging.
With no --ssid or saved car ID, plain `racecar setup networking` prompts for
this car's ID and sets the SSID to racecar-neo-<id> (so multiple cars differ).
Persistence runs BEFORE --show, so
  racecar setup networking --ssid=foo --show
saves foo, then prints the new file contents.
WARNING: this reconfigures the AP interface. If you're SSH'd over the AP,
run from a wired session or the console instead.
__RC_NET_HELP__
                        return 0
                    fi

                    if [[ "$action" == "reset" ]]; then
                        if [[ $vals_changed -eq 1 ]]; then
                            echo "racecar setup networking: --reset cannot be combined with override flags" >&2
                            return 2
                        fi
                        rm -f "$cfg_file"
                        echo "Cleared $cfg_file; defaults will apply on next run."
                        echo "Disabling the wlan1 AP..."
                        RACECAR_AP_RESET=1 bash "$net_script"
                        return $?
                    fi

                    # Prompt for this car's ID (appended to the SSID as
                    # racecar-neo-<id>) when none is known, so multiple cars
                    # don't share an SSID. Skipped when an ID or full SSID is
                    # already set (via --ssid or persisted) or stdin is not a TTY.
                    if [[ "$action" == "apply" && -z "${vals[RACECAR_AP_SSID]:-}" \
                          && -z "${vals[RACECAR_AP_ID]:-}" && -t 0 ]]; then
                        local car_id=""
                        read -r -p "Enter this car's ID (number appended to 'racecar-neo-', e.g. 1): " car_id
                        if [[ -n "$car_id" ]]; then
                            vals[RACECAR_AP_ID]="$car_id"; vals_changed=1
                        fi
                    fi

                    # Persist any new --flag values (applies to apply/show paths).
                    if [[ $vals_changed -eq 1 ]]; then
                        : > "$cfg_file"
                        chmod 600 "$cfg_file"
                        echo "# racecar networking overrides — managed by 'racecar setup networking'" >> "$cfg_file"
                        for k in RACECAR_AP_SSID RACECAR_AP_ID RACECAR_AP_PSK RACECAR_AP_CHANNEL RACECAR_AP_ADDR RACECAR_AP_IFACE RACECAR_ETH_STATIC RACECAR_ETH_MODE; do
                            if [[ -n "${vals[$k]:-}" ]]; then
                                printf '%s="%s"\n' "$k" "${vals[$k]}" >> "$cfg_file"
                            fi
                        done
                        echo "Saved overrides to $cfg_file"
                    fi

                    if [[ "$action" == "show" ]]; then
                        if [[ -s "$cfg_file" ]]; then
                            echo "Persisted networking config ($cfg_file):"
                            cat "$cfg_file"
                        else
                            echo "No persisted networking config — script defaults will apply."
                        fi
                        return 0
                    fi

                    bash "$net_script"
                    ;;
                *)
                    echo "racecar setup: unknown phase '$phase'" >&2
                    echo "  phases: all, networking, realsense" >&2
                    return 2
                    ;;
            esac
            ;;

        eth)
            # eth0 addressing mode. Static (the default) or DHCP, never both:
            # carrying a static address and a lease together is what makes the
            # static drop. All the logic lives in setup_eth.sh so this command
            # and setup_networking.sh cannot disagree about the netplan file.
            local eth_script="${RACECAR_ETH_SCRIPT:-$pkg_dir/scripts/setup_eth.sh}"
            local action="${1:-status}"
            shift || true
            case "$action" in
                static|dynamic|status)
                    bash "$eth_script" "$action" "$@"
                    ;;
                monitor)
                    # Foreground address/carrier logger. The mutual-exclusion
                    # fix is structurally right but unproven, so this is what
                    # turns that into evidence. For a multi-day soak use the
                    # racecar-eth-monitor.service unit instead.
                    python3 "$pkg_dir/scripts/eth_monitor.py" "$@"
                    ;;
                -h|--help|help)
                    cat <<'__RC_ETH_HELP__'
usage: racecar eth [action] [flags]
actions:
  status    configured mode, live addresses, routes, conflict checks (default)
  static    one fixed address (default 192.168.52.200/24), no gateway and no
            IPv6 default route; the shipped default
  dynamic   address and default route from DHCP
  monitor   log eth0 addressing and link state until interrupted; use this to
            confirm the static address stops dropping. Flags: --interval,
            --heartbeat, --log, --once. For days rather than hours, enable
            scripts/racecar-eth-monitor.service instead.
flags:
  --addr=CIDR  static address to use; persisted for later runs
  --force      skip the confirmation when this SSH session arrives on eth0
eth0 holds one IPv4 addressing mode at a time. Static is the default because a
known address is what makes a car debuggable on a bare switch; it carries no
gateway, so a static car reaches the internet over wlan0 or not at all.
WARNING: switching modes drops an SSH session arriving over eth0. Run it from
the AP (wlan1), wlan0, or an HDMI console.
__RC_ETH_HELP__
                    ;;
                *)
                    echo "racecar eth: unknown action '$action'" >&2
                    echo "actions: status, static, dynamic, monitor" >&2
                    return 2
                    ;;
            esac
            ;;

        wifi)
            # Client radio only. wlan0 is the Pi's built-in Broadcom part;
            # wlan1 carries the AP that operators are often connected over. So
            # every nmcli call here is pinned to wlan0, and `disconnect`
            # targets the device rather than a connection name that could
            # match the AP profile. Nothing this command does should be able
            # to drop the link its own operator is using.
            local wifi_iface="${RACECAR_WIFI_IFACE:-wlan0}"
            local wifi_nmcli="${RACECAR_NMCLI:-nmcli}"
            local action="${1:-status}"
            shift || true

            case "$action" in
                -h|--help|help)
                    cat <<'__RC_WIFI_HELP__'
usage: racecar wifi [action] [args]
actions:
  status              link state, SSID, addresses and DNS for wlan0 (default)
  list [--rescan]     visible networks, one row per SSID, strongest first.
                      Reads the cached scan; --rescan forces a fresh one
                      (about ten seconds).
  connect <ssid>      join a network. Brings up a saved profile as-is, or
                      creates one. Prompts for what it needs.
  disconnect          drop the wlan0 link
connect flags:
  --identity=USER     account for an enterprise (802.1X) network
  --psk=PASS          passphrase for scripted use; note that it lands in
                      shell history, so prefer the prompt
  --ca-cert=PATH      override the CA certificate for an enterprise network
  --domain-suffix-match=DOMAIN
                      override the RADIUS server suffix to require
Enterprise networks ask for an identity and password and nothing else. Server
validation is always on: profiles get system CA certificates plus a
domain-suffix-match derived from the identity's realm, so credentials are
never offered to an access point that cannot prove who it is.
Only wlan0 is ever touched; the AP on wlan1 is left alone.
__RC_WIFI_HELP__
                    return 0
                    ;;
            esac

            # A soft-blocked radio otherwise produces an opaque association
            # failure, so name the condition instead.
            if [[ "$($wifi_nmcli radio wifi 2>/dev/null)" == "disabled" ]]; then
                echo "racecar wifi: the WiFi radio is disabled (rfkill)." >&2
                echo "Re-enable it with: nmcli radio wifi on" >&2
                return 3
            fi
            if ! $wifi_nmcli -t -f DEVICE device status 2>/dev/null | grep -qx "$wifi_iface"; then
                echo "racecar wifi: $wifi_iface is not present." >&2
                echo "The client radio is the Pi's built-in adapter; wlan1 is the AP dongle." >&2
                return 3
            fi

            case "$action" in
                list)
                    local rescan="no"
                    local arg
                    for arg in "$@"; do
                        case "$arg" in
                            --rescan) rescan="yes" ;;
                            *) echo "racecar wifi list: unknown flag '$arg'" >&2; return 2 ;;
                        esac
                    done
                    local saved_ssids
                    saved_ssids=$($wifi_nmcli -t -f NAME,TYPE con show 2>/dev/null |
                        awk -F: '$2 == "802-11-wireless" { print $1 }')
                    local -a fmt_args=(--saved "$saved_ssids" --iface "$wifi_iface")
                    [[ "$rescan" == "yes" ]] && fmt_args+=(--rescanned)
                    $wifi_nmcli -t -f SSID,SIGNAL,SECURITY,IN-USE device wifi list \
                        ifname "$wifi_iface" --rescan "$rescan" 2>/dev/null |
                        python3 "$pkg_dir/scripts/wifi_scan.py" "${fmt_args[@]}"
                    ;;

                connect)
                    local ssid="$1"
                    shift || true
                    if [[ -z "$ssid" || "$ssid" == --* ]]; then
                        echo "usage: racecar wifi connect <ssid> [--identity=USER]" >&2
                        return 2
                    fi
                    local identity="" psk="" ca_cert="" domain_match=""
                    local arg
                    for arg in "$@"; do
                        case "$arg" in
                            --identity=*)             identity="${arg#*=}" ;;
                            --psk=*)                  psk="${arg#*=}" ;;
                            --ca-cert=*)              ca_cert="${arg#*=}" ;;
                            --domain-suffix-match=*)  domain_match="${arg#*=}" ;;
                            *) echo "racecar wifi connect: unknown flag '$arg'" >&2; return 2 ;;
                        esac
                    done

                    # A saved profile already carries its own security config,
                    # enterprise included, so bring it up untouched.
                    if $wifi_nmcli -t -f NAME con show 2>/dev/null | grep -qx "$ssid"; then
                        echo "Using the saved profile '$ssid'."
                        $wifi_nmcli connection up "$ssid" ifname "$wifi_iface" || return $?
                        racecar wifi status
                        return 0
                    fi

                    # Otherwise read the security type off the scan.
                    local security
                    security=$($wifi_nmcli -t -f SSID,SECURITY device wifi list \
                        ifname "$wifi_iface" 2>/dev/null |
                        awk -F: -v s="$ssid" '$1 == s { print $2; exit }')
                    if [[ -z "$security" ]]; then
                        echo "racecar wifi: '$ssid' is not visible on $wifi_iface." >&2
                        echo "Run 'racecar wifi list --rescan' to refresh the scan." >&2
                        return 4
                    fi

                    if [[ "$security" == *802.1X* ]]; then
                        if [[ -z "$identity" ]]; then
                            read -r -p "Identity for $ssid (e.g. user@school.edu): " identity
                        fi
                        if [[ -z "$identity" ]]; then
                            echo "racecar wifi: an enterprise network needs an identity." >&2
                            return 2
                        fi
                        # The RADIUS server must prove who it is before the
                        # password is offered. Without a suffix to match, any
                        # access point broadcasting this SSID could collect the
                        # credential, so refuse rather than connect blind.
                        if [[ -z "$domain_match" ]]; then
                            domain_match="${identity#*@}"
                        fi
                        if [[ -z "$domain_match" || "$domain_match" == "$identity" ]]; then
                            echo "racecar wifi: cannot derive a server domain from '$identity'." >&2
                            echo "Pass --domain-suffix-match=DOMAIN so the RADIUS server can be verified." >&2
                            return 2
                        fi
                        local eap_pw=""
                        read -r -s -p "Password for $identity: " eap_pw; echo
                        if [[ -z "$eap_pw" ]]; then
                            echo "racecar wifi: no password given." >&2
                            return 2
                        fi
                        local -a add_args=(
                            connection add type wifi
                            con-name "$ssid" ifname "$wifi_iface" ssid "$ssid"
                            wifi-sec.key-mgmt wpa-eap
                            802-1x.eap ttls
                            802-1x.phase2-auth mschapv2
                            802-1x.identity "$identity"
                            802-1x.password "$eap_pw"
                            802-1x.domain-suffix-match "$domain_match"
                        )
                        if [[ -n "$ca_cert" ]]; then
                            add_args+=(802-1x.ca-cert "$ca_cert")
                        else
                            add_args+=(802-1x.system-ca-certs yes)
                        fi
                        echo "Creating an enterprise profile for '$ssid' (server must match $domain_match)."
                        $wifi_nmcli "${add_args[@]}" >/dev/null || return $?
                        $wifi_nmcli connection up "$ssid" ifname "$wifi_iface" || return $?
                    elif [[ -z "$security" || "$security" == "--" ]]; then
                        $wifi_nmcli device wifi connect "$ssid" ifname "$wifi_iface" || return $?
                    else
                        if [[ -z "$psk" ]]; then
                            read -r -s -p "Passphrase for $ssid: " psk; echo
                        fi
                        if [[ -z "$psk" ]]; then
                            $wifi_nmcli device wifi connect "$ssid" ifname "$wifi_iface" || return $?
                        else
                            $wifi_nmcli device wifi connect "$ssid" password "$psk" \
                                ifname "$wifi_iface" || return $?
                        fi
                    fi
                    racecar wifi status
                    ;;

                disconnect)
                    $wifi_nmcli device disconnect "$wifi_iface" || return $?
                    cat <<__RC_WIFI_DISC__
Disconnected $wifi_iface. NetworkManager marks the device manually
disconnected, so it will not rejoin on its own until the next
'racecar wifi connect'. The AP on wlan1 is unaffected.
__RC_WIFI_DISC__
                    ;;

                status)
                    echo "=== wifi client ($wifi_iface) ==="
                    local dev_state
                    dev_state=$($wifi_nmcli -t -f DEVICE,STATE device status 2>/dev/null |
                        awk -F: -v d="$wifi_iface" '$1 == d { print $2 }')
                    echo "  state:      ${dev_state:-unknown}"

                    local cur_ssid
                    cur_ssid=$($wifi_nmcli -t -f ACTIVE,SSID,SIGNAL device wifi list \
                        ifname "$wifi_iface" 2>/dev/null |
                        awk -F: '$1 == "yes" { print $2 " (" $3 "%)"; exit }')
                    [[ -n "$cur_ssid" ]] && echo "  network:    $cur_ssid"

                    local ip4 gw dns
                    ip4=$($wifi_nmcli -t -f IP4.ADDRESS device show "$wifi_iface" 2>/dev/null |
                        cut -d: -f2- | paste -sd' ' -)
                    gw=$($wifi_nmcli -t -f IP4.GATEWAY device show "$wifi_iface" 2>/dev/null |
                        cut -d: -f2- | paste -sd' ' -)
                    dns=$($wifi_nmcli -t -f IP4.DNS device show "$wifi_iface" 2>/dev/null |
                        cut -d: -f2- | paste -sd' ' -)
                    echo "  address:    ${ip4:-(none)}"
                    echo "  gateway:    ${gw:-(none)}"
                    echo "  dns:        ${dns:-(none)}"

                    # A campus network handing out 10.42.0.0/24 or the eth0
                    # static subnet makes routing ambiguous and quietly breaks
                    # AP clients, so name the overlap rather than let someone
                    # discover it the hard way.
                    local w_addr ap_addr eth_addr
                    w_addr=$(ip -4 -o addr show "$wifi_iface" scope global 2>/dev/null |
                        awk '{print $4}' | head -1)
                    ap_addr=$(ip -4 -o addr show wlan1 scope global 2>/dev/null |
                        awk '{print $4}' | head -1)
                    eth_addr=$(ip -4 -o addr show eth0 scope global 2>/dev/null |
                        awk '{print $4}' | head -1)
                    if [[ -n "$w_addr" ]]; then
                        python3 - "$w_addr" "${ap_addr:-}" "${eth_addr:-}" <<'__RC_WIFI_COLLIDE__'
import ipaddress
import sys

wifi = sys.argv[1]
try:
    wnet = ipaddress.ip_interface(wifi).network
except ValueError:
    raise SystemExit(0)

for label, addr in (('the AP on wlan1', sys.argv[2]), ('eth0', sys.argv[3])):
    if not addr:
        continue
    try:
        other = ipaddress.ip_interface(addr).network
    except ValueError:
        continue
    if other.overlaps(wnet):
        print(f'  [WARN] {wnet} overlaps {label} ({addr}); routing to that')
        print('         subnet is ambiguous while both are up.')
__RC_WIFI_COLLIDE__
                    fi
                    echo
                    echo "  The AP on wlan1 is a separate radio and is not affected by this command."
                    ;;

                *)
                    echo "racecar wifi: unknown action '$action'" >&2
                    echo "actions: status, list, connect, disconnect" >&2
                    return 2
                    ;;
            esac
            ;;

        desktop)
            # The GNOME desktop ships enabled; headless users turn it off.
            #
            # The boot target is the only lever, and it is sufficient on its
            # own. The display manager unit is `static` (no [Install] section),
            # so `systemctl enable`/`disable` on it cannot work, and it is
            # pulled in by graphical.target rather than by its own enablement.
            # Booting to multi-user.target therefore never starts it.
            #
            # Deliberately reboot-scoped: there is no --now or `isolate`
            # variant, so this can never tear down a desktop session out from
            # under whoever is sitting at it. Packages are left installed, so
            # the toggle is reversible on a car with no network.
            local action="${1:-status}"
            shift || true
            local sctl="${RACECAR_SYSTEMCTL:-systemctl}"
            local sudo_cmd="${RACECAR_SUDO-sudo}"

            case "$action" in
                enable|disable)
                    local target="multi-user.target"
                    [[ "$action" == "enable" ]] && target="graphical.target"

                    local current
                    current=$($sctl get-default 2>/dev/null)
                    if [[ "$current" == "$target" ]]; then
                        echo "Default boot target is already $target; nothing to do."
                        return 0
                    fi

                    $sudo_cmd $sctl set-default "$target" || return $?
                    echo "Default boot target is now $target."
                    if [[ "$action" == "enable" ]]; then
                        echo "The desktop will start on the next boot."
                    else
                        echo "The car will boot headless; the desktop packages stay installed."
                    fi
                    echo "This takes effect after a reboot. The current session is unchanged."
                    echo "Reboot when ready:  sudo reboot"
                    ;;

                status)
                    local default_target active_target dm_unit dm_state sessions
                    default_target=$($sctl get-default 2>/dev/null)
                    if $sctl is-active graphical.target >/dev/null 2>&1; then
                        active_target="graphical.target"
                    else
                        active_target="multi-user.target"
                    fi

                    echo "=== desktop ==="
                    echo "  default target:   ${default_target:-unknown}"
                    echo "  active target:    $active_target"

                    dm_unit=$($sctl show display-manager.service -p Id --value 2>/dev/null)
                    if [[ -n "$dm_unit" ]]; then
                        dm_state=$($sctl is-active "$dm_unit" 2>&1 || true)
                        echo "  display manager:  $dm_unit ($dm_state)"
                    else
                        echo "  display manager:  none installed"
                    fi

                    sessions=$(loginctl list-sessions --no-legend 2>/dev/null | grep -c . || true)
                    echo "  login sessions:   ${sessions:-0}"
                    echo

                    if [[ "$default_target" == "graphical.target" ]]; then
                        echo "  Desktop is enabled (boots to the GNOME session)."
                    elif [[ "$default_target" == "multi-user.target" ]]; then
                        echo "  Desktop is disabled (boots headless)."
                    else
                        echo "  Default target is neither graphical nor multi-user."
                    fi

                    if [[ -n "$default_target" && "$default_target" != "$active_target" ]]; then
                        echo "  Pending: $active_target is running now; reboot to apply."
                    fi
                    ;;

                -h|--help|help)
                    cat <<'__RC_DESKTOP_HELP__'
usage: racecar desktop [enable|disable|status]
  status    default boot target, active target, display manager, and whether
            a reboot is pending (default)
  enable    boot to the GNOME desktop; the shipped default
  disable   boot headless
Changes apply on the next boot. There is no immediate variant, so this cannot
end a desktop session that someone is using. Packages are never removed, so
the toggle works on a car with no network and is reversible with one command.
__RC_DESKTOP_HELP__
                    ;;

                *)
                    echo "racecar desktop: unknown action '$action'" >&2
                    echo "actions: status, enable, disable" >&2
                    return 2
                    ;;
            esac
            ;;

        library)
            # Manage which ~/jupyter_ws/<folder>/library/ is on Python's sys.path
            # by writing a .pth file into the user site-packages directory.
            # Mirrors what the sim installer does inside its venv (see
            # racecar-neo-installer setup.sh), but uses the user-site dir since
            # the Pi has no venv.
            local jws="$HOME/jupyter_ws"
            local site_pkgs
            site_pkgs=$(python3 -c 'import site; print(site.getusersitepackages())' 2>/dev/null)
            local pth_file="$site_pkgs/racecar_student.pth"

            local action=""
            local select_target=""
            while [[ $# -gt 0 ]]; do
                local arg="$1"; shift
                case "$arg" in
                    --select)
                        action="select"
                        select_target="${1:-}"
                        [[ -n "$select_target" ]] && shift
                        ;;
                    --select=*)
                        action="select"
                        select_target="${arg#*=}"
                        ;;
                    --list)    action="list" ;;
                    --reset)   action="reset" ;;
                    --status)  action="status" ;;
                    --help|-h) action="help" ;;
                    *)
                        echo "racecar library: unknown flag '$arg'" >&2
                        return 2
                        ;;
                esac
            done

            if [[ -z "$action" ]]; then
                echo "usage: racecar library [--select <folder> | --list | --reset | --status]" >&2
                return 2
            fi

            if [[ "$action" == "help" ]]; then
                cat <<'__RC_LIB_HELP__'
usage: racecar library <action>
Manages the racecar_student.pth file in user site-packages so Python scripts
(e.g. labs/demo.py) can `import racecar_core` without sys.path hacks.

Actions:
  --select <folder>  Point the .pth at ~/jupyter_ws/<folder>/library/.
                     <folder> must contain library/racecar_core.py.
  --list             List all ~/jupyter_ws/ folders that look like valid
                     student libraries (contain library/racecar_core.py).
                     The currently-selected folder is marked with *.
  --reset            Delete the .pth file (no library on sys.path).
  --status           Show the currently-selected library, or report none.

The .pth is written to:
  ~/.local/lib/pythonX.Y/site-packages/racecar_student.pth
__RC_LIB_HELP__
                return 0
            fi

            if [[ -z "$site_pkgs" ]]; then
                echo "racecar library: could not determine user site-packages directory" >&2
                return 1
            fi

            case "$action" in
                list)
                    if [[ ! -d "$jws" ]]; then
                        echo "No ~/jupyter_ws/ directory found."
                        return 0
                    fi
                    local current=""
                    if [[ -f "$pth_file" ]]; then
                        current=$(head -n 1 "$pth_file")
                    fi
                    local found=0
                    echo "Available libraries in $jws:"
                    local d
                    for d in "$jws"/*/; do
                        [[ -d "$d" ]] || continue
                        local libdir="${d}library"
                        if [[ -f "$libdir/racecar_core.py" ]]; then
                            local name
                            name=$(basename "$d")
                            local marker="  "
                            if [[ "$current" == "${libdir%/}" ]]; then
                                marker=" *"
                            fi
                            printf "%s %s\n" "$marker" "$name"
                            found=1
                        fi
                    done
                    if [[ $found -eq 0 ]]; then
                        echo "  (none — no folder contains library/racecar_core.py)"
                    fi
                    ;;

                select)
                    if [[ -z "$select_target" ]]; then
                        echo "racecar library: --select requires a folder name" >&2
                        echo "  (run 'racecar library --list' to see candidates)" >&2
                        return 2
                    fi
                    local target_dir="$jws/$select_target"
                    local target_lib="$target_dir/library"
                    if [[ ! -d "$target_dir" ]]; then
                        echo "racecar library: '$select_target' is not a folder under $jws" >&2
                        return 2
                    fi
                    if [[ ! -f "$target_lib/racecar_core.py" ]]; then
                        echo "racecar library: '$target_lib/racecar_core.py' not found" >&2
                        echo "  (folder must contain library/racecar_core.py)" >&2
                        return 2
                    fi
                    mkdir -p "$site_pkgs"
                    echo "$target_lib" > "$pth_file"
                    echo "Selected library: $target_lib"
                    echo "  wrote $pth_file"
                    ;;

                reset)
                    if [[ -f "$pth_file" ]]; then
                        rm -f "$pth_file"
                        echo "Reset: removed $pth_file"
                    else
                        echo "Reset: no .pth file to remove ($pth_file)"
                    fi
                    ;;

                status)
                    if [[ -f "$pth_file" ]]; then
                        local current
                        current=$(head -n 1 "$pth_file")
                        echo "Current library: $current"
                        echo "  ($pth_file)"
                        if [[ ! -f "$current/racecar_core.py" ]]; then
                            echo "  WARNING: racecar_core.py not found at this path" >&2
                        fi
                    else
                        echo "No racecar library is currently selected."
                        echo "  Run: racecar library --select <folder>"
                        echo "  (or: racecar library --list)"
                    fi
                    ;;
            esac
            ;;

        cleanup)
            # Find orphaned/stale racecar processes + FastRTPS SHM segments.
            # Dry-run by default; pass --force to actually kill / remove.
            local force=0
            for arg in "$@"; do
                case "$arg" in
                    -f|--force) force=1 ;;
                    -n|--dry-run) force=0 ;;
                    -h|--help)
                        cat <<'__RC_CLEANUP_HELP__'
usage: racecar cleanup [--dry-run | --force]
  Lists racecar processes and FastRTPS SHM orphans. Default is --dry-run.
  --force kills processes (uses sudo for root-owned ones) and removes SHM.
__RC_CLEANUP_HELP__
                        return 0
                        ;;
                    *) echo "racecar cleanup: unknown flag '$arg'" >&2; return 2 ;;
                esac
            done

            # ----- Process inventory -----
            # Match any process whose cmdline mentions the racecar stack.
            local pattern='racecar_neo_ros2_driver|realsense2_camera_node|sllidar_node|ros2 launch racecar|sg dialout.*racecar'
            local matches
            matches=$(ps -eo pid,user,cmd --no-headers | grep -E "$pattern" | grep -v 'grep\|racecar cleanup' || true)

            if [[ -z "$matches" ]]; then
                echo "No racecar processes running."
            else
                echo "=== Racecar processes ==="
                echo "$matches" | awk '{printf "  pid=%-6s user=%-8s cmd=%s\n", $1, $2, substr($0, index($0,$3))}' | head -30
                local user_pids root_pids
                user_pids=$(echo "$matches" | awk -v u="$USER" '$2 == u {print $1}' | tr '\n' ' ')
                root_pids=$(echo "$matches" | awk '$2 == "root" {print $1}' | tr '\n' ' ')
                if [[ $force -eq 1 ]]; then
                    if [[ -n "$user_pids" ]]; then
                        echo "Killing user-owned: $user_pids"
                        # shellcheck disable=SC2086
                        kill -9 $user_pids 2>/dev/null || true
                    fi
                    if [[ -n "$root_pids" ]]; then
                        echo "Killing root-owned (sudo): $root_pids"
                        # shellcheck disable=SC2086
                        sudo kill -9 $root_pids 2>/dev/null || \
                            echo "  (sudo failed; run as your user: sudo kill -9 $root_pids)"
                    fi
                else
                    echo "(dry-run; pass --force to kill)"
                fi
            fi

            # ----- FastRTPS SHM orphans -----
            local shm_orphans=()
            local shm_locks=()
            for f in /dev/shm/fastrtps_port*; do
                [ -e "$f" ] || continue
                case "$f" in *_el) continue ;; esac
                # Orphan = zero-byte data segment with no live participant.
                if [ ! -s "$f" ]; then
                    shm_orphans+=("$f")
                fi
            done
            for el in /dev/shm/fastrtps_port*_el; do
                [ -e "$el" ] || continue
                local data="${el%_el}"
                # Orphan = lock segment whose data peer is gone.
                if [ ! -e "$data" ]; then
                    shm_locks+=("$el")
                fi
            done

            echo
            if [[ ${#shm_orphans[@]} -eq 0 && ${#shm_locks[@]} -eq 0 ]]; then
                echo "No FastRTPS SHM orphans in /dev/shm."
            else
                echo "=== FastRTPS SHM orphans ==="
                for f in "${shm_orphans[@]}"; do
                    echo "  zero-byte: $f"
                done
                for el in "${shm_locks[@]}"; do
                    echo "  stale lock: $el"
                done
                if [[ $force -eq 1 ]]; then
                    for f in "${shm_orphans[@]}"; do
                        local base
                        base=$(basename "$f")
                        rm -f "$f" "/dev/shm/${base}_el" "/dev/shm/sem.${base}_mutex"
                    done
                    for el in "${shm_locks[@]}"; do
                        local base
                        base=$(basename "${el%_el}")
                        rm -f "$el" "/dev/shm/sem.${base}_mutex"
                    done
                    echo "Removed."
                else
                    echo "(dry-run; pass --force to remove)"
                fi
            fi
            ;;

        status)
            echo "=== USB peripherals ==="
            lsusb | grep -iE "silicon labs|intel|global unichip|google" || echo "  (none of the expected USB devices found)"
            echo
            echo "=== Stable device symlinks ==="
            for s in neo-pit-pcb lidar; do
                if [[ -e "/dev/$s" ]]; then
                    printf "  /dev/%-14s -> %s\n" "$s" "$(readlink -f /dev/$s)"
                else
                    printf "  /dev/%-14s MISSING (run: racecar udev)\n" "$s"
                fi
            done
            echo
            echo "=== ros2 nodes running ==="
            if command -v ros2 >/dev/null; then
                ros2 node list 2>/dev/null || echo "  (no ROS daemon / no nodes)"
            else
                echo "  ros2 not on PATH"
            fi
            ;;

        help|-h|--help|"")
            cat <<'__RC_HELP__'
racecar — RACECAR Neo developer tool

Usage:
    racecar <command> [args]

Commands:
    build               Build racecar_neo_ros2_driver (--symlink-install) and source overlay.
    test                Run the package test suite with verbose results.
    source              Source the workspace overlay into the current shell.
    cd                  Change directory to the racecar_neo_ros2_driver package root.
    teleop              Launch the full teleop stack via launch_teleop.sh wrapper
                        (timestamped ~/logs/<ts>/ + FastRTPS SHM cleanup).
                        Forwards args, e.g. `racecar teleop edgetpu_enable:=false`.
    launch <name>       Shortcut for `ros2 launch racecar_neo_ros2_driver <name>.launch.py`.
                        Examples: racecar launch dotmatrix
                                  racecar launch realsense
                                  racecar launch edgetpu
    clear --dmatrix     Flash + clear the MAX7219 dot matrix display.
    udev                Re-install the udev rules (refreshes /dev/neo-pit-pcb etc.).
    watchdog            Run the node watchdog (restart-on-failure supervisor).
                        Monitors control + sensor nodes; logs to
                        ~/logs/latest/watchdog.log. Assumes teleop runs separately.
    eth [action]        eth0 addressing mode; static or DHCP, never both.
                          status    configured mode, live addresses, routes,
                                    and conflict checks (default)
                          static    fixed 192.168.52.200/24, no gateway and no
                                    IPv6 default route; the shipped default
                          dynamic   address + default route from DHCP
                          monitor   log addressing + link state to confirm the
                                    static stops dropping
                        Flags: --addr=CIDR  --force
                        Switching modes drops an SSH session on eth0; run it
                        from the AP, wlan0, or an HDMI console.
    wifi [action]       WiFi client on wlan0 (the AP on wlan1 is never touched).
                          status              link, addresses, DNS (default)
                          list [--rescan]     visible networks, one row per SSID
                          connect <ssid>      join; prompts for what it needs
                          disconnect          drop the wlan0 link
                        Enterprise networks ask only for an identity and
                        password; server validation is always on.
    setup <phase>       Run a setup script. Phases:
                          all          — setup_all.sh (the 11-phase orchestrator)
                          networking   — eth0 addressing + ALFA-dongle isolated AP.
                                         Prompts for this car's ID (SSID
                                         racecar-neo-<id>) when none is set.
                                         Flags persist to ~/.config/racecar/networking.env:
                                           --ssid=NAME   --psk=PASS   --channel=N
                                           --ap-addr=CIDR (default 10.42.0.1/24)
                                           --ap-iface=NAME (default wlan1, the ALFA dongle)
                                           --eth-static=CIDR (default 192.168.52.200/24)
                                           --eth-mode=static|dynamic
                                           --show
                                           --reset (disable the wlan1 AP + clear saved ID)
                          realsense    — flash D435i firmware from the locally-staged
                                         image (offline; for airgapped units). Reads
                                         /opt/racecar/firmware; idempotent. Flags:
                                           --check (report only)  --force
                                           --version X.Y.Z.W  --serial SN  --fw-dir DIR
    service <action>    systemd service control. Actions:
                          install              setup_services.sh (drop + enable units)
                          start [name]         default: teleop (watchdog follows)
                          stop [name]          default: teleop
                          restart [name]       default: teleop
                          enable|disable       all units
                          logs [name]          journalctl -f for racecar-<name>
                          status               active/enabled summary (default)
                        Units: teleop, watchdog, dashboard, jupyter
    library <action>    Manage racecar_student.pth in user site-packages.
                          --select <folder>   point at ~/jupyter_ws/<folder>/library
                          --list              show valid folders in ~/jupyter_ws
                          --reset             delete the .pth file
                          --status            show current selection
    cleanup             List orphaned racecar processes + FastRTPS SHM segments.
                        Defaults to a dry-run. Pass --force to actually kill/remove
                        (uses sudo for root-owned PIDs).
    status              Show USB peripherals, device symlinks, and running ros2 nodes.
    help                Show this message.

Extra args are forwarded:
    racecar build --cmake-args -DCMAKE_BUILD_TYPE=Release
    racecar launch dotmatrix dotmatrix_config:=/tmp/custom.yaml
__RC_HELP__
            ;;

        *)
            echo "racecar: unknown command '$cmd'. Try 'racecar help'." >&2
            return 2
            ;;
    esac
}

# Bash completion: subcommands at position 1, launch-file names after `launch`,
# `--dmatrix` after `clear`.
_racecar_complete() {
    local cur="${COMP_WORDS[COMP_CWORD]}"
    local prev="${COMP_WORDS[COMP_CWORD-1]}"
    local sub="${COMP_WORDS[1]:-}"

    if [[ $COMP_CWORD -eq 1 ]]; then
        COMPREPLY=( $(compgen -W "build test source cd teleop launch clear udev watchdog service setup eth wifi library cleanup status help" -- "$cur") )
        return
    fi

    case "$sub" in
        launch)
            local launch_dir="$HOME/ros2_ws/src/racecar_neo_ros2_driver/launch"
            if [[ -d "$launch_dir" ]]; then
                local names
                names=$(cd "$launch_dir" && ls *.launch.py 2>/dev/null | sed 's/\.launch\.py$//')
                COMPREPLY=( $(compgen -W "$names" -- "$cur") )
            fi
            ;;
        clear)
            COMPREPLY=( $(compgen -W "--dmatrix" -- "$cur") )
            ;;
        cleanup)
            COMPREPLY=( $(compgen -W "--dry-run --force --help" -- "$cur") )
            ;;
        library)
            if [[ "$cur" == --select=* || "$prev" == "--select" ]]; then
                # Complete with folder names under ~/jupyter_ws that contain library/racecar_core.py.
                local jws="$HOME/jupyter_ws"
                local candidates=""
                if [[ -d "$jws" ]]; then
                    local d
                    for d in "$jws"/*/; do
                        [[ -d "$d" ]] || continue
                        if [[ -f "${d}library/racecar_core.py" ]]; then
                            candidates+="$(basename "$d") "
                        fi
                    done
                fi
                if [[ "$cur" == --select=* ]]; then
                    local prefix="${cur#--select=}"
                    COMPREPLY=( $(compgen -W "$candidates" -- "$prefix") )
                else
                    COMPREPLY=( $(compgen -W "$candidates" -- "$cur") )
                fi
            else
                COMPREPLY=( $(compgen -W "--select --select= --list --reset --status --help" -- "$cur") )
            fi
            ;;
        wifi)
            if [[ $COMP_CWORD -eq 2 ]]; then
                COMPREPLY=( $(compgen -W "status list connect disconnect help" -- "$cur") )
            elif [[ "${COMP_WORDS[2]}" == "connect" && $COMP_CWORD -eq 3 ]]; then
                # Saved client profiles only; the AP profile is bound to wlan1
                # and must never be offered here.
                local ssids
                ssids=$(nmcli -t -f NAME,TYPE con show 2>/dev/null |
                    awk -F: '$2 == "802-11-wireless" { print $1 }')
                COMPREPLY=( $(compgen -W "$ssids" -- "$cur") )
            elif [[ "${COMP_WORDS[2]}" == "connect" ]]; then
                COMPREPLY=( $(compgen -W "--identity= --psk= --ca-cert= --domain-suffix-match=" -- "$cur") )
            elif [[ "${COMP_WORDS[2]}" == "list" ]]; then
                COMPREPLY=( $(compgen -W "--rescan" -- "$cur") )
            fi
            ;;
        eth)
            if [[ $COMP_CWORD -eq 2 ]]; then
                COMPREPLY=( $(compgen -W "status static dynamic monitor help" -- "$cur") )
            else
                COMPREPLY=( $(compgen -W "--addr= --force --help" -- "$cur") )
            fi
            ;;
        setup)
            if [[ $COMP_CWORD -eq 2 ]]; then
                COMPREPLY=( $(compgen -W "all networking realsense" -- "$cur") )
            elif [[ "${COMP_WORDS[2]}" == "networking" ]]; then
                COMPREPLY=( $(compgen -W "--ssid= --psk= --channel= --ap-addr= --eth-static= --eth-mode= --show --reset --help" -- "$cur") )
            elif [[ "${COMP_WORDS[2]}" == "realsense" ]]; then
                COMPREPLY=( $(compgen -W "--check --force --version --serial --fw-dir --help" -- "$cur") )
            fi
            ;;
        service)
            if [[ $COMP_CWORD -eq 2 ]]; then
                COMPREPLY=( $(compgen -W "install start stop restart enable disable logs status help" -- "$cur") )
            elif [[ $COMP_CWORD -eq 3 ]]; then
                local action="${COMP_WORDS[2]}"
                case "$action" in
                    start|stop|restart|logs)
                        COMPREPLY=( $(compgen -W "teleop watchdog dashboard jupyter" -- "$cur") )
                        ;;
                esac
            fi
            ;;
    esac
}
complete -F _racecar_complete racecar
