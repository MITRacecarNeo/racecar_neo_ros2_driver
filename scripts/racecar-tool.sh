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
            ros2 launch "$pkg" teleop.launch.py "$@"
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

        selftest)
            local target=""
            local pattern="all"
            for arg in "$@"; do
                case "$arg" in
                    --dmatrix|--dotmatrix) target="dmatrix" ;;
                    --dmatrix=*|--dotmatrix=*) target="dmatrix"; pattern="${arg#*=}" ;;
                    *) echo "racecar selftest: unknown flag '$arg'" >&2; return 2 ;;
                esac
            done
            case "$target" in
                dmatrix)
                    # Faster than `ros2 node list` (which hangs ~15s when no
                    # daemon is running). Look for the installed entry-point.
                    if ! pgrep -f 'racecar_neo_ros2_driver/lib/.*dotmatrix_node' >/dev/null; then
                        echo "racecar selftest: dotmatrix_node is not running." >&2
                        echo "Start it first: racecar launch dotmatrix" >&2
                        return 3
                    fi
                    python3 "$pkg_dir/scripts/dmatrix_patterns.py" "$pattern"
                    ;;
                "")
                    cat <<'__RC_SELFTEST_HELP__' >&2
usage: racecar selftest --dmatrix[=<pattern>]
patterns: all (default), checkerboard, all-on, sweep, module-id, font
__RC_SELFTEST_HELP__
                    return 2
                    ;;
            esac
            ;;

        status)
            echo "=== USB peripherals ==="
            lsusb | grep -iE "pololu|silicon labs|logitech|microdia|arducam|global unichip|google" || echo "  (none of the expected USB devices found)"
            echo
            echo "=== Stable device symlinks ==="
            for s in maestro lidar cam_forward cam_backward; do
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
    teleop              Launch the full teleop stack (gamepad + mux + throttle + pwm).
    launch <name>       Shortcut for `ros2 launch racecar_neo_ros2_driver <name>.launch.py`.
                        Examples: racecar launch dotmatrix
                                  racecar launch camera_forward
                                  racecar launch edgetpu
    clear --dmatrix     Flash + clear the MAX7219 dot matrix display.
    udev                Re-install the udev rules (refreshes /dev/maestro etc.).
    selftest            Hardware self-tests. Currently supported:
                          racecar selftest --dmatrix             (runs all patterns)
                          racecar selftest --dmatrix=checkerboard
                          racecar selftest --dmatrix=all-on
                          racecar selftest --dmatrix=sweep
                          racecar selftest --dmatrix=module-id
                          racecar selftest --dmatrix=font
                        Requires dotmatrix_node to be running (racecar launch dotmatrix).
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
        COMPREPLY=( $(compgen -W "build test source teleop launch clear udev selftest status help" -- "$cur") )
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
        selftest)
            COMPREPLY=( $(compgen -W "--dmatrix --dmatrix=checkerboard --dmatrix=all-on --dmatrix=sweep --dmatrix=module-id --dmatrix=font" -- "$cur") )
            ;;
    esac
}
complete -F _racecar_complete racecar
