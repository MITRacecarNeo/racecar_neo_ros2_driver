# Helpers shared by the setup scripts. Source it after SCRIPT_DIR is set:
#   source "$SCRIPT_DIR/setup_lib.sh"
# A script may set SUDO first (empty in tests); it defaults to sudo.

# The user setup is for: the caller of sudo, else the current user.
TARGET_USER="${SUDO_USER:-$USER}"
TARGET_HOME="$(getent passwd "$TARGET_USER" | cut -d: -f6)"

# install_if_changed SRC DST MODE [OWNER]
# Install SRC as DST with MODE (and OWNER:OWNER) unless DST already matches.
# Sets INSTALLED=1 when it wrote DST, else 0. Returns 1 only when the install
# fails, so `set -e` callers stop and `if !` callers can report it.
install_if_changed() {
    local src="$1" dst="$2" mode="$3" owner="${4:-}"
    local sudo="${SUDO-sudo}"
    local -a own=()
    [[ -n "$owner" ]] && own=(-o "$owner" -g "$owner")
    INSTALLED=0
    if $sudo cmp -s "$src" "$dst" 2>/dev/null; then
        echo "  $dst: already up to date"
        return 0
    fi
    if $sudo install -m "$mode" "${own[@]}" "$src" "$dst"; then
        echo "  $dst: installed"
        INSTALLED=1
        return 0
    fi
    echo "  $dst: install failed (sudo?)" >&2
    return 1
}
