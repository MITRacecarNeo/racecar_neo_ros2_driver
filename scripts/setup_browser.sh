#!/bin/bash
# The Browser app: Firefox in a virtual screen, shown in a page (scripts/browser_app.py).
#
# Installs the virtual screen (Xvfb), the screen-sharing server (x11vnc), the
# noVNC page core, and Firefox from Mozilla's APT repository; writes Firefox's policy file
# and the racecar-browser unit. Re-run after a driver upgrade; it only changes
# what differs. The unit stays stopped: the 8080 Browser tile starts it.
set -eo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "$SCRIPT_DIR/setup_lib.sh"

PACKAGES=(xvfb x11vnc novnc websockify)
missing=()
for pkg in "${PACKAGES[@]}"; do
    dpkg -s "$pkg" >/dev/null 2>&1 || missing+=("$pkg")
done
if [ ${#missing[@]} -gt 0 ]; then
    sudo apt-get install -y "${missing[@]}"
else
    echo "  ${PACKAGES[*]}: already installed"
fi

# Firefox from Mozilla's APT repository. The snap cannot open a display in a
# service's virtual screen, and Ubuntu's own `firefox` package only installs
# the snap, so Mozilla's origin is pinned above it. The snap, if present, stays.
MOZILLA_KEY=/etc/apt/keyrings/packages.mozilla.org.asc
MOZILLA_FPR=35BAA0B33E9EB396F59CA838C0BA5CE6DC6315A3
if [ ! -f "$MOZILLA_KEY" ]; then
    key_tmp="$(mktemp)"
    curl -fsSL https://packages.mozilla.org/apt/repo-signing-key.gpg -o "$key_tmp"
    # Read gpg's whole output first: with pipefail, `grep -q` closing the pipe
    # early would fail the check even on a match.
    key_info="$(gpg -n -q --import --import-options import-show "$key_tmp" 2>/dev/null || true)"
    if [[ "$key_info" != *"$MOZILLA_FPR"* ]]; then
        echo "  Mozilla's signing key does not match $MOZILLA_FPR; not installing it." >&2
        rm -f "$key_tmp"
        exit 1
    fi
    sudo install -d -m 0755 /etc/apt/keyrings
    sudo install -m 0644 "$key_tmp" "$MOZILLA_KEY"
    rm -f "$key_tmp"
fi
list_tmp="$(mktemp)"
pin_tmp="$(mktemp)"
trap 'rm -f "$list_tmp" "$pin_tmp"' EXIT
echo "deb [signed-by=$MOZILLA_KEY] https://packages.mozilla.org/apt mozilla main" > "$list_tmp"
printf 'Package: *\nPin: origin packages.mozilla.org\nPin-Priority: 1000\n' > "$pin_tmp"
install_if_changed "$list_tmp" /etc/apt/sources.list.d/mozilla.list 0644
list_changed=$INSTALLED
install_if_changed "$pin_tmp" /etc/apt/preferences.d/mozilla 0644
if [[ $list_changed -eq 1 || $INSTALLED -eq 1 ]]; then
    sudo apt-get update
fi
if [ -x /usr/lib/firefox/firefox ]; then
    echo "  firefox (Mozilla): already installed"
else
    sudo apt-get install -y --allow-downgrades firefox
fi

# Firefox reads enterprise policies from here, the snap included.
sudo mkdir -p /etc/firefox/policies
install_if_changed "$SCRIPT_DIR/firefox-policies.json" /etc/firefox/policies/policies.json 0644

install_if_changed "$SCRIPT_DIR/racecar-browser.service" /etc/systemd/system/racecar-browser.service 0644
if [[ $INSTALLED -eq 1 ]]; then
    sudo systemctl daemon-reload
    echo "  systemctl daemon-reload"
fi
echo "Browser installed. Open it from the Browser tile on the 8080 home page."
