#!/bin/bash
# The car's web server: every app at http://<car>/<path>/ on port 80.
#
# Installs nginx and writes one site from config/apps.yaml
# (racecar_neo_ros2_driver/apps.py nginx): each app's path forwards to its
# port, with its WebSocket and its client address. The apps' own ports keep
# working. Re-run after an app is added; it only rewrites a changed site.
set -eo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "$SCRIPT_DIR/setup_lib.sh"
PKG_DIR="$(dirname "$SCRIPT_DIR")"

SITE=/etc/nginx/sites-available/racecar
ENABLED=/etc/nginx/sites-enabled/racecar

if ! command -v nginx >/dev/null; then
    sudo apt-get install -y nginx-light
fi

TMP_SITE="$(mktemp)"
trap 'rm -f "$TMP_SITE"' EXIT
python3 "$PKG_DIR/racecar_neo_ros2_driver/apps.py" nginx > "$TMP_SITE"
install_if_changed "$TMP_SITE" "$SITE" 0644
sudo ln -sf "$SITE" "$ENABLED"
# The stock site also listens on port 80 as the default server.
sudo rm -f /etc/nginx/sites-enabled/default

sudo nginx -t
sudo systemctl enable nginx >/dev/null 2>&1
sudo systemctl reload-or-restart nginx

# The home page by the car's name and by each of its IPv4 addresses, no port.
# 502 means nginx answers but the 8080 dashboard is not running yet.
home_code() {
    curl -s -o /dev/null -L --max-time 3 -w '%{http_code}' "$@" || true
}
failed=0
check_home() {
    local label="$1" code
    shift
    code="$(home_code "$@")"
    if [ "$code" = "200" ]; then
        printf '  %-28s OK\n' "$label"
    else
        printf '  %-28s HTTP %s\n' "$label" "${code:-none}"
        failed=1
    fi
}
echo "Home page:"
check_home "http://$(hostname).local/" -H "Host: $(hostname).local" http://127.0.0.1/
for ip in $(hostname -I 2>/dev/null); do
    case "$ip" in *:*) continue ;; esac
    check_home "http://$ip/" "http://$ip/"
done
if [ "$failed" = 1 ] && ! systemctl is-active --quiet racecar-dashboard; then
    echo "  The dashboard is not running; the home page opens once it starts."
fi
