"""
Network settings for the 8080 Connections page (scripts/connections.html).

Reads come from `nmcli`. Secrets (a new network's password, the access point
password) go to NetworkManager over D-Bus, so they never appear on a command
line. The racecar account holds NetworkManager's network-control and
settings.modify.system permissions (scripts/setup_user_env.sh).

Only the client radio (wlan0) is joined, forgotten or switched. The access
point (wlan1, connection `racecar-neo-ap`) takes a new password only; eth0
is shown only.
"""

from __future__ import annotations

from collections.abc import Callable
import ipaddress
import json
import logging
from pathlib import Path
import re
import subprocess
import sys
import threading
import time
from typing import Any

sys.path.insert(0, str(Path(__file__).resolve().parent))

from wifi_scan import parse as parse_scan, split_nmcli  # noqa: E402  sibling script

log = logging.getLogger(__name__)

WIFI = 'wlan0'
AP = 'wlan1'
ETH = 'eth0'
AP_CONNECTION = 'racecar-neo-ap'
JOIN_TIMEOUT_SEC = 45.0
APPLY_DELAY_SEC = 1.0
INTERNET = {'full': 'online', 'limited': 'limited', 'portal': 'portal', 'none': 'none'}

Run = Callable[..., Any]


def nmcli(args: list[str], run: Run = subprocess.run, timeout: float = 15) -> str:
    """Return nmcli's output, or '' when it fails."""
    try:
        r = run(['nmcli', *args], capture_output=True, text=True, timeout=timeout)
    except (OSError, subprocess.TimeoutExpired):
        return ''
    return r.stdout if r.returncode == 0 else ''


def nmcli_ok(args: list[str], run: Run = subprocess.run, timeout: float = 30) -> bool:
    """Return True when nmcli exits 0."""
    try:
        return (
            run(['nmcli', *args], capture_output=True, text=True, timeout=timeout).returncode == 0
        )
    except (OSError, subprocess.TimeoutExpired):
        return False


def _get(fields: str, kind: str, name: str, run: Run) -> list[str]:
    """Return `nmcli -g` values for a device or connection, one per field."""
    out = nmcli(['-g', fields, kind, 'show', name], run).splitlines()
    return out + [''] * (fields.count(',') + 1 - len(out))


def device_states(run: Run = subprocess.run) -> dict[str, tuple[str, str]]:
    """Return {device: (state, connection)} from `nmcli device`."""
    out = {}
    for line in nmcli(['-t', '-f', 'DEVICE,STATE,CONNECTION', 'device'], run).splitlines():
        parts = split_nmcli(line) + ['', '']
        out[parts[0]] = (parts[1], parts[2])
    return out


def first_address(value: str) -> str:
    """Return the first address of an `IP4.ADDRESS` value (`a/24 | b/24`)."""
    return value.split('|')[0].strip()


def internet(value: str) -> str:
    """Map `GENERAL.IP4-CONNECTIVITY` (`4 (full)`) to online, limited, portal or none."""
    word = value.partition('(')[2].rstrip(')').strip()
    return INTERNET.get(word, 'none')


def security_kind(security: str) -> str:
    """Return open, psk, sae or enterprise for a scan's SECURITY field."""
    words = security.replace('--', '').split()
    if not words or security == 'open':
        return 'open'
    if '802.1X' in words:
        return 'enterprise'
    if all(w == 'WPA3' for w in words):
        return 'sae'
    return 'psk'


def saved_networks(run: Run = subprocess.run) -> list[dict[str, Any]]:
    """Return the client WiFi profiles, highest autoconnect priority first."""
    rows = []
    listing = nmcli(['-t', '-f', 'NAME,UUID,TYPE,AUTOCONNECT-PRIORITY', 'connection', 'show'], run)
    for line in listing.splitlines():
        parts = split_nmcli(line) + ['', '', '', '']
        name, uuid, kind, priority = parts[:4]
        if kind != '802-11-wireless':
            continue
        mode, ssid, key_mgmt = _get(
            '802-11-wireless.mode,802-11-wireless.ssid,802-11-wireless-security.key-mgmt',
            'connection',
            uuid,
            run,
        )
        if mode == 'ap' or name == AP_CONNECTION:
            continue
        try:
            rank = int(priority)
        except ValueError:
            rank = 0
        rows.append(
            {
                'uuid': uuid,
                'ssid': ssid or name,
                'priority': rank,
                'enterprise': key_mgmt == 'wpa-eap',
            }
        )
    return sorted(rows, key=lambda r: (-r['priority'], r['ssid'].lower()))


def visible_networks(
    saved: list[dict[str, Any]], rescan: bool = False, run: Run = subprocess.run
) -> list[dict[str, Any]]:
    """Return the networks wlan0 sees, strongest first, marked when saved."""
    out = nmcli(
        [
            '-t',
            '-f',
            'SSID,SIGNAL,SECURITY,IN-USE',
            'device',
            'wifi',
            'list',
            'ifname',
            WIFI,
            '--rescan',
            'yes' if rescan else 'no',
        ],
        run,
        timeout=30,
    )
    names = {s['ssid'] for s in saved}
    networks, _ = parse_scan(out)
    return [
        {
            'ssid': n['ssid'],
            'signal': n['signal'],
            'security': security_kind(n['security']),
            'in_use': n['in_use'],
            'saved': n['ssid'] in names,
        }
        for n in networks
    ]


def _out(argv: list[str], run: Run, timeout: float = 5) -> str:
    try:
        r = run(argv, capture_output=True, text=True, timeout=timeout)
    except (OSError, subprocess.TimeoutExpired):
        return ''
    return r.stdout if r.returncode == 0 else ''


def stations(run: Run = subprocess.run) -> list[dict[str, Any]]:
    """Return the devices joined to the access point: MAC, signal (dBm), seconds connected."""
    rows: list[dict[str, Any]] = []
    for line in _out(['iw', 'dev', AP, 'station', 'dump'], run).splitlines():
        words = line.split()
        if line.startswith('Station ') and len(words) > 1:
            rows.append({'mac': words[1].lower(), 'signal': None, 'seconds': None})
        elif rows and words[:1] == ['signal:'] and len(words) > 1:
            rows[-1]['signal'] = _int(words[1])
        elif rows and words[:2] == ['connected', 'time:'] and len(words) > 2:
            rows[-1]['seconds'] = _int(words[2])
    return rows


def _int(text: str) -> int | None:
    try:
        return int(text)
    except ValueError:
        return None


def neighbors(device: str, run: Run = subprocess.run) -> dict[str, str]:
    """Return {MAC: IPv4 address} of the devices the car has heard from on `device`."""
    out = {}
    for line in _out(['ip', '-4', 'neigh', 'show', 'dev', device], run).splitlines():
        words = line.split()
        if 'lladdr' in words and words[-1] not in ('FAILED', 'INCOMPLETE'):
            out[words[words.index('lladdr') + 1].lower()] = words[0]
    return out


def dhcp_names(run: Run = subprocess.run) -> dict[str, str]:
    """Return {MAC: host name} from the access point's DHCP log since boot; many send none."""
    names = {}
    log_lines = _out(
        ['journalctl', '-u', 'NetworkManager', '-b', '-o', 'cat', '--no-pager',
         '-g', rf'DHCPACK\({AP}\)'],
        run,
    )  # fmt: skip
    for line in log_lines.splitlines():
        words = line.split()
        if len(words) >= 4 and words[0] == f'DHCPACK({AP})':
            names[words[2].lower()] = words[3]
    return names


MAC = re.compile(r'[0-9a-f]{2}(:[0-9a-f]{2}){5}')
NAME_MAX = 32


def names_path() -> Path:
    """Return the car profile file of device nicknames, keyed by MAC."""
    from racecar_neo_ros2_driver import car_profile

    return car_profile.profile_dir() / 'devices.yaml'


def read_names(path: Path | None = None) -> dict[str, str]:
    from racecar_neo_ros2_driver import car_profile

    data = car_profile.read_yaml(path or names_path()).get('names')
    if not isinstance(data, dict):
        return {}
    return {str(k).lower(): str(v) for k, v in data.items()}


def clean_name(name: str) -> str:
    """Return `name` trimmed, without control characters, at most NAME_MAX long."""
    return ''.join(c for c in name if c.isprintable()).strip()[:NAME_MAX]


def save_names(changes: dict[str, str], path: Path | None = None) -> dict[str, str]:
    """Apply nickname changes (an empty name removes one) and return the whole table."""
    from racecar_neo_ros2_driver import car_profile

    path = path or names_path()
    names = read_names(path)
    for mac, name in changes.items():
        mac = mac.lower()
        if not MAC.fullmatch(mac):
            raise ValueError(mac)
        if clean_name(name):
            names[mac] = clean_name(name)
        else:
            names.pop(mac, None)
    lines = ['# Device nicknames from the Connections page, by MAC address.']
    lines.append('names:' if names else 'names: {}')
    lines += [f'  "{mac}": {json.dumps(name)}' for mac, name in sorted(names.items())]
    car_profile.write_text(path, '\n'.join(lines) + '\n')
    return names


def devices(
    run: Run = subprocess.run, nicknames: dict[str, str] | None = None
) -> list[dict[str, Any]]:
    """Return every device connected to the car: on the access point, then on eth0."""
    ap_ips, names = neighbors(AP, run), dhcp_names(run)
    nicknames = read_names() if nicknames is None else nicknames
    rows = [
        {'name': names.get(s['mac'], ''), 'address': ap_ips.get(s['mac'], ''), 'mac': s['mac'],
         'device': AP, 'signal': s['signal'], 'seconds': s['seconds']}
        for s in stations(run)
    ]  # fmt: skip
    rows += [
        {'name': '', 'address': ip, 'mac': mac, 'device': ETH, 'signal': None, 'seconds': None}
        for mac, ip in neighbors(ETH, run).items()
    ]
    for row in rows:
        row['nickname'] = nicknames.get(row['mac'], '')
    return rows


def via_device(client_ip: str, addresses: dict[str, str], run: Run = subprocess.run) -> str:
    """
    Return the device the page's browser reaches the car through, or '' (the car itself).

    An IPv4 address is matched to a device's network. Any other address (a
    browser on IPv6 link-local through the web server) is looked up in the
    neighbor table.
    """
    try:
        ip = ipaddress.ip_address(client_ip)
    except ValueError:
        return ''
    if ip.is_loopback:
        return ''
    for device, address in addresses.items():
        try:
            if address and ip in ipaddress.ip_interface(address).network:
                return device
        except ValueError:
            continue
    words = _out(['ip', 'neigh', 'show', 'to', str(ip)], run).split()
    return words[words.index('dev') + 1] if 'dev' in words[:-1] else ''


def ping_test(host: str = 'google.com', run: Run = subprocess.run) -> dict[str, Any]:
    """Ping `host` four times; return reached, sent, received, average ms, and the device used."""
    out = _out(['ping', '-c', '4', '-W', '2', '-q', host], run, timeout=20)
    if not out:
        try:
            r = run(['ping', '-c', '1', '-W', '2', '-q', host], capture_output=True, text=True,
                    timeout=10)  # fmt: skip
            text = r.stdout + r.stderr
        except (OSError, subprocess.TimeoutExpired):
            text = ''
        if 'name resolution' in text or 'Name or service not known' in text:
            return {'host': host, 'result': 'no-name', 'sent': 0, 'received': 0, 'avg_ms': None,
                    'device': ''}  # fmt: skip
        out = text
    sent = received = 0
    avg = None
    target = ''
    for line in out.splitlines():
        words = line.replace(',', '').split()
        if line.startswith('PING ') and len(words) > 2:
            target = words[2].strip('()')
        if 'transmitted' in words:
            sent, received = _int(words[0]) or 0, _int(words[3]) or 0
        if line.startswith(('rtt ', 'round-trip ')) and '=' in line:
            parts = line.split('=')[1].split('/')
            avg = round(float(parts[1]), 1) if len(parts) > 1 else None
    route = _out(['ip', '-o', 'route', 'get', target], run).split() if target else []
    return {
        'host': host,
        'result': 'reached' if received else 'unreached',
        'sent': sent,
        'received': received,
        'avg_ms': avg,
        'device': route[route.index('dev') + 1] if 'dev' in route[:-1] else '',
    }


def view(client_ip: str, rescan: bool = False, run: Run = subprocess.run) -> dict[str, Any]:
    """Return everything the Connections page shows."""
    states = device_states(run)
    wifi_state, wifi_con = states.get(WIFI, ('unavailable', ''))
    wifi_addr, wifi_net, wifi_auto = _get(
        'IP4.ADDRESS,GENERAL.IP4-CONNECTIVITY,GENERAL.AUTOCONNECT', 'device', WIFI, run
    )
    saved = saved_networks(run)
    visible = visible_networks(saved, rescan, run)
    current = next((n for n in visible if n['in_use']), None)
    ap_state, _ = states.get(AP, ('unavailable', ''))
    ap_ssid, ap_channel, ap_addr = _get(
        '802-11-wireless.ssid,802-11-wireless.channel,ipv4.addresses',
        'connection',
        AP_CONNECTION,
        run,
    )
    eth_state, _ = states.get(ETH, ('unavailable', ''))
    (eth_addr,) = _get('IP4.ADDRESS', 'device', ETH, run)
    addresses = {
        WIFI: first_address(wifi_addr),
        AP: first_address(ap_addr),
        ETH: first_address(eth_addr),
    }
    connected = wifi_state.startswith('connected')
    return {
        'wifi': {
            'device': WIFI,
            # `nmcli device disconnect` turns the device's autoconnect off;
            # an idle radio reads the same state.
            'on': connected
            or (wifi_state not in ('unavailable', 'unmanaged') and wifi_auto == 'yes'),
            'connected': connected,
            'network': (current or {}).get('ssid') or (wifi_con if connected else ''),
            'signal': (current or {}).get('signal'),
            'address': addresses[WIFI].split('/')[0],
            'internet': internet(wifi_net) if connected else 'none',
            'saved': saved,
            # The car's own access point is not a network to join.
            'visible': [n for n in visible if n['ssid'] != ap_ssid],
        },
        'ap': {
            'device': AP,
            'on': ap_state.startswith('connected'),
            'ssid': ap_ssid,
            'channel': ap_channel,
            'address': addresses[AP].split('/')[0],
        },
        'eth': {
            'device': ETH,
            'state': (
                'connected'
                if eth_state.startswith('connected')
                else ('no cable' if eth_state == 'unavailable' else 'not connected')
            ),
            'address': addresses[ETH].split('/')[0],
        },
        'via': via_device(client_ip, addresses, run),
        'devices': devices(run),
    }


def new_profile(
    ssid: str, security: str, password: str, identity: str, priority: int
) -> dict[str, dict[str, Any]]:
    """
    Return NetworkManager settings for a new client network on wlan0.

    `ssid` stays a str here; the D-Bus layer sends it as bytes. Enterprise
    follows `racecar wifi connect`: TTLS with MSCHAPv2, the system CA
    store, and the server's domain taken from the account (`user@domain`).
    """
    profile: dict[str, dict[str, Any]] = {
        'connection': {
            'id': ssid,
            'type': '802-11-wireless',
            'interface-name': WIFI,
            'autoconnect': True,
            'autoconnect-priority': priority,
        },
        '802-11-wireless': {'ssid': ssid, 'mode': 'infrastructure'},
        'ipv4': {'method': 'auto'},
        'ipv6': {'method': 'auto'},
    }
    if security in ('psk', 'sae'):
        profile['802-11-wireless-security'] = {
            'key-mgmt': 'wpa-psk' if security == 'psk' else 'sae',
            'psk': password,
        }
    elif security == 'enterprise':
        domain = identity.partition('@')[2]
        profile['802-11-wireless-security'] = {'key-mgmt': 'wpa-eap'}
        profile['802-1x'] = {
            'eap': ['ttls'],
            'phase2-auth': 'mschapv2',
            'identity': identity,
            'password': password,
            'domain-suffix-match': domain,
            'system-ca-certs': True,
        }
    return profile


def check_join(security: str, password: str, identity: str, saved: bool) -> str:
    """Return what is missing for a join, or ''."""
    if saved or security == 'open':
        return ''
    if security == 'enterprise':
        if '@' not in identity or not identity.partition('@')[2]:
            return 'Enter the account as name@domain.'
        return '' if password else 'Enter the password.'
    if not 8 <= len(password) <= 63:
        return 'Use 8 to 63 characters.'
    return ''


def check_ap_password(password: str) -> str:
    """Return why an access point password is refused, or ''."""
    if not 8 <= len(password) <= 63:
        return 'Use 8 to 63 characters.'
    if not all(32 <= ord(c) < 127 for c in password):
        return 'Use letters, digits and symbols only.'
    return ''


class Bus:
    """NetworkManager over the system D-Bus, for the calls that carry secrets."""

    NM = 'org.freedesktop.NetworkManager'

    def __init__(self) -> None:
        import dbus

        self.dbus = dbus
        self.bus = dbus.SystemBus()
        self.nm = dbus.Interface(
            self.bus.get_object(self.NM, '/org/freedesktop/NetworkManager'), self.NM
        )
        self.settings = dbus.Interface(
            self.bus.get_object(self.NM, '/org/freedesktop/NetworkManager/Settings'),
            f'{self.NM}.Settings',
        )

    def _wire(self, profile: dict[str, dict[str, Any]]) -> Any:
        d = self.dbus
        out = d.Dictionary({}, signature='sa{sv}')
        for group, values in profile.items():
            items = {}
            for key, value in values.items():
                if group == '802-11-wireless' and key == 'ssid':
                    value = d.ByteArray(value.encode())
                elif isinstance(value, bool):
                    value = d.Boolean(value)
                elif isinstance(value, int):
                    value = d.Int32(value)
                elif isinstance(value, list):
                    value = d.Array(value, signature='s')
                items[key] = value
            out[group] = d.Dictionary(items, signature='sv')
        return out

    def add_and_activate(self, profile: dict[str, dict[str, Any]], device: str) -> None:
        self.nm.AddAndActivateConnection(
            self._wire(profile), self.nm.GetDeviceByIpIface(device), '/'
        )

    def set_secret(self, connection_id: str, group: str, key: str, value: str) -> str:
        """Replace one secret of a saved connection, keeping the rest; return the old value."""
        path = next(
            p
            for p in self.settings.ListConnections()
            if self._connection(p).GetSettings()['connection']['id'] == connection_id
        )
        con = self._connection(path)
        settings = con.GetSettings()
        settings[group].update(con.GetSecrets(group).get(group, {}))
        old = str(settings[group].get(key, ''))
        settings[group][key] = value
        for ip in ('ipv4', 'ipv6'):
            # The deprecated forms come back beside address-data; sending both is refused.
            if ip in settings and 'address-data' in settings[ip]:
                settings[ip].pop('addresses', None)
                settings[ip].pop('routes', None)
        con.Update(settings)
        return old

    def _connection(self, path: Any) -> Any:
        return self.dbus.Interface(
            self.bus.get_object(self.NM, path), f'{self.NM}.Settings.Connection'
        )


class Jobs:
    """One network change at a time, run off the request thread; its state is polled."""

    def __init__(
        self,
        run: Run = subprocess.run,
        bus: Callable[[], Any] = Bus,
        sleep: Callable[[float], None] = time.sleep,
    ) -> None:
        self.run, self.bus, self.sleep = run, bus, sleep
        self.lock = threading.Lock()
        self.state: dict[str, Any] = {'busy': False, 'action': '', 'ok': None, 'message': ''}

    def view(self) -> dict[str, Any]:
        with self.lock:
            return dict(self.state)

    def start(self, action: str, target: Callable[[], tuple[bool, str]], failure: str) -> bool:
        """
        Start `target` in a thread; return False while another change runs.

        `target` returns (ok, message); an empty message means the page shows
        the result itself. `failure` is the message when `target` raises.
        """
        with self.lock:
            if self.state['busy']:
                return False
            self.state = {'busy': True, 'action': action, 'ok': None, 'message': ''}

        def body() -> None:
            try:
                ok, message = target()
            except Exception:  # noqa: BLE001  the page shows `failure`, the log the cause
                log.exception('Connections %s failed', action)
                ok, message = False, failure
            with self.lock:
                self.state = {'busy': False, 'action': action, 'ok': ok, 'message': message}

        threading.Thread(target=body, daemon=True).start()
        return True

    def wait_connected(self, device: str) -> bool:
        deadline = time.monotonic() + JOIN_TIMEOUT_SEC
        seen_activating = False
        while time.monotonic() < deadline:
            state = device_states(self.run).get(device, ('', ''))[0]
            if state == 'connected':
                return True
            if state.startswith('connecting'):
                seen_activating = True
            elif seen_activating and state in ('disconnected', 'unavailable'):
                return False
            self.sleep(1.0)
        return False

    def join(self, ssid: str, security: str, password: str, identity: str) -> tuple[bool, str]:
        saved = saved_networks(self.run)
        match = next((s for s in saved if s['ssid'] == ssid), None)
        if match:
            if password and not match['enterprise'] and security in ('psk', 'sae'):
                self.bus().set_secret(ssid, '802-11-wireless-security', 'psk', password)
            nmcli(
                ['connection', 'up', 'uuid', match['uuid'], 'ifname', WIFI], self.run, timeout=60
            )
        else:
            top = max((s['priority'] for s in saved), default=0) + 1
            self.bus().add_and_activate(new_profile(ssid, security, password, identity, top), WIFI)
        if self.wait_connected(WIFI):
            return True, ''
        if match is None:
            # A wrong password would otherwise leave a profile that keeps failing.
            for s in saved_networks(self.run):
                if s['ssid'] == ssid:
                    nmcli(['connection', 'delete', 'uuid', s['uuid']], self.run)
        return False, f'Could not join {ssid}.'

    def forget(self, uuid: str) -> tuple[bool, str]:
        match = next((s for s in saved_networks(self.run) if s['uuid'] == uuid), None)
        if match is None:
            # Already gone: the page reloads the list.
            return False, ''
        if not nmcli_ok(['connection', 'delete', 'uuid', uuid], self.run):
            return False, f'Could not forget {match["ssid"]}.'
        return True, ''

    def order(self, uuids: list[str]) -> tuple[bool, str]:
        known = {s['uuid'] for s in saved_networks(self.run)}
        if set(uuids) != known:
            # The list changed meanwhile: the page reloads it and drops the reorder.
            return False, ''
        for rank, uuid in enumerate(uuids):
            nmcli(
                [
                    'connection',
                    'modify',
                    'uuid',
                    uuid,
                    'connection.autoconnect-priority',
                    str(len(uuids) - rank),
                ],
                self.run,
            )
        return True, ''

    def wifi_power(self, on: bool) -> tuple[bool, str]:
        if not on:
            if not nmcli_ok(['device', 'disconnect', WIFI], self.run):
                return False, 'Could not turn off WiFi.'
            return True, ''
        # With no saved network in range `device connect` fails; autoconnect still joins one later.
        nmcli(['device', 'set', WIFI, 'autoconnect', 'yes'], self.run)
        nmcli(['device', 'connect', WIFI], self.run, timeout=60)
        return True, ''

    def ap_password(self, password: str) -> tuple[bool, str]:
        # The answer reaches a page on the access point before the restart drops it.
        self.sleep(APPLY_DELAY_SEC)
        bus = self.bus()
        old = bus.set_secret(AP_CONNECTION, '802-11-wireless-security', 'psk', password)
        if nmcli_ok(['connection', 'up', 'id', AP_CONNECTION], self.run, timeout=60):
            return True, 'Password changed.'
        # The access point still runs on the old password; keep the saved one matching it.
        if old:
            bus.set_secret(AP_CONNECTION, '802-11-wireless-security', 'psk', old)
        return False, 'Could not change the password.'
