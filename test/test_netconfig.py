"""Tests for scripts/netconfig.py, the Connections page's NetworkManager calls."""

from types import SimpleNamespace

from conftest import load_script
import pytest


@pytest.fixture(scope='module')
def net():
    return load_script('netconfig')


@pytest.fixture(autouse=True)
def names_file(net, tmp_path, monkeypatch):
    # Never read or write the real car profile.
    monkeypatch.setattr(net, 'names_path', lambda: tmp_path / 'devices.yaml')


CONNECTIONS = {
    'racecar-neo-ap': 'ap\nracecar-neo-0\nwpa-psk',
    'u-duck': 'infrastructure\nDuck\nwpa-psk',
    'u-edu': 'infrastructure\neduroam\nwpa-eap',
    'u-open': 'infrastructure\nCafe\n',
}


class FakeNm:
    """Answers nmcli and iw like a car on Duck with the access point up."""

    def __init__(self, wifi_state='connected', auto='yes'):
        self.calls = []
        self.wifi_state, self.auto = wifi_state, auto

    def __call__(self, argv, **_):
        self.calls.append(argv)
        out = self.answer(argv)
        return SimpleNamespace(returncode=0 if out is not None else 10, stdout=out or '')

    def answer(self, argv):
        args = argv[1:]
        if argv[0] == 'iw':
            return (
                'Station AA:BB:CC:00:00:01 (on wlan1)\n\tinactive time:\t212 ms\n'
                '\tsignal:  \t-44 [-44, -53] dBm\n\tconnected time:\t624 seconds\n'
                'Station aa:bb:cc:00:00:02 (on wlan1)\n\tsignal:  \t-70 dBm\n'
            )
        if argv[:4] == ['ip', '-4', 'neigh', 'show']:
            if argv[-1] == 'wlan1':
                return (
                    '10.42.0.182 lladdr aa:bb:cc:00:00:01 STALE\n'
                    '10.42.0.9 lladdr aa:bb:cc:00:00:09 FAILED\n'
                )
            return '192.168.52.10 lladdr dd:ee:ff:00:00:01 REACHABLE\n'
        if argv[:3] == ['ip', 'neigh', 'show']:
            if argv[-1] == 'fe80::1':
                return 'fe80::1 dev wlan1 lladdr aa:bb:cc:00:00:01 REACHABLE\n'
            return ''
        if argv[0] == 'journalctl':
            return (
                'DHCPACK(wlan1) 10.42.0.182 aa:bb:cc:00:00:01 student-laptop\n'
                'DHCPACK(wlan1) 10.42.0.5 aa:bb:cc:00:00:02\n'
            )
        if argv[0] == 'ping':
            return (
                'PING google.com (142.250.80.46) 56(84) bytes of data.\n\n'
                '--- google.com ping statistics ---\n'
                '4 packets transmitted, 3 received, 25% packet loss, time 3002ms\n'
                'rtt min/avg/max/mdev = 23.312/26.360/34.464/4.692 ms\n'
            )
        if argv[:4] == ['ip', '-o', 'route', 'get']:
            return '142.250.80.46 via 10.0.0.1 dev wlan0 src 10.0.0.198 uid 1000\n'
        if args[:4] == ['-t', '-f', 'DEVICE,STATE,CONNECTION', 'device']:
            return (
                f'wlan0:{self.wifi_state}:Duck\nwlan1:connected:racecar-neo-ap\n'
                'eth0:unavailable:\nlo:connected (externally):lo\n'
            )
        if args[:3] == ['-t', '-f', 'NAME,UUID,TYPE,AUTOCONNECT-PRIORITY']:
            return (
                'racecar-neo-ap:racecar-neo-ap:802-11-wireless:0\nDuck:u-duck:802-11-wireless:5\n'
                'eduroam:u-edu:802-11-wireless:9\nCafe:u-open:802-11-wireless:0\n'
                'netplan-eth0:u-eth:802-3-ethernet:0\n'
            )
        if args[0] == '-g' and args[2:4] == ['connection', 'show']:
            if args[1].startswith('802-11-wireless.ssid,802-11-wireless.channel'):
                return 'racecar-neo-0\n6\n10.42.0.1/24\n'
            return CONNECTIONS.get(args[4], '') + '\n'
        if args[0] == '-g' and args[2:4] == ['device', 'show']:
            if args[4] == 'wlan0':
                return f'10.0.0.198/24 | 10.0.0.199/24\n4 (full)\n{self.auto}\n'
            return '\n'
        if args[:3] == ['-t', '-f', 'SSID,SIGNAL,SECURITY,IN-USE']:
            return (
                'Duck:80:WPA2:*\nracecar-neo-0:70:WPA2:\nCafe:60::\n'
                'eduroam:40:WPA2 802.1X:\nNew\\:Net:30:WPA3:\n'
                'Home:20:WPA1 WPA2:\n:10:WPA2:\n'
            )
        return ''


class TestParsing:
    @pytest.mark.parametrize(
        'security, kind',
        [
            ('', 'open'),
            ('--', 'open'),
            ('open', 'open'),
            ('WPA2', 'psk'),
            ('WPA1 WPA2', 'psk'),
            ('WPA2 WPA3', 'psk'),
            ('WPA3', 'sae'),
            ('WPA2 802.1X', 'enterprise'),
        ],
    )
    def test_security_kind(self, net, security, kind):
        assert net.security_kind(security) == kind

    @pytest.mark.parametrize(
        'value, word',
        [
            ('4 (full)', 'online'),
            ('3 (limited)', 'limited'),
            ('2 (portal)', 'portal'),
            ('1 (none)', 'none'),
            ('0 (unknown)', 'none'),
            ('', 'none'),
        ],
    )
    def test_internet(self, net, value, word):
        assert net.internet(value) == word

    def test_first_address(self, net):
        assert net.first_address('10.0.0.198/24 | 10.0.0.199/24') == '10.0.0.198/24'
        assert net.first_address('') == ''

    @pytest.mark.parametrize(
        'ip, device',
        [
            ('10.42.0.23', 'wlan1'),
            ('10.0.0.50', 'wlan0'),
            ('127.0.0.1', ''),
            ('8.8.8.8', ''),
            ('nonsense', ''),
        ],
    )
    def test_via_device(self, net, ip, device):
        addresses = {'wlan0': '10.0.0.198/24', 'wlan1': '10.42.0.1/24', 'eth0': ''}
        assert net.via_device(ip, addresses) == device


class TestView:
    def test_whole_view(self, net):
        v = net.view('10.42.0.23', run=FakeNm())
        w = v['wifi']
        assert (w['on'], w['connected'], w['network'], w['signal']) == (True, True, 'Duck', 80)
        assert w['address'] == '10.0.0.198' and w['internet'] == 'online'
        assert [s['ssid'] for s in w['saved']] == ['eduroam', 'Duck', 'Cafe']
        assert w['saved'][0]['enterprise'] and not w['saved'][1]['enterprise']
        names = [(n['ssid'], n['security'], n['saved']) for n in w['visible']]
        assert names == [
            ('Duck', 'psk', True),
            ('Cafe', 'open', True),
            ('eduroam', 'enterprise', True),
            ('New:Net', 'sae', False),
            ('Home', 'psk', False),
        ]
        assert v['ap'] == {
            'device': 'wlan1',
            'on': True,
            'ssid': 'racecar-neo-0',
            'channel': '6',
            'address': '10.42.0.1',
        }
        macs = [d['mac'] for d in v['devices']]
        assert macs == ['aa:bb:cc:00:00:01', 'aa:bb:cc:00:00:02', 'dd:ee:ff:00:00:01']
        assert v['eth'] == {'device': 'eth0', 'state': 'no cable', 'address': ''}
        assert v['via'] == 'wlan1'

    def test_rescan_flag(self, net):
        run = FakeNm()
        net.view('', rescan=True, run=run)
        scan = next(c for c in run.calls if 'list' in c)
        assert scan[-2:] == ['--rescan', 'yes'] and scan[scan.index('ifname') + 1] == 'wlan0'

    def test_off_and_idle(self, net):
        off = net.view('', run=FakeNm(wifi_state='disconnected', auto='no'))['wifi']
        idle = net.view('', run=FakeNm(wifi_state='disconnected', auto='yes'))['wifi']
        assert (off['on'], off['connected'], off['internet']) == (False, False, 'none')
        assert (idle['on'], idle['connected']) == (True, False)

    def test_nmcli_failure_reads_empty(self, net):
        def broken(argv, **_):
            raise OSError('no nmcli')

        v = net.view('', run=broken)
        assert v['wifi']['saved'] == [] and v['wifi']['visible'] == [] and not v['ap']['on']


class TestProfiles:
    def test_psk(self, net):
        p = net.new_profile('Home', 'psk', 'secret123', '', 4)
        assert (
            p['connection']['interface-name'] == 'wlan0'
            and p['connection']['autoconnect-priority'] == 4
        )
        assert p['802-11-wireless-security'] == {'key-mgmt': 'wpa-psk', 'psk': 'secret123'}
        assert '802-1x' not in p

    def test_sae_and_open(self, net):
        assert (
            net.new_profile('A', 'sae', 'secret123', '', 1)['802-11-wireless-security']['key-mgmt']
            == 'sae'
        )
        assert '802-11-wireless-security' not in net.new_profile('A', 'open', '', '', 1)

    def test_enterprise_validates_the_server(self, net):
        p = net.new_profile('eduroam', 'enterprise', 'pw', 'me@mit.edu', 1)
        x = p['802-1x']
        assert x['identity'] == 'me@mit.edu' and x['domain-suffix-match'] == 'mit.edu'
        assert (
            x['system-ca-certs'] is True
            and x['eap'] == ['ttls']
            and x['phase2-auth'] == 'mschapv2'
        )

    @pytest.mark.parametrize(
        'security, password, identity, saved, ok',
        [
            ('psk', 'short', '', False, False),
            ('psk', 'longenough', '', False, True),
            ('psk', '', '', True, True),
            ('open', '', '', False, True),
            ('enterprise', 'pw', 'me', False, False),
            ('enterprise', 'pw', 'me@', False, False),
            ('enterprise', '', 'me@mit.edu', False, False),
            ('enterprise', 'pw', 'me@mit.edu', False, True),
        ],
    )
    def test_check_join(self, net, security, password, identity, saved, ok):
        assert (net.check_join(security, password, identity, saved) == '') is ok

    @pytest.mark.parametrize(
        'password, ok',
        [
            ('short', False),
            ('racecar@mit', True),
            ('x' * 64, False),
            ('caf\u00e9caf\u00e9', False),
        ],
    )
    def test_check_ap_password(self, net, password, ok):
        assert (net.check_ap_password(password) == '') is ok


class FakeBus:
    def __init__(self):
        self.added, self.secrets = [], []

    def __call__(self):
        return self

    def add_and_activate(self, profile, device):
        self.added.append((profile, device))

    def set_secret(self, connection_id, group, key, value):
        self.secrets.append((connection_id, group, key, value))


class TestJobs:
    def jobs(self, net, run=None, states=('connecting (prepare)', 'connected')):
        run = run or FakeNm()
        seq = iter(states)
        original = run.answer

        def answer(argv):
            if (
                argv[1:5] == ['-t', '-f', 'DEVICE,STATE,CONNECTION', 'device']
                and 'next' in run.__dict__
            ):
                return f'wlan0:{next(seq, states[-1])}:x\n'
            return original(argv)

        run.answer = answer
        bus = FakeBus()
        return net.Jobs(run=run, bus=bus, sleep=lambda s: None), run, bus

    def test_join_new_network_uses_dbus_and_tops_the_list(self, net):
        jobs, run, bus = self.jobs(net)
        run.next = True
        ok, message = jobs.join('Home', 'psk', 'secret123', '')
        assert ok and message == ''
        profile, device = bus.added[0]
        assert device == 'wlan0' and profile['connection']['autoconnect-priority'] == 10
        assert not any('secret123' in ' '.join(c) for c in run.calls)

    def test_join_saved_brings_it_up(self, net):
        jobs, run, bus = self.jobs(net)
        run.next = True
        assert jobs.join('Duck', 'psk', '', '')[0]
        assert ['nmcli', 'connection', 'up', 'uuid', 'u-duck', 'ifname', 'wlan0'] in run.calls
        assert bus.added == [] and bus.secrets == []

    def test_join_saved_with_a_new_password_updates_the_secret(self, net):
        jobs, run, bus = self.jobs(net)
        run.next = True
        jobs.join('Duck', 'psk', 'newpass123', '')
        assert bus.secrets == [('Duck', '802-11-wireless-security', 'psk', 'newpass123')]

    def test_failed_join_removes_the_new_profile(self, net, monkeypatch):
        jobs, run, bus = self.jobs(net, states=('connecting (need auth)', 'disconnected'))
        run.next = True
        saved = iter(
            [[], [{'ssid': 'Home', 'uuid': 'u-home', 'priority': 1, 'enterprise': False}]]
        )
        monkeypatch.setattr(net, 'saved_networks', lambda run: next(saved))
        ok, message = jobs.join('Home', 'psk', 'wrongpass', '')
        assert not ok and message == 'Could not join Home.'
        assert ['nmcli', 'connection', 'delete', 'uuid', 'u-home'] in run.calls

    def test_forget_only_client_networks(self, net):
        jobs, run, _ = self.jobs(net)
        assert not jobs.forget('racecar-neo-ap')[0]
        assert not jobs.forget('u-eth')[0]
        assert jobs.forget('u-duck') == (True, '')
        assert jobs.forget('u-missing') == (False, '')

    def test_order_sets_priorities_top_down(self, net):
        jobs, run, _ = self.jobs(net)
        assert jobs.order(['u-open', 'u-duck', 'u-edu']) == (True, '')
        mods = [c[4:] for c in run.calls if c[1:3] == ['connection', 'modify']]
        assert mods == [
            ['u-open', 'connection.autoconnect-priority', '3'],
            ['u-duck', 'connection.autoconnect-priority', '2'],
            ['u-edu', 'connection.autoconnect-priority', '1'],
        ]
        assert not jobs.order(['u-duck'])[0]

    def test_wifi_power(self, net):
        jobs, run, _ = self.jobs(net)
        assert jobs.wifi_power(False) == (True, '')
        assert run.calls[-1] == ['nmcli', 'device', 'disconnect', 'wlan0']
        jobs.wifi_power(True)
        assert ['nmcli', 'device', 'set', 'wlan0', 'autoconnect', 'yes'] in run.calls
        assert run.calls[-1] == ['nmcli', 'device', 'connect', 'wlan0']
        assert not any('wlan1' in c for c in run.calls)

    def test_ap_password_over_dbus_then_restart(self, net):
        jobs, run, bus = self.jobs(net)
        assert jobs.ap_password('newpass123') == (True, 'Password changed.')
        assert bus.secrets == [('racecar-neo-ap', '802-11-wireless-security', 'psk', 'newpass123')]
        assert run.calls[-1] == ['nmcli', 'connection', 'up', 'id', 'racecar-neo-ap']
        assert not any('newpass123' in ' '.join(c) for c in run.calls)

    def test_one_change_at_a_time(self, net):
        import threading

        jobs, _, _ = self.jobs(net)
        gate = threading.Event()
        assert jobs.start('wifi', lambda: (gate.wait(5), (True, 'done'))[1], 'x')
        assert not jobs.start('forget', lambda: (True, ''), 'x')
        assert jobs.view()['busy']
        gate.set()
        for _ in range(100):
            if not jobs.view()['busy']:
                break
            threading.Event().wait(0.01)
        assert jobs.view() == {'busy': False, 'action': 'wifi', 'ok': True, 'message': 'done'}

    def test_a_failure_is_reported(self, net):
        import threading

        jobs, _, _ = self.jobs(net)

        def boom():
            raise RuntimeError('bus down')

        jobs.start('join', boom, 'Could not join Home.')
        for _ in range(100):
            if not jobs.view()['busy']:
                break
            threading.Event().wait(0.01)
        # The cause goes to the log; the page gets the action's own line.
        assert jobs.view()['ok'] is False and jobs.view()['message'] == 'Could not join Home.'


def test_wifi_off_failure(net):
    def failing(argv, **_):
        return SimpleNamespace(returncode=10, stdout='')

    jobs = net.Jobs(run=failing, bus=FakeBus(), sleep=lambda s: None)
    assert jobs.wifi_power(False) == (False, 'Could not turn off WiFi.')


def test_devices(net):
    rows = net.devices(run=FakeNm(), nicknames={})
    assert rows[0] == {
        'name': 'student-laptop',
        'address': '10.42.0.182',
        'mac': 'aa:bb:cc:00:00:01',
        'device': 'wlan1',
        'signal': -44,
        'seconds': 624,
        'nickname': '',
    }
    assert rows[1]['name'] == '' and rows[1]['address'] == '' and rows[1]['signal'] == -70
    assert rows[2] == {
        'name': '',
        'address': '192.168.52.10',
        'mac': 'dd:ee:ff:00:00:01',
        'device': 'eth0',
        'signal': None,
        'seconds': None,
        'nickname': '',
    }


def test_via_device_ipv6_from_the_neighbor_table(net):
    assert net.via_device('fe80::1', {'wlan1': '10.42.0.1/24'}, run=FakeNm()) == 'wlan1'
    assert net.via_device('fe80::2', {'wlan1': '10.42.0.1/24'}, run=FakeNm()) == ''


def test_ping_test(net):
    assert net.ping_test(run=FakeNm()) == {
        'host': 'google.com',
        'result': 'reached',
        'sent': 4,
        'received': 3,
        'avg_ms': 26.4,
        'device': 'wlan0',
    }


def test_ping_test_without_a_name(net):
    def run(argv, **_):
        err = 'ping: google.com: Temporary failure in name resolution\n'
        return SimpleNamespace(returncode=2, stdout='', stderr=err)

    assert net.ping_test(run=run)['result'] == 'no-name'


def test_ping_test_unreached(net):
    def run(argv, **_):
        out = '4 packets transmitted, 0 received, 100% packet loss, time 3002ms\n'
        return SimpleNamespace(returncode=1, stdout=out, stderr='')

    p = net.ping_test(run=run)
    assert (p['result'], p['sent'], p['received'], p['avg_ms']) == ('unreached', 4, 0, None)


def test_ap_password_restart_refused_keeps_the_old_one(net):
    class Bus(FakeBus):
        def set_secret(self, *args):
            super().set_secret(*args)
            return 'oldpass123'

    def refused(argv, **_):
        return SimpleNamespace(returncode=4, stdout='')

    bus = Bus()
    jobs = net.Jobs(run=refused, bus=bus, sleep=lambda s: None)
    assert jobs.ap_password('newpass123') == (False, 'Could not change the password.')
    assert [x[3] for x in bus.secrets] == ['newpass123', 'oldpass123']


class TestNames:
    def test_save_read_and_remove(self, net, tmp_path):
        path = tmp_path / 'devices.yaml'
        assert net.read_names(path) == {}
        names = net.save_names(
            {'AA:BB:CC:00:00:01': '  Lab laptop\x07 ', 'aa:bb:cc:00:00:02': 'x' * 40}, path
        )
        assert names == {'aa:bb:cc:00:00:01': 'Lab laptop', 'aa:bb:cc:00:00:02': 'x' * 32}
        assert net.read_names(path) == names
        assert net.save_names({'aa:bb:cc:00:00:01': '   '}, path) == {
            'aa:bb:cc:00:00:02': 'x' * 32
        }
        net.save_names({'aa:bb:cc:00:00:02': ''}, path)
        assert net.read_names(path) == {} and 'names: {}' in path.read_text()

    def test_quotes_survive(self, net, tmp_path):
        path = tmp_path / 'devices.yaml'
        net.save_names({'aa:bb:cc:00:00:01': 'Chris\'s "phone": #1'}, path)
        assert net.read_names(path) == {'aa:bb:cc:00:00:01': 'Chris\'s "phone": #1'}

    @pytest.mark.parametrize('mac', ['', 'aa:bb', 'zz:bb:cc:00:00:01', '../../etc'])
    def test_bad_mac_refused(self, net, tmp_path, mac):
        with pytest.raises(ValueError):
            net.save_names({mac: 'x'}, tmp_path / 'devices.yaml')

    def test_devices_carry_the_nickname(self, net):
        rows = net.devices(run=FakeNm(), nicknames={'aa:bb:cc:00:00:01': 'Lab laptop'})
        assert rows[0]['nickname'] == 'Lab laptop' and rows[0]['name'] == 'student-laptop'
        assert rows[1]['nickname'] == ''
