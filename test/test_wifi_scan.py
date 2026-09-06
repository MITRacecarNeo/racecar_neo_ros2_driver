"""Tests for scripts/wifi_scan.py (the `racecar wifi list` formatter)."""

import importlib.util
from pathlib import Path

import pytest

SCRIPT = Path(__file__).parent.parent / 'scripts' / 'wifi_scan.py'

_spec = importlib.util.spec_from_file_location('wifi_scan', SCRIPT)
wifi_scan = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(wifi_scan)


# Shape of a real scan on a car parked in a lab: the same access point
# repeated across BSSIDs, several hidden networks, mixed security fields.
FIXTURE = '\n'.join([
    'Duck:100:WPA2:',
    ':100:WPA2 802.1X:',
    'FBISurveillanceVan:100:WPA2:',
    'Duck:72:WPA2:',
    ':88:WPA2:',
    'xfinitywifi:100::',
    'eduroam:97:WPA2 802.1X:*',
    'Duck:64:WPA2:',
    'FBISurveillanceVan:81:WPA2:',
    ':40:WPA1 WPA2 802.1X:',
    'vandv:52:WPA2:',
])


class TestSplitNmcli:
    def test_plain_record(self):
        assert wifi_scan.split_nmcli('Duck:100:WPA2:') == ['Duck', '100', 'WPA2', '']

    def test_escaped_colon_stays_in_field(self):
        # nmcli escapes a literal colon inside an SSID; a naive split would
        # shift every later field by one.
        fields = wifi_scan.split_nmcli(r'we\:ird:80:WPA2:')
        assert fields[0] == 'we:ird'
        assert fields[1] == '80'
        assert fields[2] == 'WPA2'


class TestParse:
    def test_deduplicates_by_ssid(self):
        networks, _ = wifi_scan.parse(FIXTURE)
        ssids = [n['ssid'] for n in networks]
        assert len(ssids) == len(set(ssids)), 'each SSID should appear once'
        assert 'Duck' in ssids

    def test_keeps_strongest_signal_per_ssid(self):
        networks, _ = wifi_scan.parse(FIXTURE)
        duck = next(n for n in networks if n['ssid'] == 'Duck')
        assert duck['signal'] == 100, 'should keep the strongest of 100/72/64'

    def test_counts_hidden_without_listing_them(self):
        networks, hidden = wifi_scan.parse(FIXTURE)
        assert hidden == 3
        assert all(n['ssid'] for n in networks), 'blank SSIDs must not be listed'

    def test_sorted_by_signal_descending(self):
        networks, _ = wifi_scan.parse(FIXTURE)
        signals = [n['signal'] for n in networks]
        assert signals == sorted(signals, reverse=True)

    def test_empty_security_reads_as_open(self):
        networks, _ = wifi_scan.parse(FIXTURE)
        xfinity = next(n for n in networks if n['ssid'] == 'xfinitywifi')
        assert xfinity['security'] == 'open'

    def test_in_use_marker_preserved(self):
        networks, _ = wifi_scan.parse(FIXTURE)
        eduroam = next(n for n in networks if n['ssid'] == 'eduroam')
        assert eduroam['in_use'] is True

    def test_in_use_survives_a_stronger_duplicate(self):
        # The associated BSSID is not always the strongest one visible; losing
        # the marker would show a connected network as unconnected.
        text = 'Net:50:WPA2:*\nNet:90:WPA2:'
        networks, _ = wifi_scan.parse(text)
        assert len(networks) == 1
        assert networks[0]['signal'] == 90
        assert networks[0]['in_use'] is True

    def test_empty_input(self):
        networks, hidden = wifi_scan.parse('')
        assert networks == []
        assert hidden == 0

    def test_short_records_do_not_raise(self):
        networks, _ = wifi_scan.parse('OnlySsid\nOther:60\n')
        assert {n['ssid'] for n in networks} == {'OnlySsid', 'Other'}

    def test_non_numeric_signal_is_tolerated(self):
        networks, _ = wifi_scan.parse('Weird:notanumber:WPA2:')
        assert networks[0]['signal'] == 0


class TestRender:
    def _render(self, saved=frozenset()):
        networks, hidden = wifi_scan.parse(FIXTURE)
        return wifi_scan.render(networks, hidden, set(saved), 'wlan0', False)

    def test_marks_saved_profiles(self):
        out = self._render(saved={'Duck'})
        duck_line = next(ln for ln in out.splitlines() if ln.strip().startswith('Duck'))
        assert 'saved' in duck_line

    def test_unsaved_network_not_marked(self):
        out = self._render(saved={'Duck'})
        line = next(ln for ln in out.splitlines() if 'vandv' in ln)
        assert 'saved' not in line

    def test_summary_reports_counts(self):
        out = self._render()
        assert '5 networks, 3 hidden' in out

    def test_scan_kind_reported(self):
        networks, hidden = wifi_scan.parse(FIXTURE)
        assert 'cached scan' in wifi_scan.render(networks, hidden, set(), 'wlan0', False)
        assert 'fresh scan' in wifi_scan.render(networks, hidden, set(), 'wlan0', True)

    def test_names_the_interface(self):
        # The command is scoped to the client radio; the output should say so
        # rather than leave the reader guessing which adapter was scanned.
        assert 'wlan0' in self._render()

    def test_empty_scan_renders(self):
        out = wifi_scan.render([], 0, set(), 'wlan0', False)
        assert 'no networks visible' in out


@pytest.mark.parametrize('ssid', ['Duck', 'FBISurveillanceVan', 'vandv'])
def test_every_listed_ssid_came_from_the_scan(ssid):
    networks, _ = wifi_scan.parse(FIXTURE)
    assert ssid in {n['ssid'] for n in networks}
