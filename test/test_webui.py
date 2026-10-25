"""Shared dashboard assets: scripts/webui.py and scripts/web/."""

from pathlib import Path
import re
from xml.etree import ElementTree

from conftest import load_script
import pytest


@pytest.fixture(scope='module')
def webui():
    return load_script('webui')


def test_serves_the_stylesheet(webui):
    body, kind = webui.static_file('/ui/rcn.css')
    assert kind.startswith('text/css') and b'--rcn-orange' in body


def test_serves_fonts_logos_and_icons(webui):
    assert webui.static_file('/ui/fonts/barlow-400.woff2')[1] == 'font/woff2'
    assert webui.static_file('/ui/brand/mark-white.png')[1] == 'image/png'
    assert webui.static_file('/ui/icons.svg?v=2')[1] == 'image/svg+xml'


@pytest.mark.parametrize(
    'path',
    [
        '/index.html',
        '/ui/',
        '/ui/missing.css',
        '/ui/../webui.py',
        '/ui/%2e%2e/webui.py',
        '/ui/fonts/../../dashboard.py',
    ],
)
def test_refuses_anything_else(webui, path):
    assert webui.static_file(path) is None


def test_refuses_unknown_types(webui, tmp_path):
    (tmp_path / 'notes.md').write_text('x')
    assert webui.static_file('/ui/notes.md', root=tmp_path) is None


def test_every_font_face_file_ships(webui):
    css = (webui.WEB_DIR / 'rcn.css').read_text()
    for name in re.findall(r'url\("(fonts/[^"]+)"\)', css):
        assert (webui.WEB_DIR / name).is_file(), name
    for license_file in ('OFL-barlow.txt', 'OFL-barlowsemicondensed.txt', 'OFL-jetbrainsmono.txt'):
        assert (webui.WEB_DIR / 'fonts' / license_file).is_file()


def test_stylesheet_has_no_blue_and_uses_brand_values(webui):
    css = (webui.WEB_DIR / 'rcn.css').read_text().lower()
    assert '#3355aa' not in css  # blue is reserved; never in RACECAR UI
    for value in ('#a01936', '#f49a4b', '#ffb84a', '#2f2d38', '#423f52', '#19686c'):
        assert value in css


def test_icons_parse_and_have_ids(webui):
    root = ElementTree.parse(webui.WEB_DIR / 'icons.svg').getroot()
    ids = {el.get('id') for el in root if el.tag.endswith('symbol')}
    assert {'map', 'notebook', 'gamepad', 'line', 'wall', 'pulse', 'check', 'alert'} <= ids


def test_pages_link_shared_files_with_the_asset_version(webui):
    # A browser keeps /ui/ files for CACHE_CONTROL; the version query is what
    # makes it fetch a changed rcn.css or charts.js at once.
    scripts = Path(__file__).resolve().parent.parent / 'scripts'
    pages = [
        scripts / 'dashboard.html',
        scripts / 'dashboard_launch.html',
        scripts / 'mapview.html',
        scripts / 'dashboards' / 'teleop_dashboard' / 'teleop.html',
        scripts / 'dashboards' / 'linefollow_dashboard' / 'linefollow.html',
        scripts / 'dashboards' / 'wallfollow_dashboard' / 'wallfollow.html',
        scripts / 'dashboards' / 'camlabel_dashboard' / 'camlabel.html',
    ]
    for page in pages:
        text = page.read_text()
        for name in ('rcn.css', 'charts.js', 'lost.js', 'override.js'):
            for link in re.findall(rf'"/ui/{re.escape(name)}[^"]*"', text):
                assert link == f'"/ui/{name}?v={webui.ASSET_VERSION}"', (page.name, link)


def test_lab_pages_show_a_closed_screen():
    # A lab app stopped from the apps page leaves its tab open; lost.js covers it.
    dashboards = Path(__file__).resolve().parent.parent / 'scripts' / 'dashboards'
    for page in (
        'teleop_dashboard/teleop.html',
        'linefollow_dashboard/linefollow.html',
        'wallfollow_dashboard/wallfollow.html',
        'camlabel_dashboard/camlabel.html',
    ):
        text = (dashboards / page).read_text()
        assert '/ui/lost.js?v=' in text and 'data-app="' in text, page


def test_driving_lab_pages_keep_the_override_alive():
    dashboards = Path(__file__).resolve().parent.parent / 'scripts' / 'dashboards'
    for page in (
        'teleop_dashboard/teleop.html',
        'linefollow_dashboard/linefollow.html',
        'wallfollow_dashboard/wallfollow.html',
    ):
        assert '/ui/override.js?v=' in (dashboards / page).read_text(), page
