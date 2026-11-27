"""scripts/handbook.py and the pages in docs/handbook/."""

from pathlib import Path

from conftest import load_script
import pytest

ROOT = Path(__file__).resolve().parent.parent
HANDBOOK = ROOT / 'docs' / 'handbook'


@pytest.fixture
def hb():
    return load_script('handbook')


class TestRenderer:
    def test_heading_anchor_slug_and_fixed(self, hb):
        page = hb.render('# Car setup\n\n## NVMe data drive {#nvme}\n\n## Gamepad /joy rate\n')
        assert page.title == 'Car setup'
        assert page.anchors == ['car-setup', 'nvme', 'gamepad-joy-rate']
        assert '<h2 id="nvme">NVMe data drive</h2>' in page.html

    def test_duplicate_anchor_is_a_problem(self, hb):
        assert 'used twice' in hb.render('## A\n\n## A\n').problems[0]

    def test_lists_nested_and_continued(self, hb):
        text = '1. First\n   still first.\n2. Second\n   - inner\n3. Third\n'
        html = hb.render(text).html
        assert html.startswith('<ol><li>First still first.</li>')
        assert '<li>Second<ul><li>inner</li></ul></li>' in html
        assert html.endswith('<li>Third</li></ol>')

    def test_table_code_and_inline(self, hb):
        text = (
            '| A | B |\n| - | - |\n| `x` | **y** |\n\n'
            '```sh\na < b\n```\n\nSee [it](health.md#reports).\n'
        )
        page = hb.render(text)
        assert '<td><code>x</code></td><td><strong>y</strong></td>' in page.html
        assert '<pre><code>a &lt; b</code></pre>' in page.html
        assert '<a href="health#reports">it</a>' in page.html
        assert page.links == ['health.md#reports']

    def test_text_is_escaped(self, hb):
        assert '&lt;b&gt;' in hb.render('Use a<b> here.\n').html

    def test_raw_html_and_deep_heading_rejected(self, hb):
        problems = hb.render('<div>x</div>\n\n##### Deep\n\n> quote\n').problems
        assert len(problems) == 3

    def test_static_links_get_html(self, hb):
        assert hb.page_link('upgrade.md#upgrade-back', '.html') == 'upgrade.html#upgrade-back'
        assert hb.page_link('../troubleshooting.md#x') == '../troubleshooting.md#x'
        assert hb.page_link('https://github.com/x') == 'https://github.com/x'


class TestPages:
    def test_every_page_in_the_subset(self, hb):
        names = [p.name for p in hb.pages()]
        assert names[:5] == [
            'index.md',
            'installation.md',
            'health.md',
            'calibration.md',
            'upgrade.md',
        ]
        assert {'installation.md', 'health.md', 'calibration.md', 'upgrade.md'} <= set(names)
        for path in hb.pages():
            assert hb.render(path.read_text()).problems == [], path.name

    def test_every_link_resolves(self, hb):
        anchors = {}
        for path in [*hb.pages(), ROOT / 'docs' / 'troubleshooting.md']:
            anchors[path.resolve()] = set(hb.render(path.read_text()).anchors)
        for path in hb.pages():
            for link in hb.render(path.read_text()).links:
                if '://' in link:
                    continue
                target, _, frag = link.partition('#')
                file = (path.parent / target).resolve() if target else path.resolve()
                assert file in anchors, f'{path.name}: {link}'
                if frag:
                    assert frag in anchors[file], f'{path.name}: {link}'

    def test_every_setup_step_has_an_anchor(self, hb):
        steps = load_script('car_setup').STEP_KEYS
        anchors = hb.render((HANDBOOK / 'installation.md').read_text()).anchors
        assert [k for k in steps if k not in anchors] == []

    def test_ascii_only(self, hb):
        for path in hb.pages():
            assert path.read_text().isascii(), path.name


def test_static_build_writes_every_page(hb, tmp_path):
    written = hb.build(tmp_path)
    assert {p.name for p in written} == {p.stem + '.html' for p in hb.pages()} | {
        'troubleshooting.html'
    }
    index = (tmp_path / 'index.html').read_text()
    assert 'href="installation.html#identity"' in index
    assert (tmp_path / 'rcn.css').exists()
    assert 'href="troubleshooting.html"' in (tmp_path / 'health.html').read_text()
    assert (tmp_path / 'troubleshooting.html').exists()
    assert not list(tmp_path.glob('*.png'))


def test_build_command(hb, tmp_path, capsys):
    assert hb.main(['build', str(tmp_path / 'out')]) == 0
    assert 'pages written' in capsys.readouterr().out


def test_racecar_handbook_dispatches():
    tool = (ROOT / 'scripts' / 'racecar-tool.sh').read_text()
    assert 'python3 "$pkg_dir/scripts/handbook.py" "$@"' in tool


class TestRealsenseLibrary:
    @pytest.fixture
    def dg(self):
        return load_script('diagnose')

    def test_versions(self, dg):
        assert dg.upstream('2.58.1-1noble.20260525.094321') == (2, 58, 1)
        assert dg.realsense_library_check('2.58.4-1noble', '4.58.4-1noble').status == dg.OK
        warn = dg.realsense_library_check('2.58.1-1noble', '4.58.4-1noble')
        assert warn.status == dg.WARN
        assert warn.detail.startswith('2.58.1, camera package needs 2.58.4. Run: sudo apt install')
        assert dg.realsense_library_check('', '4.58.4').status == dg.SKIP

    def test_setup_installs_the_library_with_the_camera(self):
        text = (ROOT / 'scripts' / 'setup_realsense.sh').read_text()
        assert 'install -y ros-jazzy-librealsense2 ros-jazzy-realsense2-camera' in text
