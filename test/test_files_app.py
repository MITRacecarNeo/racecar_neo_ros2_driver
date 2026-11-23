"""Tests for scripts/files_app.py, the read-only Files app."""

import json

from conftest import load_script
import pytest


@pytest.fixture(scope='module')
def files():
    return load_script('files_app')


@pytest.fixture(autouse=True)
def roots(files, tmp_path, monkeypatch):
    # The library and data roots of the test tree stand in for /opt/racecar-neo and /mnt/nvme/data.
    monkeypatch.setattr(
        files,
        'allowed_roots',
        lambda root=None: [(tmp_path / p).resolve() for p in ('jupyter_ws', 'opt', 'data')],
    )


@pytest.fixture
def ws(tmp_path):
    root = tmp_path / 'jupyter_ws'
    (root / 'labs').mkdir(parents=True)
    (root / 'labs' / 'drive.py').write_text('x = 1\n')
    (root / '.hidden').write_text('')
    (root / '__pycache__').mkdir()
    (root / 'README.md').write_text('# Hi\n\n[bad](javascript:alert(1)) [ok](labs/drive.py)\n')
    data = tmp_path / 'data'
    data.mkdir()
    (data / 'run.csv').write_text('a,b\n')
    (root / 'data').symlink_to(data)
    secret = tmp_path / 'secret.txt'
    secret.write_text('no')
    library = tmp_path / 'opt' / 'library-v1'
    library.mkdir(parents=True)
    (library / 'racecar_core.py').write_text('')
    (root / 'racecar-neo').mkdir()
    (root / 'racecar-neo' / 'library').symlink_to(library)
    return root


class TestResolve:
    def test_inside_and_followed_links(self, files, ws):
        assert files.resolve('', ws) == ws
        assert files.resolve('labs/drive.py', ws) == ws / 'labs' / 'drive.py'
        assert files.resolve('data/run.csv', ws) is not None

    def test_links_elsewhere_are_refused(self, files, ws, tmp_path):
        ssh = tmp_path / 'home' / '.ssh'
        ssh.mkdir(parents=True)
        (ssh / 'id_test').write_text('not a key')
        (ws / 'labs' / 'keys').symlink_to(ssh)
        (ws / 'labs' / 'etc').symlink_to(tmp_path / 'secret.txt')
        assert files.resolve('labs/keys', ws) is None
        assert files.resolve('labs/keys/id_test', ws) is None
        assert files.resolve('labs/etc', ws) is None
        assert files.listing('labs/keys', ws) is None
        assert files.view('labs/keys/id_test', ws) is None

    def test_only_get_routes(self, files):
        assert not any(
            hasattr(files.Handler, m) for m in ('do_POST', 'do_PUT', 'do_DELETE', 'do_PATCH')
        )

    def test_real_roots(self, files, monkeypatch):
        monkeypatch.undo()
        roots = files.allowed_roots()
        assert files.LIBRARY_ROOT.resolve() in roots and files.ROOT.resolve() in roots

    def test_nested_link_is_entered(self, files, ws):
        assert files.resolve('racecar-neo/library/racecar_core.py', ws) is not None
        names = [e['name'] for e in files.listing('racecar-neo/library', ws)['entries']]
        assert names == ['racecar_core.py']

    @pytest.mark.parametrize(
        'rel',
        ['..', 'labs/../../secret.txt', '%2e%2e/secret.txt', 'racecar-neo/library/..', 'missing'],
    )
    def test_refused(self, files, ws, rel):
        assert files.resolve(rel, ws) is None


def test_listing_hides_dot_files_and_puts_folders_first(files, ws):
    names = [(e['name'], e['dir']) for e in files.listing('', ws)['entries']]
    assert names == [('data', True), ('labs', True), ('racecar-neo', True), ('README.md', False)]
    assert files.listing('labs/drive.py', ws) is None
    assert files.listing('..', ws) is None


@pytest.mark.parametrize(
    'name, kind',
    [('a.ipynb', 'notebook'), ('a.md', 'markdown'), ('a.PNG', 'image'), ('a.py', 'text'),
     ('Makefile', 'text'), ('a.bag', 'binary')],
)  # fmt: skip
def test_kind_of(files, tmp_path, name, kind):
    assert files.kind_of(tmp_path / name) == kind


@pytest.mark.parametrize(
    'target, expected',
    [('labs/a.py', 'labs/a.py'), ('https://x.org', 'https://x.org'), ('javascript:alert(1)', '#'),
     (' JavaScript:alert(1)', '#'), ('data:text/html,x', '#'), ('vbscript:x', '#')],
)  # fmt: skip
def test_safe_link(files, target, expected):
    assert files.safe_link(target) == expected


class TestView:
    def test_code(self, files, ws):
        v = files.view('labs/drive.py', ws)
        assert v['kind'] == 'text' and v['text'] == 'x = 1\n' and v['language'] == 'python'

    def test_markdown_drops_script_links(self, files, ws):
        html = files.view('README.md', ws)['html']
        assert 'javascript:' not in html and 'labs/drive.py' in html

    def test_large_files_are_download_only(self, files, ws, monkeypatch):
        monkeypatch.setattr(files, 'VIEW_MAX_BYTES', 3)
        v = files.view('labs/drive.py', ws)
        assert v['kind'] == 'binary' and 'text' not in v

    def test_bad_notebook_shows_as_text(self, files, ws):
        (ws / 'bad.ipynb').write_text('{not json')
        assert files.view('bad.ipynb', ws)['kind'] == 'text'

    def test_missing(self, files, ws):
        assert files.view('labs', ws) is None and files.view('../secret.txt', ws) is None


class TestNotebook:
    def test_cells_and_outputs_are_escaped(self, files):
        nb = {'cells': [
            {'cell_type': 'markdown', 'source': ['# Title']},
            {'cell_type': 'code', 'source': ['print("<b>")'], 'outputs': [
                {'output_type': 'stream', 'text': ['<i>\n']},
                {'output_type': 'display_data', 'data': {'image/png': 'iVBORw0KGgo=\n'}},
                {'output_type': 'display_data', 'data': {'image/png': 'x" onerror="alert(1)'}},
                {'output_type': 'error', 'ename': 'ValueError', 'evalue': '<x>'},
            ]},
            {'cell_type': 'raw', 'source': ['skip me']},
        ]}  # fmt: skip
        html = files.notebook_html(nb)
        assert 'Title' in html and 'print(&quot;&lt;b&gt;&quot;)' in html and '&lt;i&gt;' in html
        assert html.count('<img') == 1 and 'base64,iVBORw0KGgo="' in html
        assert 'onerror' not in html
        assert 'ValueError: &lt;x&gt;' in html and 'skip me' not in html

    def test_view_renders_a_notebook(self, files, ws):
        cells = [{'cell_type': 'code', 'source': 'x', 'outputs': []}]
        (ws / 'lab.ipynb').write_text(json.dumps({'cells': cells}))
        v = files.view('lab.ipynb', ws)
        assert v['kind'] == 'notebook' and '<pre class="code python">x</pre>' in v['html']


def test_pages_lists_home_and_every_app(files, monkeypatch):
    monkeypatch.setattr(files, 'unit_states', lambda names: {n: 'active' for n in names})
    pages = files.pages_view()
    assert pages[0]['id'] == 'home' and pages[0]['path'] == 'home'
    ids = [p['id'] for p in pages]
    assert 'files' in ids and 'mapview' in ids
    assert {p['state'] for p in pages} == {'active'}


class TestRaw:
    def _raw(self, files, monkeypatch, ws, rel, download=False):
        if not hasattr(files.resolve, 'patched'):
            original = files.resolve

            def resolve(r, root=ws):
                return original(r, root)

            resolve.patched = True
            monkeypatch.setattr(files, 'resolve', resolve)
        h = object.__new__(files.Handler)
        h.sent = []
        h.send = lambda code, body, kind, headers=None: h.sent.append(
            (code, body, kind, headers or {})
        )
        h._raw(rel, download)
        return h.sent[0]

    def test_html_and_svg_never_run(self, files, ws, monkeypatch):
        (ws / 'page.html').write_text('<script>x</script>')
        (ws / 'pic.svg').write_text('<svg/>')
        code, body, kind, headers = self._raw(files, monkeypatch, ws, 'page.html')
        assert code == 200 and kind == 'text/plain'
        assert "default-src 'none'" in headers['Content-Security-Policy']
        _, _, kind, headers = self._raw(files, monkeypatch, ws, 'pic.svg')
        assert kind == 'image/svg+xml' and 'Content-Security-Policy' in headers

    def test_download_and_binary_are_attachments(self, files, ws, monkeypatch):
        (ws / 'run.bag').write_bytes(b'\0')
        _, _, _, headers = self._raw(files, monkeypatch, ws, 'run.bag')
        assert headers['Content-Disposition'] == "attachment; filename*=UTF-8''run.bag"
        _, _, _, headers = self._raw(files, monkeypatch, ws, 'labs/drive.py', True)
        assert 'drive.py' in headers['Content-Disposition']
        _, _, _, headers = self._raw(files, monkeypatch, ws, 'labs/drive.py')
        assert 'Content-Disposition' not in headers

    def test_outside_is_404(self, files, ws, monkeypatch):
        assert self._raw(files, monkeypatch, ws, '../secret.txt')[0] == 404
