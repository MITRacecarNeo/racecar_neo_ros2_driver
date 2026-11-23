#!/usr/bin/env python3
"""
Files app on port 8089 (/files/): the car's pages and the student workspace.

Read-only. Lists every app with its name address and port, and browses
~/jupyter_ws: folders, text and code, Markdown and notebooks rendered, images
shown, any file downloaded. Paths never step up with `..`. Links inside the
workspace are followed only into ALLOWED_ROOTS: the workspace, the student
library under /opt/racecar-neo, and the car's data root (data, maps); a link
to anywhere else (~/.ssh, /etc) is refused. Only GET routes exist.

Run by scripts/racecar-files.service. The page is scripts/files.html.
"""

from __future__ import annotations

import json
import mimetypes
from pathlib import Path
import re
import subprocess
import sys
from typing import Any
from urllib.parse import parse_qs, quote, unquote, urlparse

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))

import handbook  # noqa: E402, I100  sibling scripts, after the path insert
from racecar_neo_ros2_driver import apps, units  # noqa: E402
import webui  # noqa: E402

PAGE = HERE / 'files.html'
ROOT = Path.home() / 'jupyter_ws'
LIBRARY_ROOT = Path('/opt/racecar-neo')


def allowed_roots(root: Path = ROOT) -> list[Path]:
    """Return the real folders a workspace path may lead into."""
    from racecar_neo_ros2_driver import data_root

    return [p.resolve() for p in (root, LIBRARY_ROOT, data_root.data_dir())]


# Larger files are offered for download, not shown.
VIEW_MAX_BYTES = 2 * 1024 * 1024
TEXT_SUFFIXES = {
    '.py', '.txt', '.md', '.yaml', '.yml', '.json', '.csv', '.sh', '.cfg', '.ini', '.toml',
    '.xml', '.html', '.css', '.js', '.launch', '.log', '.rst', '.c', '.cpp', '.h', '.hpp',
}  # fmt: skip
IMAGE_SUFFIXES = {'.png', '.jpg', '.jpeg', '.gif', '.svg', '.webp', '.bmp'}
BASE64 = re.compile(r'^[A-Za-z0-9+/=]+$')
LANGUAGES = {'.py': 'python', '.sh': 'shell', '.yaml': 'yaml', '.yml': 'yaml', '.json': 'json'}


def port() -> int:
    return apps.port('files')


def _entries(folder: Path) -> list[Path]:
    try:
        return list(folder.iterdir())
    except OSError:
        return []


def resolve(rel: str, root: Path = ROOT) -> Path | None:
    """
    Return the file or folder `rel` names under the workspace, or None.

    `rel` is a path below the workspace with no `..`. Its real target must
    fall under allowed_roots, so a link the workspace holds reaches the
    library or the data root but not ~/.ssh or /etc.
    """
    parts = [p for p in unquote(rel).split('/') if p not in ('', '.')]
    if any(p == '..' for p in parts):
        return None
    target = (root / Path(*parts)) if parts else root
    try:
        real = target.resolve(strict=True)
    except OSError:
        return None
    if not any(real == r or r in real.parents for r in allowed_roots(root)):
        return None
    return target


def listing(rel: str, root: Path = ROOT) -> dict[str, Any] | None:
    """Return a folder's entries, folders first, hidden ones left out."""
    folder = resolve(rel, root)
    if folder is None or not folder.is_dir():
        return None
    out = []
    for entry in sorted(_entries(folder), key=lambda p: (not p.is_dir(), p.name.lower())):
        if entry.name.startswith('.') or entry.name == '__pycache__':
            continue
        try:
            st = entry.stat()
        except OSError:
            continue
        out.append({
            'name': entry.name,
            'dir': entry.is_dir(),
            'size': st.st_size if entry.is_file() else None,
            'mtime': int(st.st_mtime),
        })  # fmt: skip
    return {'path': '/'.join(p for p in rel.split('/') if p), 'entries': out}


def safe_link(target: str) -> str:
    """Keep a Markdown link unless it would run code (javascript:, data:)."""
    scheme = target.split(':', 1)[0].strip().lower() if ':' in target else ''
    return '#' if scheme in ('javascript', 'data', 'vbscript') else target


def kind_of(path: Path) -> str:
    suffix = path.suffix.lower()
    if suffix == '.ipynb':
        return 'notebook'
    if suffix == '.md':
        return 'markdown'
    if suffix in IMAGE_SUFFIXES:
        return 'image'
    if suffix in TEXT_SUFFIXES or not suffix:
        return 'text'
    return 'binary'


def notebook_html(data: dict[str, Any]) -> str:
    """Render a notebook's cells: Markdown through the handbook renderer, code with outputs."""
    import html

    blocks = []
    for cell in data.get('cells', []):
        source = ''.join(cell.get('source', []))
        if cell.get('cell_type') == 'markdown':
            blocks.append(f'<div class="md">{handbook.render(source).html}</div>')
            continue
        if cell.get('cell_type') != 'code':
            continue
        blocks.append(f'<pre class="code python">{html.escape(source)}</pre>')
        for out in cell.get('outputs', []):
            values = out.get('data', {})
            png = ''.join(values.get('image/png') or []).replace('\n', '')
            if png and BASE64.match(png):
                blocks.append(f'<img class="out" src="data:image/png;base64,{png}" alt="">')
            elif 'text/plain' in values or 'text' in out:
                text = ''.join(values.get('text/plain') or out.get('text') or [])
                blocks.append(f'<pre class="out">{html.escape(text)}</pre>')
            elif out.get('output_type') == 'error':
                blocks.append(f'<pre class="out err">{html.escape(out.get("ename", ""))}: '
                              f'{html.escape(out.get("evalue", ""))}</pre>')  # fmt: skip
    return '\n'.join(blocks)


def view(rel: str, root: Path = ROOT) -> dict[str, Any] | None:
    """Return how the page shows a file: its kind, and its text or HTML when it is small."""
    path = resolve(rel, root)
    if path is None or not path.is_file():
        return None
    size = path.stat().st_size
    kind = kind_of(path)
    out: dict[str, Any] = {'name': path.name, 'kind': kind, 'size': size}
    if kind in ('image', 'binary') or size > VIEW_MAX_BYTES:
        if size > VIEW_MAX_BYTES and kind != 'image':
            out['kind'] = 'binary'
        return out
    text = path.read_text(errors='replace')
    if kind == 'notebook':
        try:
            out['html'] = notebook_html(json.loads(text))
        except ValueError:
            out.update(kind='text', text=text)
    elif kind == 'markdown':
        out['html'] = handbook.render(text, link=safe_link).html
    else:
        out.update(text=text, language=LANGUAGES.get(path.suffix.lower(), ''))
    return out


def unit_states(names: list[str]) -> dict[str, str]:
    if not names:
        return {}
    out = subprocess.run(
        [*units.systemctl_argv(), 'is-active', *names], capture_output=True, text=True, check=False
    )
    return dict(zip(names, out.stdout.split(), strict=False))


def pages_view() -> list[dict[str, Any]]:
    """Return every page the car serves: name, row, name path, port, and whether it runs."""
    home = apps.registry()['home']
    listed = [{'id': 'home', 'name': 'Home', 'group': 'core', 'unit': home['unit'],
               'path': home['path'], 'port': home['port']}]  # fmt: skip
    listed += [
        {k: a[k] for k in ('id', 'name', 'group', 'unit', 'path', 'port')} for a in apps.all_apps()
    ]
    states = unit_states([p['unit'] for p in listed if p['unit']])
    for p in listed:
        p['state'] = states.get(p['unit'] or '', 'unknown')
    return listed


class Handler(webui.AppHandler):
    cache = 'no-store'

    def do_GET(self) -> None:  # noqa: N802
        if self.serve_asset():
            return
        url = urlparse(self.path)
        q = {k: v[0] for k, v in parse_qs(url.query).items()}
        rel = q.get('path', '')
        if url.path in ('/', '/index.html'):
            self.page(PAGE)
        elif url.path == '/api/pages':
            self.send_json({'pages': pages_view()})
        elif url.path == '/api/list':
            found = listing(rel)
            if found is None:
                self.send_json({'message': 'No such folder.'}, 404)
            else:
                self.send_json(found)
        elif url.path == '/api/view':
            found = view(rel)
            if found is None:
                self.send_json({'message': 'No such file.'}, 404)
            else:
                self.send_json(found)
        elif url.path == '/api/raw':
            self._raw(rel, q.get('download') == '1')
        else:
            self.send(404, b'not found', 'text/plain')

    def _raw(self, rel: str, download: bool) -> None:
        path = resolve(rel)
        if path is None or not path.is_file():
            self.send(404, b'No such file.', 'text/plain')
            return
        kind = mimetypes.guess_type(path.name)[0] or 'application/octet-stream'
        headers = {}
        if download or kind_of(path) == 'binary':
            headers['Content-Disposition'] = f"attachment; filename*=UTF-8''{quote(path.name)}"
        # SVG and HTML open as plain text, so a file in the workspace never runs as a page here.
        if kind in ('image/svg+xml', 'text/html'):
            kind = 'text/plain' if kind == 'text/html' else kind
            headers['Content-Security-Policy'] = "default-src 'none'; style-src 'unsafe-inline'"
        self.send(200, path.read_bytes(), kind, headers=headers)


def main() -> int:
    print(f'files: http://0.0.0.0:{port()}', flush=True)
    webui.serve(port(), Handler)
    return 0


if __name__ == '__main__':
    sys.exit(main())
