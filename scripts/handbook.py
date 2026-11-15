#!/usr/bin/env python3
"""
Handbook renderer: the Markdown pages in docs/handbook/ as HTML.

A small Markdown subset with no dependencies, so the car renders the pages
offline and the static build needs nothing installed:

- headings `#` to `####`, with an optional fixed anchor `{#id}` at the end;
  without one the anchor is the GitHub slug of the heading text
- paragraphs, `-` and `1.` lists (one nested level, two or more spaces in)
- fenced code blocks, tables with a header separator row
- inline code, **bold**, *emphasis* or _emphasis_, links, images

Anything else (raw HTML, block quotes, deeper headings) is a problem the
tests reject, so a page never renders differently from its source.

`racecar handbook build [DIR]` writes every page as static HTML.
"""

from __future__ import annotations

import argparse
from collections.abc import Callable
from dataclasses import dataclass, field
import html
from pathlib import Path
import re
import sys

HANDBOOK_DIR = Path(__file__).resolve().parent.parent / 'docs' / 'handbook'
IMG_DIR = HANDBOOK_DIR.parent / 'img'
CSS = Path(__file__).resolve().parent / 'web' / 'rcn.css'
# Driver docs the handbook links to; the static build includes them as pages.
LINKED_DOCS = {'../troubleshooting.md': HANDBOOK_DIR.parent / 'troubleshooting.md'}

HEADING = re.compile(r'^(#{1,6})\s+(.*?)\s*$')
ANCHOR = re.compile(r'\s*\{#([a-z0-9][a-z0-9_-]*)\}$')
FENCE = re.compile(r'^```([\w-]*)\s*$')
BULLET = re.compile(r'^(\s*)-\s+(.*)$')
NUMBERED = re.compile(r'^(\s*)\d+\.\s+(.*)$')
TABLE_RULE = re.compile(r'^\|?\s*:?-+:?\s*(\|\s*:?-+:?\s*)*\|?\s*$')
INLINE = re.compile(
    r'`(?P<code>[^`]+)`'
    r'|!\[(?P<alt>[^\]]*)\]\((?P<src>[^)\s]+)\)'
    r'|\[(?P<text>[^\]]+)\]\((?P<href>[^)\s]+)\)'
    r'|\*\*(?P<bold>[^*]+)\*\*'
    r'|(?<![\w*])\*(?P<em>[^*\s][^*]*)\*(?![\w*])'
    r'|(?<![\w_])_(?P<em2>[^_\s][^_]*)_(?![\w_])'
)


@dataclass
class Page:
    title: str = ''
    html: str = ''
    anchors: list[str] = field(default_factory=list)
    links: list[str] = field(default_factory=list)
    images: list[str] = field(default_factory=list)
    problems: list[str] = field(default_factory=list)


def slug(text: str) -> str:
    """Return the GitHub anchor of a heading."""
    plain = re.sub(r'[`*_]|\[([^\]]*)\]\([^)]*\)', r'\1', text).strip().lower()
    return re.sub(r'\s', '-', re.sub(r'[^\w\s-]', '', plain))


def page_link(target: str, suffix: str = '') -> str:
    """Point a link to another handbook page at its rendered form."""
    path, _, frag = target.partition('#')
    if path in LINKED_DOCS and suffix:
        path = Path(path).name
    if not path.endswith('.md') or '/' in path or ':' in path:
        return target
    name = path[:-3] + suffix
    return name + ('#' + frag if frag else '')


class Renderer:
    def __init__(self, link: Callable[[str], str], image: Callable[[str], str]) -> None:
        self.link = link
        self.image = image
        self.page = Page()

    def inline(self, text: str) -> str:
        out, pos = [], 0
        for m in INLINE.finditer(text):
            start = m.start()
            out.append(html.escape(text[pos:start], quote=False))
            pos = m.end()
            if m['code'] is not None:
                out.append(f'<code>{html.escape(m["code"], quote=False)}</code>')
            elif m['src'] is not None:
                self.page.images.append(m['src'])
                src = html.escape(self.image(m['src']))
                out.append(f'<img src="{src}" alt="{html.escape(m["alt"])}">')
            elif m['href'] is not None:
                self.page.links.append(m['href'])
                href = html.escape(self.link(m['href']))
                out.append(f'<a href="{href}">{self.inline(m["text"])}</a>')
            elif m['bold'] is not None:
                out.append(f'<strong>{self.inline(m["bold"])}</strong>')
            else:
                out.append(f'<em>{self.inline(m["em"] or m["em2"])}</em>')
        out.append(html.escape(text[pos:], quote=False))
        return ''.join(out)

    def heading(self, level: int, text: str, line: int) -> str:
        m = ANCHOR.search(text)
        if m:
            anchor, text = m.group(1), text[: m.start()]
        else:
            anchor = slug(text)
        if anchor in self.page.anchors:
            self.page.problems.append(f'line {line}: anchor #{anchor} used twice')
        self.page.anchors.append(anchor)
        if level == 1 and not self.page.title:
            self.page.title = re.sub(r'[`*_]', '', text)
        return f'<h{level} id="{anchor}">{self.inline(text)}</h{level}>'

    def table(self, rows: list[str]) -> str:
        def cells(row: str) -> list[str]:
            return [c.strip() for c in row.strip().strip('|').split('|')]

        head = ''.join(f'<th>{self.inline(c)}</th>' for c in cells(rows[0]))
        body = ''.join(
            '<tr>' + ''.join(f'<td>{self.inline(c)}</td>' for c in cells(r)) + '</tr>'
            for r in rows[2:]
        )
        return f'<table><thead><tr>{head}</tr></thead><tbody>{body}</tbody></table>'

    def items(self, lines: list[tuple[int, str]]) -> str:
        """Render one list; `lines` are (line number, text) from its first item on."""
        first = lines[0][1]
        indent = len(first) - len(first.lstrip())
        tag = 'ol' if NUMBERED.match(first) else 'ul'
        out, item, nested = [f'<{tag}>'], [], []

        def close() -> None:
            if item or nested:
                text = self.inline(' '.join(item))
                sub = self.items(nested) if nested else ''
                out.append(f'<li>{text}{sub}</li>')

        for number, text in lines:
            m = BULLET.match(text) or NUMBERED.match(text)
            depth = len(text) - len(text.lstrip())
            if m and depth == indent:
                close()
                item, nested = [m.group(2)], []
            elif m and depth > indent:
                nested.append((number, text))
            elif nested:
                nested.append((number, text))
            else:
                item.append(text.strip())
        close()
        out.append(f'</{tag}>')
        return ''.join(out)

    def render(self, text: str) -> Page:
        blocks: list[str] = []
        lines = text.splitlines()
        i = 0
        while i < len(lines):
            line = lines[i]
            number = i + 1
            if not line.strip():
                i += 1
                continue
            fence = FENCE.match(line)
            if fence:
                end = i + 1
                while end < len(lines) and not lines[end].startswith('```'):
                    end += 1
                if end == len(lines):
                    self.page.problems.append(f'line {number}: code block not closed')
                first = i + 1
                code = html.escape('\n'.join(lines[first:end]), quote=False)
                blocks.append(f'<pre><code>{code}</code></pre>')
                i = end + 1
                continue
            heading = HEADING.match(line)
            if heading:
                level = len(heading.group(1))
                if level > 4:
                    self.page.problems.append(f'line {number}: heading deeper than ####')
                blocks.append(self.heading(min(level, 4), heading.group(2), number))
                i += 1
                continue
            if line.lstrip().startswith(('<', '>')):
                self.page.problems.append(f'line {number}: raw HTML or block quote')
                i += 1
                continue
            if line.startswith('|') and i + 1 < len(lines) and TABLE_RULE.match(lines[i + 1]):
                end = i
                while end < len(lines) and lines[end].startswith('|'):
                    end += 1
                blocks.append(self.table(lines[i:end]))
                i = end
                continue
            if BULLET.match(line) or NUMBERED.match(line):
                end = i
                while end < len(lines) and lines[end].strip():
                    nxt = lines[end + 1] if end + 1 < len(lines) else ''
                    end += 1
                    # A blank line inside a list continues it when the next line is
                    # another item or an indented continuation.
                    if not nxt.strip() and end + 1 < len(lines):
                        after = lines[end + 1]
                        if after.startswith((' ', '-')) or NUMBERED.match(after):
                            end += 1
                blocks.append(
                    self.items([(k + 1, lines[k]) for k in range(i, end) if lines[k].strip()])
                )
                i = end
                continue
            end = i
            while (
                end < len(lines)
                and lines[end].strip()
                and not HEADING.match(lines[end])
                and not FENCE.match(lines[end])
                and not (end > i and (BULLET.match(lines[end]) or NUMBERED.match(lines[end])))
            ):
                end += 1
            blocks.append(f'<p>{self.inline(" ".join(s.strip() for s in lines[i:end]))}</p>')
            i = end
        self.page.html = '\n'.join(blocks)
        return self.page


def render(
    text: str,
    link: Callable[[str], str] = page_link,
    image: Callable[[str], str] = lambda src: src,
) -> Page:
    return Renderer(link, image).render(text)


def pages(src: Path = HANDBOOK_DIR) -> list[Path]:
    """Return the handbook pages, index first."""
    files = sorted(src.glob('*.md'))
    return sorted(files, key=lambda p: p.name != 'index.md')


STATIC_PAGE = """<!doctype html>
<html lang="en">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>{title} - RACECAR Neo handbook</title>
<link rel="stylesheet" href="rcn.css">
<style>
  body {{ background: var(--rcn-bg); color: var(--rcn-text); margin: 0; }}
  main {{ max-width: 860px; margin: 0 auto; padding: 16px; }}
  nav {{ display: flex; flex-wrap: wrap; gap: 12px; padding: 12px 16px;
        border-bottom: 1px solid var(--rcn-border); }}
  table {{ border-collapse: collapse; display: block; overflow-x: auto; }}
  th, td {{ border: 1px solid var(--rcn-border); padding: 4px 8px; text-align: left; }}
  pre {{ overflow-x: auto; padding: 8px; background: var(--rcn-surface, #00000010); }}
  img {{ max-width: 100%; }}
</style>
</head>
<body>
<nav>{nav}</nav>
<main>
{body}
</main>
</body>
</html>
"""


def build(out: Path, src: Path = HANDBOOK_DIR) -> list[Path]:
    """Write every page as static HTML with the shared style and images."""
    out.mkdir(parents=True, exist_ok=True)
    files = [*pages(src), *(p for p in LINKED_DOCS.values() if p.exists())]
    rendered = {
        p.stem: render(p.read_text(), lambda t: page_link(t, '.html'), lambda s: Path(s).name)
        for p in files
    }
    nav = ' '.join(
        f'<a href="{name}.html">{html.escape(page.title or name)}</a>'
        for name, page in rendered.items()
    )
    images = {name for page in rendered.values() for name in page.images}
    written = []
    for name, page in rendered.items():
        path = out / f'{name}.html'
        path.write_text(
            STATIC_PAGE.format(title=html.escape(page.title or name), nav=nav, body=page.html)
        )
        written.append(path)
    if CSS.exists():
        (out / 'rcn.css').write_text(CSS.read_text())
    for name in sorted(images):
        image = IMG_DIR / Path(name).name
        if image.exists():
            (out / image.name).write_bytes(image.read_bytes())
    return written


def default_out() -> Path:
    sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
    from racecar_neo_ros2_driver import data_root

    return data_root.data_dir('handbook')


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(prog='racecar handbook', description=__doc__.split('\n')[1])
    sub = parser.add_subparsers(dest='cmd', required=True)
    b = sub.add_parser('build', help='write the handbook as static HTML')
    b.add_argument(
        'dir', nargs='?', type=Path, help='output folder (default: <data root>/handbook)'
    )
    args = parser.parse_args(argv)
    problems = [f'{p.name}: {msg}' for p in pages() for msg in render(p.read_text()).problems]
    if problems:
        print('\n'.join(problems), file=sys.stderr)
        return 1
    out = args.dir or default_out()
    written = build(out)
    print(f'{len(written)} pages written to {out}')
    return 0


if __name__ == '__main__':
    sys.exit(main())
