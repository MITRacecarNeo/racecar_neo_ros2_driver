"""
JupyterLab server extension: the port's old addresses lead under /jupyter/.

JupyterLab serves under /jupyter/ (scripts/racecar-jupyter.service,
`--ServerApp.base_url`) so the car's web server can forward /jupyter/ to it.
Links to http://<car>:8888/ and http://<car>:8888/lab... still open it. The
unit puts this folder on PYTHONPATH and enables the extension by name.
"""

from __future__ import annotations

from typing import Any

from tornado import web

BASE = '/jupyter/'
OLD_PATHS = r'/((?:lab|tree|notebooks|edit|files)(?:/.*)?)?'


class _Redirect(web.RequestHandler):
    def get(self, rest: str | None = None) -> None:
        query = f'?{self.request.query}' if self.request.query else ''
        self.redirect(f'{BASE}{rest or ""}{query}')


def _jupyter_server_extension_points() -> list[dict[str, str]]:
    return [{'module': __name__}]


def _load_jupyter_server_extension(serverapp: Any) -> None:
    if serverapp.base_url == BASE:
        serverapp.web_app.add_handlers('.*$', [(OLD_PATHS, _Redirect)])
