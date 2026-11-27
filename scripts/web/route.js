/* Addresses of the car's apps. Served at /ui/route.js and loaded first in
   every page's head by scripts/webui.py.

   RcnRoute.app(id, sub)   the address of app `id` (home, profile, mapview, ...)
                           with `sub` after it. A page reached through the
                           car's web server (port 80) gets the name address,
                           http://<car>/<path>/sub; a page reached on a port
                           gets the port address, http://<car>:<port>/sub.
   RcnRoute.named          whether this page came through port 80.

   The paths and ports come from config/apps.yaml, filled in when the script
   is sent. */
(function () {
  'use strict';
  const ROUTES = {{routes}};
  const named = location.port === '' || location.port === '80' || location.port === '443';

  function app(id, sub) {
    const [path, port, prefixed] = ROUTES[id];
    const rest = (sub || '').replace(/^\/+/, '');
    if (named) return `${location.protocol}//${location.host}/${path}/${rest}`;
    return `${location.protocol}//${location.hostname}:${port}/${prefixed ? `${path}/` : ''}${rest}`;
  }

  window.RcnRoute = { app, named };
})();
