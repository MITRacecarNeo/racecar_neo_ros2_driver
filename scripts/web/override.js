/* Safety-override banner for 8080 and the pages that drive the car. Served at /ui/override.js.

   The override is turned on from the apps page on 8080 and lapses 2 s after
   the last keepalive. Every page that loads this file sends one each second,
   so the override stays on while any of them is open, and shows the banner
   with Turn off while it is on. A keepalive never turns the override on.

   RcnOverride.send('start' | 'stop' | 'keep') posts an action;
   RcnOverride.onChange(o) is called with each reply, and RcnOverride.onLost
   when a start or stop cannot reach 8080 (the 8080 tile and messages). */
(function () {
  'use strict';
  const PERIOD_MS = 1000;
  const url = RcnRoute.app('home', 'api/override');
  const ON = 'Safety override is on. The car may move without warning.';
  let epoch = 0;  // Replies to requests sent before the last start or stop are ignored.
  let banner = null;

  const api = { onChange: null, onLost: null, send };
  window.RcnOverride = api;

  function build() {
    banner = document.createElement('div');
    banner.className = 'rcn-override';
    banner.id = 'override-banner';
    banner.setAttribute('role', 'alert');
    banner.hidden = true;
    banner.innerHTML = `<svg class="rcn-icon"><use href="ui/icons.svg?v={{asset}}#alert"/></svg>
      <span>${ON}</span>
      <button class="rcn-btn small" type="button">Turn off</button>`;
    banner.querySelector('button').onclick = () => send('stop');
    // A page whose main is covered (a map) names the place with #override-slot.
    const slot = document.getElementById('override-slot') || document.querySelector('main');
    (slot || document.body).prepend(banner);
  }

  function render(o) {
    const on = !!(o && o.on);
    // Until the car confirms, the page that started it grays its tile; the banner waits.
    banner.hidden = !on || o.confirmed === false;
    if (api.onChange) api.onChange(o || { on: false });
  }

  async function send(action) {
    const mine = action === 'keep' ? epoch : ++epoch;
    try {
      const r = await fetch(url, { method: 'POST', body: JSON.stringify({ action }) });
      const o = await r.json();
      if (mine === epoch) render(o);
    } catch (err) {
      // 8080 unreachable: the override lapses on the car, so hide the banner.
      if (mine === epoch) render(null);
      if (action !== 'keep' && api.onLost) api.onLost();
    }
  }

  build();
  send('keep');
  setInterval(() => send('keep'), PERIOD_MS);
})();
