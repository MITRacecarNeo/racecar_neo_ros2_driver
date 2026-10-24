/* Safety-override banner for the driving lab pages. Served at /ui/override.js.

   The override is turned on from the apps page on 8080 and lapses 2 s after
   the last keepalive. Every page that loads this file sends one each second,
   so the override stays on while any of them is open, and shows the banner
   with Turn off while it is on. A keepalive never turns the override on. */
(function () {
  'use strict';
  const PERIOD_MS = 1000;
  const url = `${location.protocol}//${location.hostname}:8080/api/override`;
  let epoch = 0;
  let banner = null;

  function build() {
    banner = document.createElement('div');
    banner.className = 'rcn-override';
    banner.setAttribute('role', 'alert');
    banner.hidden = true;
    banner.innerHTML = `<svg class="rcn-icon"><use href="/ui/icons.svg#alert"/></svg>
      <span>Safety switch override is on. The car may move without warning. Do not leave it unattended.</span>
      <button class="rcn-btn small" type="button">Turn off</button>`;
    banner.querySelector('button').onclick = () => send('stop');
    const main = document.querySelector('main');
    if (main) main.prepend(banner); else document.body.prepend(banner);
  }

  async function send(action) {
    const mine = action === 'keep' ? epoch : ++epoch;
    try {
      const r = await fetch(url, { method: 'POST', body: JSON.stringify({ action }) });
      const o = await r.json();
      if (mine === epoch) banner.hidden = !o.on;
    } catch (err) {
      // 8080 unreachable: the override lapses on the car, so hide the banner.
      if (mine === epoch) banner.hidden = true;
    }
  }

  build();
  send('keep');
  setInterval(() => send('keep'), PERIOD_MS);
})();
