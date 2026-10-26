/* Low-battery banner for the app pages. Served at /ui/battery.js.

   Asks the home page on 8080 for the battery guard's estimate every 3 s and
   shows a banner while the battery is low, counting down each second during
   a shutdown countdown. The count runs on this page from shutdown_in, so it
   does not depend on the poll or on the car's clock. The cancel button is on
   the 8080 page only. */
(function () {
  'use strict';
  const PERIOD_MS = 3000;
  const url = `${location.protocol}//${location.hostname}:8080/api/battery`;
  const FINAL = 'Turning the car off now. Turn off the power switch.';
  let estimate = null;
  let deadline = null;
  let banner = null;

  function build() {
    banner = document.createElement('div');
    banner.className = 'rcn-override';
    banner.setAttribute('role', 'alert');
    banner.hidden = true;
    banner.innerHTML = '<svg class="rcn-icon"><use href="/ui/icons.svg#battery"/></svg><span></span>';
    const main = document.querySelector('main');
    if (main) main.prepend(banner); else document.body.prepend(banner);
  }

  function render() {
    const e = estimate;
    let text = '';
    if (e && e.level === 'off') text = FINAL;
    else if (e && e.level === 'countdown' && deadline !== null) {
      text = `Shutting down in ${Math.max(0, Math.ceil((deadline - performance.now()) / 1000))} s.`;
    } else if (e && e.level === 'warn') {
      text = e.minutes_left !== null
        ? `Low battery: ${Math.round(e.minutes_left)} min left. Charge or swap the battery.`
        : 'Low battery: Charge or swap the battery.';
    }
    banner.querySelector('span').textContent = text;
    banner.hidden = !text;
  }

  async function poll() {
    try {
      const b = (await (await fetch(url)).json()).battery;
      estimate = (b && b.estimate) || null;
    } catch (err) {
      // 8080 unreachable: no estimate to show.
      estimate = null;
    }
    if (estimate && estimate.level === 'countdown' && estimate.shutdown_in !== null) {
      const at = performance.now() + estimate.shutdown_in * 1000;
      if (deadline === null || Math.abs(at - deadline) > 1500) deadline = at;
    } else deadline = null;
    render();
  }

  build();
  poll();
  setInterval(poll, PERIOD_MS);
  setInterval(render, 250);
})();
