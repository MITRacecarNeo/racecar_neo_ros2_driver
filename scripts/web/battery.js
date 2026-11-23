/* Low-battery banner for every page. Served at /ui/battery.js.

   Asks the home page on 8080 for the battery guard's estimate every 3 s and
   shows a banner while the battery is low, counting down each second during
   a shutdown countdown. The count runs on this page from shutdown_in, so it
   does not depend on the poll or on the car's clock.

   The 8080 page (<body data-home>) polls the estimate itself, so this file
   does not: it passes each estimate in with RcnBattery.update(), sets
   RcnBattery.onCancel to show Cancel shutdown, and calls RcnBattery.final()
   once Shut Down is accepted. */
(function () {
  'use strict';
  const PERIOD_MS = 3000;
  const url = `${location.protocol}//${location.hostname}:{{home}}/api/battery`;
  const FINAL = 'Turning the car off now. Turn off the power switch.';
  let estimate = null;
  let deadline = null;
  let turningOff = false;
  let banner = null;
  let cancel = null;

  const api = { onCancel: null, update, final };
  window.RcnBattery = api;

  function build() {
    banner = document.createElement('div');
    banner.className = 'rcn-override';
    banner.id = 'battery-banner';
    banner.setAttribute('role', 'alert');
    banner.hidden = true;
    banner.innerHTML = '<svg class="rcn-icon"><use href="/ui/icons.svg?v={{asset}}#battery"/></svg><span></span>'
      + '<button class="rcn-btn small" type="button" hidden>Cancel shutdown</button>';
    cancel = banner.querySelector('button');
    cancel.onclick = () => { if (api.onCancel) api.onCancel(); };
    const main = document.querySelector('main');
    if (main) main.prepend(banner); else document.body.prepend(banner);
  }

  function render() {
    const e = estimate;
    let text = '';
    if (turningOff || (e && e.level === 'off')) text = FINAL;
    else if (e && e.level === 'countdown' && deadline !== null) {
      text = `Shutting down in ${Math.max(0, Math.ceil((deadline - performance.now()) / 1000))} s.`;
    } else if (e && e.level === 'warn') {
      text = e.minutes_left !== null
        ? `Low battery: ${Math.round(e.minutes_left)} min left. Charge or swap the battery.`
        : 'Low battery: Charge or swap the battery.';
    }
    banner.querySelector('span').textContent = text;
    cancel.hidden = !(api.onCancel && e && e.can_cancel && !turningOff);
    banner.hidden = !text;
  }

  function update(e) {
    estimate = e || null;
    if (estimate && estimate.level === 'countdown' && estimate.shutdown_in !== null) {
      const at = performance.now() + estimate.shutdown_in * 1000;
      // Keep the running count unless the car's figure differs by more than a tick.
      if (deadline === null || Math.abs(at - deadline) > 1500) deadline = at;
    } else deadline = null;
    render();
  }

  function final() {
    turningOff = true;
    render();
  }

  async function poll() {
    let e = null;
    try {
      const b = (await (await fetch(url)).json()).battery;
      e = (b && b.estimate) || null;
    } catch (err) {
      // 8080 unreachable: no estimate to show.
    }
    update(e);
  }

  build();
  setInterval(render, 250);
  if (!document.body.hasAttribute('data-home')) {
    poll();
    setInterval(poll, PERIOD_MS);
  }
})();
