/* Stall banner for every page. Served at /ui/stall.js.

   Asks the home page on 8080 for the firmware's stall protection state each
   second and shows a banner while the throttle is held at neutral: tripped
   until the throttle returns to neutral, then the cooldown or lockout with
   its seconds counting down on this page. */
(function () {
  'use strict';
  const PERIOD_MS = 1000;
  const url = RcnRoute.app('home', 'api/stall');
  let stall = null;
  let deadline = null;
  let banner = null;

  function build() {
    banner = document.createElement('div');
    banner.className = 'rcn-override';
    banner.id = 'stall-banner';
    banner.setAttribute('role', 'alert');
    banner.hidden = true;
    banner.innerHTML = '<svg class="rcn-icon"><use href="ui/icons.svg?v={{asset}}#alert"/></svg><span></span>';
    const main = document.querySelector('main');
    if (main) main.prepend(banner); else document.body.prepend(banner);
  }

  function render() {
    const state = stall && stall.state;
    let text = '';
    const left = deadline === null ? 0 : Math.max(0, Math.ceil((deadline - performance.now()) / 1000));
    if (state === 'tripped') text = 'Car stalled. Return the throttle stick to neutral.';
    else if (state === 'cooldown') text = `Car stalled. Driving resumes in ${left} s.`;
    else if (state === 'lockout') text = `Car stalled and locked out. Driving resumes in ${left} s.`;
    banner.querySelector('span').textContent = text;
    banner.classList.toggle('warn', state === 'tripped' || state === 'cooldown');
    banner.hidden = !text;
  }

  async function poll() {
    try {
      stall = (await (await fetch(url)).json()).stall || null;
    } catch (err) {
      // 8080 unreachable: nothing to show.
      stall = null;
    }
    if (stall && stall.seconds_left !== null && stall.seconds_left !== undefined) {
      const at = performance.now() + stall.seconds_left * 1000;
      if (deadline === null || Math.abs(at - deadline) > 1500) deadline = at;
    } else deadline = null;
    render();
  }

  build();
  poll();
  setInterval(poll, PERIOD_MS);
  setInterval(render, 250);
})();
