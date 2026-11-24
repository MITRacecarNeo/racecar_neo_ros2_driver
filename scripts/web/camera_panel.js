/* The color camera beside a map, for the Mapper and the Planner. Served at
   /ui/camera_panel.js by scripts/webui.py; styles are .rcn-camera in rcn.css.

   const panel = RcnCamera.mount(el, { key })
     fills `el` with a panel headed Camera that Show and Hide open and close
     (remembered under `key`, hidden at first). A tap on the picture enlarges
     it, another shrinks it.
   panel.setEnabled(on)   grayed while the camera is off

   Frames come from 8080 (api/camera.jpg), one request in flight, only while
   the panel is open and the page is in front. The picture grays out when the
   last good frame was asked for over STALE_MS ago, so a slow network grays it too. */
(function () {
  'use strict';
  const PERIOD_MS = 120;
  const RETRY_MS = 200;
  const STALE_MS = 700;  // plus STALE_SEC in scripts/overview.py: one second in all
  const GIVE_UP_MS = 3000;

  function mount(el, { key }) {
    el.classList.add('rcn-camera');
    el.innerHTML = `<div class="chead"><b>Camera</b>
      <button class="rcn-btn quiet small" type="button" aria-label="Show or hide the camera"></button></div>
      <div class="cpic" role="button" tabindex="0"><img alt="Color camera"></div>`;
    const toggle = el.querySelector('button'), pic = el.querySelector('.cpic');
    let shown = false, enabled = true, last = 0, request = 0;

    const live = () => shown && enabled && !document.hidden;
    function paint() {
      el.classList.toggle('stale', live() && Date.now() - last > STALE_MS);
    }
    function next(id) {
      if (id !== request || !live()) return;
      const started = Date.now(), img = new Image();
      img.alt = 'Color camera';
      const again = (ms) => { if (id === request) setTimeout(() => next(id), ms); };
      img.onload = () => {
        if (id !== request) return;
        const took = Date.now() - started;
        if (took <= STALE_MS) { pic.replaceChildren(img); last = started; paint(); }
        again(Math.max(20, PERIOD_MS - took));
      };
      img.onerror = () => again(RETRY_MS);
      img.src = RcnRoute.app('home', `api/camera.jpg?t=${started}`);
      // A request that never ends must not hold the picture still for good.
      setTimeout(() => { if (id === request && !img.complete) restart(); }, GIVE_UP_MS);
    }
    function restart() { request += 1; if (live()) next(request); }

    function setShown(on) {
      shown = on;
      el.classList.toggle('folded', !on);
      if (!on) setBig(false);
      toggle.textContent = on ? 'Hide' : 'Show';
      RcnUi.store.set(key, on ? '1' : '0');
      last = Date.now();
      paint(); restart();
    }
    function setBig(on) {
      el.classList.toggle('big', on);
      pic.setAttribute('aria-label', on ? 'Shrink the picture' : 'Enlarge the picture');
    }
    toggle.onclick = () => setShown(!shown);
    pic.onclick = () => setBig(!el.classList.contains('big'));
    pic.onkeydown = (e) => { if (e.key === 'Enter') setBig(!el.classList.contains('big')); };
    document.addEventListener('visibilitychange', () => { paint(); restart(); });
    setInterval(paint, 100);
    setBig(false);
    setShown(RcnUi.store.get(key, '0') === '1');

    return {
      setEnabled(on) {
        if (on === enabled) return;
        enabled = on;
        el.classList.toggle('off', !on);
        toggle.disabled = !on;
        el.title = on ? '' : 'The camera is off';
        last = Date.now();
        paint(); restart();
      },
    };
  }

  window.RcnCamera = { mount };
})();
