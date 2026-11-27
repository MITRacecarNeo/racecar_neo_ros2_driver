/* One on-screen joystick, for touch or mouse. Served at /ui/joystick.js by
   scripts/webui.py.

   RcnJoystick.mount(el, { size, deadzone, onMove })
     draws a round base with a knob inside `el` and returns { stop }.
     onMove(x, y) gets x (right +) and y (up +), each -1..1, linear outside
     the dead zone (a share of the reach, default 0.1). The knob springs to
     the center on release, on a cancelled touch, when the page is hidden or
     loses focus, and onMove(0, 0) is sent then.

   RcnJoystick.shape(v, deadzone) is the per-axis mapping, for tests. */
(function () {
  'use strict';

  function shape(v, deadzone) {
    const a = Math.abs(v);
    if (a <= deadzone) return 0;
    return Math.sign(v) * Math.min(1, (a - deadzone) / (1 - deadzone));
  }

  function mount(el, options) {
    const size = (options && options.size) || 180;
    const deadzone = options && options.deadzone !== undefined ? options.deadzone : 0.1;
    const onMove = (options && options.onMove) || (() => {});
    const knobSize = Math.round(size * 0.38);
    const reach = (size - knobSize) / 2;
    el.innerHTML = '';
    const base = document.createElement('div');
    base.className = 'rcn-joy';
    base.style.cssText = `position:relative;width:${size}px;height:${size}px;border-radius:50%;`
      + 'background:var(--rcn-code-bg);border:2px solid var(--rcn-border);touch-action:none;'
      + 'user-select:none;-webkit-user-select:none;margin:0 auto;';
    const knob = document.createElement('div');
    knob.style.cssText = `position:absolute;width:${knobSize}px;height:${knobSize}px;border-radius:50%;`
      + `left:${(size - knobSize) / 2}px;top:${(size - knobSize) / 2}px;background:var(--rcn-accent);`
      + 'box-shadow:0 2px 6px rgba(0,0,0,.4);pointer-events:none;';
    base.append(knob);
    el.append(base);

    let pointer = null;
    function place(dx, dy) {
      knob.style.transform = `translate(${dx}px, ${dy}px)`;
    }
    function move(e) {
      const rect = base.getBoundingClientRect();
      let dx = e.clientX - (rect.left + rect.width / 2);
      let dy = e.clientY - (rect.top + rect.height / 2);
      const d = Math.hypot(dx, dy);
      if (d > reach) { dx *= reach / d; dy *= reach / d; }
      place(dx, dy);
      onMove(shape(dx / reach, deadzone), shape(-dy / reach, deadzone));
    }
    function stop() {
      if (pointer === null) return;
      pointer = null;
      place(0, 0);
      onMove(0, 0);
    }
    base.addEventListener('pointerdown', (e) => {
      e.preventDefault();
      pointer = e.pointerId;
      base.setPointerCapture(e.pointerId);
      move(e);
    });
    base.addEventListener('pointermove', (e) => { if (e.pointerId === pointer) move(e); });
    for (const ev of ['pointerup', 'pointercancel', 'lostpointercapture']) {
      base.addEventListener(ev, (e) => { if (e.pointerId === pointer) stop(); });
    }
    base.addEventListener('contextmenu', (e) => e.preventDefault());
    window.addEventListener('blur', stop);
    document.addEventListener('visibilitychange', () => { if (document.hidden) stop(); });
    return { stop };
  }

  window.RcnJoystick = { mount, shape };
})();
