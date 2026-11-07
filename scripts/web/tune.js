/* Tuning sliders for the lab dashboards. Served at /ui/tune.js by
   scripts/webui.py; needs /ui/api.js. Styles are .rcn-slider in rcn.css.

   const form = RcnTune.sliders(el, specs)
     specs: [{key, label, tip, min, max, step, int}]; label defaults to key.
     form.sync(params)  first call builds the sliders; later calls move each
                        slider to what the app applied (it clamps), except
                        the one being dragged
     form.use(specs)    swap the sliders, for a page whose settings depend on a mode
   RcnTune.queue(key, value)  send one setting; a drag sends at most one
                        POST /params per SEND_MS, always the newest values */
(function () {
  'use strict';
  const SEND_MS = 60;
  const esc = (s) => String(s).replace(/[&<>"]/g, (c) => ({ '&': '&amp;', '<': '&lt;', '>': '&gt;', '"': '&quot;' }[c]));
  let pending = {}, timer = null;

  function queue(key, value) {
    pending[key] = value;
    if (timer) return;
    timer = setTimeout(() => {
      const body = pending;
      pending = {}; timer = null;
      RcnApi.post('/params', body);
    }, SEND_MS);
  }

  function sliders(el, specs) {
    let list = specs, built = false;
    const input = (s) => document.getElementById(`in_${s.key}`);
    const value = (s) => document.getElementById(`val_${s.key}`);

    function build(p) {
      el.innerHTML = list.map((s) => `<label class="rcn-slider" title="${esc(s.tip || '')}">${esc(s.label || s.key)}
        <b id="val_${s.key}">${p[s.key]}</b>
        <input id="in_${s.key}" type="range" min="${s.min}" max="${s.max}" step="${s.step}" value="${p[s.key]}"></label>`).join('');
      for (const s of list) {
        input(s).oninput = (e) => {
          value(s).textContent = e.target.value;
          queue(s.key, s.int ? parseInt(e.target.value) : parseFloat(e.target.value));
        };
      }
      built = true;
    }

    function sync(p) {
      if (!built) { build(p); return; }
      if (document.querySelector('input:active')) return;
      for (const s of list) {
        const i = input(s);
        if (i && parseFloat(i.value) !== p[s.key]) { i.value = p[s.key]; value(s).textContent = p[s.key]; }
      }
    }

    function use(next) {
      if (next === list) return;
      list = next; built = false;
    }

    return { sync, use };
  }

  window.RcnTune = { sliders, queue };
})();
