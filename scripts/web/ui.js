/* Page helpers shared by the pages the car serves. Served at /ui/ui.js by
   scripts/webui.py; load it before the page's own script.

   RcnUi.$(id), esc(text), icon(name, cls), pill(kind, word)
   RcnUi.setHtml(id, html)   touches the DOM only when the markup changes
   RcnUi.store.get(key, d), store.set(key, v)   localStorage, never throws
   RcnUi.token(name)         a CSS custom property, for drawing on a canvas
   RcnUi.font(size, weight)  the sans font at a size, for a canvas
   RcnUi.toast(text)         a short confirmation at the bottom of the page
   RcnUi.frames(img, path, ms)   live camera frames, one request in flight */
(function () {
  'use strict';
  const $ = (id) => document.getElementById(id);
  const esc = (s) => String(s).replace(/[&<>"]/g, (c) => ({ '&': '&amp;', '<': '&lt;', '>': '&gt;', '"': '&quot;' }[c]));
  const icon = (name, cls = 'rcn-icon') => `<svg class="${cls}"><use href="/ui/icons.svg#${name}"/></svg>`;
  const STATUS_ICON = { ok: 'check', warn: 'alert', err: 'cross', off: 'info', info: 'info' };
  // kind: ok | warn | err | off, each with its icon and word.
  const pill = (kind, word) => `<span class="rcn-status ${kind}">${icon(STATUS_ICON[kind])}${esc(word)}</span>`;

  const shown = new Map();
  function setHtml(id, html) {
    if (shown.get(id) === html) return;
    shown.set(id, html);
    $(id).innerHTML = html;
  }

  const store = {
    get: (k, d) => { try { return localStorage.getItem(k) ?? d; } catch (err) { return d; } },
    set: (k, v) => { try { localStorage.setItem(k, v); } catch (err) { /* not remembered */ } },
  };

  const token = (name) => getComputedStyle(document.documentElement).getPropertyValue(name).trim();
  const font = (size, weight = 600) => `${weight} ${size}px ${token('--rcn-font-sans') || 'Barlow, Arial, sans-serif'}`;

  let toastTimer = null;
  function toast(text) {
    let el = $('rcn-toast');
    if (!el) {
      el = document.createElement('div');
      el.id = 'rcn-toast';
      el.className = 'rcn-toast';
      el.setAttribute('role', 'status');
      document.body.append(el);
    }
    el.textContent = text;
    el.classList.add('show');
    clearTimeout(toastTimer);
    toastTimer = setTimeout(() => el.classList.remove('show'), 2200);
  }

  // A congested link lowers the frame rate instead of building a queue.
  // Pauses while the tab is hidden. img may be off the page (drawn by its onload).
  function frames(img, path, ms) {
    (function tick() {
      if (document.hidden) { setTimeout(tick, 250); return; }
      const t = Date.now();
      const done = () => {
        img.removeEventListener('load', done);
        img.removeEventListener('error', done);
        setTimeout(tick, Math.max(20, ms - (Date.now() - t)));
      };
      img.addEventListener('load', done);
      img.addEventListener('error', done);
      img.src = `${path}?${t}`;
    })();
  }

  window.RcnUi = { $, esc, icon, pill, STATUS_ICON, setHtml, store, token, font, toast, frames };
})();
