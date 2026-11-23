/* Menu of the Car Profile app. Served at /ui/profile_nav.js by
   scripts/webui.py.

   RcnProfileNav.mount(current, sub)
     adds the menu button at the right end of the header and the menu on the
     right side of the page. `current` is the page id (home, handbook, setup,
     calibration, checks); `sub` is the handbook page shown, if any. The
     handbook pages (/api/handbook) are listed under Handbook.

   The menu starts closed on narrow screens and open on wide ones; the choice
   is kept per browser. */
(function () {
  'use strict';
  const KEY = 'rcn-profile-nav';
  const WIDE = '(min-width: 900px)';
  const PAGES = [
    { id: 'home', label: 'Car', href: '/', icon: 'car' },
    { id: 'handbook', label: 'Handbook', href: '/handbook/index', icon: 'book' },
    { id: 'setup', label: 'Setup', href: '/setup', icon: 'check' },
    { id: 'calibration', label: 'Calibration', href: '/calibration', icon: 'target' },
    { id: 'checks', label: 'Checks', href: '/checks', icon: 'pulse' },
  ];

  function stored() {
    try { return localStorage.getItem(KEY); } catch (e) { return null; }
  }
  function store(value) {
    try { localStorage.setItem(KEY, value); } catch (e) { /* private window */ }
  }
  function icon(name) {
    return `<svg class="rcn-icon"><use href="/ui/icons.svg?v={{asset}}#${name}"/></svg>`;
  }
  function esc(s) {
    return String(s).replace(/[&<>"]/g, (c) => ({ '&': '&amp;', '<': '&lt;', '>': '&gt;', '"': '&quot;' }[c]));
  }

  function mount(current, sub) {
    const style = document.createElement('style');
    style.textContent = `
      .pnav { position: fixed; top: 0; bottom: 0; right: 0; width: 240px; z-index: 20;
              background: var(--rcn-surface); border-left: 1px solid var(--rcn-border);
              transform: translateX(100%); transition: transform 0.15s; overflow-y: auto;
              padding: var(--rcn-space-4) var(--rcn-space-3); }
      body.pnav-open .pnav { transform: none; }
      .pnav .top { display: flex; align-items: center; justify-content: space-between;
                   margin-bottom: var(--rcn-space-3); }
      .pnav a, .pnav span.off { display: flex; align-items: center; gap: var(--rcn-space-2);
                   padding: var(--rcn-space-2) var(--rcn-space-3); border-radius: var(--rcn-radius-md);
                   color: var(--rcn-text); text-decoration: none; font-weight: 600; }
      .pnav a:hover { background: var(--rcn-steel); }
      .pnav a.on { background: var(--rcn-accent); color: var(--rcn-on-accent); }
      .pnav span.off { opacity: 0.45; }
      .pnav .children a { font-weight: 500; padding-left: 44px; }
      @media (min-width: 900px) {
        body.pnav-open { padding-right: 240px; }
        body.pnav-open .pnav-close { display: none; }
      }`;
    document.head.append(style);
    const nav = document.createElement('nav');
    nav.className = 'pnav';
    nav.setAttribute('aria-label', 'Car Profile');
    nav.innerHTML = `<div class="top"><b>Car Profile</b>
      <button class="rcn-btn quiet small pnav-close" aria-label="Close menu">${icon('cross')}</button></div>`
      + PAGES.map((p) => {
        const on = p.id === current && !(p.id === 'handbook' && sub && sub !== 'index');
        const link = p.href
          ? `<a href="${p.href}"${on ? ' class="on"' : ''}>${icon(p.icon)}${p.label}</a>`
          : `<span class="off">${icon(p.icon)}${p.label}</span>`;
        return link + (p.id === 'handbook' ? '<div class="children" id="pnav-handbook"></div>' : '');
      }).join('');
    document.body.append(nav);

    fetch('/api/handbook').then((r) => r.json()).then((d) => {
      nav.querySelector('#pnav-handbook').innerHTML = d.pages
        .filter((p) => p.name !== 'index')
        .map((p) => `<a href="/handbook/${esc(p.name)}"${p.name === sub ? ' class="on"' : ''}>${esc(p.title)}</a>`)
        .join('');
    }).catch(() => {});

    const toggle = document.createElement('button');
    toggle.className = 'rcn-btn quiet small pnav-menu';
    toggle.setAttribute('aria-label', 'Menu');
    toggle.innerHTML = icon('menu');
    const header = document.querySelector('.rcn-header');
    if (header) header.append(toggle);

    const saved = stored();
    const open = saved === null ? window.matchMedia(WIDE).matches : saved === 'open';
    document.body.classList.toggle('pnav-open', open);
    function set(value) {
      document.body.classList.toggle('pnav-open', value);
      store(value ? 'open' : 'closed');
      window.dispatchEvent(new Event('resize'));
    }
    toggle.addEventListener('click', () => set(!document.body.classList.contains('pnav-open')));
    nav.querySelector('.pnav-close').addEventListener('click', () => set(false));
  }

  window.RcnProfileNav = { mount };
})();
