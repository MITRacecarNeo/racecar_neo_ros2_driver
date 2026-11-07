/* Home shortcut for every page header. Served at /ui/home.js and added to
   each page by scripts/webui.py.

   The RACECAR Neo mark in the header becomes a link: it closes this tab,
   which leaves the 8080 home page that opened it in front. A tab the browser
   will not close (opened from a bookmark or a typed address) loads the home
   page instead. Left alone on the home page (its <body> has data-home). */
(function () {
  'use strict';
  const home = `${location.protocol}//${location.hostname}:{{home}}/`;
  if (document.body.hasAttribute('data-home')) return;
  const mark = document.querySelector('.rcn-header .rcn-mark');
  if (!mark) return;
  const link = document.createElement('a');
  link.className = 'rcn-home';
  link.href = home;
  link.title = 'Home';
  link.setAttribute('aria-label', 'Home');
  link.addEventListener('click', (event) => {
    event.preventDefault();
    window.close();
    setTimeout(() => { location.href = home; }, 300);
  });
  mark.replaceWith(link);
  link.append(mark);
})();
