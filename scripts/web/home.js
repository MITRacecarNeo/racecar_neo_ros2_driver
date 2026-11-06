/* Home button for every page header. Served at /ui/home.js and added to
   each page by scripts/webui.py.

   Closes this tab, which leaves the 8080 home page that opened it in
   front. A tab the browser will not close (opened from a bookmark or a
   typed address) loads the home page instead. Not shown on the home page
   (its <body> has data-home). */
(function () {
  'use strict';
  const home = `${location.protocol}//${location.hostname}:{{home}}/`;
  if (document.body.hasAttribute('data-home')) return;
  const header = document.querySelector('.rcn-header');
  if (!header) return;
  const link = document.createElement('a');
  link.className = 'rcn-btn quiet small rcn-home';
  link.href = home;
  link.title = 'Close this tab and go back to the home page';
  link.setAttribute('aria-label', 'Home');
  link.innerHTML = '<svg class="rcn-icon"><use href="/ui/icons.svg#home"/></svg>'
    + '<span class="label">Home</span>';
  link.addEventListener('click', (event) => {
    event.preventDefault();
    window.close();
    setTimeout(() => { location.href = home; }, 300);
  });
  const mark = header.querySelector('.rcn-mark');
  if (mark) mark.after(link); else header.prepend(link);
})();
