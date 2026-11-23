/* Page footer for every page the car serves. Served at /ui/footer.js and
   added to each page by scripts/webui.py.

   Shows the car (hostname, and its nickname when it has one), the driver
   version (filled in when the script is sent), and the Teensy firmware
   version; the car and the firmware come from the home page on 8080, once a
   minute. On a
   page that is not a flex column the footer is pinned to the bottom edge
   (body.rcn-footed) and the body keeps room for it. */
(function () {
  'use strict';
  const PERIOD_MS = 60000;
  const url = `${location.protocol}//${location.hostname}:{{home}}/api/about`;
  const footer = document.createElement('footer');
  footer.className = 'rcn-footer';
  footer.innerHTML = '<span class="rcn-car"></span><span>racecar OS</span>'
    + '<span>Driver {{driver}}</span><span>Firmware <b>...</b></span>';
  const firmware = footer.querySelector('b');
  const car = footer.querySelector('.rcn-car');
  document.body.append(footer);

  if (getComputedStyle(document.body).display !== 'flex') {
    document.body.classList.add('rcn-footed');
    const room = () => { document.body.style.paddingBottom = `${footer.offsetHeight}px`; };
    room();
    window.addEventListener('resize', room);
  }

  async function poll() {
    try {
      const about = await (await fetch(url)).json();
      firmware.textContent = about.firmware || 'unknown';
      if (about.car) car.textContent = about.car;
    } catch (err) {
      // 8080 unreachable: keep the last value.
    }
  }

  poll();
  setInterval(poll, PERIOD_MS);
})();
