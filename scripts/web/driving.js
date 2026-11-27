/* Who drives, for the pages that drive the car. Served at /ui/driving.js.

   Asks the home page on 8080 each second who holds the drive lease
   (racecar_neo_ros2_driver/drive_lease.py) and shows "X is driving." while
   another source holds it. The page's own name is its <html data-app>.
   RcnDriving.blocked() is true meanwhile, so a Drive panel can gray out; a
   car held for the operator (a paused mission) leaves it free. On a page
   with a #driver pill (8080) the pill shows the holder instead. */
(function () {
  'use strict';
  const PERIOD_MS = 1000;
  const url = RcnRoute.app('home', 'api/driver');
  const self = document.documentElement.dataset.app || '';
  let driver = null;
  let banner = null;

  function other() {
    return !!driver && driver.name !== self;
  }

  function build() {
    banner = document.createElement('div');
    banner.className = 'rcn-override warn';
    banner.id = 'driving-banner';
    banner.setAttribute('role', 'status');
    banner.hidden = true;
    banner.innerHTML = '<svg class="rcn-icon"><use href="ui/icons.svg?v={{asset}}#alert"/></svg><span></span>';
    // A map page covers its main; the banner goes under the override banner there.
    const slot = document.getElementById('override-slot');
    const main = document.querySelector('main');
    if (slot) slot.append(banner); else if (main) main.prepend(banner); else document.body.prepend(banner);
  }

  async function poll() {
    try {
      driver = (await (await fetch(url)).json()).driver || null;
    } catch (err) {
      // 8080 unreachable: nothing to show.
      driver = null;
    }
    const pill = document.getElementById('driver');
    if (pill) {
      pill.textContent = driver ? `Driving: ${driver.name}` : '';
      pill.hidden = !driver;
      return;
    }
    const shown = other() && !driver.share;
    banner.querySelector('span').textContent = shown ? `${driver.name} is driving.` : '';
    banner.hidden = !shown;
  }

  window.RcnDriving = { blocked: () => other() && !driver.share };
  if (!document.getElementById('driver')) build();
  poll();
  setInterval(poll, PERIOD_MS);
})();
