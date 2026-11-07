/* Top-down lidar view: the car in the middle, nose up, rings every meter.
   Served at /ui/lidar.js by scripts/webui.py; needs /ui/ui.js. Points come
   in the lab convention: [deg, m], 0 at the nose, positive to the right.

   const view = RcnLidar.view(canvas, rangeEl, {dots})
     view.draw(scan, overlay)  overlay(api) draws on top before the car;
                               api: {x, cx, cy, k, range, ray(deg, m, color, width)}
     view.zoom(factor)         >1 shows more, <1 less; 1 to 12 m. The wheel zooms too. */
(function () {
  'use strict';
  const MIN_M = 1, MAX_M = 12;

  function view(canvas, rangeEl, opts = {}) {
    let range = 4.0, last = [[], null];
    const x = canvas.getContext('2d');

    function zoom(f) {
      range = Math.min(MAX_M, Math.max(MIN_M, range * f));
      draw(...last);
    }
    canvas.addEventListener('wheel', (e) => { e.preventDefault(); zoom(e.deltaY > 0 ? 1.2 : 1 / 1.2); });

    function draw(scan, overlay) {
      last = [scan, overlay];
      const cx = canvas.width / 2, cy = canvas.height / 2, k = (cx - 10) / range;
      x.clearRect(0, 0, canvas.width, canvas.height);
      x.lineWidth = 1;
      x.strokeStyle = 'rgba(227, 227, 235, 0.18)';
      for (let r = 1; r <= range; r++) { x.beginPath(); x.arc(cx, cy, r * k, 0, 2 * Math.PI); x.stroke(); }
      x.fillStyle = RcnUi.token(opts.dots || '--rcn-mist');
      for (const [deg, r] of scan) {
        const a = deg * Math.PI / 180;
        x.fillRect(cx + Math.sin(a) * r * k - 1.5, cy - Math.cos(a) * r * k - 1.5, 3, 3);
      }
      if (overlay) {
        const ray = (deg, r, color, width) => {
          const a = deg * Math.PI / 180;
          x.strokeStyle = color; x.lineWidth = width; x.beginPath();
          x.moveTo(cx, cy); x.lineTo(cx + Math.sin(a) * r * k, cy - Math.cos(a) * r * k); x.stroke();
        };
        overlay({ x, cx, cy, k, range, ray });
      }
      x.fillStyle = RcnUi.token('--rcn-paper');
      x.beginPath(); x.moveTo(cx, cy - 10); x.lineTo(cx - 7, cy + 8); x.lineTo(cx + 7, cy + 8); x.fill();
      if (rangeEl) rangeEl.textContent = range.toFixed(1);
    }

    return { draw, zoom };
  }

  window.RcnLidar = { view };
})();
