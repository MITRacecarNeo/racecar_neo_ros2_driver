/* Map view shared by the Mapper and the Planner: a map in the map frame on a
   canvas, with pan, zoom, rotate and car-up, and a drag tool for poses.

   const mv = RcnMapView.create(canvas, { draw: () => myLayers(mv) });
     mv.view            {cx, cy, scale, rot, carUp, fitted}: screen = (world - center) * scale, y up
     mv.toScreen(x, y)  world (m) to canvas pixels, before the rotation
     mv.toWorld(px, py) a pointer offset (CSS px) to world (m), after undoing the rotation
     mv.fetchGrid(url)  {info, image} from a grid endpoint (X-Map-Info header + int8 body), or null
     mv.fit(grid)       center and zoom so the grid fills the canvas
     mv.frame(fn)       clear, apply the rotation, run fn(ctx), restore, then the scale bar
     mv.drawGrid, drawCar, drawOrigin, drawLine, drawDots: the layers a page picks from
     mv.tool = { start(w), move(w), end(start, end) } turns one-finger drags into a tool
       (the page clears it; end gets null when the drag was too short)
     mv.bindButtons({ zin, zout, center, rotl, rotr, carup }, () => [x, y] | null)
     mv.redraw()        draw now (also on resize)

   Colors come from the page's --map-* CSS tokens. */
(function () {
  'use strict';
  const css = (name) => RcnUi.token(name);
  const ROT_STEP = Math.PI / 12;  // 15 degrees per press
  const ROT_REPEAT_MS = 120;
  const MIN_SCALE = 2, MAX_SCALE = 400;
  const DRAG_PX = 12;  // a pose drag counts once it is this long

  function create(canvas, opts) {
    const ctx = canvas.getContext('2d');
    const view = { cx: 0, cy: 0, scale: 40, fitted: false, rot: 0, carUp: false };
    const mv = { canvas, ctx, view, tool: null, carPose: null, scaleLeft: () => 0 };

    mv.redraw = () => opts.draw(mv);
    function resize() {
      const r = canvas.getBoundingClientRect();
      canvas.width = Math.round(r.width * devicePixelRatio);
      canvas.height = Math.round(r.height * devicePixelRatio);
      mv.redraw();
    }
    // The canvas area changes height with the page's bars, not only with the window;
    // a stale buffer would be drawn stretched.
    new ResizeObserver(resize).observe(canvas);

    mv.toScreen = (x, y) => {
      const d = devicePixelRatio;
      return [canvas.width / 2 + (x - view.cx) * view.scale * d, canvas.height / 2 - (y - view.cy) * view.scale * d];
    };
    mv.toWorld = (px, py) => {
      // Undo the map rotation about the canvas center, then the pan and zoom.
      const cx = canvas.clientWidth / 2, cy = canvas.clientHeight / 2;
      const c = Math.cos(-view.rot), s = Math.sin(-view.rot);
      const dx = px - cx, dy = py - cy;
      const ux = cx + dx * c - dy * s, uy = cy + dx * s + dy * c;
      return [view.cx + (ux - cx) / view.scale, view.cy - (uy - cy) / view.scale];
    };
    // A screen drag (dx, dy) as a pan of the unrotated map.
    function unrotate(dx, dy) {
      const c = Math.cos(-view.rot), s = Math.sin(-view.rot);
      return [dx * c - dy * s, dx * s + dy * c];
    }

    mv.buildImage = (info, data) => {
      const off = document.createElement('canvas');
      off.width = info.width; off.height = info.height;
      const octx = off.getContext('2d');
      const img = octx.createImageData(info.width, info.height);
      const rgb = (c) => [parseInt(c.slice(1, 3), 16), parseInt(c.slice(3, 5), 16), parseInt(c.slice(5, 7), 16)];
      const free = rgb(css('--map-free')), wall = rgb(css('--map-wall')), unk = rgb(css('--map-unknown'));
      for (let r = 0; r < info.height; r++) {
        const dst = (info.height - 1 - r) * info.width;  // OccupancyGrid rows start at the bottom
        for (let c = 0; c < info.width; c++) {
          const v = data[r * info.width + c];
          const col = v < 0 ? unk : (v >= 65 ? wall : free);
          const i = (dst + c) * 4;
          img.data[i] = col[0]; img.data[i + 1] = col[1]; img.data[i + 2] = col[2]; img.data[i + 3] = 255;
        }
      }
      octx.putImageData(img, 0, 0);
      return { info, image: off };
    };
    mv.fetchGrid = async (url) => {
      const res = await fetch(url, { cache: 'no-store' });
      if (res.status !== 200) return null;
      const info = JSON.parse(res.headers.get('X-Map-Info'));
      return mv.buildImage(info, new Int8Array(await res.arrayBuffer()));
    };
    mv.fit = (g) => {
      const i = g.info;
      view.cx = i.origin[0] + i.width * i.resolution / 2;
      view.cy = i.origin[1] + i.height * i.resolution / 2;
      view.scale = Math.max(5, Math.min(canvas.clientWidth / (i.width * i.resolution),
        canvas.clientHeight / (i.height * i.resolution)) * 0.92);
      view.fitted = true;
    };

    mv.drawGrid = (g, alpha) => {
      const i = g.info, d = devicePixelRatio;
      const [x0, y1] = mv.toScreen(i.origin[0], i.origin[1] + i.height * i.resolution);
      ctx.save();
      if (alpha !== undefined) ctx.globalAlpha = alpha;
      ctx.imageSmoothingEnabled = false;
      ctx.drawImage(g.image, x0, y1, i.width * i.resolution * view.scale * d, i.height * i.resolution * view.scale * d);
      ctx.restore();
    };
    mv.drawCar = (x, y, yaw, color, alpha) => {
      const d = devicePixelRatio, s = view.scale * d;
      const [px, py] = mv.toScreen(x, y);
      ctx.save();
      ctx.globalAlpha = alpha;
      ctx.translate(px, py);
      ctx.rotate(-yaw);
      // Footprint from frames.yaml: rear -0.06, front 0.29, half width 0.105 (base_link at the rear axle).
      ctx.fillStyle = color;
      ctx.strokeStyle = css('--map-outline');
      ctx.lineWidth = 1.5 * d;
      const w = Math.max(0.105 * s, 5 * d), front = Math.max(0.29 * s, 12 * d), rear = Math.max(0.06 * s, 3 * d);
      ctx.beginPath();
      ctx.moveTo(-rear, -w); ctx.lineTo(front * 0.7, -w); ctx.lineTo(front, 0); ctx.lineTo(front * 0.7, w);
      ctx.lineTo(-rear, w); ctx.closePath();
      ctx.fill(); ctx.stroke();
      ctx.restore();
    };
    mv.drawLine = (points, color, width) => {
      if (!points || points.length < 2) return;
      const d = devicePixelRatio;
      ctx.strokeStyle = color; ctx.lineWidth = width * d; ctx.lineJoin = 'round'; ctx.beginPath();
      points.forEach((p, i) => { const [x, y] = mv.toScreen(p[0], p[1]); i ? ctx.lineTo(x, y) : ctx.moveTo(x, y); });
      ctx.stroke();
    };
    mv.drawDots = (points, color) => {
      const d = devicePixelRatio;
      ctx.fillStyle = color;
      for (const p of points || []) { const [x, y] = mv.toScreen(p[0], p[1]); ctx.fillRect(x - d, y - d, 2 * d, 2 * d); }
    };
    mv.drawOrigin = () => {
      const d = devicePixelRatio, [ox, oy] = mv.toScreen(0, 0), l = 18 * d;
      ctx.lineWidth = 2 * d;
      ctx.strokeStyle = css('--map-x'); ctx.beginPath(); ctx.moveTo(ox, oy); ctx.lineTo(ox + l, oy); ctx.stroke();
      ctx.strokeStyle = css('--map-y'); ctx.beginPath(); ctx.moveTo(ox, oy); ctx.lineTo(ox, oy - l); ctx.stroke();
    };
    function drawScale() {
      const d = devicePixelRatio;
      const steps = [0.1, 0.2, 0.5, 1, 2, 5, 10, 20, 50];
      const m = steps.find((s) => s * view.scale >= 60) || 50;
      const len = m * view.scale * d, x = (mv.scaleLeft() + 14) * d, y = canvas.height - 16 * d;
      ctx.strokeStyle = css('--rcn-paper'); ctx.fillStyle = css('--rcn-paper'); ctx.lineWidth = 2 * d;
      ctx.beginPath(); ctx.moveTo(x, y); ctx.lineTo(x + len, y); ctx.moveTo(x, y - 5 * d); ctx.lineTo(x, y);
      ctx.moveTo(x + len, y - 5 * d); ctx.lineTo(x + len, y); ctx.stroke();
      ctx.font = `600 ${12 * d}px Barlow, Arial, sans-serif`;
      ctx.fillText(m >= 1 ? `${m} m` : `${m * 100} cm`, x, y - 8 * d);
    }
    // Clear, rotate about the center, draw the page's layers, then the scale bar on top.
    mv.frame = (layers) => {
      ctx.fillStyle = css('--map-unknown');
      ctx.fillRect(0, 0, canvas.width, canvas.height);
      if (view.carUp && mv.carPose) {
        view.cx = mv.carPose[0]; view.cy = mv.carPose[1];
        view.rot = mv.carPose[2] - Math.PI / 2;
      }
      ctx.save();
      ctx.translate(canvas.width / 2, canvas.height / 2);
      ctx.rotate(view.rot);
      ctx.translate(-canvas.width / 2, -canvas.height / 2);
      layers(ctx);
      ctx.restore();
      drawScale();
    };

    // -- pan, zoom, drag tool --
    const pointers = new Map();
    let pinch = null, drag = null;
    canvas.addEventListener('pointerdown', (e) => {
      canvas.setPointerCapture(e.pointerId);
      pointers.set(e.pointerId, [e.offsetX, e.offsetY]);
      if (mv.tool && pointers.size === 1) {
        drag = { start: mv.toWorld(e.offsetX, e.offsetY), end: null };
        if (mv.tool.start) mv.tool.start(drag.start);
        mv.redraw(); return;
      }
      if (pointers.size === 2) {
        const [p, q] = [...pointers.values()];
        pinch = { dist: Math.hypot(p[0] - q[0], p[1] - q[1]), scale: view.scale,
          angle: Math.atan2(q[1] - p[1], q[0] - p[0]), rot: view.rot };
      }
    });
    canvas.addEventListener('pointermove', (e) => {
      if (!pointers.has(e.pointerId)) return;
      const prev = pointers.get(e.pointerId);
      pointers.set(e.pointerId, [e.offsetX, e.offsetY]);
      if (mv.tool && drag && pointers.size === 1) {
        const w = mv.toWorld(e.offsetX, e.offsetY);
        if (Math.hypot(w[0] - drag.start[0], w[1] - drag.start[1]) * view.scale > DRAG_PX) {
          drag.end = w;
          if (mv.tool.move) mv.tool.move(drag.start, w);
        }
        mv.redraw(); return;
      }
      if (pointers.size === 2 && pinch) {
        const [p, q] = [...pointers.values()];
        view.scale = Math.max(MIN_SCALE, Math.min(MAX_SCALE, pinch.scale * Math.hypot(p[0] - q[0], p[1] - q[1]) / pinch.dist));
        // Two fingers turning turn the map, unless the car-up lock holds it.
        if (!view.carUp) view.rot = pinch.rot + Math.atan2(q[1] - p[1], q[0] - p[0]) - pinch.angle;
      } else if (pointers.size === 1 && !view.carUp) {
        const [dx, dy] = unrotate(e.offsetX - prev[0], e.offsetY - prev[1]);
        view.cx -= dx / view.scale; view.cy += dy / view.scale;
      }
      mv.redraw();
    });
    function pointerEnd(e) {
      pointers.delete(e.pointerId);
      if (pointers.size < 2) pinch = null;
      if (mv.tool && drag && pointers.size === 0) {
        const done = drag;
        drag = null;
        mv.tool.end(done.start, done.end);
      }
    }
    canvas.addEventListener('pointerup', pointerEnd);
    canvas.addEventListener('pointercancel', pointerEnd);
    canvas.addEventListener('wheel', (e) => {
      e.preventDefault();
      const before = mv.toWorld(e.offsetX, e.offsetY);
      view.scale = Math.max(MIN_SCALE, Math.min(MAX_SCALE, view.scale * (e.deltaY < 0 ? 1.15 : 1 / 1.15)));
      const after = mv.toWorld(e.offsetX, e.offsetY);
      view.cx += before[0] - after[0]; view.cy += before[1] - after[1];
      mv.redraw();
    }, { passive: false });
    // The drag in progress (start, end) for drawing a pose arrow, or null.
    mv.dragging = () => drag;

    mv.setCarUp = (on, button) => {
      view.carUp = on;
      if (button) { button.classList.toggle('on', on); button.setAttribute('aria-pressed', String(on)); }
      mv.redraw();
    };
    // A press turns one step; holding keeps turning. The map stays where the lock left it.
    function holdToRotate(button, step, carupButton) {
      let timer = null;
      const turn = () => { if (view.carUp) mv.setCarUp(false, carupButton); view.rot += step; mv.redraw(); };
      const stop = () => { clearInterval(timer); timer = null; };
      button.addEventListener('pointerdown', (e) => {
        e.preventDefault();
        button.setPointerCapture(e.pointerId);
        turn();
        stop();
        timer = setInterval(turn, ROT_REPEAT_MS);
      });
      for (const ev of ['pointerup', 'pointercancel', 'lostpointercapture']) button.addEventListener(ev, stop);
      button.addEventListener('contextmenu', (e) => e.preventDefault());
    }
    mv.bindButtons = (b, carCenter, fallbackGrid) => {
      b.zin.onclick = () => { view.scale = Math.min(MAX_SCALE, view.scale * 1.3); mv.redraw(); };
      b.zout.onclick = () => { view.scale = Math.max(MIN_SCALE, view.scale / 1.3); mv.redraw(); };
      b.center.onclick = () => {
        const c = carCenter();
        if (c) { view.cx = c[0]; view.cy = c[1]; } else { const g = fallbackGrid(); if (g) mv.fit(g); }
        mv.redraw();
      };
      holdToRotate(b.rotl, -ROT_STEP, b.carup);
      holdToRotate(b.rotr, ROT_STEP, b.carup);
      b.carup.onclick = () => mv.setCarUp(!view.carUp, b.carup);
    };
    return mv;
  }

  window.RcnMapView = { create };
})();
