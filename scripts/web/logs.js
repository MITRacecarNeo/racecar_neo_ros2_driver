/* The Logs section of a lab dashboard: save the live chart as LogN.csv, pick
   a saved log, and draw it. Served at /ui/logs.js by scripts/webui.py; needs
   /ui/ui.js, /ui/api.js and /ui/charts.js.

   RcnLogs.mount(el, render) fills `el` (a .rcn-section) with the tools and a chart.
     render(rows, marks, name) -> {info, series, opts, legend}
       rows: number arrays from the CSV, time first; marks: [[t, label], ...]
       info: the line shown beside the picker; series and opts as for
       RcnCharts.timeChart (the time axis counts seconds from the first row);
       legend: items for RcnCharts.legendHtml, or HTML. */
(function () {
  'use strict';

  function parse(text) {
    const lines = text.trim().split('\n');
    const marks = lines.filter((l) => l.startsWith('# mark')).map((l) => {
      const p = l.split(',');
      return [parseFloat(p[1]), p.slice(2).join(',')];
    });
    const rows = lines.filter((l) => !l.startsWith('#')).slice(1).map((l) => l.split(',').map(Number));
    return { rows, marks };
  }

  function mount(el, render) {
    const id = (name) => `${el.id}-${name}`;
    el.insertAdjacentHTML('beforeend', `
      <div class="rcn-tools" style="margin-top: 0">
        <button class="rcn-btn quiet small" id="${id('save')}">Save log</button>
        <select class="rcn-input" id="${id('sel')}" aria-label="Log file"></select>
        <button class="rcn-btn quiet small" id="${id('load')}">Load log</button>
        <span class="rcn-caption" id="${id('info')}"></span>
      </div>
      <div class="rcn-chart" id="${id('chart')}" style="height: 200px; margin-top: var(--rcn-space-3)"></div>
      <div class="rcn-legend" id="${id('legend')}"></div>`);
    const $ = RcnUi.$;

    function load() {
      const name = $(id('sel')).value;
      if (!name) return;
      fetch('/logs/' + encodeURIComponent(name)).then((r) => r.text()).then((text) => {
        const { rows, marks } = parse(text);
        const out = render(rows, marks, name);
        $(id('info')).textContent = out.info;
        if (!rows.length) return;
        const t0 = rows[0][0], t1 = rows[rows.length - 1][0];
        RcnCharts.timeChart($(id('chart')), t0, t1, out.series, RcnCharts.elapsedTicks(t0), { gapSec: 0.5, marks, ...out.opts });
        RcnUi.setHtml(id('legend'), typeof out.legend === 'string' ? out.legend : RcnCharts.legendHtml(out.legend));
      });
    }

    function refresh(loadLatest) {
      fetch('/logs').then((r) => r.json()).then((names) => {
        const sel = $(id('sel'));
        sel.replaceChildren(...names.map((n) => new Option(n, n)));
        sel.selectedIndex = names.length - 1;
        if (loadLatest && names.length) load();
      });
    }

    $(id('save')).onclick = () => RcnApi.post('/logs/save').then(() => { refresh(true); RcnUi.toast('Log saved.'); });
    $(id('load')).onclick = load;
    refresh(false);
    return { load, refresh };
  }

  window.RcnLogs = { mount, parse };
})();
