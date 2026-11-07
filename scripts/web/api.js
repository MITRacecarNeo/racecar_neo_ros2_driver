/* Requests from a page to its own app. Served at /ui/api.js by
   scripts/webui.py.

   RcnApi.post(path, body)       POST JSON (body {} when left out); the Response
   RcnApi.postJson(path, body)   the reply's JSON, {ok: false, message} on any failure
   RcnApi.poll(path, ms, onData, onError)
       GET path every ms with one request in flight; skips while the tab is
       hidden and asks again as soon as it is shown. Returns {now()} to ask at once.
   RcnApi.LOST                   the words for a page that cannot reach the car */
(function () {
  'use strict';
  const LOST = 'Lost contact with the car.';

  function post(path, body) {
    return fetch(path, {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify(body || {}),
    });
  }

  async function postJson(path, body) {
    try {
      const res = await post(path, body);
      return await res.json();
    } catch (err) {
      return { ok: false, message: LOST };
    }
  }

  function poll(path, ms, onData, onError) {
    let busy = false, timer = null;
    function ask() {
      clearTimeout(timer);
      if (document.hidden) { timer = null; return; }
      if (busy) return;
      busy = true;
      fetch(path).then((r) => r.json()).then((d) => { busy = false; onData(d); })
        .catch((err) => { busy = false; if (onError) onError(err); })
        .finally(() => { if (!document.hidden) timer = setTimeout(ask, ms); });
    }
    document.addEventListener('visibilitychange', () => { if (!document.hidden) ask(); });
    ask();
    return { now: ask };
  }

  window.RcnApi = { post, postJson, poll, LOST };
})();
