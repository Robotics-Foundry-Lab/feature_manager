// "Detection" tab: live annotated image of the object detection container and, when it runs
// NanoOWL, the text query. The image is an MJPEG stream from the feature manager
// (api/detection/stream); it only runs while the tab is open.

const STALE_S = 3;          // no new image for this long: show a badge on the last one
const POLL_MS = 1000;
const RETRY_MS = 2000;
const THRESHOLD_SEND_MS = 250;  // debounce while dragging the slider
// Replacing the src aborts the MJPEG request
const BLANK = 'data:image/gif;base64,R0lGODlhAQABAIAAAAAAAP///yH5BAEAAAAALAAAAAABAAEAAAIBRAA7';

const $ = (root, sel) => root.querySelector(sel);

export class DetectionView {
  /**
   * @param {HTMLElement} root  #view-detection
   * @param {{ featureId: string, feature: () => object, status: () => object }} opts
   *   feature(): config of the object detection feature, status(): its live status
   */
  constructor(root, { featureId, feature, status }) {
    this.featureId = featureId;
    this.feature = feature;
    this.status = status;
    this.running = false;
    this.streaming = false;
    this.retryTimer = null;
    this.pollTimer = null;
    this.state = null;        // GET api/detection
    this.stateError = null;
    this.edited = false;      // the user typed into the query field and has not sent it yet
    this.message = null;      // {text, tone} after sending a query
    this.thresholdTimer = null;
    this.thresholdEdited = false;  // slider moved, value not sent or not confirmed yet
    this.thresholdMessage = null;

    this.el = {
      image: $(root, '.det-image'),
      badge: $(root, '.det-badge'),
      empty: $(root, '.det-empty'),
      emptyTitle: $(root, '.det-empty strong'),
      emptyText: $(root, '.det-empty p'),
      emptyLink: $(root, '.det-empty a'),
      meta: $(root, '.det-meta'),
      info: $(root, '.det-info'),
      form: $(root, '.det-query'),
      input: $(root, '#det-query-input'),
      submit: $(root, '.det-query [type="submit"]'),
      revert: $(root, '.det-revert'),
      queryState: $(root, '.det-query-state'),
      threshold: $(root, '.det-threshold'),
      thresholdInput: $(root, '#det-threshold-input'),
      thresholdValue: $(root, '.det-threshold-value'),
      thresholdState: $(root, '.det-threshold-state'),
    };

    this.el.image.addEventListener('error', () => {
      if (!this.streaming) return;
      // Stream broke (feature manager restarted, network): reconnect
      this.streaming = false;
      clearTimeout(this.retryTimer);
      this.retryTimer = setTimeout(() => this.update(), RETRY_MS);
    });
    this.el.input.addEventListener('input', () => {
      this.edited = true;
      this.message = null;
      this.render();
    });
    this.el.revert.addEventListener('click', () => {
      this.edited = false;
      this.render();
    });
    this.el.thresholdInput.addEventListener('input', () => {
      this.thresholdEdited = true;
      this.thresholdMessage = null;
      this.el.thresholdValue.textContent = Number(this.el.thresholdInput.value).toFixed(2);
      clearTimeout(this.thresholdTimer);
      this.thresholdTimer = setTimeout(() => this.sendThreshold(), THRESHOLD_SEND_MS);
    });
    this.el.form.addEventListener('submit', (ev) => {
      ev.preventDefault();
      this.sendQuery();
    });
  }

  start() {
    if (this.running) return;
    this.running = true;
    this.poll();
    this.pollTimer = setInterval(() => this.poll(), POLL_MS);
    this.update();
  }

  stop() {
    this.running = false;
    clearInterval(this.pollTimer);
    clearTimeout(this.retryTimer);
    this.setStream(false);
  }

  // Options the container runs with (a stopped container keeps them), else the defaults
  options() {
    const st = this.status() || {};
    const out = {};
    for (const opt of (this.feature() || { options: [] }).options) out[opt.key] = opt.default;
    return { ...out, ...(st.active_options || {}) };
  }

  // VIS_MODE is a YOLOv8 setting, NanoOWL always publishes its annotated image
  visNone() {
    const opts = this.options();
    return (opts.DETECTOR || 'yolov8') === 'yolov8' && opts.VIS_MODE === 'none';
  }

  /** Called on every status change of the features and every poll. */
  update() {
    if (!this.running) return;
    const st = this.status() || {};
    this.setStream(Boolean(st.running) && !this.visNone());
    this.render();
  }

  setStream(on) {
    if (on === this.streaming) return;
    this.streaming = on;
    clearTimeout(this.retryTimer);
    if (on) {
      this.el.image.src = `api/detection/stream?t=${Date.now()}`;
    } else {
      this.el.image.src = BLANK;
    }
  }

  async poll() {
    try {
      const res = await fetch('api/detection');
      if (!res.ok) throw new Error((await res.json().catch(() => ({}))).detail || res.statusText);
      this.state = await res.json();
      this.stateError = null;
    } catch (err) {
      this.state = null;
      this.stateError = String(err.message || err);
    }
    this.update();
  }

  async sendQuery() {
    const query = this.el.input.value;
    this.el.submit.disabled = true;
    this.message = { text: 'Sending…', tone: '' };
    this.render();
    try {
      const res = await fetch('api/detection/query', {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({ query }),
      });
      const body = await res.json().catch(() => ({}));
      if (!res.ok) throw new Error(body.detail || res.statusText);
      this.edited = false;
      this.message = null;
      this.state = { ...(this.state || {}), wanted_query: body.query, query_pending: true };
    } catch (err) {
      this.message = { text: String(err.message || err), tone: 'bad' };
    }
    this.el.submit.disabled = false;
    this.render();
    this.poll();
  }

  async sendThreshold() {
    const threshold = Number(this.el.thresholdInput.value);
    try {
      const res = await fetch('api/detection/threshold', {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({ threshold }),
      });
      const body = await res.json().catch(() => ({}));
      if (!res.ok) throw new Error(body.detail || res.statusText);
      this.state = { ...(this.state || {}), wanted_threshold: body.threshold,
        threshold_pending: true };
    } catch (err) {
      this.thresholdEdited = false;
      this.thresholdMessage = { text: String(err.message || err), tone: 'bad' };
    }
    this.render();
    this.poll();
  }

  renderThreshold(ds, listening) {
    const active = ds.active_threshold;
    const wanted = ds.wanted_threshold;
    // Keep the slider where the user put it until the node confirmed that value
    if (this.thresholdEdited && !ds.threshold_pending && active != null
        && wanted != null && Math.abs(active - wanted) < 0.005) this.thresholdEdited = false;
    const input = this.el.thresholdInput;
    if (!this.thresholdEdited && document.activeElement !== input && active != null) {
      input.value = active;
    }
    this.el.thresholdValue.textContent = Number(input.value).toFixed(2);
    input.disabled = !listening;

    let text = '';
    let tone = '';
    if (this.thresholdMessage) ({ text, tone } = this.thresholdMessage);
    else if (ds.threshold_pending) text = 'Sending…';
    else if (wanted != null && active != null && Math.abs(active - wanted) >= 0.005) {
      text = `NanoOWL did not confirm ${wanted.toFixed(2)}. Active: ${active.toFixed(2)}`;
      tone = 'bad';
    }
    this.el.thresholdState.textContent = text;
    this.el.thresholdState.dataset.tone = tone;
  }

  render() {
    const st = this.status() || {};
    const opts = this.options();
    const detector = opts.DETECTOR || 'yolov8';
    const ds = this.state || {};
    const live = Boolean(st.running);
    const frameFresh = ds.age_s != null && ds.age_s < STALE_S;
    const isOwl = detector === 'nanoowl';

    // Header: detector and image rate
    const name = isOwl ? 'NanoOWL' : `YOLOv8 (${opts.YOLO_MODEL || 'yolov8n'})`;
    this.el.meta.textContent = live && frameFresh ? `${name} · ${ds.rate_hz} Hz` : name;

    // Image area: empty state, or the stream with a badge when it stalls
    let empty = null;
    if (this.stateError && !live) {
      empty = ['No live image', this.stateError];
    } else if (!live) {
      empty = ['Object Detection is not running', isOwl
        ? 'Start it in Features.'
        : 'Start it in Features. Choose Detector "nanoowl" to search for objects by text.'];
    } else if (this.visNone()) {
      empty = ['No annotated image',
        'Visualization is set to "none". Choose "sparse" or "full" in Features to see the image.'];
    } else if (ds.age_s == null) {
      empty = ['Waiting for the first image…', isOwl
        ? 'NanoOWL needs about 15 s to load. The very first start builds its TensorRT engine, which takes several minutes.'
        : 'YOLOv8 needs a few seconds to start. Check the logs in Features if nothing arrives.'];
    }
    this.el.empty.hidden = !empty;
    // Browsers fire no reliable load event for MJPEG, so the server's frame age decides
    this.el.image.hidden = Boolean(empty) || !this.streaming;
    if (empty) {
      [this.el.emptyTitle.textContent, this.el.emptyText.textContent] = empty;
      this.el.emptyLink.hidden = live && !this.visNone();
    }
    const stale = !empty && ds.age_s != null && ds.age_s >= STALE_S;
    this.el.badge.hidden = !stale;
    if (stale) this.el.badge.textContent = `No new image for ${Math.round(ds.age_s)} s`;

    // Side panel
    this.el.info.textContent = isOwl
      ? 'NanoOWL detects whatever you describe. Each label is drawn with its score.'
      : 'YOLOv8 detects a fixed set of 80 everyday object classes. To search for anything by text, set Detector to "nanoowl" in Features.';
    this.el.form.hidden = !isOwl;
    this.el.threshold.hidden = !isOwl;
    if (!isOwl) return;

    const active = ds.active_query;
    if (!this.edited && document.activeElement !== this.el.input) {
      this.el.input.value = active || ds.wanted_query || '';
    }
    this.el.revert.hidden = !this.edited;
    const listening = live && ds.query_listeners > 0;
    this.el.submit.disabled = !listening || this.el.input.value.trim() === '';
    this.renderThreshold(ds, listening);

    let text;
    let tone = '';
    if (this.message) {
      ({ text, tone } = this.message);
    } else if (!live) {
      text = 'Start Object Detection to change the query.';
    } else if (!listening) {
      text = 'NanoOWL is still loading…';
    } else if (ds.query_pending) {
      text = 'Sending…';
    } else if (ds.wanted_query && active && active !== ds.wanted_query) {
      text = `NanoOWL did not confirm "${ds.wanted_query}". Active: ${active}`;
      tone = 'bad';
    } else if (active) {
      text = `Active: ${active}`;
      tone = 'ok';
    } else {
      text = '';
    }
    this.el.queryState.textContent = text;
    this.el.queryState.dataset.tone = tone;
  }
}
