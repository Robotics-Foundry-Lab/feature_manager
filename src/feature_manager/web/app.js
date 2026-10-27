'use strict';

// Exit codes of a regular `docker stop` (SIGTERM / SIGKILL after timeout) are not failures
const CLEAN_EXIT = new Set([0, 130, 137, 143]);
const MAX_LOG_LINES = 2000;

const state = {
  features: [],          // config from /api/features, in display order
  byId: {},
  status: {},            // feature id -> live status
  clockOffset: 0,        // server time - browser time, seconds
  drafts: {},            // feature id -> {option key: value} edited but not applied
  cards: {},             // feature id -> card elements
  logs: {},              // feature id -> {source, startedAt}
};

const $ = (root, sel) => root.querySelector(sel);

// ---------------------------------------------------------------- formatting

function formatDuration(seconds) {
  if (seconds == null || seconds < 0) return '–';
  const d = Math.floor(seconds / 86400);
  const h = Math.floor((seconds % 86400) / 3600);
  const m = Math.floor((seconds % 3600) / 60);
  const s = Math.floor(seconds % 60);
  if (d) return `${d}d ${h}h`;
  if (h) return `${h}h ${m}m`;
  if (m) return `${m}m ${s}s`;
  return `${s}s`;
}

function formatBytes(bytes) {
  if (bytes == null) return '–';
  const units = ['B', 'KB', 'MB', 'GB'];
  let i = 0;
  while (bytes >= 1024 && i < units.length - 1) { bytes /= 1024; i++; }
  return `${bytes.toFixed(bytes < 10 && i ? 1 : 0)} ${units[i]}`;
}

function nowServer() { return Date.now() / 1000 + state.clockOffset; }

// ---------------------------------------------------------------- state helpers

// The values the controls show when the user hasn't edited them: what the container actually
// runs with (also kept by a stopped container), otherwise the configured defaults.
function baseline(feature) {
  const active = (state.status[feature.id] || {}).active_options || {};
  const out = {};
  for (const opt of feature.options) out[opt.key] = opt.key in active ? active[opt.key] : opt.default;
  return out;
}

function currentOptions(feature) {
  return { ...baseline(feature), ...(state.drafts[feature.id] || {}) };
}

function dirtyKeys(feature) {
  const base = baseline(feature);
  const draft = state.drafts[feature.id] || {};
  return Object.keys(draft).filter((k) => draft[k] !== base[k]);
}

function describeState(st) {
  if (!st) return { text: '–', tone: 'idle' };
  if (st.pending) return { text: `${st.pending}…`, tone: 'warn' };
  if (st.state === 'running') {
    if (st.health === 'unhealthy') return { text: 'unhealthy', tone: 'bad' };
    if (st.health === 'starting') return { text: 'starting…', tone: 'warn' };
    return { text: 'running', tone: 'ok' };
  }
  if (st.state === 'restarting') return { text: 'restarting…', tone: 'warn' };
  if (st.state === 'exited' && st.exit_code != null && !CLEAN_EXIT.has(st.exit_code)) {
    return { text: `crashed (exit ${st.exit_code})`, tone: 'bad' };
  }
  if (st.state === 'not created') return { text: 'not created', tone: 'idle' };
  if (st.state === 'unknown') return { text: 'docker unreachable', tone: 'bad' };
  return { text: 'stopped', tone: 'idle' };
}

// ---------------------------------------------------------------- rendering

function buildOption(feature, opt, card) {
  const wrap = document.createElement('div');
  wrap.className = 'option';
  wrap.dataset.key = opt.key;

  const row = document.createElement('div');
  row.className = 'option-row';
  const label = document.createElement('label');
  label.className = 'option-label';
  label.textContent = opt.label;
  const inputId = `opt-${feature.id}-${opt.key}`;
  label.htmlFor = inputId;
  row.append(label);

  const desc = document.createElement('p');
  desc.className = 'option-desc';

  let input;
  if (opt.type === 'bool') {
    const sw = document.createElement('span');
    sw.className = 'switch';
    input = document.createElement('input');
    input.type = 'checkbox';
    input.setAttribute('role', 'switch');
    const track = document.createElement('span');
    track.className = 'track';
    sw.append(input, track);
    row.append(sw);
    desc.textContent = opt.description;
    input.addEventListener('change', () => setDraft(feature, opt.key, input.checked));
  } else {
    input = document.createElement('select');
    for (const choice of opt.choices) {
      const o = document.createElement('option');
      o.value = choice.value;
      o.textContent = choice.value;
      input.append(o);
    }
    row.append(input);
    input.addEventListener('change', () => setDraft(feature, opt.key, input.value));
  }
  input.id = inputId;
  wrap.append(row, desc);
  card.optionInputs[opt.key] = { opt, input, wrap, desc };
  return wrap;
}

function buildCard(feature) {
  const node = document.getElementById('card-template').content.firstElementChild.cloneNode(true);
  node.dataset.feature = feature.id;
  const card = { node, optionInputs: {} };
  $(node, '.name').textContent = feature.display_name;
  $(node, '.description').textContent = feature.description;

  if (feature.options.length) {
    $(node, '.options').hidden = false;
    const list = $(node, '.option-list');
    for (const opt of feature.options) list.append(buildOption(feature, opt, card));
    $(node, '.reset').addEventListener('click', () => {
      delete state.drafts[feature.id];
      updateCard(feature);
    });
  }

  for (const btn of node.querySelectorAll('[data-action]')) {
    btn.addEventListener('click', () => runAction(feature, btn.dataset.action));
  }

  const logs = $(node, '.logs');
  logs.addEventListener('toggle', () => (logs.open ? openLogs(feature) : closeLogs(feature)));

  card.pill = $(node, '.pill');
  card.uptime = $(node, '.m-uptime');
  card.uptimeLabel = $(node, '.m-uptime-label');
  card.cpu = $(node, '.m-cpu');
  card.mem = $(node, '.m-mem');
  card.restarts = $(node, '.m-restarts');
  card.hints = $(node, '.hints');
  card.error = $(node, '.error');
  card.logView = $(node, '.log-view');
  card.logState = $(node, '.log-state');
  card.logs = logs;
  card.buttons = Object.fromEntries(
    [...node.querySelectorAll('[data-action]')].map((b) => [b.dataset.action, b]));
  state.cards[feature.id] = card;
  return node;
}

function hintsFor(feature, options) {
  const hints = [];
  const isRunning = (id) => (state.status[id] || {}).running;
  const name = (id) => (state.byId[id] || { display_name: id }).display_name;

  const missing = feature.requires.filter((id) => !isRunning(id));
  if (missing.length) hints.push(`Needs ${missing.map(name).join(', ')} — not running.`);

  for (const opt of feature.options) {
    if (opt.type !== 'choice') continue;
    const choice = opt.choices.find((c) => c.value === options[opt.key]);
    const conflicts = (choice && choice.warn_if_running || []).filter(isRunning);
    if (conflicts.length) {
      hints.push(`${opt.label} "${choice.value}" while ${conflicts.map(name).join(', ')} is running.`);
    }
  }
  return hints;
}

function updateCard(feature) {
  const card = state.cards[feature.id];
  const st = state.status[feature.id] || {};
  const { text, tone } = describeState(st);
  card.pill.textContent = text;
  card.pill.dataset.tone = tone;
  card.node.dataset.tone = tone;

  card.cpu.textContent = st.cpu_percent != null ? `${st.cpu_percent.toFixed(1)} %` : '–';
  card.mem.textContent = formatBytes(st.mem_bytes);
  card.restarts.textContent = st.restart_count != null ? st.restart_count : '–';
  updateUptime(feature);

  // Options: show the active/default value unless the user edited the control
  const options = currentOptions(feature);
  const dirty = new Set(dirtyKeys(feature));
  for (const { opt, input, wrap, desc } of Object.values(card.optionInputs)) {
    const value = options[opt.key];
    if (opt.type === 'bool') input.checked = !!value;
    else {
      input.value = value;
      const choice = opt.choices.find((c) => c.value === value);
      desc.textContent = [opt.description, choice && choice.description].filter(Boolean).join(' ');
    }
    input.disabled = !!st.pending;
    wrap.dataset.dirty = dirty.has(opt.key);
  }
  if (feature.options.length) $(card.node, '.reset').hidden = dirty.size === 0;

  const hints = hintsFor(feature, options);
  card.hints.replaceChildren(...hints.map((h) => {
    const p = document.createElement('p');
    p.className = 'hint';
    p.textContent = h;
    return p;
  }));

  card.error.hidden = !st.last_error;
  $(card.error, 'pre').textContent = st.last_error || '';

  const running = st.state === 'running' || st.state === 'restarting';
  const busy = !!st.pending;
  const b = card.buttons;
  b.start.hidden = running;
  b.stop.hidden = !running;
  b.restart.hidden = !running;
  b.apply.hidden = !(running && dirty.size);
  for (const btn of Object.values(b)) btn.disabled = busy || st.state === 'unknown';

  // Reconnect an open log view when the container (re)started
  const log = state.logs[feature.id];
  if (card.logs.open && st.running && (!log || log.startedAt !== st.started_at)) openLogs(feature);
}

function updateUptime(feature) {
  const st = state.status[feature.id] || {};
  const card = state.cards[feature.id];
  const running = st.running && st.started_at;
  card.uptimeLabel.textContent = running || !st.finished_at ? 'Uptime' : 'Stopped';
  if (running) card.uptime.textContent = formatDuration(nowServer() - st.started_at);
  else if (st.finished_at) card.uptime.textContent = formatDuration(nowServer() - st.finished_at);
  else card.uptime.textContent = '–';
}

function updateSummary() {
  const total = state.features.length;
  const running = state.features.filter((f) => (state.status[f.id] || {}).running).length;
  const pending = state.features.filter((f) => (state.status[f.id] || {}).pending).length;
  document.getElementById('summary').textContent =
    `${running} of ${total} running${pending ? ` · ${pending} busy` : ''}`;
}

function applySnapshot(snapshot) {
  state.status = snapshot.features || {};
  if (snapshot.timestamp) state.clockOffset = snapshot.timestamp - Date.now() / 1000;
  for (const f of state.features) updateCard(f);
  updateSummary();
}

// ---------------------------------------------------------------- actions

function setDraft(feature, key, value) {
  state.drafts[feature.id] = { ...(state.drafts[feature.id] || {}), [key]: value };
  updateCard(feature);
}

async function runAction(feature, action) {
  const st = state.status[feature.id] || {};
  const body = {};
  let endpoint = action;
  if (action === 'start' || action === 'apply') {
    endpoint = 'start';
    body.options = currentOptions(feature);
  }
  if (action === 'stop' && !confirm(`Stop ${feature.display_name}?`)) return;

  // Optimistic pending state until the next status event arrives
  state.status[feature.id] = { ...st, pending: { start: 'starting', apply: 'restarting',
    stop: 'stopping', restart: 'restarting' }[action], last_error: null };
  updateCard(feature);
  try {
    const res = await fetch(`api/features/${encodeURIComponent(feature.id)}/${endpoint}`, {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify(body),
    });
    if (!res.ok) {
      const detail = (await res.json().catch(() => ({}))).detail || res.statusText;
      throw new Error(detail);
    }
    if (endpoint === 'start') delete state.drafts[feature.id];
  } catch (err) {
    state.status[feature.id] = { ...st, pending: null, last_error: String(err.message || err) };
    updateCard(feature);
  }
}

// ---------------------------------------------------------------- logs

function lineClass(line) {
  if (/\b(ERROR|FATAL|Traceback)\b|\[ERROR\]/.test(line)) return 'l-err';
  if (/\bWARN(ING)?\b|\[WARN\]/.test(line)) return 'l-warn';
  return '';
}

function appendLogLines(view, lines) {
  const atBottom = view.scrollHeight - view.scrollTop - view.clientHeight < 40;
  const frag = document.createDocumentFragment();
  for (const line of lines) {
    const span = document.createElement('span');
    const cls = lineClass(line);
    if (cls) span.className = cls;
    span.textContent = line + '\n';
    frag.append(span);
  }
  view.append(frag);
  while (view.childNodes.length > MAX_LOG_LINES) view.firstChild.remove();
  if (atBottom) view.scrollTop = view.scrollHeight;
}

function openLogs(feature) {
  closeLogs(feature);
  const card = state.cards[feature.id];
  const st = state.status[feature.id] || {};
  const source = new EventSource(`api/features/${encodeURIComponent(feature.id)}/logs/stream?tail=300`);
  state.logs[feature.id] = { source, startedAt: st.started_at };
  card.logState.textContent = 'connecting…';

  source.addEventListener('meta', (ev) => {
    const meta = JSON.parse(ev.data);
    if (meta.reset) {
      card.logView.replaceChildren();
      card.logState.textContent = meta.exists ? '● live' : '';
      if (!meta.exists) card.logView.textContent = 'No container yet — start the feature first.';
    }
    if (meta.ended) {
      // Container stopped: keep the last lines; updateCard reconnects once it runs again
      source.close();
      card.logState.textContent = '(container stopped)';
    }
  });
  source.addEventListener('lines', (ev) => appendLogLines(card.logView, JSON.parse(ev.data)));
  source.onerror = () => { card.logState.textContent = 'reconnecting…'; };
}

function closeLogs(feature) {
  const log = state.logs[feature.id];
  if (log) log.source.close();
  delete state.logs[feature.id];
  state.cards[feature.id].logState.textContent = '';
}

// ---------------------------------------------------------------- startup

function setConnection(mode) {
  const el = document.getElementById('connection');
  el.dataset.state = mode;
  $(el, '.label').textContent = { online: 'live', offline: 'offline – retrying', connecting: 'connecting' }[mode];
}

function connectEvents() {
  const source = new EventSource('api/events');
  source.addEventListener('status', (ev) => {
    setConnection('online');
    applySnapshot(JSON.parse(ev.data));
  });
  source.onopen = () => setConnection('online');
  source.onerror = () => setConnection('offline');
}

async function init() {
  try {
    const res = await fetch('api/features');
    const data = await res.json();
    state.features = data.features;
    state.byId = Object.fromEntries(data.features.map((f) => [f.id, f]));
    const grid = document.getElementById('features');
    grid.replaceChildren(...data.features.map(buildCard));
    applySnapshot(data.status);
    connectEvents();
    setInterval(() => state.features.forEach(updateUptime), 1000);
  } catch (err) {
    setConnection('offline');
    document.getElementById('summary').textContent = `Could not load features: ${err.message}`;
    setTimeout(init, 3000);
  }
}

init();
