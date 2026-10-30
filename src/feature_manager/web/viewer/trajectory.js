// Waypoint trajectories in the 3D map. "Trajectory" opens the planner: a short tap on the map
// appends a waypoint, dragging a waypoint moves it, tapping waypoint 1 closes the loop. Presses
// that do not start on a waypoint still turn and move the camera. The robot first drives to
// waypoint 1 and then through all others without stopping; a closed loop can repeat until it is
// stopped. The feature manager drives it with Nav2 (see trajectory.py) and stores named
// trajectories on the robot, so every browser sees them.

import * as THREE from 'three';
import { ACTIVE, groundPoint, post, replayPress, request } from './goal.js';

const DRAG_PX = 12;       // shorter moves are taps
const TAP_MS = 500;       // longer presses turn the camera instead of adding a waypoint
const HIT_PX = { mouse: 16, pen: 20, touch: 28 };
const MIN_WAYPOINTS = 2;
const MIN_LOOP = 3;
const MAX_WAYPOINTS = 200;
const DRAFT_COLOR = 0xf0b350;
const ACTIVE_COLOR = 0x34c47c;
const DRAFT_KEY = 'feature-manager.trajectory-draft';
const EMPTY = { frame_id: null, points: [], closed: false, repeat: false, name: '', dirty: false };

const MARKER_PX = 26;     // waypoint markers keep this size on screen at every zoom
const markerMaterials = new Map();

// Round marker with the waypoint number; `target` adds a white ring (where the robot drives to)
function markerMaterial(label, color, target) {
  const key = `${label}|${color}|${target}`;
  if (!markerMaterials.has(key)) {
    const canvas = document.createElement('canvas');
    canvas.width = canvas.height = 64;
    const ctx = canvas.getContext('2d');
    ctx.fillStyle = `#${color.toString(16).padStart(6, '0')}`;
    ctx.beginPath();
    ctx.arc(32, 32, target ? 25 : 30, 0, Math.PI * 2);
    ctx.fill();
    if (target) {
      ctx.strokeStyle = '#ffffff';
      ctx.lineWidth = 5;
      ctx.stroke();
    }
    ctx.fillStyle = '#14171c';
    ctx.font = `700 ${label.length > 2 ? 22 : 30}px system-ui, sans-serif`;
    ctx.textAlign = 'center';
    ctx.textBaseline = 'middle';
    ctx.fillText(label, 32, 34);
    const map = new THREE.CanvasTexture(canvas);
    map.colorSpace = THREE.SRGBColorSpace;
    markerMaterials.set(key, new THREE.SpriteMaterial({
      map, depthTest: false, transparent: true, sizeAttenuation: false,
    }));
  }
  return markerMaterials.get(key);
}

function routeLength(points, closed) {
  let length = 0;
  for (let i = 1; i < points.length; i++) {
    length += Math.hypot(points[i].x - points[i - 1].x, points[i].y - points[i - 1].y);
  }
  if (closed && points.length > 2) {
    const [a, b] = [points[points.length - 1], points[0]];
    length += Math.hypot(a.x - b.x, a.y - b.y);
  }
  return length;
}

function loadDraft() {
  try {
    const draft = JSON.parse(localStorage.getItem(DRAFT_KEY));
    if (draft && Array.isArray(draft.points)) {
      const points = draft.points.filter((p) => Number.isFinite(p.x) && Number.isFinite(p.y));
      return { ...EMPTY, ...draft, points };
    }
  } catch {
    // storage unavailable or a broken entry: start empty
  }
  return EMPTY;
}

function saveDraft(draft) {
  try {
    localStorage.setItem(DRAFT_KEY, JSON.stringify(draft));
  } catch {
    // storage unavailable (private mode); the draft just isn't remembered
  }
}

const round = (v) => Math.round(v * 1000) / 1000;

// Waypoint markers and the line through them
class RouteView {
  constructor(parent, color) {
    this.color = color;
    this.group = new THREE.Group();
    this.group.matrixAutoUpdate = false;
    this.group.visible = false;
    this.line = new THREE.Line(
      new THREE.BufferGeometry(),
      new THREE.LineBasicMaterial({ color, depthTest: false, transparent: true }));
    this.line.renderOrder = 10;
    this.line.frustumCulled = false;
    this.markers = new THREE.Group();
    this.group.add(this.line, this.markers);
    parent.add(this.group);
    this.route = null;
    this.scale = 0;
  }

  // route: {frame_id, points: [{x, y}], closed, target (waypoint index or undefined)}; returns
  // the transform from the route's frame into the fixed frame, null when hidden
  // the transform from the route's frame into the fixed frame, null when hidden. `scale` is the
  // marker size as a fraction of the view (sprites without size attenuation).
  show(tf, fixedFrame, route, scale) {
    const m = route && tf.lookup(fixedFrame, route.frame_id);
    this.group.visible = Boolean(m);
    if (!m) return null;
    this.group.matrix.copy(m);
    this.group.matrixWorldNeedsUpdate = true;
    if (route !== this.route) this.build(route);
    if (scale !== this.scale) {
      this.scale = scale;
      for (const marker of this.markers.children) marker.scale.set(scale, scale, 1);
    }
    return m;
  }

  build(route) {
    this.route = route;
    const points = route.points.map((p) => new THREE.Vector3(p.x, p.y, 0.03));
    if (route.closed) points.push(points[0]);
    this.line.geometry.dispose();
    this.line.geometry = new THREE.BufferGeometry().setFromPoints(points);
    this.markers.clear();
    route.points.forEach((p, i) => {
      const marker = new THREE.Sprite(markerMaterial(String(i + 1), this.color, i === route.target));
      marker.position.set(p.x, p.y, 0.05);
      marker.scale.set(this.scale, this.scale, 1);
      marker.renderOrder = i === route.target ? 12 : 11;
      this.markers.add(marker);
    });
  }
}

export class TrajectoryTool {
  constructor(viewer) {
    this.viewer = viewer;
    const $ = (sel) => viewer.root.querySelector(sel);
    this.el = {
      open: $('.traj-open'), panel: $('.traj-panel'), close: $('.traj-close'), saved: $('.traj-saved'),
      remove: $('.traj-delete'), hint: $('.traj-hint'), list: $('.traj-list'), summary: $('.traj-summary'),
      closed: $('.traj-closed'), repeat: $('.traj-repeat'), name: $('.traj-name'), save: $('.traj-save'),
      undo: $('.traj-undo'), clear: $('.traj-clear'), drive: $('.traj-drive'), message: $('.traj-message'),
    };
    if (!this.el.panel || !this.el.open) throw new Error('index.html has no trajectory planner');
    this.canvas = viewer.renderer.domElement;
    this.area = viewer.el.canvas;
    this.editing = false;
    this.draft = loadDraft();  // {frame_id, points: [{x, y}], closed, repeat, name, dirty}
    this.saved = [];           // stored trajectories from the feature manager
    this.press = null;         // pointer that may become a tap or a waypoint drag
    this.pointers = new Set(); // pointers down on the map; a second one zooms or pans
    this.busy = false;
    this.message = ['', ''];
    this.listPoints = null;    // points the waypoint list shows

    this.draftView = new RouteView(viewer.world, DRAFT_COLOR);
    this.activeView = new RouteView(viewer.world, ACTIVE_COLOR);
    // Dashed line from the robot to waypoint 1: where the robot drives first. Updated in place
    // every frame; new attributes per frame would leak GPU buffers.
    const approach = new THREE.BufferGeometry();
    approach.setAttribute('position', new THREE.BufferAttribute(new Float32Array(6), 3));
    approach.setAttribute('lineDistance', new THREE.BufferAttribute(new Float32Array(2), 1));
    this.approach = new THREE.Line(approach, new THREE.LineDashedMaterial({
      color: DRAFT_COLOR, dashSize: 0.15, gapSize: 0.12, depthTest: false, transparent: true, opacity: 0.8,
    }));
    this.approach.renderOrder = 10;
    this.approach.frustumCulled = false;
    this.approach.visible = false;
    viewer.world.add(this.approach);
    this.setNavigation(viewer.options.navigation());

    // Capture phase on the canvas' parent: runs before OrbitControls, which listens on the canvas
    this.area.addEventListener('pointerdown', (e) => this.onPointerDown(e), true);
    this.area.addEventListener('pointermove', (e) => this.onPointerMove(e), true);
    this.area.addEventListener('pointerup', (e) => this.onPointerUp(e), true);
    this.area.addEventListener('pointercancel', (e) => this.onPointerCancel(e), true);
    window.addEventListener('keydown', (e) => {
      if (e.key === 'Escape' && this.editing && !this.viewer.root.hidden) this.close();
    });

    const { el } = this;
    el.open.addEventListener('click', () => (this.editing ? this.close() : this.open()));
    el.close.addEventListener('click', () => this.close());
    el.saved.addEventListener('change', () => this.load(el.saved.value));
    el.remove.addEventListener('click', () => this.deleteSaved());
    el.closed.addEventListener('change', () => this.change({ closed: el.closed.checked }));
    el.repeat.addEventListener('change', () => this.change({ repeat: el.repeat.checked }));
    el.name.addEventListener('input', () => this.render());
    el.name.addEventListener('keydown', (e) => e.key === 'Enter' && this.save());
    el.save.addEventListener('click', () => this.save());
    el.undo.addEventListener('click', () => this.change({ points: this.draft.points.slice(0, -1) }));
    el.clear.addEventListener('click', () => {
      if (this.draft.points.length > 2 && !confirm('Remove all waypoints?')) return;
      this.change({ points: [], frame_id: null });
    });
    el.drive.addEventListener('click', () => this.drive());
    el.name.value = this.draft.name;
    this.render();
  }

  setNavigation(nav) {
    this.nav = nav;
    const t = nav && nav.trajectory;
    this.activeRoute = t ? { frame_id: t.frame_id, points: t.waypoints, closed: t.closed, target: t.target } : null;
    this.render();
  }

  open() {
    if (this.viewer.goal) this.viewer.goal.setMode('idle');
    this.editing = true;
    this.say('');
    this.render();
    this.loadSaved();
  }

  close() {
    this.cancelPress();
    this.editing = false;
    this.render();
  }

  say(text, tone = '') {
    this.message = [text, tone];
    this.render();
  }

  // Apply changes to the draft; it is remembered in the browser until it is saved
  change(patch, persist = true) {
    const draft = { ...this.draft, ...patch, dirty: true };
    if (draft.points.length < MIN_LOOP) draft.closed = false;
    if (!draft.closed) draft.repeat = false;
    if (!draft.points.length) draft.frame_id = null;
    this.draft = draft;
    this.message = ['', ''];
    if (persist) saveDraft(draft);
    this.render();
  }

  // ------------------------------------------------------------ pointer input

  frameMatrix() {
    return this.draft.frame_id && this.viewer.tf.lookup(this.viewer.fixedFrame(), this.draft.frame_id);
  }

  // Waypoints are kept in the frame of the first one, also when the fixed frame changes later
  toDraftFrame(point) {
    if (!point) return null;
    const fixed = this.viewer.fixedFrame();
    const frame = this.draft.frame_id || fixed;
    const m = this.viewer.tf.lookup(frame, fixed);
    if (!m) return null;
    if (!this.draft.frame_id) this.draft = { ...this.draft, frame_id: frame };
    point.applyMatrix4(m);
    return { x: round(point.x), y: round(point.y) };
  }

  hitWaypoint(e) {
    const m = this.frameMatrix();
    if (!m) return null;
    const rect = this.canvas.getBoundingClientRect();
    const v = new THREE.Vector3();
    let best = null;
    let bestDist = HIT_PX[e.pointerType] || HIT_PX.mouse;
    this.draft.points.forEach((p, i) => {
      v.set(p.x, p.y, 0).applyMatrix4(m).project(this.viewer.camera);
      if (v.z > 1) return; // behind the camera
      const d = Math.hypot(rect.left + ((v.x + 1) / 2) * rect.width - e.clientX,
        rect.top + ((1 - v.y) / 2) * rect.height - e.clientY);
      if (d < bestDist) [best, bestDist] = [i, d];
    });
    return best;
  }

  onPointerDown(e) {
    if (!this.editing || this.handingOver) return;
    if (e.isPrimary) this.pointers.clear();
    this.pointers.add(e.pointerId);
    const press = this.press;
    if (press) {
      // A second finger zooms or pans: no tap, and a dragged waypoint goes back
      this.cancelPress();
      if (press.index != null && press.pointerType === 'touch') this.handOver(press.last);
      return;
    }
    if (e.pointerType === 'mouse' && e.button !== 0) return;
    const index = this.hitWaypoint(e);
    this.press = {
      pointerId: e.pointerId, pointerType: e.pointerType, startX: e.clientX, startY: e.clientY,
      time: performance.now(), index, moved: false, last: e, before: this.draft,
    };
    if (index == null) return; // OrbitControls turns the camera; a short press adds a waypoint
    e.stopPropagation();
    e.preventDefault();
    this.canvas.setPointerCapture(e.pointerId);
  }

  onPointerMove(e) {
    const press = this.press;
    if (!press || e.pointerId !== press.pointerId) return;
    press.last = e;
    if (Math.hypot(e.clientX - press.startX, e.clientY - press.startY) >= DRAG_PX) press.moved = true;
    if (press.index == null) return;
    e.stopPropagation();
    if (!press.moved) return;
    const point = this.toDraftFrame(groundPoint(this.viewer, e));
    if (!point) return;
    const points = [...this.draft.points];
    points[press.index] = point;
    this.change({ points }, false);
  }

  onPointerUp(e) {
    this.pointers.delete(e.pointerId);
    const press = this.press;
    if (!press || e.pointerId !== press.pointerId) return;
    this.press = null;
    if (press.index != null) {
      e.stopPropagation();
      this.release(press);
      if (press.moved) saveDraft(this.draft);
      else this.tapWaypoint(press.index);
      return;
    }
    if (press.moved || performance.now() - press.time > TAP_MS) return;
    this.addWaypoint(this.toDraftFrame(groundPoint(this.viewer, e)));
  }

  onPointerCancel(e) {
    this.pointers.delete(e.pointerId);
    if (this.press && e.pointerId === this.press.pointerId) this.cancelPress();
  }

  release(press) {
    if (this.canvas.hasPointerCapture(press.pointerId)) this.canvas.releasePointerCapture(press.pointerId);
  }

  cancelPress() {
    const press = this.press;
    if (!press) return;
    this.press = null;
    if (press.index == null) return;
    this.release(press);
    if (press.moved) {
      this.draft = press.before;
      this.render();
    }
  }

  handOver(first) {
    this.handingOver = true;
    try {
      replayPress(this.canvas, first);
    } finally {
      this.handingOver = false;
    }
  }

  addWaypoint(point) {
    if (!point) return;
    if (this.draft.points.length >= MAX_WAYPOINTS) {
      this.say(`A trajectory has at most ${MAX_WAYPOINTS} waypoints.`, 'bad');
      return;
    }
    this.change({ points: [...this.draft.points, point] });
  }

  tapWaypoint(index) {
    if (index === 0 && this.draft.points.length >= MIN_LOOP) {
      this.change({ closed: !this.draft.closed });
    } else {
      this.say(`Waypoint ${index + 1}: drag it to move it, ✕ in the list removes it.`);
    }
  }

  removeWaypoint(index) {
    this.change({ points: this.draft.points.filter((_, i) => i !== index) });
  }

  // ------------------------------------------------------------ saved trajectories

  async loadSaved() {
    try {
      this.saved = (await request('GET', 'api/trajectories')).trajectories;
    } catch (err) {
      this.say(`Saved trajectories unavailable: ${err.message}`, 'bad');
    }
    this.renderSaved();
  }

  load(name) {
    const t = this.saved.find((s) => s.name === name);
    if (!t) return;
    if (this.draft.dirty && this.draft.points.length && !confirm(`Replace the current waypoints with “${name}”?`)) {
      this.renderSaved();
      return;
    }
    this.draft = {
      frame_id: t.frame_id, points: t.waypoints.map((p) => ({ x: p.x, y: p.y })),
      closed: t.closed, repeat: t.repeat, name: t.name, dirty: false,
    };
    saveDraft(this.draft);
    this.el.name.value = t.name;
    this.say(`Loaded “${t.name}”.`);
  }

  async save() {
    const name = this.el.name.value.trim();
    const { draft } = this;
    if (!name) {
      this.say('Enter a name to save the trajectory.', 'bad');
      this.el.name.focus();
      return;
    }
    if (draft.points.length < MIN_WAYPOINTS) return;
    if (name !== draft.name && this.saved.some((s) => s.name === name)
        && !confirm(`Overwrite the saved trajectory “${name}”?`)) return;
    this.busy = true;
    this.render();
    try {
      const saved = await request('PUT', `api/trajectories/${encodeURIComponent(name)}`, {
        waypoints: draft.points, closed: draft.closed, repeat: draft.repeat, frame_id: draft.frame_id,
      });
      this.draft = { ...this.draft, name: saved.name, dirty: false };
      saveDraft(this.draft);
      this.el.name.value = saved.name;
      await this.loadSaved();
      this.say(`Saved “${saved.name}”.`, 'ok');
    } catch (err) {
      this.say(`Not saved: ${err.message}`, 'bad');
    } finally {
      this.busy = false;
      this.render();
    }
  }

  async deleteSaved() {
    const name = this.el.saved.value;
    if (!name || !confirm(`Delete the saved trajectory “${name}”?`)) return;
    try {
      await request('DELETE', `api/trajectories/${encodeURIComponent(name)}`);
      if (this.draft.name === name) this.draft = { ...this.draft, dirty: true };
      await this.loadSaved();
      this.say(`Deleted “${name}”. The waypoints stay in the planner.`);
    } catch (err) {
      this.say(`Not deleted: ${err.message}`, 'bad');
    }
  }

  // ------------------------------------------------------------ driving

  async drive() {
    const { draft } = this;
    this.busy = true;
    this.say('');
    try {
      await post('api/navigation/trajectory', {
        name: this.el.name.value.trim(), waypoints: draft.points, closed: draft.closed,
        repeat: draft.repeat, frame_id: draft.frame_id,
      });
      this.close();
    } catch (err) {
      this.message = [`Not started: ${err.message}`, 'bad'];
    } finally {
      this.busy = false;
      this.render();
    }
  }

  // ------------------------------------------------------------ view

  renderSaved() {
    const { saved } = this.el;
    const names = this.saved.map((s) => s.name);
    saved.replaceChildren(
      new Option(names.length ? `Saved (${names.length})` : 'No saved trajectories', ''),
      ...names.map((n) => new Option(n, n)));
    saved.value = !this.draft.dirty && names.includes(this.draft.name) ? this.draft.name : '';
    this.el.remove.disabled = !saved.value;
  }

  renderList() {
    const { points } = this.draft;
    if (points === this.listPoints) return;
    this.listPoints = points;
    this.el.list.replaceChildren(...points.map((p, i) => {
      const li = document.createElement('li');
      const num = document.createElement('span');
      num.className = 'traj-num';
      num.textContent = i + 1;
      const xy = document.createElement('span');
      xy.className = 'traj-xy';
      xy.textContent = `x ${p.x.toFixed(2)}  y ${p.y.toFixed(2)}`;
      const remove = document.createElement('button');
      remove.type = 'button';
      remove.className = 'link traj-remove';
      remove.textContent = '✕';
      remove.setAttribute('aria-label', `Remove waypoint ${i + 1}`);
      remove.addEventListener('click', () => this.removeWaypoint(i));
      li.append(num, xy, remove);
      return li;
    }));
  }

  render() {
    const { el, draft, nav } = this;
    el.open.setAttribute('aria-pressed', String(this.editing));
    el.panel.hidden = !this.editing;
    if (!this.editing) return;

    const n = draft.points.length;
    this.renderList();
    el.hint.textContent = n
      ? 'Tap to add · drag a waypoint to move it · tap waypoint 1 to close the loop'
      : 'Tap the map to add waypoints. The robot drives to waypoint 1 first, then through all '
        + 'others without stopping.';
    el.summary.textContent = n
      ? `${n} waypoint${n === 1 ? '' : 's'} · ${routeLength(draft.points, draft.closed).toFixed(1)} m`
        + `${draft.closed ? ' per lap' : ''}${draft.dirty && draft.name ? ' · not saved' : ''}`
      : '';
    el.closed.checked = draft.closed;
    el.closed.disabled = n < MIN_LOOP;
    el.repeat.checked = draft.repeat;
    el.repeat.disabled = !draft.closed;
    el.undo.disabled = !n;
    el.clear.disabled = !n;
    el.save.disabled = n < MIN_WAYPOINTS || this.busy;
    const ready = Boolean(nav && nav.trajectory_ready);
    el.drive.disabled = n < MIN_WAYPOINTS || this.busy || !ready;
    el.drive.textContent = draft.repeat ? 'Drive loop' : 'Drive';
    el.remove.disabled = !el.saved.value;

    let [text, tone] = this.message;
    if (!text && n === 1) text = 'Add at least one more waypoint. A single waypoint is a goal.';
    else if (!text && !ready) {
      text = nav && nav.mode === 'topic'
        ? 'Driving trajectories needs nav2_msgs in the feature manager image.'
        : 'Nav2 is not running: you can plan and save, driving needs Navigation with Lidar SLAM + Nav2.';
    }
    el.message.textContent = text;
    el.message.dataset.tone = tone;
  }

  // Called every frame: put the routes where their frames are now
  update(fixedFrame) {
    const { tf, camera } = this.viewer;
    const nav = this.nav;
    // A sprite without size attenuation is `scale` / tan(fov / 2) half view heights tall
    const height = this.canvas.clientHeight || 1;
    const scale = (2 * MARKER_PX / height) * Math.tan(THREE.MathUtils.degToRad(camera.fov) / 2);
    this.activeView.show(tf, fixedFrame, nav && ACTIVE.includes(nav.status) ? this.activeRoute : null, scale);
    const draft = this.editing && this.draft.points.length ? this.draft : null;
    const m = this.draftView.show(tf, fixedFrame, draft, scale);
    const robot = this.viewer.robotFrame();
    this.approach.visible = Boolean(m) && tf.has(robot);
    if (!this.approach.visible) return;
    const from = this.viewer.robotPosition();
    const first = this.draft.points[0];
    const to = new THREE.Vector3(first.x, first.y, 0).applyMatrix4(m);
    const { position, lineDistance } = this.approach.geometry.attributes;
    position.setXYZ(0, from.x, from.y, 0.03);
    position.setXYZ(1, to.x, to.y, 0.03);
    lineDistance.setX(1, Math.hypot(to.x - from.x, to.y - from.y));
    position.needsUpdate = lineDistance.needsUpdate = true;
  }
}
