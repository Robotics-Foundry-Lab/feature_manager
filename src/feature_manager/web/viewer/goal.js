// Navigation goals in the 3D map. After "Set goal", press on the goal position and drag towards
// the direction the robot should face, with the mouse or one finger; a short tap faces away from
// the robot. Two fingers, the wheel and the right mouse button keep moving the camera. The
// feature manager sends the goal to Nav2 (NavigateToPose action, see nav_goal.py). Waypoint
// trajectories are planned in trajectory.js; their progress is shown here in the goal bar.

import * as THREE from 'three';

const DRAG_PX = 12;       // shorter drags only place the goal
const MAX_RANGE = 200;    // ignore ground hits near the horizon
const GROUND = new THREE.Plane(new THREE.Vector3(0, 0, 1), 0);
export const ACTIVE = ['sending', 'active', 'sent'];
const TONES = { succeeded: 'ok', aborted: 'bad', rejected: 'bad', failed: 'bad' };

function makeMarker(color) {
  // Flat arrow along +x with a ring around the goal position
  const shape = new THREE.Shape();
  shape.moveTo(0, -0.05);
  shape.lineTo(0.45, -0.05);
  shape.lineTo(0.45, -0.14);
  shape.lineTo(0.7, 0);
  shape.lineTo(0.45, 0.14);
  shape.lineTo(0.45, 0.05);
  shape.lineTo(0, 0.05);
  shape.closePath();
  const material = new THREE.MeshBasicMaterial({
    color, depthTest: false, transparent: true, opacity: 0.95, side: THREE.DoubleSide,
  });
  const marker = new THREE.Group();
  marker.matrixAutoUpdate = false;
  marker.visible = false;
  for (const geometry of [new THREE.ShapeGeometry(shape), new THREE.RingGeometry(0.2, 0.26, 40)]) {
    const mesh = new THREE.Mesh(geometry, material);
    mesh.position.z = 0.04;
    mesh.renderOrder = 11;
    marker.add(mesh);
  }
  return marker;
}

function goalMatrix(goal) {
  return new THREE.Matrix4().makeRotationZ(goal.yaw).setPosition(goal.x, goal.y, 0);
}

function describeGoal(goal) {
  const deg = Math.round(THREE.MathUtils.radToDeg(goal.yaw));
  return `x ${goal.x.toFixed(2)} m, y ${goal.y.toFixed(2)} m, heading ${deg}°`;
}

function describeTrajectory(nav) {
  const t = nav.trajectory;
  const fb = nav.feedback;
  const name = t.name ? `“${t.name}”` : 'Trajectory';
  const n = t.waypoints.length;
  const parts = [t.closed && !t.repeat && t.lap > 1
    ? `${name} · back to waypoint 1` : `${name} · to waypoint ${t.target + 1} of ${n}`];
  if (t.repeat) parts.push(`lap ${t.lap}`);
  if (fb && fb.distance_to_waypoint != null) parts.push(`${fb.distance_to_waypoint.toFixed(1)} m`);
  if (fb && fb.recoveries) parts.push(`${fb.recoveries} recover${fb.recoveries === 1 ? 'y' : 'ies'}`);
  return parts.join(' · ');
}

export async function request(method, url, body) {
  const res = await fetch(url, {
    method,
    headers: { 'Content-Type': 'application/json' },
    body: body === undefined ? undefined : JSON.stringify(body),
  });
  const data = await res.json().catch(() => ({}));
  if (!res.ok) {
    const detail = data.detail;
    throw new Error(Array.isArray(detail) ? detail.map((d) => d.msg).join('; ') : detail || res.statusText);
  }
  return data;
}

export const post = (url, body = {}) => request('POST', url, body);

const raycaster = new THREE.Raycaster();

// Where the pointer event hits the ground (z = 0 of the fixed frame), null near the horizon
export function groundPoint(viewer, e) {
  const rect = viewer.renderer.domElement.getBoundingClientRect();
  const ndc = new THREE.Vector2(
    ((e.clientX - rect.left) / rect.width) * 2 - 1,
    -((e.clientY - rect.top) / rect.height) * 2 + 1);
  raycaster.setFromCamera(ndc, viewer.camera);
  const point = raycaster.ray.intersectPlane(GROUND, new THREE.Vector3());
  return point && point.distanceTo(viewer.camera.position) < MAX_RANGE ? point : null;
}

// A tool kept the first finger from OrbitControls; replay its press before the second one arrives,
// so the two fingers zoom and pan
export function replayPress(canvas, first) {
  canvas.dispatchEvent(new PointerEvent('pointerdown', {
    pointerId: first.pointerId, pointerType: 'touch', isPrimary: true, bubbles: true,
    clientX: first.clientX, clientY: first.clientY, screenX: first.screenX, screenY: first.screenY,
    button: 0, buttons: 1,
  }));
}

export class GoalTool {
  constructor(viewer) {
    this.viewer = viewer;
    const $ = (sel) => viewer.root.querySelector(sel);
    this.el = {
      bar: $('.goal-bar'), status: $('.goal-status'), start: $('.goal-start'),
      send: $('.goal-send'), discard: $('.goal-discard'), stop: $('.goal-stop'), trajectory: $('.traj-open'),
      reset: $('.map-reset'), resetButton: $('.reset-map'), resetInfo: $('.reset-map-info'),
    };
    this.canvas = viewer.renderer.domElement;
    this.area = viewer.el.canvas;
    this.nav = viewer.options.navigation();
    this.mode = 'idle';     // idle | pick (waiting for a press) | drag | draft (placed, not sent)
    this.draft = null;      // {x, y, yaw, frame_id}
    this.drag = null;       // pointer that places the goal
    this.error = '';
    this.busy = false;

    this.draftMarker = makeMarker(0xf0b350);
    this.goalMarker = makeMarker(0x34c47c);
    viewer.world.add(this.draftMarker, this.goalMarker);

    // Capture phase on the canvas' parent: runs before OrbitControls, which listens on the canvas
    this.area.addEventListener('pointerdown', (e) => this.onPointerDown(e), true);
    this.area.addEventListener('pointermove', (e) => this.onPointerMove(e), true);
    this.area.addEventListener('pointerup', (e) => this.onPointerUp(e), true);
    this.area.addEventListener('pointercancel', (e) => this.drag && e.pointerId === this.drag.pointerId && this.cancelDrag(), true);
    window.addEventListener('keydown', (e) => {
      if (e.key === 'Escape' && this.mode !== 'idle' && !this.viewer.root.hidden) this.setMode('idle');
    });

    this.el.start.addEventListener('click', () => this.setMode('pick'));
    this.el.discard.addEventListener('click', () => this.setMode('idle'));
    this.el.send.addEventListener('click', () => this.sendGoal());
    this.el.stop.addEventListener('click', () => this.stopNavigation());
    this.el.resetButton.addEventListener('click', () => this.resetMap());
    this.render();
  }

  setNavigation(nav) {
    this.nav = nav;
    this.render();
  }

  setMode(mode) {
    if (mode !== 'idle' && this.viewer.trajectory) this.viewer.trajectory.close();
    if (mode === 'idle' || mode === 'pick') this.draft = null;
    if (this.drag) this.releaseDrag();
    this.mode = mode;
    this.error = '';
    this.area.dataset.goal = mode === 'idle' ? '' : 'pick';
    this.render();
  }

  // ------------------------------------------------------------ pointer input

  groundPoint(e) {
    return groundPoint(this.viewer, e);
  }

  onPointerDown(e) {
    if (this.mode === 'idle' || this.handingOver) return;
    if (this.drag) {
      // A second finger means zoom or pan: hand both touches to the camera controls
      if (e.pointerType === 'touch' && this.drag.pointerType === 'touch') this.handOver();
      return;
    }
    if (e.pointerType === 'mouse' && e.button !== 0) return;
    const point = this.groundPoint(e);
    if (!point) return;
    e.stopPropagation();
    e.preventDefault();
    this.canvas.setPointerCapture(e.pointerId);
    this.drag = {
      pointerId: e.pointerId, pointerType: e.pointerType, start: point,
      startX: e.clientX, startY: e.clientY, last: e, previous: this.draft, previousMode: this.mode,
    };
    // Until the pointer is dragged, the robot faces away from where it is now
    const robot = this.viewer.robotPosition();
    const yaw = robot.distanceTo(point) > 0.1 ? Math.atan2(point.y - robot.y, point.x - robot.x) : 0;
    this.placeDraft(point, yaw);
    this.mode = 'drag';
    this.render();
  }

  onPointerMove(e) {
    const drag = this.drag;
    if (!drag || e.pointerId !== drag.pointerId) return;
    e.stopPropagation();
    drag.last = e;
    if (Math.hypot(e.clientX - drag.startX, e.clientY - drag.startY) < DRAG_PX) return;
    const point = this.groundPoint(e);
    if (!point) return;
    this.placeDraft(drag.start, Math.atan2(point.y - drag.start.y, point.x - drag.start.x));
    this.render();
  }

  onPointerUp(e) {
    if (!this.drag || e.pointerId !== this.drag.pointerId) return;
    e.stopPropagation();
    this.releaseDrag();
    this.mode = 'draft';
    this.render();
  }

  placeDraft(point, yaw) {
    this.draft = { x: point.x, y: point.y, yaw, frame_id: this.viewer.fixedFrame() };
  }

  releaseDrag() {
    if (this.canvas.hasPointerCapture(this.drag.pointerId)) this.canvas.releasePointerCapture(this.drag.pointerId);
    this.drag = null;
  }

  cancelDrag() {
    if (!this.drag) return;
    this.draft = this.drag.previous;
    this.mode = this.drag.previousMode;
    this.releaseDrag();
    this.render();
  }

  handOver() {
    const first = this.drag.last;
    this.cancelDrag();
    this.handingOver = true;
    try {
      replayPress(this.canvas, first);
    } finally {
      this.handingOver = false;
    }
  }

  // ------------------------------------------------------------ actions

  async sendGoal() {
    if (!this.draft) return;
    this.busy = true;
    this.error = '';
    this.render();
    try {
      await post('api/navigation/goal', this.draft);
      this.setMode('idle');
    } catch (err) {
      this.error = `Goal not sent: ${err.message}`;
    } finally {
      this.busy = false;
      this.render();
    }
  }

  async stopNavigation() {
    try {
      await post('api/navigation/cancel');
    } catch (err) {
      this.error = `Stop failed: ${err.message}`;
      this.render();
    }
  }

  async resetMap() {
    const ok = confirm('Reset the 2D map?\n\nThe robot forgets the occupancy map built so far and '
      + 'starts a new one from where it stands now. A running navigation goal is stopped.');
    if (!ok) return;
    const { resetButton, resetInfo } = this.el;
    resetButton.disabled = true;
    clearTimeout(this.resetInfoTimer);
    resetInfo.dataset.tone = '';
    resetInfo.textContent = 'Resetting…';
    try {
      await post('api/navigation/reset_map');
      resetInfo.textContent = 'Map reset. The new map appears in a few seconds.';
    } catch (err) {
      resetInfo.dataset.tone = 'bad';
      resetInfo.textContent = `Reset failed: ${err.message}`;
    } finally {
      resetButton.disabled = false;
      this.resetInfoTimer = setTimeout(() => { resetInfo.textContent = ''; }, 10000);
    }
  }

  // ------------------------------------------------------------ view

  statusText() {
    const nav = this.nav;
    if (this.error) return [this.error, 'bad'];
    if (this.mode === 'pick') return ['Press on the goal and drag towards the direction the robot should face.', ''];
    if (this.draft) return [`Goal ${describeGoal(this.draft)}. Drag again to change it.`, 'draft'];
    if (!nav.ready) return ['Nav2 is not running. Start Navigation with Lidar SLAM + Nav2.', ''];
    if (nav.status === 'sending') return [nav.trajectory ? 'Sending trajectory…' : 'Sending goal…', ''];
    if (nav.status === 'active' && nav.trajectory) return [nav.message || describeTrajectory(nav), 'active'];
    if (nav.status === 'active') {
      const fb = nav.feedback;
      if (nav.message || !fb) return [nav.message || 'Driving to the goal…', 'active'];
      const parts = [`Driving to the goal · ${fb.distance_remaining.toFixed(1)} m left`];
      if (fb.eta_s > 0) parts.push(`~${Math.round(fb.eta_s)} s`);
      if (fb.recoveries) parts.push(`${fb.recoveries} recover${fb.recoveries === 1 ? 'y' : 'ies'}`);
      return [parts.join(' · '), 'active'];
    }
    return [nav.message, TONES[nav.status] || (nav.status === 'sent' ? 'active' : '')];
  }

  render() {
    const { bar, status, start, send, discard, stop } = this.el;
    const nav = this.nav;
    bar.hidden = !nav; // the feature manager runs without ROS
    this.el.reset.hidden = !nav;
    if (!nav) return;
    const picking = this.mode !== 'idle';
    start.hidden = picking;
    if (this.el.trajectory) this.el.trajectory.hidden = picking;
    start.disabled = !nav.ready;
    send.hidden = !picking;
    send.disabled = !this.draft || this.mode === 'drag' || this.busy || !nav.ready;
    discard.hidden = !picking;
    // Stop stays available while anything drives, also while a new goal is being placed
    stop.hidden = !ACTIVE.includes(nav.status);
    const [text, tone] = this.statusText();
    status.textContent = text || '';
    status.dataset.tone = tone;
  }

  // Called every frame: put the markers where their frames are now
  update(fixedFrame) {
    const nav = this.nav;
    this.place(this.draftMarker, fixedFrame, this.mode !== 'idle' ? this.draft : null);
    this.place(this.goalMarker, fixedFrame, nav && ACTIVE.includes(nav.status) ? nav.goal : null);
  }

  place(marker, fixedFrame, goal) {
    const m = goal && this.viewer.tf.lookup(fixedFrame, goal.frame_id);
    marker.visible = Boolean(m);
    if (!m) return;
    marker.matrix.copy(m).multiply(goalMatrix(goal));
    marker.matrixWorldNeedsUpdate = true;
  }
}
