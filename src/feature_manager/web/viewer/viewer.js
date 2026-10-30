// 3D map view: renders nvblox voxel layers, the 2D map, costmaps, paths and the robot from the
// foxglove_bridge of the Navigation container. Rendering happens in the browser; the robot only
// streams the topics that are switched on. Navigation goals are set in goal.js, waypoint
// trajectories in trajectory.js.

import * as THREE from 'three';
import { OrbitControls } from '../vendor/OrbitControls.js';
import { FoxgloveClient } from './foxglove.js';
import { GoalTool } from './goal.js';
import { TrajectoryTool } from './trajectory.js';
import { toFloat32 } from './ros2msg.js';

const SETTINGS_KEY = 'feature-manager.viewer';
const TF_SCHEMA = 'tf2_msgs/msg/TFMessage';

// Layers shown in the panel. `topic` is the preferred topic; any topic with the same schema can
// be selected instead.
const LAYERS = [
  { id: 'voxels', label: '3D voxel map', kind: 'voxels', schema: 'nvblox_msgs/msg/VoxelBlockLayer',
    topic: '/nvblox_node/color_layer', enabled: true,
    hint: 'Turn on 3D Mapping (NVBLOX) in Navigation.' },
  { id: 'mesh', label: '3D mesh', kind: 'mesh', schema: 'nvblox_msgs/msg/Mesh',
    topic: '/nvblox_node/mesh', enabled: false, hint: 'Turn on 3D Mapping (NVBLOX) in Navigation.' },
  { id: 'map', label: '2D map', kind: 'grid', schema: 'nav_msgs/msg/OccupancyGrid', topic: '/map',
    enabled: true, palette: 'map', z: -0.02, hint: 'Turn on Lidar SLAM + Nav2 in Navigation.' },
  { id: 'global_costmap', label: 'Global costmap', kind: 'grid', schema: 'nav_msgs/msg/OccupancyGrid',
    topic: '/global_costmap/costmap', enabled: false, palette: 'costmap', z: -0.01 },
  { id: 'local_costmap', label: 'Local costmap', kind: 'grid', schema: 'nav_msgs/msg/OccupancyGrid',
    topic: '/local_costmap/costmap', enabled: false, palette: 'costmap', z: 0.0 },
  { id: 'plan', label: 'Planned path', kind: 'path', schema: 'nav_msgs/msg/Path', topic: '/plan',
    enabled: true, color: 0x34c47c },
  { id: 'slam_path', label: 'VSLAM trajectory', kind: 'path', schema: 'nav_msgs/msg/Path',
    topic: '/visual_slam/tracking/slam_path', enabled: false, color: 0xf0b350 },
];

const LIFETIMES = { 10: '10 s (like Foxglove)', 60: '60 s', 0: 'Forever' };

const IDENTITY = new THREE.Matrix4();
const UNIT_SCALE = new THREE.Vector3(1, 1, 1);
const BOX = new THREE.BoxGeometry(1, 1, 1);

const stripSlash = (frame) => (frame || '').replace(/^\//, '');

function loadSettings() {
  try {
    return JSON.parse(localStorage.getItem(SETTINGS_KEY)) || {};
  } catch {
    return {};
  }
}

function saveSettings(settings) {
  try {
    localStorage.setItem(SETTINGS_KEY, JSON.stringify(settings));
  } catch {
    // storage unavailable (private mode); settings just aren't remembered
  }
}

// nvblox colors are sRGB; three.js shades in linear space
const SRGB_TO_LINEAR = new Float32Array(1024).map((_, i) => {
  const c = i / 1023;
  return c <= 0.04045 ? c / 12.92 : ((c + 0.055) / 1.055) ** 2.4;
});
const toLinear = (c) => SRGB_TO_LINEAR[Math.min(1023, Math.max(0, (c * 1023 + 0.5) | 0))];

function poseMatrix(position, orientation, target = new THREE.Matrix4()) {
  return target.compose(
    new THREE.Vector3(position.x, position.y, position.z),
    new THREE.Quaternion(orientation.x, orientation.y, orientation.z, orientation.w),
    UNIT_SCALE);
}

// ------------------------------------------------------------------------------------- TF

class TfBuffer {
  constructor() {
    this.edges = new Map(); // child -> {parent, matrix: T_parent_child}
    this.version = 0;
  }

  add(msg) {
    for (const t of msg.transforms) {
      const child = stripSlash(t.child_frame_id);
      const parent = stripSlash(t.header.frame_id);
      if (!child || !parent || child === parent) continue;
      const prev = this.edges.get(child);
      const matrix = poseMatrix(t.transform.translation, t.transform.rotation, prev && prev.matrix);
      if (!prev || prev.parent !== parent) this.version++;
      this.edges.set(child, { parent, matrix });
    }
  }

  has(frame) {
    return this.edges.has(frame) || [...this.edges.values()].some((e) => e.parent === frame);
  }

  frames() {
    const set = new Set();
    for (const [child, e] of this.edges) set.add(child).add(e.parent);
    return [...set].sort();
  }

  toRoot(frame) {
    const matrix = new THREE.Matrix4();
    let current = frame;
    for (let i = 0; i < 64; i++) {
      const edge = this.edges.get(current);
      if (!edge) return { root: current, matrix };
      matrix.premultiply(edge.matrix);
      current = edge.parent;
    }
    return null; // loop in the tree
  }

  // T_target_source, or null if the frames are not connected
  lookup(target, source) {
    if (!source || source === target) return IDENTITY;
    const a = this.toRoot(source);
    const b = this.toRoot(target);
    if (!a || !b || a.root !== b.root) return null;
    return b.matrix.invert().multiply(a.matrix);
  }
}

// ---------------------------------------------------------------------------------- layers

class FramedLayer {
  constructor(parent) {
    this.group = new THREE.Group();
    this.group.matrixAutoUpdate = false;
    this.frameId = '';
    parent.add(this.group);
  }

  place(tf, fixedFrame, visible) {
    const m = visible ? tf.lookup(fixedFrame, this.frameId) : null;
    this.group.visible = Boolean(m);
    if (m) {
      this.group.matrix.copy(m);
      this.group.matrixWorldNeedsUpdate = true;
    }
  }

  stats() {
    return null;
  }
}

class VoxelLayer extends FramedLayer {
  constructor(parent) {
    super(parent);
    this.blocks = new Map(); // "x,y,z" -> {centers, colors, t}
    this.voxelSize = 0.05;
    this.mode = 'cubes';
    this.lifetimeMs = 10000;
    this.dirty = false;
    this.count = 0;
    this.capacity = 0;
    this.object = null;
    this.cubeMaterial = new THREE.MeshLambertMaterial();
    this.pointMaterial = new THREE.PointsMaterial({ size: 0.05, sizeAttenuation: true, vertexColors: true });
  }

  clear() {
    this.blocks.clear();
    this.dirty = true;
  }

  onMessage(msg) {
    if (msg.clear) this.blocks.clear();
    this.frameId = stripSlash(msg.header.frame_id);
    this.voxelSize = msg.voxel_size_m > 0 ? msg.voxel_size_m : msg.block_size_m / 8;
    const idx = msg.block_indices;
    const index = ArrayBuffer.isView(idx) ? idx : Int32Array.from(idx.flatMap((i) => [i.x, i.y, i.z]));
    const now = performance.now();
    msg.blocks.forEach((block, i) => {
      const key = `${index[3 * i]},${index[3 * i + 1]},${index[3 * i + 2]}`;
      const centers = toFloat32(block.centers, ['x', 'y', 'z']);
      // An empty block means nvblox removed it
      if (!centers.length) this.blocks.delete(key);
      else {
        this.blocks.set(key, {
          centers: centers.slice(), colors: toFloat32(block.colors, ['r', 'g', 'b', 'a']).slice(), t: now,
        });
      }
    });
    this.dirty = true;
  }

  expire(now) {
    if (!this.lifetimeMs) return;
    for (const [key, block] of this.blocks) {
      if (now - block.t > this.lifetimeMs) {
        this.blocks.delete(key);
        this.dirty = true;
      }
    }
  }

  setMode(mode) {
    if (mode === this.mode) return;
    this.mode = mode;
    this.disposeObject();
    this.dirty = true;
  }

  disposeObject() {
    if (!this.object) return;
    this.group.remove(this.object);
    if (this.object.isPoints) this.object.geometry.dispose();
    else this.object.dispose();
    this.object = null;
    this.capacity = 0;
  }

  ensureCapacity(n) {
    if (this.object && this.capacity >= n) return;
    this.disposeObject();
    const cap = Math.max(65536, Math.ceil(n * 1.5));
    if (this.mode === 'cubes') {
      const mesh = new THREE.InstancedMesh(BOX, this.cubeMaterial, cap);
      mesh.instanceMatrix.setUsage(THREE.DynamicDrawUsage);
      mesh.instanceColor = new THREE.InstancedBufferAttribute(new Float32Array(cap * 3), 3);
      mesh.instanceColor.setUsage(THREE.DynamicDrawUsage);
      mesh.count = 0;
      this.object = mesh;
    } else {
      const geometry = new THREE.BufferGeometry();
      geometry.setAttribute('position', new THREE.BufferAttribute(new Float32Array(cap * 3), 3).setUsage(THREE.DynamicDrawUsage));
      geometry.setAttribute('color', new THREE.BufferAttribute(new Float32Array(cap * 3), 3).setUsage(THREE.DynamicDrawUsage));
      this.object = new THREE.Points(geometry, this.pointMaterial);
    }
    this.object.frustumCulled = false;
    this.capacity = cap;
    this.group.add(this.object);
  }

  rebuild() {
    this.dirty = false;
    let n = 0;
    for (const b of this.blocks.values()) n += b.centers.length / 3;
    this.count = n;
    this.ensureCapacity(n);
    const s = this.voxelSize;
    let positions;
    let colors;
    let matrices = null;
    if (this.mode === 'cubes') {
      matrices = this.object.instanceMatrix.array;
      colors = this.object.instanceColor.array;
    } else {
      positions = this.object.geometry.attributes.position.array;
      colors = this.object.geometry.attributes.color.array;
      this.pointMaterial.size = s;
    }
    let i = 0;
    for (const { centers, colors: rgba } of this.blocks.values()) {
      for (let j = 0, k = 0; j < centers.length; j += 3, k += 4, i++) {
        if (matrices) {
          const o = i * 16;
          matrices[o] = s; matrices[o + 5] = s; matrices[o + 10] = s; matrices[o + 15] = 1;
          matrices[o + 12] = centers[j]; matrices[o + 13] = centers[j + 1]; matrices[o + 14] = centers[j + 2];
        } else {
          positions[i * 3] = centers[j]; positions[i * 3 + 1] = centers[j + 1]; positions[i * 3 + 2] = centers[j + 2];
        }
        const has = k + 2 < rgba.length;
        colors[i * 3] = has ? toLinear(rgba[k]) : 0.5;
        colors[i * 3 + 1] = has ? toLinear(rgba[k + 1]) : 0.5;
        colors[i * 3 + 2] = has ? toLinear(rgba[k + 2]) : 0.5;
      }
    }
    if (matrices) {
      this.object.count = n;
      this.object.instanceMatrix.needsUpdate = true;
      this.object.instanceColor.needsUpdate = true;
    } else {
      const geo = this.object.geometry;
      geo.setDrawRange(0, n);
      geo.attributes.position.needsUpdate = true;
      geo.attributes.color.needsUpdate = true;
    }
  }

  stats() {
    return `${this.count.toLocaleString()} voxels in ${this.blocks.size.toLocaleString()} blocks`;
  }
}

class MeshLayer extends FramedLayer {
  constructor(parent) {
    super(parent);
    this.blocks = new Map(); // "x,y,z" -> {mesh, t}
    this.lifetimeMs = 10000;
    this.material = new THREE.MeshLambertMaterial({ vertexColors: true, side: THREE.DoubleSide });
  }

  remove(key) {
    const block = this.blocks.get(key);
    if (!block) return;
    this.group.remove(block.mesh);
    block.mesh.geometry.dispose();
    this.blocks.delete(key);
  }

  clear() {
    for (const key of [...this.blocks.keys()]) this.remove(key);
  }

  onMessage(msg) {
    if (msg.clear) this.clear();
    this.frameId = stripSlash(msg.header.frame_id);
    const idx = msg.block_indices;
    const index = ArrayBuffer.isView(idx) ? idx : Int32Array.from(idx.flatMap((i) => [i.x, i.y, i.z]));
    const now = performance.now();
    msg.blocks.forEach((block, i) => {
      const key = `${index[3 * i]},${index[3 * i + 1]},${index[3 * i + 2]}`;
      this.remove(key);
      const vertices = toFloat32(block.vertices, ['x', 'y', 'z']);
      if (!vertices.length) return;
      const geometry = new THREE.BufferGeometry();
      geometry.setAttribute('position', new THREE.BufferAttribute(vertices.slice(), 3));
      const normals = toFloat32(block.normals, ['x', 'y', 'z']);
      if (normals.length === vertices.length) geometry.setAttribute('normal', new THREE.BufferAttribute(normals.slice(), 3));
      else geometry.computeVertexNormals();
      const rgba = toFloat32(block.colors, ['r', 'g', 'b', 'a']);
      const rgb = new Float32Array(vertices.length).fill(0.5);
      for (let v = 0; v * 4 + 2 < rgba.length && v * 3 < rgb.length; v++) {
        rgb[v * 3] = toLinear(rgba[v * 4]);
        rgb[v * 3 + 1] = toLinear(rgba[v * 4 + 1]);
        rgb[v * 3 + 2] = toLinear(rgba[v * 4 + 2]);
      }
      geometry.setAttribute('color', new THREE.BufferAttribute(rgb, 3));
      const tri = block.triangles;
      geometry.setIndex(new THREE.BufferAttribute(Uint32Array.from(tri), 1));
      const mesh = new THREE.Mesh(geometry, this.material);
      this.group.add(mesh);
      this.blocks.set(key, { mesh, t: now });
    });
  }

  expire(now) {
    if (!this.lifetimeMs) return;
    for (const [key, block] of this.blocks) if (now - block.t > this.lifetimeMs) this.remove(key);
  }

  stats() {
    return `${this.blocks.size.toLocaleString()} mesh blocks`;
  }
}

const PALETTES = {
  // Like RViz/Foxglove "map": free = light, occupied = dark, unknown = transparent
  map(v, out, o) {
    if (v < 0) { out[o + 3] = 0; return; }
    const g = 255 - Math.round(v * 2.55);
    out[o] = g; out[o + 1] = g; out[o + 2] = g; out[o + 3] = 90 + Math.round(v * 1.4);
  },
  // Like RViz "costmap": blue (low cost) -> red (high), cyan = inscribed, magenta = lethal
  costmap(v, out, o) {
    if (v <= 0) { out[o + 3] = 0; return; }
    if (v >= 100) { out[o] = 255; out[o + 1] = 0; out[o + 2] = 255; out[o + 3] = 220; return; }
    if (v === 99) { out[o] = 0; out[o + 1] = 255; out[o + 2] = 255; out[o + 3] = 200; return; }
    const t = v / 98;
    out[o] = Math.round(255 * t); out[o + 1] = 40; out[o + 2] = Math.round(255 * (1 - t)); out[o + 3] = 150;
  },
};

class GridLayer extends FramedLayer {
  constructor(parent, palette, z) {
    super(parent);
    this.palette = PALETTES[palette];
    this.z = z;
    this.mesh = null;
    this.size = null;
  }

  clear() {
    if (!this.mesh) return;
    this.group.remove(this.mesh);
    this.mesh.geometry.dispose();
    this.mesh.material.map.dispose();
    this.mesh.material.dispose();
    this.mesh = null;
    this.size = null;
  }

  onMessage(msg) {
    const { width: w, height: h, resolution: res, origin } = msg.info;
    if (!w || !h) return;
    this.frameId = stripSlash(msg.header.frame_id);
    const rgba = new Uint8Array(w * h * 4);
    const data = msg.data;
    for (let i = 0; i < w * h; i++) this.palette(data[i], rgba, i * 4);

    const size = `${w}x${h}x${res}`;
    if (this.size !== size) {
      this.clear();
      const texture = new THREE.DataTexture(rgba, w, h, THREE.RGBAFormat);
      texture.magFilter = THREE.NearestFilter;
      texture.colorSpace = THREE.SRGBColorSpace;
      const material = new THREE.MeshBasicMaterial({
        map: texture, transparent: true, depthWrite: false, side: THREE.DoubleSide,
      });
      this.mesh = new THREE.Mesh(new THREE.PlaneGeometry(w * res, h * res), material);
      this.mesh.matrixAutoUpdate = false;
      this.mesh.renderOrder = -1;
      this.group.add(this.mesh);
      this.size = size;
    } else {
      this.mesh.material.map.image.data = rgba;
    }
    this.mesh.material.map.needsUpdate = true;
    // Cell (0, 0) is at the origin pose; the plane geometry is centered
    const m = poseMatrix(origin.position, origin.orientation);
    m.multiply(new THREE.Matrix4().makeTranslation(w * res / 2, h * res / 2, this.z));
    this.mesh.matrix.copy(m);
    this.mesh.matrixWorldNeedsUpdate = true;
    this.info = `${w}×${h} cells, ${res.toFixed(2)} m`;
  }

  stats() {
    return this.info || null;
  }
}

class PathLayer extends FramedLayer {
  constructor(parent, color) {
    super(parent);
    this.line = new THREE.Line(
      new THREE.BufferGeometry(),
      new THREE.LineBasicMaterial({ color, depthTest: false, transparent: true }));
    this.line.renderOrder = 10;
    this.line.frustumCulled = false;
    this.group.add(this.line);
    this.length = 0;
  }

  clear() {
    this.line.geometry.setAttribute('position', new THREE.Float32BufferAttribute([], 3));
    this.length = 0;
  }

  onMessage(msg) {
    this.frameId = stripSlash(msg.header.frame_id);
    const pts = new Float32Array(msg.poses.length * 3);
    msg.poses.forEach((p, i) => {
      pts[i * 3] = p.pose.position.x;
      pts[i * 3 + 1] = p.pose.position.y;
      pts[i * 3 + 2] = p.pose.position.z + 0.03;
    });
    this.line.geometry.setAttribute('position', new THREE.BufferAttribute(pts, 3));
    this.length = msg.poses.length;
  }

  stats() {
    return this.length ? `${this.length} poses` : null;
  }
}

function makeRobot() {
  const robot = new THREE.Group();
  robot.matrixAutoUpdate = false;
  const body = new THREE.Mesh(
    new THREE.BoxGeometry(0.45, 0.35, 0.18),
    new THREE.MeshLambertMaterial({ color: 0x4d8bf5, transparent: true, opacity: 0.6 }));
  body.position.z = 0.09;
  const heading = new THREE.ArrowHelper(new THREE.Vector3(1, 0, 0), new THREE.Vector3(0, 0, 0.2), 0.45, 0xffffff, 0.12, 0.08);
  robot.add(body, heading, new THREE.AxesHelper(0.3));
  return robot;
}

// Some browsers/drivers only give a context with conservative settings
function createRenderer() {
  const attempts = [
    { antialias: true },
    { antialias: false, powerPreference: 'low-power' },
    { antialias: false, powerPreference: 'low-power', precision: 'mediump', failIfMajorPerformanceCaveat: false },
  ];
  for (const options of attempts) {
    try {
      return new THREE.WebGLRenderer(options);
    } catch (err) {
      console.warn('WebGL context failed with', options, err);
    }
  }
  const error = new Error('This browser does not allow WebGL (3D graphics).');
  error.code = 'webgl';
  throw error;
}

// ---------------------------------------------------------------------------------- viewer

export class MapViewer {
  constructor(root, options) {
    this.root = root;
    this.options = options;
    this.settings = {
      enabled: {}, topics: {}, mode: 'cubes', lifetime: 10, follow: false, panelOpen: true,
      ...loadSettings(),
    };
    this.tf = new TfBuffer();
    this.layers = [];
    this.running = false;
    this.lastFrames = '';
    this.robotPos = null;
  }

  // ------------------------------------------------------------ setup

  init() {
    const $ = (sel) => this.root.querySelector(sel);
    this.el = {
      canvas: $('.map-canvas'), overlay: $('.map-overlay'), overlayTitle: $('.map-overlay strong'),
      overlayText: $('.map-overlay p'), panel: $('.map-panel'), layerList: $('.layer-list'),
      fixedFrame: $('.fixed-frame'), mode: $('.render-mode'), lifetime: $('.lifetime'),
      follow: $('.follow'), stats: $('.map-stats'), bridgeState: $('.bridge-state'),
      bridgeUrl: $('.bridge-url'), panelToggle: $('.panel-toggle'),
    };

    this.renderer = createRenderer();
    this.renderer.setPixelRatio(Math.min(window.devicePixelRatio, 2));
    this.el.canvas.append(this.renderer.domElement);

    this.scene = new THREE.Scene();
    this.camera = new THREE.PerspectiveCamera(60, 1, 0.05, 1000);
    this.camera.up.set(0, 0, 1);
    this.camera.position.set(-5, -5, 5);
    this.controls = new OrbitControls(this.camera, this.renderer.domElement);
    this.controls.enableDamping = true;
    this.controls.screenSpacePanning = false; // pan along the ground plane
    this.controls.maxDistance = 300;

    this.scene.add(new THREE.HemisphereLight(0xffffff, 0x606060, 2.4));
    const sun = new THREE.DirectionalLight(0xffffff, 1.6);
    sun.position.set(3, 2, 6);
    this.scene.add(sun);

    this.world = new THREE.Group(); // = fixed frame
    this.scene.add(this.world);
    this.applyTheme();
    window.matchMedia('(prefers-color-scheme: dark)').addEventListener('change', () => this.applyTheme());

    for (const def of LAYERS) {
      let obj;
      if (def.kind === 'voxels') obj = new VoxelLayer(this.world);
      else if (def.kind === 'mesh') obj = new MeshLayer(this.world);
      else if (def.kind === 'grid') obj = new GridLayer(this.world, def.palette, def.z);
      else obj = new PathLayer(this.world, def.color);
      this.layers.push({ def, obj, topic: null, count: 0, history: [], rate: 0, row: null });
    }
    this.voxels = this.layers.find((l) => l.def.kind === 'voxels').obj;
    this.mesh = this.layers.find((l) => l.def.kind === 'mesh').obj;
    this.robot = makeRobot();
    this.world.add(this.robot);

    this.buildPanel();
    try {
      this.goal = new GoalTool(this);
    } catch (err) {
      // The map must still work without goals (e.g. a cached index.html without the goal bar)
      console.error('Navigation goals unavailable:', err);
    }
    try {
      this.trajectory = new TrajectoryTool(this);
    } catch (err) {
      console.error('Trajectories unavailable:', err);
    }
    this.client = new FoxgloveClient(this.bridgeUrl());
    this.client.addEventListener('state', () => this.updateOverlay());
    this.client.addEventListener('channels', () => this.syncLayers());

    new ResizeObserver(() => this.resize()).observe(this.el.canvas);
    this.resize();
  }

  bridgeUrl() {
    const template = this.settings.bridgeUrl || this.options.bridgeUrl || 'ws://{host}:8765';
    return template.replace('{host}', window.location.hostname || 'localhost');
  }

  applyTheme() {
    const css = getComputedStyle(document.documentElement);
    const color = (name, fallback) => new THREE.Color(css.getPropertyValue(name).trim() || fallback);
    this.scene.background = color('--map-bg', '#101318');
    if (this.grid) {
      this.world.remove(this.grid);
      this.grid.dispose();
    }
    this.grid = new THREE.GridHelper(40, 40, color('--map-grid-center', '#4a5260'), color('--map-grid', '#2a3039'));
    this.grid.rotation.x = Math.PI / 2; // GridHelper lies in XZ, ROS ground is XY
    this.world.add(this.grid);
  }

  buildPanel() {
    const s = this.settings;
    const rows = this.layers.map((layer) => {
      const { def } = layer;
      const row = document.createElement('div');
      row.className = 'layer';
      row.innerHTML = `
        <label class="layer-head"><input type="checkbox"><span></span><em class="layer-rate"></em></label>
        <select class="layer-topic"></select>
        <p class="layer-info"></p>`;
      row.querySelector('span').textContent = def.label;
      const check = row.querySelector('input');
      check.checked = s.enabled[def.id] ?? def.enabled;
      check.addEventListener('change', () => {
        s.enabled[def.id] = check.checked;
        saveSettings(s);
        this.syncLayers();
      });
      const select = row.querySelector('select');
      select.addEventListener('change', () => {
        s.topics[def.id] = select.value;
        saveSettings(s);
        this.syncLayers();
      });
      layer.row = { row, check, select, rate: row.querySelector('.layer-rate'), info: row.querySelector('.layer-info') };
      return row;
    });
    this.el.layerList.replaceChildren(...rows);

    this.el.mode.value = s.mode;
    this.el.mode.addEventListener('change', () => {
      s.mode = this.el.mode.value;
      saveSettings(s);
      this.voxels.setMode(s.mode);
    });
    this.voxels.setMode(s.mode);

    this.el.lifetime.replaceChildren(...Object.entries(LIFETIMES).map(([value, label]) => new Option(label, value)));
    this.el.lifetime.value = String(s.lifetime);
    const applyLifetime = () => {
      this.voxels.lifetimeMs = this.mesh.lifetimeMs = Number(s.lifetime) * 1000;
    };
    this.el.lifetime.addEventListener('change', () => {
      s.lifetime = Number(this.el.lifetime.value);
      saveSettings(s);
      applyLifetime();
    });
    applyLifetime();

    this.el.follow.checked = s.follow;
    this.el.follow.addEventListener('change', () => {
      s.follow = this.el.follow.checked;
      saveSettings(s);
    });

    this.el.fixedFrame.addEventListener('change', () => {
      s.fixedFrame = this.el.fixedFrame.value;
      saveSettings(s);
      this.resetView();
    });

    this.el.bridgeUrl.value = s.bridgeUrl || this.options.bridgeUrl || 'ws://{host}:8765';
    this.el.bridgeUrl.addEventListener('change', () => {
      s.bridgeUrl = this.el.bridgeUrl.value.trim();
      saveSettings(s);
      this.client.close();
      this.client.url = this.bridgeUrl();
      this.client.connect();
    });

    for (const btn of this.root.querySelectorAll('[data-view]')) {
      btn.addEventListener('click', () => (btn.dataset.view === 'top' ? this.topView() : this.resetView()));
    }

    const narrow = window.matchMedia('(max-width: 640px)').matches;
    this.setPanelOpen(narrow ? false : s.panelOpen);
    this.el.panelToggle.addEventListener('click', () => {
      s.panelOpen = !this.panelOpen;
      saveSettings(s);
      this.setPanelOpen(s.panelOpen);
    });
  }

  setPanelOpen(open) {
    this.panelOpen = open;
    this.el.panel.dataset.open = open;
    this.el.panelToggle.textContent = open ? 'Hide' : 'Show';
  }

  // ------------------------------------------------------------ data

  syncLayers() {
    const s = this.settings;
    for (const layer of this.layers) {
      const { def, row } = layer;
      const topics = this.client.topicsWithSchema(def.schema);
      const preferred = s.topics[def.id] || def.topic;
      const topic = topics.includes(preferred) ? preferred : (topics.includes(def.topic) ? def.topic : topics[0]);
      const options = topics.length ? topics : ['not published'];
      if (row.select.options.length !== options.length || [...row.select.options].some((o, i) => o.value !== options[i])) {
        row.select.replaceChildren(...options.map((t) => new Option(t, t)));
      }
      row.select.disabled = !topics.length;
      row.select.value = topic || 'not published';
      row.select.hidden = topics.length <= 1;
      layer.available = topics.length > 0;

      const want = row.check.checked && topic ? topic : null;
      if (want !== layer.topic) {
        if (layer.topic) this.client.unsubscribe(layer.topic);
        layer.obj.clear();
        layer.history = [];
        layer.topic = want;
        if (want) {
          this.client.subscribe(want, (msg) => {
            layer.count++;
            layer.obj.onMessage(msg);
          });
        }
      }
    }
    this.client.subscribe('/tf', (msg) => this.tf.add(msg));
    this.client.subscribe('/tf_static', (msg) => this.tf.add(msg));
    this.updateLayerInfo();
  }

  updateLayerInfo() {
    const fixed = this.fixedFrame();
    const open = this.client.state === 'open';
    for (const layer of this.layers) {
      const { def, obj, row } = layer;
      let info = '';
      if (row.check.checked && open) {
        if (!layer.available) info = def.hint || 'Not published.';
        else if (obj.frameId && !this.tf.lookup(fixed, obj.frameId)) info = `No transform from ${obj.frameId} to ${fixed}.`;
        else info = obj.stats() || 'Waiting for data…';
      }
      row.info.textContent = info;
      const rate = layer.rate >= 10 ? layer.rate.toFixed(0) : layer.rate.toFixed(1);
      row.rate.textContent = row.check.checked && layer.topic && open ? `${rate} Hz` : '';
    }
  }

  fixedFrame() {
    const frames = this.tf.frames();
    const wanted = this.settings.fixedFrame || this.options.fixedFrame || 'map';
    if (frames.includes(wanted)) return wanted;
    return ['map', 'odom'].find((f) => frames.includes(f)) || this.voxels.frameId || frames[0] || 'map';
  }

  robotFrame() {
    const candidates = [this.options.robotFrame, 'base_link', 'base_footprint'].filter(Boolean);
    return candidates.find((f) => this.tf.has(f)) || candidates[0];
  }

  // ------------------------------------------------------------ view

  resize() {
    const { clientWidth: w, clientHeight: h } = this.el.canvas;
    if (!w || !h) return;
    this.renderer.setSize(w, h, false);
    this.camera.aspect = w / h;
    this.camera.updateProjectionMatrix();
  }

  robotPosition() {
    const m = this.tf.lookup(this.fixedFrame(), this.robotFrame());
    return m ? new THREE.Vector3().setFromMatrixPosition(m) : new THREE.Vector3();
  }

  resetView() {
    const p = this.robotPosition();
    this.controls.target.copy(p);
    this.camera.position.copy(p).add(new THREE.Vector3(-4, -4, 4));
    this.robotPos = null;
  }

  topView() {
    const p = this.robotPosition();
    this.controls.target.copy(p);
    this.camera.position.copy(p).add(new THREE.Vector3(0, -0.01, 14));
    this.robotPos = null;
  }

  updateOverlay() {
    const state = this.client.state;
    this.el.bridgeState.textContent = { open: '● live', connecting: 'connecting…', error: 'offline', closed: '' }[state];
    this.el.bridgeState.dataset.state = state;
    if (state === 'open') this.everConnected = true;
    this.updateLayerInfo();
    const show = state !== 'open';
    this.el.overlay.hidden = !show;
    if (!show) return;
    const navRunning = this.options.navigationRunning();
    if (state === 'connecting' && !this.everConnected) {
      this.el.overlayTitle.textContent = 'Connecting to the robot…';
      this.el.overlayText.textContent = `Foxglove bridge at ${this.client.url}`;
    } else if (navRunning) {
      this.el.overlayTitle.textContent = 'Waiting for the Foxglove bridge';
      this.el.overlayText.textContent = `Navigation is running, but ${this.client.url} does not answer yet. The bridge starts a few seconds after the container.`;
    } else {
      this.el.overlayTitle.textContent = 'Navigation is not running';
      this.el.overlayText.textContent = 'The 3D map comes from the Foxglove bridge inside the Navigation container. Start Navigation with 3D Mapping (NVBLOX) to see it.';
    }
  }

  updateStats() {
    const fixed = this.fixedFrame();
    const frames = this.tf.frames();
    const key = frames.join(',') + '|' + fixed;
    if (key !== this.lastFrames) {
      this.lastFrames = key;
      const list = frames.length ? frames : [fixed];
      this.el.fixedFrame.replaceChildren(...list.map((f) => new Option(f, f)));
      this.el.fixedFrame.value = fixed;
    }
    // Message rate averaged over the last 5 s (maps and costmaps arrive slower than 1 Hz)
    for (const layer of this.layers) {
      layer.history.push(layer.count);
      if (layer.history.length > 5) layer.history.shift();
      layer.count = 0;
      layer.rate = layer.history.reduce((a, b) => a + b, 0) / layer.history.length;
    }
    this.updateLayerInfo();

    const robot = this.robotFrame();
    const hasRobot = this.tf.has(robot);
    const noTf = this.client.state === 'open' && !this.client.topicsWithSchema(TF_SCHEMA).length;
    this.el.stats.textContent = noTf ? 'No /tf published, so layers cannot be placed.'
      : `Robot frame: ${robot}${hasRobot ? '' : ' (not in TF yet)'} · ${frames.length} frames`;
    if (!this.firstViewDone && hasRobot) {
      this.firstViewDone = true;
      if (this.firstView === 'top') this.topView();
      else this.resetView();
    }
  }

  frame = () => {
    if (!this.running) return;
    this.raf = requestAnimationFrame(this.frame);
    const now = performance.now();
    if (now - (this.lastSecond || 0) > 1000) {
      this.lastSecond = now;
      this.voxels.expire(now);
      this.mesh.expire(now);
      this.updateStats();
    }
    if (this.voxels.dirty && now - (this.lastRebuild || 0) > 200) {
      this.lastRebuild = now;
      this.voxels.rebuild();
    }

    const fixed = this.fixedFrame();
    for (const layer of this.layers) layer.obj.place(this.tf, fixed, layer.row.check.checked && layer.topic);

    const m = this.tf.lookup(fixed, this.robotFrame());
    this.robot.visible = Boolean(m) && this.tf.has(this.robotFrame());
    if (m) {
      this.robot.matrix.copy(m);
      this.robot.matrixWorldNeedsUpdate = true;
      const pos = new THREE.Vector3().setFromMatrixPosition(m);
      if (this.settings.follow && this.robotPos) {
        const delta = pos.clone().sub(this.robotPos);
        this.camera.position.add(delta);
        this.controls.target.add(delta);
      }
      this.robotPos = pos;
    }

    if (this.goal) this.goal.update(fixed);
    if (this.trajectory) this.trajectory.update(fixed);
    this.controls.update();
    this.renderer.render(this.scene, this.camera);
  };

  setNavigation(nav) {
    if (this.goal) this.goal.setNavigation(nav);
    if (this.trajectory) this.trajectory.setNavigation(nav);
  }

  // ------------------------------------------------------------ lifecycle

  start() {
    if (!this.renderer) this.init();
    if (this.running) return;
    this.running = true;
    this.client.connect();
    this.updateOverlay();
    this.resize();
    this.syncLayers();
    this.raf = requestAnimationFrame(this.frame);
  }

  // Leaving the tab closes the connection, so the robot stops streaming the map
  stop() {
    this.running = false;
    cancelAnimationFrame(this.raf);
    if (this.goal) this.goal.cancelDrag();
    if (this.trajectory) this.trajectory.cancelPress();
    if (this.client) {
      this.client.close();
      for (const layer of this.layers) layer.topic = null;
    }
  }
}
