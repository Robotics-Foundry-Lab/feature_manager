// Decode ROS 2 messages (CDR) using the .msg schema text that foxglove_bridge advertises.
//
// Sequences of numeric primitives and of "flat" structs whose fields all share one numeric type
// (geometry_msgs/Point32, std_msgs/ColorRGBA, nvblox_msgs/Index3D, ...) are returned as typed
// arrays (x0, y0, z0, x1, ...) instead of arrays of objects. That keeps big point clouds fast.

const PRIMITIVES = {
  bool: { size: 1, get: (dv, o) => dv.getUint8(o) !== 0, array: Uint8Array },
  byte: { size: 1, get: (dv, o) => dv.getUint8(o), array: Uint8Array },
  char: { size: 1, get: (dv, o) => dv.getUint8(o), array: Uint8Array },
  int8: { size: 1, get: (dv, o) => dv.getInt8(o), array: Int8Array },
  uint8: { size: 1, get: (dv, o) => dv.getUint8(o), array: Uint8Array },
  int16: { size: 2, get: (dv, o, le) => dv.getInt16(o, le), array: Int16Array },
  uint16: { size: 2, get: (dv, o, le) => dv.getUint16(o, le), array: Uint16Array },
  int32: { size: 4, get: (dv, o, le) => dv.getInt32(o, le), array: Int32Array },
  uint32: { size: 4, get: (dv, o, le) => dv.getUint32(o, le), array: Uint32Array },
  int64: { size: 8, get: (dv, o, le) => Number(dv.getBigInt64(o, le)), array: null },
  uint64: { size: 8, get: (dv, o, le) => Number(dv.getBigUint64(o, le)), array: null },
  float32: { size: 4, get: (dv, o, le) => dv.getFloat32(o, le), array: Float32Array },
  float64: { size: 8, get: (dv, o, le) => dv.getFloat64(o, le), array: Float64Array },
};

// Some bridges leave out the definitions of the builtin types
const BUILTINS = {
  'builtin_interfaces/Time': 'int32 sec\nuint32 nanosec',
  'builtin_interfaces/Duration': 'int32 sec\nuint32 nanosec',
};

const textDecoder = new TextDecoder();

function normalizeName(name) {
  // "pkg/msg/Type" -> "pkg/Type"
  const parts = name.split('/');
  return parts.length === 3 ? `${parts[0]}/${parts[2]}` : name;
}

function parseFields(body, pkg) {
  const fields = [];
  for (let line of body.split('\n')) {
    line = line.replace(/#.*$/, '').trim();
    if (!line) continue;
    if (/^\S+\s+\w+\s*=/.test(line)) continue; // constant
    const [rawType, name] = line.split(/\s+/);
    const m = rawType.match(/^([^[\]]+)(\[(<=)?(\d*)\])?$/);
    if (!m || !name) throw new Error(`Cannot parse field "${line}"`);
    let base = m[1].replace(/<=\d+$/, ''); // bounded string
    const isArray = Boolean(m[2]);
    const fixedLength = isArray && !m[3] && m[4] ? Number(m[4]) : null;
    if (base === 'string' || base === 'wstring' || base in PRIMITIVES) {
      // primitive
    } else if (base === 'Header') {
      base = 'std_msgs/Header';
    } else if (base.includes('/')) {
      base = normalizeName(base);
    } else {
      base = `${pkg}/${base}`;
    }
    fields.push({ name, type: base, isArray, fixedLength });
  }
  return fields;
}

export function parseSchema(rootName, text) {
  const types = new Map();
  const sections = text.split(/^={10,}\s*$/m);
  sections.forEach((section, i) => {
    let name = rootName;
    let body = section;
    if (i > 0) {
      const m = section.match(/^\s*MSG:\s*(\S+)\s*$/m);
      if (!m) return;
      name = m[1];
      body = section.slice(section.indexOf(m[0]) + m[0].length);
    }
    name = normalizeName(name);
    types.set(name, parseFields(body, name.split('/')[0]));
  });
  for (const [name, def] of Object.entries(BUILTINS)) {
    if (!types.has(name)) types.set(name, parseFields(def, 'builtin_interfaces'));
  }
  return types;
}

// Element type of a struct whose fields are all scalars of one numeric primitive, else null
function flatPrimitive(types, typeName) {
  const fields = types.get(typeName);
  if (!fields || !fields.length) return null;
  const kind = fields[0].type;
  if (!PRIMITIVES[kind] || !PRIMITIVES[kind].array || kind === 'bool') return null;
  return fields.every((f) => f.type === kind && !f.isArray) ? kind : null;
}

class Reader {
  constructor(buffer, littleEndian) {
    this.buffer = buffer;
    this.dv = new DataView(buffer);
    this.le = littleEndian;
    // The payload was copied to offset 4, so the CDR data (after the 4-byte encapsulation
    // header) starts at offset 8 and CDR alignment equals alignment inside the ArrayBuffer.
    this.origin = 8;
    this.pos = 8;
  }

  align(n) {
    const r = (this.pos - this.origin) % n;
    if (r) this.pos += n - r;
  }

  primitive(kind) {
    const p = PRIMITIVES[kind];
    this.align(p.size);
    const v = p.get(this.dv, this.pos, this.le);
    this.pos += p.size;
    return v;
  }

  string() {
    const len = this.primitive('uint32');
    const bytes = new Uint8Array(this.buffer, this.pos, Math.max(len - 1, 0));
    this.pos += len;
    return textDecoder.decode(bytes);
  }

  // `count` elements of a numeric primitive as a typed array (copy-free on little endian)
  typedArray(kind, count) {
    const p = PRIMITIVES[kind];
    if (count === 0) return new p.array(0);
    this.align(p.size);
    let out;
    if (this.le || p.size === 1) {
      out = new p.array(this.buffer, this.pos, count);
    } else {
      out = new p.array(count);
      for (let i = 0; i < count; i++) out[i] = p.get(this.dv, this.pos + i * p.size, false);
    }
    this.pos += count * p.size;
    return out;
  }
}

export class MessageDecoder {
  constructor(schemaName, schemaText) {
    this.rootName = normalizeName(schemaName);
    this.types = parseSchema(this.rootName, schemaText);
    this.flat = new Map();
    for (const name of this.types.keys()) this.flat.set(name, flatPrimitive(this.types, name));
  }

  decode(payload) {
    // Copy to offset 4 of a fresh buffer: aligned typed-array views become possible
    const buffer = new ArrayBuffer(payload.byteLength + 4);
    new Uint8Array(buffer, 4).set(payload);
    const littleEndian = (new Uint8Array(buffer)[5] & 1) === 1;
    return this.readType(new Reader(buffer, littleEndian), this.rootName);
  }

  readType(r, typeName) {
    const fields = this.types.get(typeName);
    if (!fields) throw new Error(`Unknown message type ${typeName}`);
    const out = {};
    for (const f of fields) out[f.name] = f.isArray ? this.readArray(r, f) : this.readValue(r, f.type);
    return out;
  }

  readValue(r, type) {
    if (type === 'string' || type === 'wstring') return r.string();
    if (type in PRIMITIVES) return r.primitive(type);
    return this.readType(r, type);
  }

  readArray(r, f) {
    const count = f.fixedLength ?? r.primitive('uint32');
    const prim = PRIMITIVES[f.type];
    if (prim && prim.array) return r.typedArray(f.type, count);
    const flatKind = this.flat.get(f.type);
    if (flatKind) return r.typedArray(flatKind, count * this.types.get(f.type).length);
    const out = new Array(count);
    for (let i = 0; i < count; i++) out[i] = this.readValue(r, f.type);
    return out;
  }
}

// Turn either a packed typed array or an array of {x, y, z} objects into a Float32Array
export function toFloat32(values, keys) {
  if (ArrayBuffer.isView(values)) return values instanceof Float32Array ? values : Float32Array.from(values);
  const out = new Float32Array(values.length * keys.length);
  values.forEach((v, i) => keys.forEach((k, j) => { out[i * keys.length + j] = v[k]; }));
  return out;
}
