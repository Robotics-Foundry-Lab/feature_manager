#!/usr/bin/env python3
"""Shrink the CAD export of the robot into the model shown in the 3D map of the web UI.

The CAD export (binary STL in mm, per-face colors in the Materialise "COLOR=" format, robot facing
-X, wheels standing on z = min) is moved into base_link (m, x forward, z = 0 on the ground,
origin in the middle of the wheels). It is decimated by clustering the vertices of each color on
a grid: parts smaller than the grid (screws, SMD components) vanish and colors stay separate.
Quadric decimation looked worse, as it cannot shrink these thousands of tiny closed parts.

Usage (on any machine with numpy):
    python3 tools/compress_robot_model.py ../../Roberta.stl web/models/roberta.stl
"""

import argparse

import numpy as np

STL_FACE = np.dtype([('normal', '<f4', 3), ('v', '<f4', (3, 3)), ('attr', '<u2')])


def read_stl(path):
    with open(path, 'rb') as f:
        header = f.read(80)
        count = int(np.frombuffer(f.read(4), '<u4')[0])
        faces = np.fromfile(f, STL_FACE, count)
    if b'COLOR=' not in header:
        raise SystemExit(f'{path} has no per-face colors (Materialise "COLOR=" header)')
    return faces


def to_base_link(v, forward):
    """Mm in CAD coordinates -> m in base_link."""
    lo, hi = v.reshape(-1, 3).min(0), v.reshape(-1, 3).max(0)
    center = np.array([(lo[0] + hi[0]) / 2, (lo[1] + hi[1]) / 2, lo[2]])
    v = v - center
    if forward == '-x':  # turn by 180 deg about z
        v = v * np.array([-1.0, -1.0, 1.0])
    return (v / 1000.0).astype(np.float32)


def cluster(tris, grid):
    """Merge the vertices in each grid cell (m) and drop collapsed and duplicate triangles."""
    points, index = np.unique(np.round(tris.reshape(-1, 3) / grid), axis=0, return_inverse=True)
    faces = index.reshape(-1, 3)
    faces = faces[(faces[:, 0] != faces[:, 1]) & (faces[:, 1] != faces[:, 2])
                  & (faces[:, 0] != faces[:, 2])]
    _, first = np.unique(np.sort(faces, axis=1), axis=0, return_index=True)
    return (points * grid)[faces[np.sort(first)]].astype(np.float32)


def write_stl(path, parts):
    tris = np.concatenate([t for t, _ in parts])
    out = np.zeros(len(tris), STL_FACE)
    out['v'] = tris
    n = np.cross(tris[:, 1] - tris[:, 0], tris[:, 2] - tris[:, 0])
    out['normal'] = n / np.maximum(np.linalg.norm(n, axis=1, keepdims=True), 1e-12)
    out['attr'] = np.concatenate([np.full(len(t), attr, '<u2') for t, attr in parts])
    # Default color (for faces without their own) mid grey, opaque
    header = b'COLOR=' + bytes([128, 128, 128, 255]) + b' base_link, m'
    with open(path, 'wb') as f:
        f.write(header.ljust(80, b' '))
        f.write(np.uint32(len(out)).tobytes())
        out.tofile(f)
    return len(out)


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument('src', help='CAD export (binary STL with colors, mm)')
    parser.add_argument('dst', help='compressed STL for the web UI')
    parser.add_argument('--grid', type=float, default=2.5,
                        help='vertex clustering grid in mm; larger = smaller file')
    parser.add_argument('--forward', choices=['-x', '+x'], default='-x',
                        help='CAD axis the robot faces')
    args = parser.parse_args()

    faces = read_stl(args.src)
    tris = to_base_link(faces['v'], args.forward)
    parts = []
    for attr in np.unique(faces['attr']):
        part = cluster(tris[faces['attr'] == attr], args.grid / 1000.0)
        if len(part):
            parts.append((part, attr))
    written = write_stl(args.dst, parts)
    lo, hi = tris.reshape(-1, 3).min(0), tris.reshape(-1, 3).max(0)
    print(f'{len(faces)} -> {written} triangles, {len(parts)} colors, '
          f'extent {np.round(lo, 3)} .. {np.round(hi, 3)} m')


if __name__ == '__main__':
    main()
