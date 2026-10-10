// Small vector/quaternion helpers shared by the physics build and the renderer.
// Everything handed to Rapier goes through rnd() so the starting state is
// bit-identical in every JS engine (Math.sin/cos may differ in the last ulp).

export const rnd = (v) => Math.round(v * 1e7) / 1e7;

export const v3 = (x = 0, y = 0, z = 0) => [x, y, z];
export const add = (a, b) => [a[0] + b[0], a[1] + b[1], a[2] + b[2]];
export const sub = (a, b) => [a[0] - b[0], a[1] - b[1], a[2] - b[2]];
export const scale = (a, s) => [a[0] * s, a[1] * s, a[2] * s];
export const dot = (a, b) => a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
export const cross = (a, b) => [a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]];
export const len = (a) => Math.hypot(a[0], a[1], a[2]);
export const norm = (a) => { const l = len(a) || 1; return [a[0] / l, a[1] / l, a[2] / l]; };
export const lerp3 = (a, b, t) => [a[0] + (b[0] - a[0]) * t, a[1] + (b[1] - a[1]) * t, a[2] + (b[2] - a[2]) * t];

// Quaternions are {x,y,z,w}, the shape Rapier and THREE both accept.
export const qId = () => ({ x: 0, y: 0, z: 0, w: 1 });
export function qAxis(ax, ang) {
  const n = norm(ax), s = Math.sin(ang / 2);
  return { x: n[0] * s, y: n[1] * s, z: n[2] * s, w: Math.cos(ang / 2) };
}
export function qMul(a, b) {
  return {
    x: a.w * b.x + a.x * b.w + a.y * b.z - a.z * b.y,
    y: a.w * b.y - a.x * b.z + a.y * b.w + a.z * b.x,
    z: a.w * b.z + a.x * b.y - a.y * b.x + a.z * b.w,
    w: a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z,
  };
}
export function qRot(q, v) {
  const [x, y, z] = v;
  const ix = q.w * x + q.y * z - q.z * y;
  const iy = q.w * y + q.z * x - q.x * z;
  const iz = q.w * z + q.x * y - q.y * x;
  const iw = -q.x * x - q.y * y - q.z * z;
  return [
    ix * q.w + iw * -q.x + iy * -q.z - iz * -q.y,
    iy * q.w + iw * -q.y + iz * -q.x - ix * -q.z,
    iz * q.w + iw * -q.z + ix * -q.y - iy * -q.x,
  ];
}
// Rotation taking +Y onto dir (used for capsules and cylinders).
export function qYTo(dir) {
  const [ux, uy, uz] = norm(dir);
  const w = 1 + uy;
  if (w < 1e-8) return { x: 1, y: 0, z: 0, w: 0 };
  const n = Math.hypot(uz, ux, w);
  return { x: uz / n, y: 0, z: -ux / n, w: w / n };
}
// Rotation whose local axes map to the given orthonormal basis (columns X,Y,Z).
export function qBasis(X, Y, Z) {
  const m00 = X[0], m01 = Y[0], m02 = Z[0];
  const m10 = X[1], m11 = Y[1], m12 = Z[1];
  const m20 = X[2], m21 = Y[2], m22 = Z[2];
  const tr = m00 + m11 + m22;
  let x, y, z, w;
  if (tr > 0) {
    const s = 0.5 / Math.sqrt(tr + 1);
    w = 0.25 / s; x = (m21 - m12) * s; y = (m02 - m20) * s; z = (m10 - m01) * s;
  } else if (m00 > m11 && m00 > m22) {
    const s = 2 * Math.sqrt(1 + m00 - m11 - m22);
    w = (m21 - m12) / s; x = 0.25 * s; y = (m01 + m10) / s; z = (m02 + m20) / s;
  } else if (m11 > m22) {
    const s = 2 * Math.sqrt(1 + m11 - m00 - m22);
    w = (m02 - m20) / s; x = (m01 + m10) / s; y = 0.25 * s; z = (m12 + m21) / s;
  } else {
    const s = 2 * Math.sqrt(1 + m22 - m00 - m11);
    w = (m10 - m01) / s; x = (m02 + m20) / s; y = (m12 + m21) / s; z = 0.25 * s;
  }
  const n = Math.hypot(x, y, z, w);
  return { x: x / n, y: y / n, z: z / n, w: w / n };
}
export const rq = (q) => ({ x: rnd(q.x), y: rnd(q.y), z: rnd(q.z), w: rnd(q.w) });
export const rv = (p) => ({ x: rnd(p[0]), y: rnd(p[1]), z: rnd(p[2]) });

// Tilt of a body's local +Y axis away from world up, in radians.
export function tiltOf(rot) {
  const up = qRot(rot, [0, 1, 0]);
  return Math.acos(Math.max(-1, Math.min(1, up[1])));
}

// Deterministic PRNG (mulberry32) for pip patterns and perturbation tests.
export function rng(seed) {
  let a = seed >>> 0;
  return () => {
    a = (a + 0x6d2b79f5) >>> 0;
    let t = a;
    t = Math.imul(t ^ (t >>> 15), t | 1);
    t ^= t + Math.imul(t ^ (t >>> 7), t | 61);
    return ((t ^ (t >>> 14)) >>> 0) / 4294967296;
  };
}
