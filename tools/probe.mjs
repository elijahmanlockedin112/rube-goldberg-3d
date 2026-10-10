// Quick probe: ball rolling on capsule rails + domino topple, timing.
import R from '@dimforge/rapier3d-compat';
await R.init();
const world = new R.World({ x: 0, y: -9.81, z: 0 });
const ip = world.integrationParameters;
console.log('defaults', { dt: ip.dt, lengthUnit: ip.lengthUnit, iters: ip.numSolverIterations, err: ip.normalizedAllowedLinearError, pred: ip.normalizedPredictionDistance });
world.timestep = 1 / 240;
ip.lengthUnit = 0.05;
const ground = world.createRigidBody(R.RigidBodyDesc.fixed());
world.createCollider(R.ColliderDesc.cuboid(2, 0.01, 2).setTranslation(0, -0.01, 0), ground);
// two rails along x, sloped 4%
const r = 0.0125, rr = 0.003, s = 0.016;
const d = Math.sqrt((r + rr) ** 2 - (s / 2) ** 2);
const L = 1, slope = 0.04;
function capsule(a, b, rad) {
  const dx = b[0] - a[0], dy = b[1] - a[1], dz = b[2] - a[2];
  const len = Math.hypot(dx, dy, dz);
  const ux = dx / len, uy = dy / len, uz = dz / len;
  // quat rotating (0,1,0) to u
  const w = 1 + uy; let q;
  if (w < 1e-6) q = { x: 1, y: 0, z: 0, w: 0 };
  else { const n = Math.hypot(uz, 0, -ux, w); q = { x: uz / n, y: 0, z: -ux / n, w: w / n }; }
  return R.ColliderDesc.capsule(len / 2, rad).setTranslation((a[0] + b[0]) / 2, (a[1] + b[1]) / 2, (a[2] + b[2]) / 2).setRotation(q);
}
for (const side of [1, -1]) {
  for (let i = 0; i < 20; i++) {
    const x0 = i * L / 20, x1 = (i + 1) * L / 20;
    world.createCollider(capsule([x0, 0.3 - slope * x0 - d, side * s / 2], [x1, 0.3 - slope * x1 - d, side * s / 2], rr), ground);
  }
}
const ball = world.createRigidBody(R.RigidBodyDesc.dynamic().setTranslation(0.05, 0.3 - slope * 0.05 + 0.0002, 0).setCcdEnabled(true));
world.createCollider(R.ColliderDesc.ball(r).setDensity(7850).setFriction(0.3).setRestitution(0.3), ball);
const t0 = performance.now();
for (let i = 0; i < 240 * 1.5; i++) {
  world.step();
  if (i % 60 === 0) { const p = ball.translation(), v = ball.linvel(); console.log((i / 240).toFixed(2), p.x.toFixed(3), p.y.toFixed(4), p.z.toFixed(4), 'v', Math.hypot(v.x, v.y, v.z).toFixed(3)); }
}
console.log('ms/step', ((performance.now() - t0) / 360).toFixed(3));
const a = 9.81 * Math.sin(Math.atan(slope)) / (1 + 0.4 * r * r / (r * (d / (r + rr))) ** 2);
console.log('theory accel', a.toFixed(3), 'v at 1.5s from 0', (a * 1.5).toFixed(3));
