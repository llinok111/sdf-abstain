import test from 'node:test';
import assert from 'node:assert/strict';
import {
  closestPointOnTriangle, distanceToMesh, distanceBrute, signedDistance, windingNumber,
  meshStats, buildBVH, uvSphere, openSphere, cube,
} from '../src/core.mjs';

test('closest point on a triangle: face, edge, vertex', () => {
  const a = [0, 0, 0], b = [1, 0, 0], c = [0, 1, 0];
  assert.deepEqual(closestPointOnTriangle([0.2, 0.2, 1], a, b, c).map(x => +x.toFixed(12)), [0.2, 0.2, 0]);
  assert.deepEqual(closestPointOnTriangle([2, 0, 0], a, b, c), [1, 0, 0]);
  assert.deepEqual(closestPointOnTriangle([-1, -1, 0], a, b, c), [0, 0, 0]);
});

test('sphere is closed and consistently oriented after welding', () => {
  const s = uvSphere(32, 16);
  const st = meshStats(s);
  assert.equal(st.boundary, 0);
  assert.equal(st.nonManifold, 0);
  assert.equal(st.flipped, 0);
  assert.equal(st.watertight, true);
});

test('cube is closed and signed distance has the right sign', () => {
  const c = cube(2);
  assert.equal(meshStats(c).watertight, true);
  assert.equal(signedDistance([0, 0, 0], c).signed, -1);
  assert.equal(signedDistance([2, 0, 0], c).signed, 1);
});

test('sphere SDF matches the analytic distance', () => {
  const s = uvSphere(64, 32);
  const bvh = buildBVH(s);
  for (const p of [[0, 0, 0], [0.5, 0, 0], [2, 0, 0], [0.3, 0.4, 0.5], [-0.9, 0.1, 0.2]]) {
    const got = distanceToMesh(p, s, bvh);
    const want = Math.abs(Math.hypot(...p) - 1);
    assert.ok(Math.abs(got - want) < 5e-3, `${p}: ${got} vs ${want}`);
  }
});

test('BVH and brute force agree', () => {
  const s = uvSphere(32, 16);
  const bvh = buildBVH(s);
  let worst = 0;
  for (let i = 0; i < 500; i++) {
    const p = [Math.sin(i * 1.7) * 1.4, Math.cos(i * 0.9) * 1.4, Math.sin(i * 0.31) * 1.4];
    worst = Math.max(worst, Math.abs(distanceToMesh(p, s, bvh) - distanceBrute(p, s)));
  }
  assert.ok(worst < 1e-9, `max error ${worst}`);
});

test('open mesh refuses to return a sign', () => {
  const o = openSphere(16, 8);
  assert.equal(meshStats(o).watertight, false);
  const r = signedDistance([0, 0, 0], o);
  assert.equal(r.sign, null);
  assert.equal(r.signed, null);
  assert.match(r.reason, /boundary=\d+/);
});

test('winding number is 1 inside and 0 outside a sphere', () => {
  const s = uvSphere(32, 16);
  assert.ok(Math.abs(windingNumber([0, 0, 0], s) - 1) < 1e-9);
  assert.ok(Math.abs(windingNumber([2, 0, 0], s)) < 1e-9);
});

test('sign agrees with the analytic sphere oracle', () => {
  const s = uvSphere(32, 16);
  const bvh = buildBVH(s);
  for (let i = 0; i < 200; i++) {
    const p = [Math.sin(i) * 1.3, Math.cos(i * 0.7) * 1.3, Math.sin(i * 0.3) * 1.3];
    const r = signedDistance(p, s, bvh);
    assert.equal(r.sign, Math.hypot(...p) < 1 ? -1 : 1);
  }
});
