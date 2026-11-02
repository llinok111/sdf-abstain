// Bench: BVH point-mesh distance vs brute force, plus sign agreement against the
// analytic sphere.  Run: node bench/bench.mjs  (writes bench/result.json)

import {writeFileSync} from 'node:fs';
import {
  uvSphere, openSphere, buildBVH, distanceToMesh, distanceBrute, signedDistance, meshStats,
} from '../src/core.mjs';

const sphere = uvSphere(64, 32);
const stats = meshStats(sphere);
const bvh = buildBVH(sphere);
const open = openSphere(32, 16);

function points(n, seed = 12345) {
  let a = seed >>> 0;
  const rnd = () => {
    a = (a + 0x6D2B79F5) | 0;
    let t = Math.imul(a ^ (a >>> 15), 1 | a);
    t = (t + Math.imul(t ^ (t >>> 7), 61 | t)) ^ t;
    return ((t ^ (t >>> 14)) >>> 0) / 4294967296;
  };
  const out = [];
  for (let i = 0; i < n; i++) {
    out.push([(rnd() * 2 - 1) * 1.5, (rnd() * 2 - 1) * 1.5, (rnd() * 2 - 1) * 1.5]);
  }
  return out;
}

const pts = points(20000);
const smallPts = pts.slice(0, 1000);

function time(fn) {
  const t0 = process.hrtime.bigint();
  const out = fn();
  return {seconds: Number(process.hrtime.bigint() - t0) / 1e9, out};
}

const bvhRun = time(() => pts.map(p => distanceToMesh(p, sphere, bvh)));
const bruteRun = time(() => smallPts.map(p => distanceBrute(p, sphere)));
// Scale the brute-force time to 20k points so the comparison is explicit.
const bruteProjected = bruteRun.seconds * (pts.length / smallPts.length);

let worstError = 0;
for (let i = 0; i < smallPts.length; i++) {
  worstError = Math.max(worstError, Math.abs(bvhRun.out[i] - distanceBrute(smallPts[i], sphere)));
}

let signChecked = 0, signMismatch = 0;
for (const p of smallPts) {
  const r = signedDistance(p, sphere, bvh);
  if (r.sign === null) continue;
  signChecked++;
  const analytic = Math.hypot(...p) < 1 ? -1 : 1;
  if (r.sign !== analytic) signMismatch++;
}

let abstained = 0;
for (const p of smallPts) {
  if (signedDistance(p, open).sign === null) abstained++;
}

const result = {
  sphere: {vertices: stats.vertices, faces: stats.faces, watertight: stats.watertight},
  open_sphere: {...meshStats(open)},
  queries: {distance: pts.length, brute: smallPts.length, sign: smallPts.length},
  timing: {
    bvh_20k_seconds: bvhRun.seconds,
    brute_1k_seconds: bruteRun.seconds,
    brute_projected_20k_seconds: bruteProjected,
    speedup_projected: bruteProjected / bvhRun.seconds,
  },
  max_abs_distance_error_vs_brute: worstError,
  sign: {checked: signChecked, mismatches: signMismatch},
  open_mesh_abstained: abstained,
};

console.log(JSON.stringify(result, null, 2));
writeFileSync(new URL('./result.json', import.meta.url), JSON.stringify(result, null, 2) + '\n');
