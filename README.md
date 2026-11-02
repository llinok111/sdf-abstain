# sdf-abstain

**Point-to-mesh signed distance that refuses to guess a sign on an open mesh.**
The distance is a BVH-accelerated closest-triangle query; the sign is a winding-number
integral. If the mesh is not closed and consistently oriented, the result has
`sign: null` and a counted reason instead of a plausible-looking negative number.

Upstream: 原创 (original implementation of standard closest-point / winding-number
geometry; the abstention contract and the open-mesh oracle are this repository's).

## What it does

```js
import {signedDistance, uvSphere, openSphere} from './src/core.mjs';

const closed = uvSphere(64, 32);
signedDistance([0, 0, 0], closed);
// {distance: 0.9976, sign: -1, signed: -0.9976, watertight: true, winding: 1}

const open = openSphere(32, 16);
signedDistance([0, 0, 0], open);
// {distance: 0.9626, sign: null, signed: null, watertight: false,
//  reason: "mesh is not closed/oriented: boundary=32, ..."}
```

- `distanceToMesh(p, mesh, bvh)` / `distanceBrute(p, mesh)` — BVH vs all-triangle
  reference.
- `signedDistance(...)` — distance + sign, or an explicit abstention.
- `meshStats(mesh)` — boundary edges, non-manifold edges, orientation flips,
  degenerate faces, watertight verdict.
- `windingNumber(p, mesh)` — the sign oracle, exposed separately.
- `uvSphere`, `openSphere`, `cube` — deterministic fixtures, no data files.

## Measured results

Sphere 64×32: **1,986 vertices / 3,968 faces**, welded and closed. 20,000 query points,
1,000-point brute-force sample, CPU Node 24:

| metric | value |
|---|---|
| BVH, 20,000 queries | **0.459 s** |
| brute force, 1,000 queries | 0.218 s |
| brute force projected to 20,000 | 4.355 s |
| projected speedup | **9.48×** |
| max absolute distance error vs brute | **0** (bit-identical on 1,000 points) |
| sign vs analytic sphere (1,000 points) | **0 mismatches** |
| open mesh (32 boundary edges), 1,000 queries | **1,000/1,000 abstained** |

The brute-force 20k number is **projected** from a measured 1k run, not measured at
20k; that is labelled in `bench/result.json` as `brute_projected_20k_seconds`.

## Why abstention is the feature

A signed distance is only meaningful with a defined inside and outside. For a mesh with
a hole, a flipped triangle, or a non-manifold edge, the winding number can be 0.3 or
0.7 — no sign is defensible. Most libraries return the sign of the nearest triangle
normal or round the winding number; both produce silent, plausible garbage that then
propagates into collision, offsetting, or SDF baking.

This tool separates the two questions:

1. **distance** — always defined, and verified against brute force;
2. **sign** — returned only when `meshStats()` says closed, oriented, manifold and
   non-degenerate; otherwise `null` plus the exact boundary/flip counts.

## Run

Node 20+, no dependencies.

```bash
node --test tests/*.test.mjs    # 8 tests
node bench/bench.mjs            # writes bench/result.json
```

## Limits

- Winding number is O(F) per query; `signedDistance` pays it after the BVH distance.
  The 9.48× figure is for distance only.
- No pseudo-normal or edge-case epsilon policy for points exactly on the surface;
  `sign` at distance 0 is whichever side the winding integral lands on.
- The BVH is rebuilt per call unless you pass one; build once for batches.
- Meshes are `{vertices: [[x,y,z]], faces: [[a,b,c]]}`; no file format loader.

## License

MIT.
