// Point-to-mesh signed distance with an explicit abstention rule.
//
// The distance part is a BVH over triangles.  The sign part is a winding-number
// integral; if the mesh is not closed and consistently oriented, `signedDistance`
// refuses to return a sign instead of guessing one.  Pure ES module.

const EPS = 1e-12;
const sub = (a, b) => [a[0] - b[0], a[1] - b[1], a[2] - b[2]];
const dot = (a, b) => a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
const cross = (a, b) => [a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]];
const sq = (v) => v[0] * v[0] + v[1] * v[1] + v[2] * v[2];

export function closestPointOnTriangle(p, a, b, c) {
  const ab = sub(b, a), ac = sub(c, a), ap = sub(p, a);
  const d1 = dot(ab, ap), d2 = dot(ac, ap);
  if (d1 <= 0 && d2 <= 0) return a;

  const bp = sub(p, b), d3 = dot(ab, bp), d4 = dot(ac, bp);
  if (d3 >= 0 && d4 <= d3) return b;

  const vc = d1 * d4 - d3 * d2;
  if (vc <= 0 && d1 >= 0 && d3 <= 0) {
    const v = d1 / (d1 - d3);
    return [a[0] + ab[0] * v, a[1] + ab[1] * v, a[2] + ab[2] * v];
  }

  const cp = sub(p, c), d5 = dot(ab, cp), d6 = dot(ac, cp);
  if (d6 >= 0 && d5 <= d6) return c;

  const vb = d5 * d2 - d1 * d6;
  if (vb <= 0 && d2 >= 0 && d6 <= 0) {
    const w = d2 / (d2 - d6);
    return [a[0] + ac[0] * w, a[1] + ac[1] * w, a[2] + ac[2] * w];
  }

  const va = d3 * d6 - d5 * d4;
  if (va <= 0 && (d4 - d3) >= 0 && (d5 - d6) >= 0) {
    const w = (d4 - d3) / ((d4 - d3) + (d5 - d6));
    return [b[0] + (c[0] - b[0]) * w, b[1] + (c[1] - b[1]) * w, b[2] + (c[2] - b[2]) * w];
  }

  const denom = 1 / (va + vb + vc);
  const v = vb * denom, w = vc * denom;
  return [a[0] + ab[0] * v + ac[0] * w,
          a[1] + ab[1] * v + ac[1] * w,
          a[2] + ab[2] * v + ac[2] * w];
}

export function meshStats(mesh) {
  const {vertices, faces} = mesh;
  let min = [Infinity, Infinity, Infinity], max = [-Infinity, -Infinity, -Infinity];
  for (const v of vertices) {
    for (let i = 0; i < 3; i++) {
      if (v[i] < min[i]) min[i] = v[i];
      if (v[i] > max[i]) max[i] = v[i];
    }
  }
  const edges = new Map();
  let degenerate = 0;
  for (const f of faces) {
    const [a, b, c] = f;
    if (a === b || b === c || a === c) { degenerate++; continue; }
    for (const [u, v] of [[a, b], [b, c], [c, a]]) {
      const key = u < v ? `${u},${v}` : `${v},${u}`;
      const sign = u < v ? 1 : -1;
      const rec = edges.get(key) || {count: 0, balance: 0};
      rec.count++;
      rec.balance += sign;
      edges.set(key, rec);
    }
  }
  let boundary = 0, nonManifold = 0;
  for (const rec of edges.values()) {
    if (rec.count === 1) boundary++;
    else if (rec.count > 2) nonManifold++;
  }
  let flipped = 0;
  for (const rec of edges.values()) if (rec.count === 2 && rec.balance !== 0) flipped++;
  return {vertices: vertices.length, faces: faces.length, degenerate, boundary,
          nonManifold, flipped, watertight: boundary === 0 && nonManifold === 0 && flipped === 0 && degenerate === 0,
          min, max};
}

function triBox(tri, box) {
  // Conservative triangle AABB for BVH construction.
  const min = [Math.min(...tri.map(v => v[0])), Math.min(...tri.map(v => v[1])), Math.min(...tri.map(v => v[2]))];
  const max = [Math.max(...tri.map(v => v[0])), Math.max(...tri.map(v => v[1])), Math.max(...tri.map(v => v[2]))];
  box.min = min; box.max = max;
  return box;
}

function boxDistance2(p, box) {
  let d2 = 0;
  for (let i = 0; i < 3; i++) {
    const d = p[i] < box.min[i] ? box.min[i] - p[i] : (p[i] > box.max[i] ? p[i] - box.max[i] : 0);
    d2 += d * d;
  }
  return d2;
}

export function buildBVH(mesh, leafSize = 8) {
  const tris = mesh.faces.map(f => [mesh.vertices[f[0]], mesh.vertices[f[1]], mesh.vertices[f[2]]]);
  const boxes = tris.map(t => triBox(t, {min: null, max: null}));
  const centroids = boxes.map(b => [(b.min[0] + b.max[0]) / 2, (b.min[1] + b.max[1]) / 2, (b.min[2] + b.max[2]) / 2]);

  function build(ids) {
    let min = [Infinity, Infinity, Infinity], max = [-Infinity, -Infinity, -Infinity];
    for (const id of ids) {
      const b = boxes[id];
      for (let i = 0; i < 3; i++) { min[i] = Math.min(min[i], b.min[i]); max[i] = Math.max(max[i], b.max[i]); }
    }
    if (ids.length <= leafSize) return {min, max, ids, left: null, right: null};
    let axis = 0, best = -1;
    for (let i = 0; i < 3; i++) {
      let lo = Infinity, hi = -Infinity;
      for (const id of ids) { lo = Math.min(lo, centroids[id][i]); hi = Math.max(hi, centroids[id][i]); }
      if (hi - lo > best) { best = hi - lo; axis = i; }
    }
    ids.sort((a, b) => centroids[a][axis] - centroids[b][axis]);
    const mid = ids.length >> 1;
    return {min, max, ids: null, left: build(ids.slice(0, mid)), right: build(ids.slice(mid))};
  }
  if (tris.length === 0) {
    const empty = {min: [0, 0, 0], max: [0, 0, 0], ids: [], left: null, right: null};
    empty.tris = tris;
    return empty;
  }
  const root = build(tris.map((_, i) => i));
  root.tris = tris;
  return root;
}

export function distanceToMesh(p, mesh, bvh = buildBVH(mesh)) {
  const tris = bvh.tris || (bvh.tris = mesh.faces.map(f => [mesh.vertices[f[0]], mesh.vertices[f[1]], mesh.vertices[f[2]]]));
  let best = Infinity;
  const stack = [bvh];
  while (stack.length) {
    const node = stack.pop();
    if (boxDistance2(p, node) >= best) continue;
    if (node.ids) {
      for (const id of node.ids) {
        const t = tris[id];
        const q = closestPointOnTriangle(p, t[0], t[1], t[2]);
        const d2 = sq(sub(p, q));
        if (d2 < best) best = d2;
      }
    } else {
      stack.push(node.left, node.right);
    }
  }
  return Math.sqrt(best);
}

export function distanceBrute(p, mesh) {
  let best = Infinity;
  for (const f of mesh.faces) {
    const t = f.map(i => mesh.vertices[i]);
    const q = closestPointOnTriangle(p, t[0], t[1], t[2]);
    const d2 = sq(sub(p, q));
    if (d2 < best) best = d2;
  }
  return Math.sqrt(best);
}

export function windingNumber(p, mesh) {
  let sum = 0;
  for (const f of mesh.faces) {
    const a = sub(mesh.vertices[f[0]], p);
    const b = sub(mesh.vertices[f[1]], p);
    const c = sub(mesh.vertices[f[2]], p);
    const la = Math.sqrt(sq(a)), lb = Math.sqrt(sq(b)), lc = Math.sqrt(sq(c));
    const num = dot(a, cross(b, c));
    const den = la * lb * lc + dot(a, b) * lc + dot(b, c) * la + dot(c, a) * lb;
    sum += 2 * Math.atan2(num, den);
  }
  return sum / (4 * Math.PI);
}

export function signedDistance(p, mesh, bvh = buildBVH(mesh)) {
  const stats = mesh._stats || (mesh._stats = meshStats(mesh));
  const distance = distanceToMesh(p, mesh, bvh);
  if (!stats.watertight) {
    return {distance, sign: null, signed: null, watertight: false,
            reason: `mesh is not closed/oriented: boundary=${stats.boundary}, nonManifold=${stats.nonManifold}, flipped=${stats.flipped}`};
  }
  const w = windingNumber(p, mesh);
  const inside = Math.abs(w) > 0.5;
  return {distance, sign: inside ? -1 : 1, signed: inside ? -distance : distance,
          watertight: true, winding: w};
}

// -- deterministic meshes used by tests and the benchmark --------------------


export function weld(mesh, eps = 1e-9) {
  const keyOf = v => v.map(x => Math.round(x / eps)).join(',');
  const map = new Map(), vertices = [], remap = [];
  for (const v of mesh.vertices) {
    const k = keyOf(v);
    if (!map.has(k)) { map.set(k, vertices.length); vertices.push(v); }
    remap.push(map.get(k));
  }
  const faces = [];
  for (const f of mesh.faces) {
    const a = remap[f[0]], b = remap[f[1]], c = remap[f[2]];
    if (a !== b && b !== c && a !== c) faces.push([a, b, c]);
  }
  return {vertices, faces};
}

export function uvSphere(segments = 64, rings = 32, radius = 1) {
  const vertices = [], faces = [];
  for (let j = 0; j <= rings; j++) {
    const phi = (j / rings) * Math.PI;
    for (let i = 0; i <= segments; i++) {
      const theta = (i / segments) * 2 * Math.PI;
      vertices.push([radius * Math.sin(phi) * Math.cos(theta),
                     radius * Math.cos(phi),
                     radius * Math.sin(phi) * Math.sin(theta)]);
    }
  }
  const row = segments + 1;
  for (let j = 0; j < rings; j++) {
    for (let i = 0; i < segments; i++) {
      const a = j * row + i, b = a + 1, c = a + row, d = c + 1;
      if (j !== 0) faces.push([a, b, c]);
      if (j !== rings - 1) faces.push([b, d, c]);
    }
  }
  return weld({vertices, faces});
}

export function openSphere(segments = 32, rings = 16, radius = 1) {
  // Drop the last ring of quads: the mesh keeps a boundary and must be refused.
  const mesh = uvSphere(segments, rings, radius);
  // Cut a cap off: the remaining mesh has a real boundary edge loop.
  mesh.faces = mesh.faces.filter(f => {
    const y = (mesh.vertices[f[0]][1] + mesh.vertices[f[1]][1] + mesh.vertices[f[2]][1]) / 3;
    return y > -0.70 * radius;
  });
  return mesh;
}

export function cube(size = 1) {
  const s = size / 2;
  const vertices = [
    [-s, -s, -s], [s, -s, -s], [s, s, -s], [-s, s, -s],
    [-s, -s, s], [s, -s, s], [s, s, s], [-s, s, s],
  ];
  const faces = [
    [0, 3, 2], [0, 2, 1], [4, 5, 6], [4, 6, 7],
    [0, 1, 5], [0, 5, 4], [2, 3, 7], [2, 7, 6],
    [1, 2, 6], [1, 6, 5], [0, 4, 7], [0, 7, 3],
  ];
  return {vertices, faces};
}
