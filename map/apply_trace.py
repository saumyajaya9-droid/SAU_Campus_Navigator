"""Apply trace_result.json to campus_navigator.py (edge lengths) and campus_geo.py (road shapes).
Always starts from the pre-trace originals in /tmp so it can be re-run."""
import importlib.util, json, re, sys
def load(path, name):
    sp = importlib.util.spec_from_file_location(name, path); m = importlib.util.module_from_spec(sp); sp.loader.exec_module(m); return m
C0 = load("/tmp/nav_before_trace.py", "nav0"); G0 = load("/tmp/geo_before_trace.py", "geo0")
R = json.load(open("/home/claude/trace_result.json"))
is_j = lambda n: re.fullmatch(r"J\d+|Gate\d", n) is not None
shapes = dict(G0.ROAD_SHAPES); newlen = {}; log = []
for key, v in R["roads"].items():
    a, b = key.split("|")
    if key != "J1|J2" and max(v["snaps"]) > 12: log.append(("keep drawn shape (junction off the road)", key, v["snaps"])); continue
    shapes["|".join(sorted((a, b)))] = v["shape"]
    if key == "J1|J2": newlen[(a, b)] = round(v["length"] * 0.88, 1)       # real road goes round the buildings; traced lines run ~12% long
for a, b, L, *_ in C0.EDGES:
    if is_j(a) == is_j(b): continue
    n, j = (b, a) if is_j(a) else (a, b)
    if n in ("SecurityRoom", "SuffleStore", "Tapri"): continue
    t = R["links"].get(f"{n}|{j}")
    if not t or L < 25 or t["length"] > 2.2 * L + 10: continue
    shapes["|".join(sorted((n, j)))] = t["shape"]; newlen[(a, b)] = max(L, t["length"])

# ---------------------------------------------------------------- entrance points on the roads (SNAP)
import math
SNAP_MAX = 35.0           # an entrance within this many metres of a road gets a stub onto it
MERGE = 8.0               # entrance points closer than this along a road share one point
END_TOL = 6.0             # a point this close to a junction just uses the junction
EXCLUDE = {"SecurityRoom", "SuffleStore", "Tapri", "Gate2"}
lat0, lon0 = G0.GEO["Gate1"]; kx = 111320 * math.cos(math.radians(lat0))
to_m = lambda ll: ((ll[1] - lon0) * kx, (ll[0] - lat0) * 110540)
to_ll = lambda p: [round(lat0 + p[1] / 110540, 6), round(lon0 + p[0] / kx, 6)]
orig_len = {frozenset((a, b)): newlen.get((a, b), L) for a, b, L, *_ in C0.EDGES}
roads = {}
for a, b, L, *_ in C0.EDGES:
    if is_j(a) and is_j(b):
        sh = shapes.get("|".join(sorted((a, b))))
        pts = [to_m(p) for p in sh] if sh else [to_m(G0.GEO[a]), to_m(G0.GEO[b])]
        if math.dist(pts[0], to_m(G0.GEO[a])) > math.dist(pts[0], to_m(G0.GEO[b])): pts = pts[::-1]
        roads[(a, b)] = (pts, newlen.get((a, b), L))
def project(p):
    best = None
    for e, (pts, L) in roads.items():
        cum = 0.0
        for u, v in zip(pts, pts[1:]):
            dx, dy = v[0] - u[0], v[1] - u[1]; l2 = dx * dx + dy * dy; seg = math.sqrt(l2)
            t = 0 if l2 == 0 else max(0, min(1, ((p[0] - u[0]) * dx + (p[1] - u[1]) * dy) / l2)); q = (u[0] + t * dx, u[1] + t * dy)
            d = math.dist(p, q)
            if best is None or d < best[0]: best = (d, e, cum + t * seg)
            cum += seg
    return best
def point_at(pts, s):
    cum = 0.0
    for u, v in zip(pts, pts[1:]):
        seg = math.dist(u, v)
        if cum + seg >= s - 1e-9 and seg > 0:
            t = (s - cum) / seg; return (u[0] + t * (v[0] - u[0]), u[1] + t * (v[1] - u[1]))
        cum += seg
    return pts[-1]
def cut(pts, s0, s1):
    out = [point_at(pts, s0)]; cum = 0.0
    for u, v in zip(pts, pts[1:]):
        cum += math.dist(u, v)
        if s0 + 1e-6 < cum < s1 - 1e-6: out.append(v)
    return out + [point_at(pts, s1)]
places = [n for n in G0.GEO if not is_j(n) and n not in EXCLUDE]
by_road = {}; dropped_links = set(); snapped = {}
for n in sorted(places):
    d, e, s = project(to_m(G0.GEO[n]))
    if d <= SNAP_MAX: by_road.setdefault(e, []).append((s, n, d)); snapped[n] = d
snap_nodes, snap_edges, snap_chains, drop_edges, new_geo = {}, [], {}, set(), {}
counter = 101
ceil1 = lambda x: math.ceil(x * 10 - 1e-9) / 10
for e in sorted(by_road):
    pts, L = roads[e]; Lp = sum(math.dist(u, v) for u, v in zip(pts, pts[1:])); a, b = e
    items = sorted(by_road[e]); clusters = []          # [position, [(place, dist)...], endpoint or None]
    for s, n, d in items:
        if s < END_TOL: clusters.append([0.0, [(n, d)], a]); continue
        if s > Lp - END_TOL: clusters.append([Lp, [(n, d)], b]); continue
        if clusters and clusters[-1][2] is None and s - clusters[-1][0] < MERGE:
            c = clusters[-1]; c[1].append((n, d)); c[0] = (c[0] * (len(c[1]) - 1) + s) / len(c[1])
        else: clusters.append([s, [(n, d)], None])
    chain = [a]; marks = [0.0]; names = {a: a}
    for pos, members, endpoint in clusters:
        if endpoint: node = endpoint
        else:
            node = f"J{counter}"; counter += 1; chain.append(node); marks.append(pos)
            xy = point_at(pts, pos); snap_nodes[node] = (round(xy[0], 1), round(xy[1], 1)); new_geo[node] = to_ll(xy)
        for n, d in members:
            p = to_m(G0.GEO[n]); q = snap_nodes[node] if node in snap_nodes else to_m(G0.GEO[node])
            snap_edges.append((n, node, max(ceil1(math.dist(p, q)), 1.0), True, None, None))
    chain.append(b); marks.append(Lp)
    if len(chain) > 2:
        snap_chains[(a, b)] = chain; drop_edges.add(frozenset((a, b))); shapes.pop("|".join(sorted((a, b))), None)
        pos_xy = lambda nd: snap_nodes[nd] if nd in snap_nodes else to_m(G0.GEO[nd])
        for (u, v), s0, s1 in zip(zip(chain, chain[1:]), marks, marks[1:]):
            length = max(ceil1(L * (s1 - s0) / Lp), ceil1(math.dist(pos_xy(u), pos_xy(v))))
            snap_edges.append((u, v, length, True, None, None))
            sub = cut(pts, s0, s1); sub[0] = pos_xy(u); sub[-1] = pos_xy(v)
            shapes["|".join(sorted((u, v)))] = [to_ll(p) for p in sub]
# the old straight links of the snapped entrances are replaced by the stubs
for a_, b_, L_, *_ in C0.EDGES:
    if is_j(a_) != is_j(b_):
        n_ = b_ if is_j(a_) else a_
        if n_ in snapped: drop_edges.add(frozenset((a_, b_)))
block = ("# ---- SNAP BEGIN (generated by sau_map/apply_trace.py): entrance points on the roads.\n"
         "# Each entrance within 35 m of a road gets a short stub onto it, so a route can step off the road right next to\n"
         "# the entrance. J101 and up are those points; they are hidden in the app. Roads they split become chains.\n"
         f"SNAP_NODES = {snap_nodes!r}\n"
         f"_SNAP_DROP = {sorted(sorted(p) for p in drop_edges)!r}\n"
         f"SNAP_EDGES = {snap_edges!r}\n"
         f"SNAP_CHAINS.update({ {k: v for k, v in snap_chains.items()}!r})\n"
         "NODES.update({k: tuple(v) for k, v in SNAP_NODES.items()})\n"
         "_drop = {frozenset(p) for p in _SNAP_DROP}\n"
         "EDGES[:] = [e for e in EDGES if frozenset(e[:2]) not in _drop] + [tuple(e) for e in SNAP_EDGES]\n"
         "# ---- SNAP END\n")
src = open("/home/claude/campus_navigator.py").read()
src = re.sub(r"# ---- SNAP BEGIN.*?# ---- SNAP END\n", "", src, flags=re.S)
for a, b, L, *_ in C0.EDGES:
    new = newlen.get((a, b), L)
    pat = re.compile(r'\("%s", "%s", [\d.]+,' % (re.escape(a), re.escape(b)))
    assert pat.search(src), (a, b); src = pat.sub('("%s", "%s", %s,' % (a, b, new), src, count=1)
marker = "# INDOOR links: a building can"
assert marker in src; src = src.replace(marker, block + "\n" + marker, 1)
open("/home/claude/campus_navigator.py", "w").write(src)
geo = {k: v for k, v in G0.GEO.items()}; geo.update(new_geo)
open("/home/claude/campus_geo.py", "w").write('"""Generated by build_geo.py + apply_trace.py. GPS position of every place and the road shapes (for the satellite view)."""\nGEO = '
    + json.dumps(geo, indent=1) + "\nROAD_SHAPES = " + json.dumps({k: shapes[k] for k in sorted(shapes)}) + "\n")
for l in log: print(*l)
print(len(newlen), "edge lengths changed;", len(snap_nodes), "entrance points;", len(snap_edges), "snap edges;", len(snapped), "places snapped")
print("not snapped (more than 35 m from a road):", sorted(set(places) - set(snapped)))
