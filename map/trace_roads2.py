"""Trace every path on the real road lines (Google's grey roads in the satellite screenshot).
Writes /tmp trace_result.json: road shapes (lat/lon), new lengths and new access links. Nothing is patched here."""
import cv2, json, math, re, sys, numpy as np, networkx as nx
from scipy.spatial import cKDTree
sys.path.insert(0, "/home/claude")
from campus_geo import GEO
import campus_navigator as C
S = "/tmp/claude-0/-home-claude/87d7c525-6fa4-5412-b0ec-77f6fe7d9c58/scratchpad/"
im = cv2.imread(S + "img/sat.png"); sk = np.load(S + "out/sk_clean.npy")
k = 111320 * math.cos(math.radians(28.485))
def metres(n): return np.array([GEO[n][1] * k, GEO[n][0] * 110540])
g1, g3 = metres("Gate1"), metres("Gate3"); p1, p3 = np.array([880., 1190.]), np.array([130., 185.])
q = complex(p3[0] - p1[0], -(p3[1] - p1[1])) / complex(*(g3 - g1))
sim = lambda lat, lon: (lambda z: np.array([p1[0] + z.real, p1[1] - z.imag]))(complex(*(np.array([lon * k, lat * 110540]) - g1)) * q)
# Refine with ICP: fit scale + shift (the screenshot is north-up) from the junction GPS points to the nearest road pixels.
# (Two gates alone put every road ~15 m too far east; a free affine fit absorbed GPS noise as 7% skew.)
_names = [n for n in GEO if re.fullmatch(r"J\d+|Gate\d", n) and n != "Gate2"]
_X = np.array([metres(n) for n in _names]) * np.array([1, -1]); _P = np.array([sim(*GEO[n]) for n in _names])
_ys, _xs = np.nonzero(sk); _tree0 = cKDTree(np.c_[_xs, _ys].astype(float)); _cur = _P.copy()
for _ in range(60):
    _d, _i = _tree0.query(_cur); _w = _d < 40; _Y = _tree0.data[_i][_w]; _Z = _X[_w]
    _s = ((_Y - _Y.mean(0)) * (_Z - _Z.mean(0))).sum() / ((_Z - _Z.mean(0)) ** 2).sum(); _tt = _Y.mean(0) - _s * _Z.mean(0)
    _cur = _X * _s + _tt
_M = np.array([[_s, 0], [0, -_s]]); _t = _tt; _Mi = np.linalg.inv(_M)
print("scale+shift fit: median distance to road %.1f px" % np.median(_tree0.query(_cur)[0]))
def to_pix(lat, lon): return _M @ np.array([lon * k, lat * 110540]) + _t
def to_geo(x, y):
    m_ = _Mi @ (np.array([x, y]) - _t); return [round(float(m_[1] / 110540), 6), round(float(m_[0] / k), 6)]
np.save("/home/claude/affine.npy", np.r_[_M.ravel(), _t])
PPM = float(np.sqrt(abs(np.linalg.det(_M)))); print("px per metre %.3f" % PPM)
# ---- skeleton graph
ys, xs = np.nonzero(sk); pts = list(zip(xs.tolist(), ys.tolist())); ps = set(pts)
G = nx.Graph()
for x, y in pts:
    for dx, dy in ((1, 0), (0, 1), (1, 1), (1, -1)):
        if (x + dx, y + dy) in ps: G.add_edge((x, y), (x + dx, y + dy), weight=math.hypot(dx, dy))
# bridge the little gaps left by the junction dots: join two different pieces when their ends are within 60 px
comp = {}
for i, c in enumerate(nx.connected_components(G)):
    for u in c: comp[u] = i
nl = list(G.nodes); tr0 = cKDTree(np.array(nl)); best = {}
for i, j in tr0.query_pairs(60):
    a_, b_ = nl[i], nl[j]
    if comp[a_] != comp[b_]:
        kk = tuple(sorted((comp[a_], comp[b_]))); d_ = math.dist(a_, b_)
        if kk not in best or d_ < best[kk][0]: best[kk] = (d_, a_, b_)
for d_, a_, b_ in best.values(): G.add_edge(a_, b_, weight=d_)
print("bridged", len(best), "gaps; pieces left:", nx.number_connected_components(G))
tree = cKDTree(np.array(pts))
def snap(n):
    p = to_pix(*GEO[n]); d, i = tree.query(p); return pts[i], d / PPM
is_j = lambda n: re.fullmatch(r"J\d+|Gate\d", n) is not None
backbone = [n for n in GEO if is_j(n) and n != "Gate2"]
SN = {n: snap(n) for n in backbone}
straight = lambda a, b: float(np.linalg.norm(metres(a) - metres(b)))
def simplify(path, tol=1.2):
    return [tuple(map(float, r)) for r in cv2.approxPolyDP(np.array(path, np.float32).reshape(-1, 1, 2), tol, False).reshape(-1, 2)]
out = {"roads": {}, "links": {}, "report": []}
# ---- roads: every existing J-J / Gate-J edge
old = {}
for a, b, L, *_ in C.EDGES:
    if is_j(a) and is_j(b): old[(a, b)] = L
for (a, b), L in old.items():
    if a == "Gate2" or b == "Gate2": continue                         # Gate2 is the same place as J10
    (pa, da), (pb, db) = SN[a], SN[b]
    if da > 32 or db > 32: out["report"].append((a, b, "KEEP old: node too far from a road", round(da), round(db))); continue
    try: path = nx.shortest_path(G, pa, pb, weight="weight")
    except nx.NetworkXNoPath: out["report"].append((a, b, "KEEP old: no road connects them")); continue
    length = sum(math.dist(u, v) for u, v in zip(path, path[1:])) / PPM + da + db
    blocked = [m for m in backbone if m not in (a, b) and SN[m][1] < 12 and SN[m][0] != pa and SN[m][0] != pb and min(math.dist(SN[m][0], p) for p in path[::2]) < 7]
    ratio = length / max(L, 1)
    if (blocked or ratio > 1.6 or ratio < 0.6) and f"{a}|{b}" not in ("J1|J2",):
        out["report"].append((a, b, f"KEEP old: traced {length:.0f} m vs old {L:g} m, passes {blocked}")); continue
    sh = [tuple(to_pix(*GEO[a]))] + simplify(path) + [tuple(to_pix(*GEO[b]))]
    out["roads"][f"{a}|{b}"] = {"snaps": [round(da, 1), round(db, 1)], "shape": [to_geo(*p) for p in sh], "length": round(max(length, straight(a, b)), 1), "old": L}
    out["report"].append((a, b, f"traced {length:.0f} m (path only {length-da-db:.0f}, snaps {da:.0f}+{db:.0f}) old {L:g}"))
# ---- access links for everything that is not a junction/gate
others = [n for n in GEO if not is_j(n)]
bb_snaps = {n: SN[n][0] for n in backbone if SN[n][1] < 32 and n != "Gate2"}
snap_to_nodes = {}
for n, p in bb_snaps.items(): snap_to_nodes.setdefault(p, []).append(n)
for n in others:
    p, d = snap(n)
    if d > 60: out["report"].append((n, "no road within 60 m, keep straight link", round(d))); continue
    dist, paths = nx.single_source_dijkstra(G, p, cutoff=400 * PPM)
    cand = []
    for jn, jp in bb_snaps.items():
        if jp not in dist: continue
        path = paths[jp]; # adjacent only: the road path may not pass another junction snap
        if any(m != jn and bb_snaps[m] != jp and bb_snaps[m] != p and min(math.dist(bb_snaps[m], u) for u in path[::2]) < 7 for m in bb_snaps):
            continue
        cand.append((dist[jp] / PPM + d, jn, path))
    cand.sort(key=lambda t: t[0])
    chosen = []
    for L, jn, path in cand:
        if len(chosen) < 2 and (not chosen or L < 2.2 * chosen[0][0] + 20): chosen.append((L, jn, path))
    for L, jn, path in chosen:
        sh = [tuple(to_pix(*GEO[n]))] + simplify(path) + [tuple(to_pix(*GEO[jn]))]
        out["links"][f"{n}|{jn}"] = {"shape": [to_geo(*u) for u in sh], "length": round(max(L + SN[jn][1], straight(n, jn)), 1)}
json.dump(out, open("/home/claude/trace_result.json", "w"))
for r in out["report"]: print(*r)
print(len(out["roads"]), "roads traced;", len(out["links"]), "links")
