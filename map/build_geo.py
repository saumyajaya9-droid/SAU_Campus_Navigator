"""Road shapes on the satellite: warp your drawn red paths (schematic space) onto real GPS using the junction pins,
then write campus_geo.py (node lat/lon + road polylines) for the satellite view in the app."""
import json, math, csv, numpy as np
from scipy.interpolate import RBFInterpolator
src = open("draw_map_v2.py").read().split("def draw_base")[0]; ns = {}; exec(src, ns)
Mimg = np.array(json.load(open("/tmp/calib.json"))["M"])
Iimg = np.linalg.inv(np.vstack([Mimg.T, [0, 0, 1]]))[:2, :].T
to_sch = lambda p: np.array([p[0], p[1], 1.0]) @ Iimg                      # red image pixel -> schematic coords
red = json.load(open("red_graph.json")); G = json.load(open("final_graph.json")); P, K = G["pos"], G["kind"]
rows = list(csv.DictReader(open("nodes_gps_v3.csv", encoding="cp1252")))
gps = {r["name"].strip(): (float(r["lat"]), float(r["lon"])) for r in rows if r["lat"]}
lat0, lon0 = gps["Gate1"]
to_m = lambda la, lo: ((lo - lon0) * 111320 * math.cos(math.radians(lat0)), (la - lat0) * 110540)
to_ll = lambda x, y: (lat0 + y / 110540, lon0 + x / (111320 * math.cos(math.radians(lat0))))
name_of = lambda k: "Gate1" if k == 17 else f"J{k}"
# control points: schematic position -> real metres
src_pts, dst_pts, label = [], [], []
for k, p in red["nodes"].items():
    k = int(k); src_pts.append(to_sch(p)[:2]); dst_pts.append(to_m(*gps[name_of(k)])); label.append(name_of(k))
for g in ("Gate3", "Gate2"):
    src_pts.append(np.array(ns["GATES"][g])); dst_pts.append(to_m(*gps[g])); label.append(g)
src_pts, dst_pts = np.array(src_pts), np.array(dst_pts)
S = 1e-3
warp = RBFInterpolator(src_pts, dst_pts, kernel="thin_plate_spline", smoothing=S)
loo = []
for i in range(len(src_pts)):                                              # leave-one-out accuracy of the warp
    m = np.arange(len(src_pts)) != i
    w = RBFInterpolator(src_pts[m], dst_pts[m], kernel="thin_plate_spline", smoothing=S)
    loo.append(float(np.hypot(*(w(src_pts[i:i+1])[0] - dst_pts[i]))))
print("warp leave-one-out error (m): median %.0f, max %.0f (%s)" % (np.median(loo), max(loo), label[int(np.argmax(loo))]))
# red polylines -> metres
poly = {}
for e in red["edges"]:
    sch = np.array([to_sch(q)[:2] for q in e["poly"]])
    poly[frozenset((name_of(e["a"]), name_of(e["b"])))] = (name_of(e["a"]), warp(sch))
def oriented(a, b):
    first, pts = poly[frozenset((a, b))]; pts = pts.copy()
    pa, pb = np.array(P[a]), np.array(P[b])
    if np.hypot(*(pts[0] - pa)) + np.hypot(*(pts[-1] - pb)) > np.hypot(*(pts[0] - pb)) + np.hypot(*(pts[-1] - pa)): pts = pts[::-1]
    return pts
def cut(pts, point):                                                     # index of the vertex nearest to point
    return int(np.argmin(np.hypot(*(pts - np.array(point)).T)))
def dp(pts, tol=1.0):                                                    # Douglas-Peucker simplify
    if len(pts) < 3: return pts
    a, b = pts[0], pts[-1]; v = b - a; w = pts - a; d = np.abs(v[0] * w[:, 1] - v[1] * w[:, 0]) / (np.hypot(*v) + 1e-9)
    i = int(np.argmax(d))
    if d[i] <= tol: return np.array([a, b])
    return np.vstack([dp(pts[:i + 1], tol)[:-1], dp(pts[i:], tol)])
def deform(pts, via):
    """Bend a polyline so it passes exactly through the points in `via` (nodes that sit ON this road).
    The sideways correction is spread smoothly along the road (zero at both ends), so there is no spike."""
    seg = np.hypot(*np.diff(pts, axis=0).T); s = np.concatenate([[0], np.cumsum(seg)])
    ks = sorted(((cut(pts, v), np.array(v)) for v in via), key=lambda t: t[0])
    knots_s = [0.0] + [s[k] for k, _ in ks] + [s[-1]]
    knots_d = [np.zeros(2)] + [v - pts[k] for k, v in ks] + [np.zeros(2)]
    out = pts.copy()
    for dim in (0, 1):
        out[:, dim] += np.interp(s, knots_s, [d[dim] for d in knots_d])
    for k, v in ks: out[k] = v
    return out


def shape(a, b):
    """Polyline from node a to node b in metres (exact at both ends)."""
    key = frozenset((a, b))
    if key in poly: pts = oriented(a, b)
    elif key == frozenset(("J2", "J18")) or key == frozenset(("J18", "J5")):
        pts = deform(oriented("J2", "J5"), [P["J18"]]); i = cut(pts, P["J18"])
        part = pts[:i + 1] if (a == "J2" or b == "J2") else pts[i:]
        pts = part
    elif key in (frozenset(("J3", "Gate3")), frozenset(("Gate3", "J1")), frozenset(("J1", "J4"))):
        pts = deform(oriented("J3", "J4"), [P["Gate3"], P["J1"]]); i, j = cut(pts, P["Gate3"]), cut(pts, P["J1"])
        pts = {frozenset(("J3", "Gate3")): pts[:i + 1], frozenset(("Gate3", "J1")): pts[i:j + 1], frozenset(("J1", "J4")): pts[j:]}[key]
    else: pts = None
    if pts is None or len(pts) < 2: pts = np.array([P[a], P[b]])
    pa, pb = np.array(P[a]), np.array(P[b])
    if np.hypot(*(pts[0] - pa)) + np.hypot(*(pts[-1] - pb)) > np.hypot(*(pts[0] - pb)) + np.hypot(*(pts[-1] - pa)): pts = pts[::-1]
    pts = pts.copy(); pts[0], pts[-1] = pa, pb
    return dp(pts)
shapes, report = {}, []
for a, b, L, t in G["edges"]:
    if t != "road": continue
    pts = shape(a, b); ln = float(np.sum(np.hypot(*np.diff(pts, axis=0).T)))
    shapes["|".join(sorted((a, b)))] = [list(map(lambda v: round(v, 6), to_ll(*q))) for q in pts]
    report.append((a, b, L, ln))
print("%-14s %8s %8s" % ("road", "used m", "shape m"))
for a, b, L, ln in sorted(report): print("%-14s %8.0f %8.0f %s" % (a + "-" + b, L, ln, " <-- differs" if abs(ln - L) > 0.25 * L else ""))
geo = {n: tuple(round(v, 6) for v in to_ll(*P[n])) for n in P if K[n] != "building_centre" and n != "J17"}
with open("../campus_geo.py", "w") as f:
    f.write('"""Generated by build_geo.py. GPS position of every place and the road shapes (for the satellite view)."""\n')
    f.write("GEO = " + json.dumps(geo, indent=1) + "\n\nROAD_SHAPES = " + json.dumps(shapes) + "\n")
json.dump({"shapes": shapes, "geo": geo}, open("geo_check.json", "w"))
print("wrote campus_geo.py:", len(geo), "places,", len(shapes), "road shapes")
