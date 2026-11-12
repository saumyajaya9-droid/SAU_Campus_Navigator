"""Final SAU graph: junction roads (your red paths) + access links (entrance/shop/gate -> nearest junction)."""
import csv, math, json, heapq
rows = list(csv.DictReader(open("nodes_gps_v3.csv", encoding="cp1252")))
fix = {"Faculty Housing": "FacHousing", "Safal  Store": "SuffleStore", "HostelA_w": "HostelA_W", "Amphitheatre": "Amphitheater",
       "library": "Library_W"}
pos, kind = {}, {}
for r in rows:
    if not r["lat"]: continue
    n = r["name"].strip(); n = fix.get(n, n)
    k = r["kind"]
    if k.startswith("caf"): k = "cafe"
    if k == "entrance" and n in ("GuestHouse",): n = "GuestHouse_S"          # entrance row named like its building
    pos[n], kind[n] = (float(r["lat"]), float(r["lon"])), k
lat0, lon0 = pos["Gate1"]
xy = {n: ((lo-lon0)*111320*math.cos(math.radians(lat0)), (la-lat0)*110540) for n, (la, lo) in pos.items()}
dist = lambda a, b: math.dist(xy[a], xy[b])
# Gate 1 and J17 are the same place (12 m apart) -> keep Gate1, drop J17
alias = {"J17": "Gate1"}
A = lambda n: alias.get(n, n)
edges = {}
def add(a, b, L, typ):
    a, b = A(a), A(b)
    if a != b: edges[tuple(sorted((a, b)))] = (round(L, 1), typ)
for r in csv.DictReader(open("edges_user_paths.csv")):
    a, b, est = r["from"], r["to"], float(r["length_m_estimate"])
    if (a, b) in (("J2", "J5"),): continue       # J18 sits on this road: split below
    if (a, b) == ("J3", "J4"): continue          # this path passes through Gate 3 and J1: split below
    add(a, b, max(est, dist(a, b)), "road")
# J2 -> J18 -> J5 (J18 lies on the old J2-J5 road; its 189.7 m estimate is shared in proportion to GPS distance)
_t = dist("J2", "J18") + dist("J18", "J5")
add("J2", "J18", max(189.7*dist("J2", "J18")/_t, dist("J2", "J18")), "road"); add("J18", "J5", max(189.7*dist("J18", "J5")/_t, dist("J18", "J5")), "road")
# long west/north path J3 -> Gate3 -> J1 -> J4 (lengths from the drawn polyline)
add("J3", "Gate3", 328, "road"); add("Gate3", "J1", 64, "road"); add("J1", "J4", 442, "road")   # measured on Google Maps: 442 m
# J12 <-> J14: added at your request (length = straight-line GPS distance; measure it on Google Maps to refine)
add("J12", "J14", dist("J12", "J14"), "road")
# access links: nearest junction (building entrances, shops, gates, amphitheater); building centre -> its entrances
J = [n for n in pos if kind[n] == "junction" and n != "J17"] + ["Gate1"]
J = [j for j in J if j in pos]
links = []
for n in pos:
    if kind[n] in ("entrance", "shop", "cafe", "security", "open_space", "building", "gate") and n not in ("Gate1", "Gate3"):
        j = min((x for x in J if x != n), key=lambda x: dist(n, x)); links.append((n, j, dist(n, j)))
        add(n, j, dist(n, j), "access")
ent_of = {}
for n in pos:
    if kind[n] == "entrance":
        b = n.rsplit("_", 1)[0]; b = {"Library": "Library", "GuestHouse": "GuestHouse"}.get(b, b); ent_of.setdefault(b, []).append(n)
for b, ens in ent_of.items():
    c = b if b in pos else None
    if c is None: continue
    for e in ens: add(c, e, dist(c, e), "inside")
for r in rows:   # building centres without entrances rows: link to nearest junction
    pass
json.dump({"pos": xy, "kind": kind, "edges": [[a, b, L, t] for (a, b), (L, t) in edges.items()]}, open("final_graph.json", "w"))
with open("edges_final_graph.csv", "w", newline="") as f:
    w = csv.writer(f); w.writerow(["from", "to", "length_m", "type"])
    for (a, b), (L, t) in sorted(edges.items()): w.writerow([a, b, L, t])
# report
print("nodes", len(pos) - 1, "edges", len(edges))
print("long access links (>90 m):", [(n, j, round(d)) for n, j, d in links if d > 90])
adj = {}
for (a, b), (L, t) in edges.items(): adj.setdefault(a, []).append(b); adj.setdefault(b, []).append(a)
seen, st = set(), ["Gate1"]
while st:
    x = st.pop()
    if x in seen: continue
    seen.add(x); st += adj.get(x, [])
print("unreachable from Gate1:", sorted(set(adj) - seen), "| nodes without any edge:", sorted(set(pos) - set(adj) - {"J17"}))
