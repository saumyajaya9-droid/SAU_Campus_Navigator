"""Regenerate SECTION 1 (NODES, BUILDINGS, EDGES) of campus_navigator.py from sau_map/final_graph.json.
Keeps the two PLACEHOLDER settings below until you replace them with real data."""
import json, re
PLACEHOLDERS = {}   # (a, b): (accessible, min_min, max_min, comment). Empty: no step or crowd data recorded yet.
g = json.load(open("sau_map/final_graph.json")); P, K = g["pos"], g["kind"]
drop = {n for n, k in K.items() if k == "building_centre"} | {"J17"}
nodes = {n: p for n, p in P.items() if n not in drop}
edges = [(a, b, L) for a, b, L, t in g["edges"] if t != "inside" and a not in drop and b not in drop]
bld = {}
for n, k in K.items():
    if k == "entrance": bld.setdefault(n.rsplit("_", 1)[0], []).append(n)
order = lambda n: (0 if n.startswith("Gate") else 1 if re.fullmatch(r"J\d+", n) else 2, int(n[1:]) if re.fullmatch(r"J\d+", n) else 0, n)
L = ["NODES = {   # name -> (x, y) metres east/north of Gate 1, converted from your GPS file"]
L += [f'    "{n}": ({nodes[n][0]:.1f}, {nodes[n][1]:.1f}),' for n in sorted(nodes, key=order)]
L += ["}\n", "# BUILDINGS: a building is reached through ANY of its entrances (so 'AC1' means the nearest of AC1_N / AC1_E / AC1_S).", "BUILDINGS = {"]
L += [f'    "{b}": {sorted(bld[b])},' for b in sorted(bld)]
L += ["}\n", "# EDGES: (from, to, length_m, accessible, min_min, max_min). Walkable both ways.\n"
      "#   J-to-J / Gate-to-J edges = your red paths; entrance/shop/gate edges = short walking links to the nearest junction.\n"
      "#   accessible=True everywhere (no step data recorded yet); None, None = derive uncertainty from length.", "EDGES = ["]
for a, b, ln in sorted(edges):
    acc, lo, hi, note = PLACEHOLDERS.get(frozenset((a, b)), (True, None, None, ""))
    L.append(f'    ("{a}", "{b}", {ln}, {acc}, {lo}, {hi}),' + (f"   # {note}" if note else ""))
L += ["]\n", "# Paths closed right now. Can also be changed while running: agent.close(a, b)\nCLOSED = []\n",
      "# Walking speeds in meters/minute.  1.5 m/s = 90 m/min.\nTYPICAL_SPEED = 90.0    # average student, used for normal time estimates\n"
      "MAX_SPEED = 90.0        # A* heuristic pace: straight-line distance / this = optimistic minutes left (see heuristic())"]
s = open("campus_navigator.py").read()
a = s.index("NODES = {"); b = s.index("# =============================================================================\n# END OF SECTION 1")
open("campus_navigator.py", "w").write(s[:a] + "\n".join(L) + "\n" + s[b:])
print(len(nodes), "nodes", len(edges), "edges", len(bld), "buildings")
