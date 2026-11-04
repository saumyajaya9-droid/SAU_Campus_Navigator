"""
campus_navigator.py  --  Smart Campus Navigator Agent (single file, standard library only)

HOW TO USE WITH YOUR REAL MAP
-----------------------------
1. Scroll to SECTION 1 and replace NODES and EDGES with your campus data.
   Nothing below SECTION 1 needs to change.
2. Run `python campus_navigator.py check`   -> validates your map (typos, islands, bad coordinates)
3. Run `python campus_navigator.py demo`    -> shows everything working on your map
4. Or use it from your own code:

       from campus_navigator import NavigatorAgent
       agent = NavigatorAgent()
       print(agent.route("MainGate", "AdminBlock", accessible=True))
       print(agent.multi_stop("MainGate", {"Hostel": 5, "PCMB1": 10}))

WHAT IT CONTAINS
----------------
  Search   : BFS (uninformed), Uniform-Cost (uninformed), A* (informed, admissible heuristic)
  Real-world handling : closed paths, wheelchair-accessible routing, travel-time uncertainty
  CSP      : multi-stop planner. Variables = stops, constraint = arrive before each deadline.
             Backtracking + forward checking + branch-and-bound -> fastest order that meets
             every deadline. If no order can, falls back to Simulated Annealing (local search)
             that minimises total lateness.
"""

import heapq
import math
import random
import sys
from collections import deque
from dataclasses import dataclass


# =============================================================================
# SECTION 1 - YOUR MAP (REPLACE THIS SECTION WITH THE REAL CAMPUS)
# =============================================================================
# NODES: every place a person can stand/choose (buildings, gates, junctions).
#        name -> (x, y) in METERS on any consistent flat grid.
#        From GPS: convert lat/lon to meters (x = (lon-lon0)*111320*cos(lat0), y = (lat-lat0)*110540).
#        From a traced satellite image: use pixel position * (meters per pixel).
NODES = {   # name -> (x, y) metres east/north of Gate 1, converted from your GPS file
    "Gate1": (0.0, 0.0),
    "Gate2": (-316.0, 276.3),
    "Gate3": (-412.2, 663.0),
    "J1": (-374.9, 644.6),
    "J2": (-380.6, 429.0),
    "J3": (-429.8, 384.2),
    "J4": (-109.7, 334.7),
    "J5": (-247.3, 343.9),
    "J6": (-38.6, 280.1),
    "J7": (127.1, 276.9),
    "J8": (-167.3, 288.7),
    "J9": (-135.0, 239.8),
    "J10": (-311.3, 281.8),
    "J11": (24.1, 228.3),
    "J12": (-3.5, 191.5),
    "J13": (83.1, 183.6),
    "J14": (-32.2, 154.8),
    "J15": (47.0, 89.0),
    "J16": (-73.6, 69.2),
    "J18": (-296.7, 383.2),
    "AC1_E": (-297.2, 353.2),
    "AC1_N": (-306.5, 370.5),
    "AC1_S": (-343.5, 319.2),
    "AC2_S": (-400.0, 363.3),
    "AC2_W": (-370.1, 411.0),
    "AC3_S": (-384.8, 435.0),
    "AC3_W": (-446.2, 440.6),
    "AC4_E": (-382.4, 497.3),
    "AC4_N": (-376.8, 546.6),
    "AC4_W": (-434.5, 532.4),
    "AC5_E": (-342.7, 581.0),
    "AC5_N": (-352.9, 607.7),
    "AC5_S": (-371.7, 560.7),
    "AC6_N": (-252.3, 436.4),
    "AC6_S": (-302.0, 405.0),
    "AC7_N": (-201.9, 396.8),
    "AC7_S": (-222.4, 344.7),
    "AC8_N": (-78.5, 406.1),
    "AC8_S": (-130.6, 381.4),
    "Admin_E": (-141.3, 327.1),
    "Admin_N": (-145.3, 356.9),
    "Amphitheater": (-341.3, 459.7),
    "ConventionCentre": (-311.5, 562.8),
    "Court1": (-116.6, 275.5),
    "Court2": (-61.4, 119.6),
    "FacHousing_E": (54.3, 277.1),
    "FacHousing_W": (6.1, 318.5),
    "GuestHouse_S": (99.5, 239.3),
    "HostelA_E": (15.8, 55.7),
    "HostelA_W": (-39.2, 106.9),
    "HostelB_E": (54.9, 137.2),
    "HostelB_W": (5.7, 185.7),
    "Library_W": (-55.1, 324.1),
    "Playground": (-63.3, 237.9),
    "SecurityRoom": (-6.4, 7.8),
    "SuffleStore": (-17.0, 14.1),
    "Tapri": (-30.5, 25.8),
}

# BUILDINGS: a building is reached through ANY of its entrances (so 'AC1' means the nearest of AC1_N / AC1_E / AC1_S).
BUILDINGS = {
    "AC1": ['AC1_E', 'AC1_N', 'AC1_S'],
    "AC2": ['AC2_S', 'AC2_W'],
    "AC3": ['AC3_S', 'AC3_W'],
    "AC4": ['AC4_E', 'AC4_N', 'AC4_W'],
    "AC5": ['AC5_E', 'AC5_N', 'AC5_S'],
    "AC6": ['AC6_N', 'AC6_S'],
    "AC7": ['AC7_N', 'AC7_S'],
    "AC8": ['AC8_N', 'AC8_S'],
    "Admin": ['Admin_E', 'Admin_N'],
    "FacHousing": ['FacHousing_E', 'FacHousing_W'],
    "GuestHouse": ['GuestHouse_S'],
    "HostelA": ['HostelA_E', 'HostelA_W'],
    "HostelB": ['HostelB_E', 'HostelB_W'],
    "Library": ['Library_W'],
}

# EDGES: (from, to, length_m, accessible, min_min, max_min). Walkable both ways.
#   J-to-J / Gate-to-J edges = your red paths; entrance/shop/gate edges = short walking links to the nearest junction.
#   accessible=True everywhere (no step data recorded yet); None, None = derive uncertainty from length.
EDGES = [
    ("AC1_E", "J18", 40.3, True, None, None),
    ("AC1_N", "J18", 16.0, True, None, None),
    ("AC1_S", "J10", 58.5, True, None, None),
    ("AC2_S", "J3", 56.8, True, None, None),
    ("AC2_W", "J2", 20.9, True, None, None),
    ("AC3_S", "J2", 7.3, True, None, None),
    ("AC3_W", "J3", 97.2, True, None, None),
    ("AC4_E", "J2", 79.4, True, None, None),
    ("AC4_N", "J1", 185.4, True, None, None),
    ("AC4_W", "J2", 116.6, True, None, None),
    ("AC5_E", "J1", 71.2, True, None, None),
    ("AC5_N", "J1", 77.2, True, None, None),
    ("AC5_S", "J1", 190.4, True, None, None),
    ("AC6_N", "J18", 69.2, True, None, None),
    ("AC6_S", "J18", 22.4, True, None, None),
    ("AC7_N", "J5", 69.7, True, None, None),
    ("AC7_S", "J5", 24.9, True, None, None),
    ("AC8_N", "J4", 77.9, True, None, None),
    ("AC8_S", "J4", 84.3, True, None, None),
    ("Admin_E", "J4", 72.5, True, None, None),
    ("Admin_N", "J4", 66.1, True, None, None),
    ("Amphitheater", "J2", 49.9, True, None, None),
    ("ConventionCentre", "J1", 170.9, True, None, None),
    ("Court1", "J9", 40.2, True, None, None),
    ("Court2", "J14", 61.7, True, None, None),
    ("FacHousing_E", "J11", 75.6, True, None, None),
    ("FacHousing_W", "J6", 132.0, True, None, None),
    ("Gate1", "J15", 170.3, True, None, None),
    ("Gate1", "SecurityRoom", 10.1, True, None, None),
    ("Gate1", "SuffleStore", 22.1, True, None, None),
    ("Gate1", "Tapri", 39.9, True, None, None),
    ("Gate2", "J10", 7.2, True, None, None),
    ("Gate3", "J1", 64, True, None, None),
    ("Gate3", "J3", 328, True, None, None),
    ("GuestHouse_S", "J7", 58.9, True, None, None),
    ("HostelA_E", "J15", 107.5, True, None, None),
    ("HostelA_W", "J14", 65.6, True, None, None),
    ("HostelB_E", "J15", 82.8, True, None, None),
    ("HostelB_W", "J12", 10.8, True, None, None),
    ("J1", "J2", 369.7, True, None, None),
    ("J1", "J4", 442, True, None, None),
    ("J10", "J3", 211.4, True, None, None),
    ("J10", "J5", 116.1, True, None, None),
    ("J11", "J12", 46.0, True, None, None),
    ("J11", "J13", 93.4, True, None, None),
    ("J4", "J6", 99.2, True, None, None),
    ("J6", "J11", 88.5, True, None, None),
    ("J12", "J14", 46.6, True, None, None),
    ("J13", "J15", 101.3, True, None, None),
    ("J13", "J7", 136.5, True, None, None),
    ("J14", "J15", 109.3, True, None, None),
    ("J14", "J16", 128.5, True, None, None),
    ("J14", "J9", 165.6, True, None, None),
    ("J16", "J9", 252.2, True, None, None),
    ("J18", "J2", 114.2, True, None, None),
    ("J18", "J5", 75.5, True, None, None),
    ("J2", "J3", 112.4, True, None, None),
    ("J4", "J8", 100.8, True, None, None),
    ("J5", "J8", 97.1, True, None, None),
    ("J6", "J7", 197.6, True, None, None),
    ("J6", "Library_W", 81.6, True, None, None),
    ("J6", "Playground", 48.9, True, None, None),
    ("J8", "J9", 58.7, True, None, None),
]

SNAP_CHAINS = {}      # road (a, b) -> [a, ..., b] when entrance points split it (filled in by the SNAP block below)


# ---- SNAP BEGIN (generated by sau_map/apply_trace.py): entrance points on the roads.
# Each entrance within 35 m of a road gets a short stub onto it, so a route can step off the road right next to
# the entrance. J101 and up are those points; they are hidden in the app. Roads they split become chains.
SNAP_NODES = {'J101': (35.2, 43.8), 'J102': (-465.8, 434.6), 'J103': (-453.2, 537.3), 'J104': (-339.2, 634.2), 'J105': (-292.4, 563.2), 'J106': (-246.9, 446.7), 'J107': (-200.6, 399.1), 'J108': (-142.7, 362.2), 'J109': (-348.6, 312.7), 'J110': (-405.6, 356.0), 'J111': (65.6, 133.2), 'J112': (110.7, 233.6), 'J113': (-54.7, 114.7), 'J114': (-81.6, 210.7), 'J115': (-311.4, 392.1), 'J116': (-356.2, 425.0), 'J117': (-285.0, 372.9), 'J118': (-63.4, 308.6), 'J119': (-130.0, 314.1), 'J120': (-228.0, 335.6), 'J121': (4.3, 310.8), 'J122': (59.1, 298.3), 'J123': (-143.1, 254.5)}
_SNAP_DROP = [['AC1_E', 'J18'], ['AC1_N', 'J18'], ['AC1_S', 'J10'], ['AC2_S', 'J3'], ['AC2_W', 'J2'], ['AC3_S', 'J2'], ['AC3_W', 'J3'], ['AC4_W', 'J2'], ['AC5_N', 'J1'], ['AC6_N', 'J18'], ['AC6_S', 'J18'], ['AC7_N', 'J5'], ['AC7_S', 'J5'], ['AC8_S', 'J4'], ['Admin_E', 'J4'], ['Admin_N', 'J4'], ['ConventionCentre', 'J1'], ['Court1', 'J9'], ['Court2', 'J14'], ['FacHousing_E', 'J11'], ['FacHousing_W', 'J6'], ['Gate1', 'J15'], ['Gate3', 'J3'], ['GuestHouse_S', 'J7'], ['HostelA_E', 'J15'], ['HostelA_W', 'J14'], ['HostelB_E', 'J15'], ['HostelB_W', 'J12'], ['J1', 'J2'], ['J1', 'J4'], ['J10', 'J3'], ['J13', 'J15'], ['J13', 'J7'], ['J14', 'J16'], ['J14', 'J9'], ['J18', 'J2'], ['J18', 'J5'], ['J4', 'J6'], ['J4', 'J8'], ['J5', 'J8'], ['J6', 'J7'], ['J6', 'Library_W'], ['J6', 'Playground'], ['J8', 'J9']]
SNAP_EDGES = [('HostelA_E', 'J101', 22.9, True, None, None), ('Gate1', 'J101', 92.1, True, None, None), ('J101', 'J15', 78.3, True, None, None), ('AC3_W', 'J102', 20.6, True, None, None), ('Gate3', 'J102', 259.8, True, None, None), ('J102', 'J3', 68.3, True, None, None), ('AC4_W', 'J103', 19.4, True, None, None), ('J1', 'J103', 138.8, True, None, None), ('J103', 'J2', 231.0, True, None, None), ('AC5_N', 'J104', 29.9, True, None, None), ('ConventionCentre', 'J105', 19.2, True, None, None), ('AC6_N', 'J106', 11.7, True, None, None), ('AC7_N', 'J107', 2.6, True, None, None), ('AC8_S', 'J108', 22.7, True, None, None), ('Admin_N', 'J108', 5.9, True, None, None), ('J1', 'J104', 40.7, True, None, None), ('J104', 'J105', 89.3, True, None, None), ('J105', 'J106', 132.2, True, None, None), ('J106', 'J107', 66.5, True, None, None), ('J107', 'J108', 68.7, True, None, None), ('J108', 'J4', 52.3, True, None, None), ('AC1_S', 'J109', 8.3, True, None, None), ('AC2_S', 'J110', 9.3, True, None, None), ('J10', 'J109', 62.0, True, None, None), ('J109', 'J110', 91.7, True, None, None), ('J110', 'J3', 57.8, True, None, None), ('HostelB_W', 'J12', 10.9, True, None, None), ('HostelB_E', 'J111', 11.5, True, None, None), ('J13', 'J111', 53.4, True, None, None), ('J111', 'J15', 48.1, True, None, None), ('GuestHouse_S', 'J112', 12.6, True, None, None), ('J13', 'J112', 75.4, True, None, None), ('J112', 'J7', 61.2, True, None, None), ('Court2', 'J113', 8.3, True, None, None), ('HostelA_W', 'J113', 17.4, True, None, None), ('J14', 'J113', 62.2, True, None, None), ('J113', 'J16', 66.4, True, None, None), ('Playground', 'J114', 32.8, True, None, None), ('J14', 'J114', 93.1, True, None, None), ('J114', 'J9', 72.6, True, None, None), ('AC1_N', 'J18', 16.1, True, None, None), ('AC6_S', 'J115', 16.1, True, None, None), ('AC2_W', 'J116', 19.8, True, None, None), ('AC3_S', 'J2', 7.4, True, None, None), ('J18', 'J115', 20.1, True, None, None), ('J115', 'J116', 60.6, True, None, None), ('J116', 'J2', 33.7, True, None, None), ('AC1_E', 'J117', 23.2, True, None, None), ('J18', 'J117', 19.5, True, None, None), ('J117', 'J5', 56.1, True, None, None), ('Library_W', 'J118', 17.6, True, None, None), ('J4', 'J118', 53.2, True, None, None), ('J118', 'J6', 46.8, True, None, None), ('Admin_E', 'J119', 17.3, True, None, None), ('J4', 'J119', 39.5, True, None, None), ('J119', 'J8', 61.4, True, None, None), ('AC7_S', 'J120', 10.7, True, None, None), ('J5', 'J120', 21.0, True, None, None), ('J120', 'J8', 76.7, True, None, None), ('FacHousing_W', 'J121', 7.9, True, None, None), ('FacHousing_E', 'J122', 21.8, True, None, None), ('J6', 'J121', 65.4, True, None, None), ('J121', 'J122', 58.4, True, None, None), ('J122', 'J7', 74.0, True, None, None), ('Court1', 'J123', 33.8, True, None, None), ('J8', 'J123', 42.0, True, None, None), ('J123', 'J9', 17.0, True, None, None)]
SNAP_CHAINS.update({('Gate1', 'J15'): ['Gate1', 'J101', 'J15'], ('Gate3', 'J3'): ['Gate3', 'J102', 'J3'], ('J1', 'J2'): ['J1', 'J103', 'J2'], ('J1', 'J4'): ['J1', 'J104', 'J105', 'J106', 'J107', 'J108', 'J4'], ('J10', 'J3'): ['J10', 'J109', 'J110', 'J3'], ('J13', 'J15'): ['J13', 'J111', 'J15'], ('J13', 'J7'): ['J13', 'J112', 'J7'], ('J14', 'J16'): ['J14', 'J113', 'J16'], ('J14', 'J9'): ['J14', 'J114', 'J9'], ('J18', 'J2'): ['J18', 'J115', 'J116', 'J2'], ('J18', 'J5'): ['J18', 'J117', 'J5'], ('J4', 'J6'): ['J4', 'J118', 'J6'], ('J4', 'J8'): ['J4', 'J119', 'J8'], ('J5', 'J8'): ['J5', 'J120', 'J8'], ('J6', 'J7'): ['J6', 'J121', 'J122', 'J7'], ('J8', 'J9'): ['J8', 'J123', 'J9']})
NODES.update({k: tuple(v) for k, v in SNAP_NODES.items()})
_drop = {frozenset(p) for p in _SNAP_DROP}
EDGES[:] = [e for e in EDGES if frozenset(e[:2]) not in _drop] + [tuple(e) for e in SNAP_EDGES]
# ---- SNAP END
# ---- GATE1 ROAD BEGIN
# The road leaving Gate1 to the north-west passes the Security room, the Suffle store and the Tapri (they stand beside it),
# then carries on to J16. Before, those three were dead-end spurs, so Gate1 had only one way out.
EDGES[:] = [e for e in EDGES if not (e[0] == "Gate1" and e[1] in ("SecurityRoom", "SuffleStore", "Tapri"))]
EDGES += [("Gate1", "SecurityRoom", 10.2, True, None, None),
          ("SecurityRoom", "SuffleStore", 12.4, True, None, None),
          ("SuffleStore", "Tapri", 18.0, True, None, None),
          ("Tapri", "J16", 64.0, True, None, None)]
SNAP_CHAINS[("Gate1", "J16")] = ["Gate1", "SecurityRoom", "SuffleStore", "Tapri", "J16"]
# ---- GATE1 ROAD END

# INDOOR links: a building can be crossed from one of its entrances to another. Length = straight line x 1.2 (corridors
# are not straight), so it never undercuts the A* heuristic. The app can switch them off (it just closes them).
INDOOR_FACTOR = 1.2
INDOOR = []
for _b, _ents in BUILDINGS.items():
    for _i, _a in enumerate(_ents):
        for _c in _ents[_i + 1:]:
            _d = math.hypot(NODES[_a][0] - NODES[_c][0], NODES[_a][1] - NODES[_c][1])
            INDOOR.append((_a, _c, round(max(_d * INDOOR_FACTOR, _d + 0.1), 1)))
EDGES += [(a, b, L, True, None, None) for a, b, L in INDOOR]
INDOOR_PAIRS = {frozenset((a, b)) for a, b, _ in INDOOR}

# Paths closed right now. Can also be changed while running: agent.close(a, b)
CLOSED = []

# Walking speeds in meters/minute.  1.5 m/s = 90 m/min.
TYPICAL_SPEED = 90.0    # average student, used for normal time estimates
MAX_SPEED = 90.0        # A* heuristic pace: straight-line distance / this = optimistic minutes left (see heuristic())
# =============================================================================
# END OF SECTION 1 - nothing below needs editing for a new map
# =============================================================================


# =============================================================================
# SECTION 2 - GRAPH
# =============================================================================
@dataclass
class Edge:
    to: str
    distance_m: float
    accessible: bool
    min_time: float          # fastest plausible walking time, minutes
    max_time: float          # slowest plausible walking time, minutes
    base_time: float         # normal walking time, minutes
    closed: bool = False


class CampusGraph:
    def __init__(self, nodes, edges, closed=(), buildings=None):
        self.nodes = dict(nodes)
        self.buildings = dict(buildings or {})
        self.adj = {n: [] for n in self.nodes}
        self._hscale = None
        for a, b, dist, acc, tmin, tmax in edges:
            if a not in self.nodes or b not in self.nodes:
                raise ValueError(f"Edge ({a}, {b}) uses a node that is not in NODES (typo?)")
            base = dist / TYPICAL_SPEED
            if tmin is None or tmax is None:
                tmin, tmax = base * 0.85, base * 1.35
            else:
                base = (tmin + tmax) / 2          # uncertain path: typical = middle of its range
            self.adj[a].append(Edge(b, dist, acc, tmin, tmax, base))
            self.adj[b].append(Edge(a, dist, acc, tmin, tmax, base))
        for a, b in closed:
            self.close(a, b)

    def resolve(self, place):
        """A place is either a single node (gate, junction, shop...) or a building.
        A building is reached through ANY of its entrances, so it resolves to a list of nodes."""
        if place in self.buildings:
            return list(self.buildings[place])
        if place in self.nodes:
            return [place]
        raise ValueError(f"Unknown place '{place}'. Known: "
                         f"{', '.join(sorted(set(self.nodes) | set(self.buildings)))}")

    # ---- closures -----------------------------------------------------------
    def _pieces(self, a, b):
        """The edge(s) that make up the road a<->b. Normally one. A road with entrance points on it was split into a
        chain (J4 - J101 - J102 - J6); naming its two ends then means the whole chain."""
        if any(e.to == b for e in self.adj.get(a, [])):
            return [(a, b)]
        chain = SNAP_CHAINS.get((a, b)) or (SNAP_CHAINS.get((b, a)) or [])[::-1]
        if not chain:
            raise ValueError(f"No path between {a} and {b}")
        return list(zip(chain, chain[1:]))

    def _set_closed(self, a, b, value):
        for u, v in self._pieces(a, b):
            for x, y in ((u, v), (v, u)):
                for e in self.adj.get(x, []):
                    if e.to == y:
                        e.closed = value

    def close(self, a, b):
        self._set_closed(a, b, True)

    def _edges_between(self, a, b):
        found = [e for u, v in ((a, b), (b, a)) for e in self.adj.get(u, []) if e.to == v]
        if not found:
            raise ValueError(f"No path between {a} and {b}")
        return found

    def set_accessible(self, a, b, value):
        """Record that the path a<->b is step-free (True) or has steps (False)."""
        for u, v in self._pieces(a, b):
            for e in self._edges_between(u, v):
                e.accessible = value

    def set_uncertain(self, a, b, min_time, max_time):
        """Record that the path a<->b takes between min_time and max_time minutes (crowds, construction...).
        A road split into pieces shares the time between its pieces in proportion to their length."""
        pieces = self._pieces(a, b)
        total = sum(self.edge(u, v).distance_m for u, v in pieces)
        for u, v in pieces:
            share = self.edge(u, v).distance_m / total if len(pieces) > 1 else 1.0
            for e in self._edges_between(u, v):
                e.min_time, e.max_time, e.base_time = min_time * share, max_time * share, (min_time + max_time) / 2 * share

    def reopen(self, a, b):
        self._set_closed(a, b, False)

    # ---- queries ------------------------------------------------------------
    def neighbors(self, node, accessible=False):
        """Edges you may walk out of `node` right now."""
        for e in self.adj[node]:
            if e.closed:
                continue
            if accessible and not e.accessible:
                continue
            yield e

    def edge(self, a, b):
        for e in self.adj[a]:
            if e.to == b:
                return e
        raise KeyError((a, b))

    def straight_line(self, a, b):
        (x1, y1), (x2, y2) = self.nodes[a], self.nodes[b]
        return math.hypot(x1 - x2, y1 - y2)

    def heuristic_scale(self):
        """A* needs h(n) <= true remaining time, always. Straight-line distance is a safe
        lower bound ONLY IF no path is shorter than the straight line between its ends.
        If your coordinates are slightly off, this finds the factor (<=1) that restores safety."""
        if self._hscale is None:
            s = 1.0
            for a, edges in self.adj.items():
                for e in edges:
                    sl = self.straight_line(a, e.to)
                    if sl > 0:
                        s = min(s, e.distance_m / sl)
            self._hscale = s
        return self._hscale

    def heuristic(self, a, b):
        """Estimated minimum minutes from a to b = straight-line distance / fastest pace."""
        return self.heuristic_scale() * self.straight_line(a, b) / MAX_SPEED

    def heuristic_to_any(self, a, goals):
        """Several goals (a building with many entrances): optimistic = the closest one."""
        return min(self.heuristic(a, t) for t in goals)

    # ---- validation ---------------------------------------------------------
    def check(self):
        """Returns a list of human-readable problems with the map ([] = healthy)."""
        problems = []
        seen, stack = set(), [next(iter(self.nodes))]
        while stack:
            n = stack.pop()
            if n in seen:
                continue
            seen.add(n)
            stack.extend(e.to for e in self.adj[n])
        for n in self.nodes:
            if n not in seen:
                problems.append(f"'{n}' is not connected to the rest of the map (missing edge?)")
        s = self.heuristic_scale()
        if s < 0.9:
            problems.append(
                f"Coordinates and path lengths disagree (heuristic scaled to {s:.2f}). "
                "A* still gives correct routes but explores more nodes. Check that a path is never "
                "shorter than the straight-line distance between its end nodes.")
        return problems


# =============================================================================
# SECTION 3 - SEARCH ALGORITHMS
# =============================================================================
# A "cost function" turns an Edge into minutes. Normal planning uses typical time; cautious
# planning uses the slow end of the uncertainty range.
def typical_cost(e):
    return e.base_time


def cautious_cost(e):
    return e.max_time


def _as_list(x):
    return [x] if isinstance(x, str) else list(x)


def _rebuild(came_from, goal):
    path = [goal]
    while came_from[path[-1]] is not None:
        path.append(came_from[path[-1]])
    return path[::-1]


def bfs(g, starts, goals, accessible=False):
    """UNINFORMED. Fewest hops. Ignores distance, so NOT guaranteed fastest.
    starts / goals are lists of nodes (a building = all its entrances)."""
    starts, goals = _as_list(starts), set(_as_list(goals))
    queue, came_from, expanded = deque(starts), {s: None for s in starts}, 0
    while queue:
        node = queue.popleft()
        expanded += 1
        if node in goals:
            return _rebuild(came_from, node), expanded
        for e in g.neighbors(node, accessible):
            if e.to not in came_from:
                came_from[e.to] = node
                queue.append(e.to)
    return None, expanded


def best_first(g, starts, goals, accessible=False, cost_fn=typical_cost, use_heuristic=True):
    """Uniform-Cost Search (use_heuristic=False) and A* (use_heuristic=True) share one engine.
         priority f(n) = g(n) + h(n)      g = minutes walked so far, h = optimistic minutes left
       With h = 0 this is UCS / Dijkstra. Both return the fastest route; A* looks at fewer nodes
       because h pulls the search toward the goal."""
    starts, goals = _as_list(starts), set(_as_list(goals))
    h = (lambda n: g.heuristic_to_any(n, goals)) if use_heuristic else (lambda n: 0.0)
    frontier = [(h(s), 0.0, s) for s in starts]
    heapq.heapify(frontier)
    came_from, best_g, expanded = {s: None for s in starts}, {s: 0.0 for s in starts}, 0
    while frontier:
        _, cost, node = heapq.heappop(frontier)
        if cost > best_g.get(node, math.inf):
            continue                      # stale queue entry
        expanded += 1
        if node in goals:
            return _rebuild(came_from, node), cost, expanded
        for e in g.neighbors(node, accessible):
            new_cost = cost + cost_fn(e)
            if new_cost < best_g.get(e.to, math.inf):
                best_g[e.to] = new_cost
                came_from[e.to] = node
                heapq.heappush(frontier, (new_cost + h(e.to), new_cost, e.to))
    return None, math.inf, expanded


ALGORITHMS = ("astar", "ucs", "bfs")


def search(g, algorithm, starts, goals, accessible=False, cost_fn=typical_cost):
    """Run BFS, UCS or A* the same way. Returns (path, minutes, places_examined); path is None if no route."""
    if algorithm == "bfs":
        path, expanded = bfs(g, starts, goals, accessible)
        minutes = path_minutes(g, path, cost_fn) if path else math.inf   # BFS ignores cost; measure it afterwards
        return path, minutes, expanded
    if algorithm in ("ucs", "astar"):
        return best_first(g, starts, goals, accessible, cost_fn, use_heuristic=(algorithm == "astar"))
    raise ValueError("algorithm must be 'astar', 'ucs' or 'bfs'")


def path_minutes(g, path, cost_fn=typical_cost):
    return sum(cost_fn(g.edge(a, b)) for a, b in zip(path, path[1:]))


# =============================================================================
# SECTION 4 - CSP MULTI-STOP PLANNER (deadlines)
# =============================================================================
# A stop can be a building, and a building has several entrances. WHERE you arrive decides where
# the next leg starts, so the planner tracks the actual node you are standing on:
#     step(node, place) -> (minutes, node_you_arrive_at)      or None if unreachable
def solve_csp(start, stops, step, dwell=0.0, end=None, options=None):
    """
    CSP formulation
      Variables : the stops, assigned one at a time to visiting positions 1, 2, 3, ...
      Domain    : stops not yet visited
      Constraint: arrival time at each stop <= that stop's deadline
      step      : walking minutes (and arrival entrance) from where you stand to a stop (A* inside)
      dwell     : minutes spent at each stop (class length, errand time) before moving on

    Search techniques
      * Backtracking     - extend the order one stop at a time, undo on dead ends.
      * Forward checking - after choosing a stop, check every UNVISITED stop can still
                           meet its deadline even if walked to directly. If one cannot, prune now.
      * MRV ordering     - try stops with the earliest deadline first (fail fast).
      * Branch & bound   - keep searching for a feasible order that FINISHES earlier than the best
                           found, so the returned plan is the fastest feasible one.

    end: optional final destination. It must be in `stops` (for its deadline) but is always
         visited LAST; every other stop can be visited in any order.

    options(node, place): optional; every (minutes, entrance) way of arriving at the place. Without it the planner
         only tries the nearest entrance, which can be a worse start for the next leg.

    Returns (order, arrival_times, arrival_nodes) or (None, None, None) if the deadlines cannot all be met.
    """
    names = [s for s in stops if s != end]

    def hops(cur, place):
        if options is not None:
            return sorted(options(cur, place))
        h = step(cur, place)
        return [] if h is None else [h]
    best = {"finish": math.inf, "order": None, "arr": None, "via": None}

    def extend(cur, remaining, now, order, arr, via):
        if not remaining:
            if end is not None:
                found = hops(cur, end)
                if not found or now + found[0][0] > stops[end] + 1e-9:
                    return
                hop = found[0]
                order, arr, via, now = order + [end], arr + [now + hop[0]], via + [hop[1]], now + hop[0]
            if now < best["finish"]:
                best.update(finish=now, order=order, arr=arr, via=via)
            return
        if now >= best["finish"]:
            return                                              # bound: cannot beat best
        for nxt in sorted(remaining, key=lambda s: stops[s]):   # MRV: tightest deadline first
            for hop in hops(cur, nxt):                          # each entrance you could arrive by
                arrive = now + hop[0]
                if arrive > stops[nxt] + 1e-9:
                    continue                                    # violates own deadline
                leave = arrive + dwell
                rest = [s for s in remaining if s != nxt]
                # forward check: can every other stop still be reached in time?
                if any((h := step(hop[1], r)) is None or leave + h[0] > stops[r] + 1e-9 for r in rest):
                    continue
                extend(hop[1], rest, leave, order + [nxt], arr + [arrive], via + [hop[1]])

    extend(start, names, 0.0, [], [], [])
    return best["order"], best["arr"], best["via"]


def solve_local_search(start, stops, step, dwell=0.0, iterations=4000, seed=1, end=None):
    """Simulated annealing over visiting orders. Used when no order meets every deadline.
    Objective: minimise total lateness = sum of max(0, arrival - deadline).
    Moves: swap two stops. Worse moves are accepted with probability exp(-delta/temperature),
    which lets the search escape local minima; temperature cools over time."""
    rng = random.Random(seed)

    def evaluate(order):
        t, cur, late, arr, via = 0.0, start, 0.0, [], []
        for s in order + ([end] if end is not None else []):
            hop = step(cur, s)
            if hop is None:                                    # closures / accessibility cut this stop off
                return [], [], math.inf
            t += hop[0]
            arr.append(t)
            via.append(hop[1])
            late += max(0.0, t - stops[s])
            t += dwell
            cur = hop[1]
        return arr, via, late

    current = [s for s in stops if s != end]
    rng.shuffle(current)
    cur_arr, cur_via, cur_late = evaluate(current)
    best, best_arr, best_via, best_late = current[:], cur_arr, cur_via, cur_late
    temp = 50.0
    for _ in range(iterations):
        if len(current) < 2 or best_late == 0:
            break
        i, j = rng.sample(range(len(current)), 2)
        cand = current[:]
        cand[i], cand[j] = cand[j], cand[i]
        arr, via, late = evaluate(cand)
        delta = late - cur_late
        if delta < 0 or rng.random() < math.exp(-delta / max(temp, 1e-9)):
            current, cur_arr, cur_via, cur_late = cand, arr, via, late
            if late < best_late:
                best, best_arr, best_via, best_late = cand[:], arr, via, late
        temp *= 0.995
    if best_late == math.inf:                                  # some stop cannot be reached at all
        return None, None, None, math.inf
    return best + ([end] if end is not None else []), best_arr, best_via, best_late


# =============================================================================
# SECTION 5 - THE AGENT (what you actually call)
# =============================================================================
class NavigatorAgent:
    def __init__(self, nodes=None, edges=None, closed=None):
        self.graph = CampusGraph(NODES if nodes is None else nodes,
                                 EDGES if edges is None else edges,
                                 CLOSED if closed is None else closed,
                                 BUILDINGS)

    # ---- live updates -------------------------------------------------------
    def close(self, a, b):
        self.graph.close(a, b)

    def reopen(self, a, b):
        self.graph.reopen(a, b)

    def set_accessible(self, a, b, value):
        self.graph.set_accessible(a, b, value)

    def set_uncertain(self, a, b, min_time, max_time):
        self.graph.set_uncertain(a, b, min_time, max_time)

    # ---- single route -------------------------------------------------------
    def route(self, start, goal, accessible=False, cautious=False, algorithm="astar"):
        """Plan one route. Returns a dict (path, minutes, worst-case minutes, nodes expanded).
           accessible=True : avoid stairs-only paths.
           cautious=True   : plan against slow-end walking times.
           algorithm       : 'astar' (default), 'ucs' or 'bfs'."""
        g = self.graph
        starts, goals = g.resolve(start), g.resolve(goal)      # a building = any of its entrances
        cost_fn = cautious_cost if cautious else typical_cost
        path, _, expanded = search(g, algorithm, starts, goals, accessible, cost_fn)
        if path is None:
            return {"found": False, "algorithm": algorithm, "expanded": expanded,
                    "reason": "No route (closures or accessibility rules block every path)."}
        return {
            "found": True, "algorithm": algorithm, "path": path,
            "minutes": round(path_minutes(g, path), 2),
            "fastest_minutes": round(path_minutes(g, path, lambda e: e.min_time), 2),
            "worst_minutes": round(path_minutes(g, path, cautious_cost), 2),
            "meters": round(sum(g.edge(a, b).distance_m for a, b in zip(path, path[1:])), 1),
            "expanded": expanded,
        }

    # ---- multi-stop under deadlines ----------------------------------------
    def multi_stop(self, start, stops, accessible=False, cautious=False, dwell=0.0, end=None, algorithm="astar"):
        """stops = {place: latest_arrival_minutes_from_now}. Returns the visiting order.
           method == 'csp'     : an order meeting every deadline was found (fastest such order)
           method == 'anneal'  : impossible; best-effort order that minimises total lateness
           end                 : optional final destination (must be a key of `stops`); it is
                                 always visited last, the other stops in the best order.
           A place may be a building: you arrive at one of its entrances and the next leg starts there.
           algorithm: which search finds each walking leg ('astar', 'ucs' or 'bfs'); the CSP then orders the stops
                      using those leg times. 'expanded' in the result = places examined by all those searches."""
        g = self.graph
        cost_fn = cautious_cost if cautious else typical_cost
        for p in [start] + list(stops):
            g.resolve(p)                                        # raises a clear error on a typo
        table = {}                                              # (node, place) -> (minutes, arrival node, path)
        examined = [0]                                          # places looked at by all leg searches

        def step(node, place):
            key = (node, place)
            if key not in table:
                if node in g.resolve(place):
                    table[key] = (0.0, node, [node])            # already standing at that place
                else:
                    path, cost, n = search(g, algorithm, [node], g.resolve(place), accessible, cost_fn)
                    examined[0] += n
                    table[key] = None if path is None else (cost, path[-1], path)
            hit = table[key]
            return None if hit is None else (hit[0], hit[1])

        def leg(node, entrance):
            """Cheapest walk from node to one specific entrance (cached)."""
            key = ("to", node, entrance)
            if key not in table:
                if node == entrance:
                    table[key] = (0.0, entrance, [node])
                else:
                    path, cost, n = search(g, algorithm, [node], [entrance], accessible, cost_fn)
                    examined[0] += n
                    table[key] = None if path is None else (cost, entrance, path)
            return table[key]

        def options(node, place):
            found = [leg(node, e) for e in g.resolve(place)]
            return [(h[0], h[1]) for h in found if h is not None]

        best = None
        for s0 in g.resolve(start):                             # a building start: try each entrance
            order, arr, via = solve_csp(s0, stops, step, dwell, end, options)
            method, late = "csp", 0.0
            if order is None:
                method = "anneal"
                order, arr, via, late = solve_local_search(s0, stops, step, dwell, end=end)
                if order is None:
                    continue                                    # a stop is unreachable from this start
            if any(step(n, p) is None for n, p in zip([s0] + via, order)):
                continue
            finish = arr[-1] if arr else 0.0
            key = (method != "csp", late, finish)
            if best is None or key < best[0]:
                best = (key, s0, order, arr, via, method)
        if best is None:
            return {"found": False,
                    "reason": "Cannot walk to every stop with the current closures/accessibility rules."}
        _, s0, order, arr, via, method = best

        legs, cur = [], s0
        for p, v in zip(order, via):
            legs.append(leg(cur, v)[2])
            cur = v
        # Re-time the chosen order under slow walking so the user sees the risk.
        worst_t, plan = 0.0, []
        for p, a, leg in zip(order, arr, legs):
            worst_t += path_minutes(g, leg, cautious_cost)
            plan.append({"stop": p, "arrive": round(a, 2), "deadline": stops[p],
                         "late_by": round(max(0.0, a - stops[p]), 2),
                         "via": leg[-1],                        # entrance/node you arrive at
                         "arrive_if_slow": round(worst_t, 2),
                         "at_risk": a <= stops[p] < worst_t})
            worst_t += dwell
        return {"found": True, "method": method, "plan": plan, "legs": legs,
                "algorithm": algorithm, "expanded": examined[0],
                "all_deadlines_met": all(p["late_by"] == 0 for p in plan),
                "total_lateness": round(sum(p["late_by"] for p in plan), 2)}


# =============================================================================
# SECTION 6 - PRETTY PRINTING + COMMAND LINE
# =============================================================================
def show_route(r):
    if not r["found"]:
        print(f"   {r['algorithm']:>5}: {r['reason']}")
        return
    print(f"   {r['algorithm']:>5}: {' -> '.join(r['path'])}")
    print(f"          {r['minutes']} min typical ({r['fastest_minutes']}-{r['worst_minutes']} range), "
          f"{r['meters']} m, looked at {r['expanded']} nodes")


def show_multi(r):
    if not r["found"]:
        print("   ", r["reason"])
        return
    if r["method"] == "csp":
        print("   All deadlines can be met (CSP backtracking, fastest feasible order):")
    else:
        print(f"   No order meets every deadline. Best effort (simulated annealing), "
              f"total lateness {r['total_lateness']} min:")
    for i, p in enumerate(r["plan"], 1):
        flag = f"LATE by {p['late_by']}" if p["late_by"] else ("on time, RISKY if slow" if p["at_risk"] else "on time")
        print(f"   {i}. {p['stop']:<10} via {p['via']:<10} arrive {p['arrive']:>6} min  (deadline {p['deadline']:>5}, "
              f"slow-walk {p['arrive_if_slow']:>6})  {flag}")


def demo():
    agent = NavigatorAgent()
    g = agent.graph
    print("=" * 72, "\nSAU CAMPUS NAVIGATOR  -  1. SEARCH ALGORITHMS ON THE REAL MAP\n" + "=" * 72)
    print(f"{sum(1 for n in g.nodes if not (n[0] == 'J' and n[1:].isdigit() and int(n[1:]) >= 100))} places, {sum(len(v) for v in g.adj.values()) // 2} paths, "
          f"{len(g.buildings)} buildings, walking speed {TYPICAL_SPEED / 60:.1f} m/s")
    print("map check:", "; ".join(g.check()) or "healthy", f"| heuristic scale {g.heuristic_scale():.2f}")
    trips = [("Gate1", "GuestHouse"), ("Gate1", "Gate3"), ("Gate2", "Library"),
             ("Gate3", "HostelA"), ("Tapri", "ConventionCentre"), ("AC4", "Admin")]
    print("\nNodes examined (fewer = smarter search).  Same minutes = same fastest route.")
    print(f"{'trip':<28}{'BFS':>10}{'UCS':>10}{'A*':>10}   {'minutes':>8}  BFS route is fastest?")
    for s_, t_ in trips:
        r = {alg: agent.route(s_, t_, algorithm=alg) for alg in ("bfs", "ucs", "astar")}
        same = abs(r["bfs"]["minutes"] - r["ucs"]["minutes"]) < 0.05
        print(f"{s_ + ' -> ' + t_:<28}{r['bfs']['expanded']:>10}{r['ucs']['expanded']:>10}"
              f"{r['astar']['expanded']:>10}   {r['astar']['minutes']:>8}  {'yes' if same else 'NO (' + str(r['bfs']['minutes']) + ' min)'}")
    print("\nExample, A* route Gate1 -> Gate3:")
    show_route(agent.route("Gate1", "Gate3"))

    print("\n" + "=" * 72, "\n2. CLOSURES  Gate1 -> Gate3 (A* replans around a closed path)\n" + "=" * 72)
    show_route(agent.route("Gate1", "Gate3"))
    agent.close("J4", "J6")
    print("   after closing J4 <-> J6 (e.g. maintenance):")
    show_route(agent.route("Gate1", "Gate3"))
    agent.close("Gate1", "J16")
    print("   after ALSO closing the north-west road Gate1 <-> J16 (Gate 1 uses its other road, via J15):")
    show_route(agent.route("Gate1", "Gate3"))
    agent.close("Gate1", "J15")
    print("   after ALSO closing Gate1 <-> J15 (both roads out of Gate 1 are shut, so Gate 1 is cut off):")
    show_route(agent.route("Gate1", "Gate3"))
    agent.reopen("J4", "J6"); agent.reopen("Gate1", "J16"); agent.reopen("Gate1", "J15")

    print("\n" + "=" * 72, "\n3. ACCESSIBILITY  Gate1 -> Gate3  (what-if scenario)\n" + "=" * 72)
    print("   No real step data is recorded yet, so imagine J4 <-> J6 has steps. In the app, mark real ones\n"
          "   under More options > 'Paths with steps'.")
    show_route(agent.route("Gate1", "Gate3"))
    agent.set_accessible("J4", "J6", False)
    print("   wheelchair-accessible only, with J4 <-> J6 marked as steps:")
    show_route(agent.route("Gate1", "Gate3", accessible=True))
    agent.set_accessible("J4", "J6", True)

    print("\n" + "=" * 72, "\n4. TRAVEL-TIME UNCERTAINTY  Gate1 -> Library  (what-if scenario)\n" + "=" * 72)
    print("   Imagine J6 <-> J11 is crowded at class change: 1.2 to 4.0 min instead of about 1.0.")
    agent.set_uncertain("J6", "J11", 1.2, 4.0)
    show_route(agent.route("Gate1", "Library"))
    print("   cautious (plans on slow-end times, avoids the unpredictable path):")
    show_route(agent.route("Gate1", "Library", cautious=True))
    agent.set_uncertain("J6", "J11", 88.5 / TYPICAL_SPEED * 0.85, 88.5 / TYPICAL_SPEED * 1.35)

    print("\n" + "=" * 72, "\n5. MULTI-STOP CSP  Gate1 -> four stops, deadlines in minutes from now\n" + "=" * 72)
    stops = {"Library": 12, "Admin": 20, "AC1": 26, "HostelA": 45}
    print("   stops:", stops)
    show_multi(agent.multi_stop("Gate1", stops))
    print("\n   same stops but Library must be reached by minute 4 (impossible):")
    show_multi(agent.multi_stop("Gate1", {**stops, "Library": 4}))
    print("\n   stops in any order, must END at Gate3:")
    show_multi(agent.multi_stop("Gate1", {"Library": 30, "AC1": 30, "Gate3": 40}, end="Gate3"))


def main(argv):
    usage = ("usage:\n"
             "  python campus_navigator.py check\n"
             "  python campus_navigator.py demo\n"
             "  python campus_navigator.py route FROM TO [--accessible] [--cautious] [--alg astar|ucs|bfs]\n"
             "  python campus_navigator.py multi FROM STOP=DEADLINE [STOP=DEADLINE ...] [--accessible] [--cautious]")
    if len(argv) < 2:
        # No command given (e.g. pressing "Run" in VS Code): run the demo.
        demo()
        print("\n" + "-" * 72)
        print(usage)
        return
    cmd, rest = argv[1], argv[2:]
    flags = {a for a in rest if a.startswith("--")}
    alg = "astar"
    if "--alg" in rest:
        alg = rest[rest.index("--alg") + 1]
        rest = [a for i, a in enumerate(rest) if a != "--alg" and (i == 0 or rest[i - 1] != "--alg")]
    pos = [a for a in rest if not a.startswith("--")]
    agent = NavigatorAgent()
    if cmd == "check":
        problems = agent.graph.check()
        print(f"{len(agent.graph.nodes)} places, {sum(len(v) for v in agent.graph.adj.values()) // 2} paths.")
        print("\n".join("PROBLEM: " + p for p in problems) if problems else "Map looks healthy.")
    elif cmd == "demo":
        demo()
    elif cmd == "route" and len(pos) == 2:
        show_route(agent.route(pos[0], pos[1], "--accessible" in flags, "--cautious" in flags, alg))
    elif cmd == "multi" and len(pos) >= 2:
        stops = {k: float(v) for k, v in (p.split("=") for p in pos[1:])}
        show_multi(agent.multi_stop(pos[0], stops, "--accessible" in flags, "--cautious" in flags))
    else:
        print(usage)


if __name__ == "__main__":
    main(sys.argv)
