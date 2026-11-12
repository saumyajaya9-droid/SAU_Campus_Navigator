"""Tests for campus_navigator.py.  Run:  python test_campus_navigator.py   (or: python -m pytest)"""
import itertools, math, random
import campus_navigator as c


def fresh():
    return c.NavigatorAgent()


def no_indoor(agent):
    """Close every walk-through-the-building link (the app does this when 'Walk through buildings' is off)."""
    for pair in c.INDOOR_PAIRS:
        agent.close(*sorted(pair))
    return agent


def uses_road(path, a, b):
    """Does the path walk along the road a-b? (A road with entrance points on it is a chain of pieces.)"""
    chain = c.SNAP_CHAINS.get((a, b)) or (c.SNAP_CHAINS.get((b, a)) or [])[::-1] or [a, b]
    pieces = {frozenset(p) for p in zip(chain, chain[1:])}
    return any(frozenset(p) in pieces for p in zip(path, path[1:]))


def test_map_is_healthy():
    g = fresh().graph
    assert g.check() == []
    assert len(g.nodes) >= 57 and len(g.buildings) == 14        # 57 places + the hidden points where entrances meet the roads


def test_astar_matches_ucs_on_every_pair():
    """A* must be as good as Uniform-Cost (optimal) for every pair of places, with a never-overestimating heuristic."""
    a = fresh(); g = a.graph
    names = list(g.nodes)
    for s, t in itertools.combinations(names, 2):
        u = c.best_first(g, [s], [t], use_heuristic=False)
        h = c.best_first(g, [s], [t], use_heuristic=True)
        assert abs(u[1] - h[1]) < 1e-9, (s, t, u[1], h[1])


def test_heuristic_never_overestimates():
    g = fresh().graph
    for s in g.nodes:
        for t in g.nodes:
            if s != t:
                _, true_cost, _ = c.best_first(g, [s], [t], use_heuristic=False)
                assert g.heuristic(s, t) <= true_cost + 1e-9, (s, t)


def test_bfs_has_fewest_hops():
    g = fresh().graph
    names = list(g.nodes)
    for s, t in random.Random(1).sample(list(itertools.permutations(names, 2)), 300):
        bfs_path, _ = c.bfs(g, [s], [t])
        ucs_path, _, _ = c.best_first(g, [s], [t], use_heuristic=False)
        assert len(bfs_path) <= len(ucs_path)


def test_building_is_reached_through_any_entrance():
    a = fresh()
    r = a.route("Gate1", "GuestHouse")
    assert r["path"][-1] in a.graph.buildings["GuestHouse"]
    r = a.route("Gate3", "AC1")
    assert r["path"][-1] in a.graph.buildings["AC1"]


def test_closed_path_is_never_used_and_reopening_restores():
    a = no_indoor(fresh())
    base = a.route("Gate1", "Gate3")
    assert uses_road(base["path"], "J4", "J6")
    a.close("J4", "J6")
    r = a.route("Gate1", "Gate3")
    assert r["found"] and not uses_road(r["path"], "J4", "J6")
    assert r["minutes"] >= base["minutes"]
    a.reopen("J4", "J6")
    assert a.route("Gate1", "Gate3")["path"] == base["path"]


def test_no_route_when_gate_is_cut_off():
    a = no_indoor(fresh())
    a.close("Gate1", "J15"); a.close("Gate1", "J16")      # Gate1 has two ways out; close both
    assert a.route("Gate1", "Gate3")["found"] is False


def test_gate1_has_a_second_road_when_one_is_closed():
    a = no_indoor(fresh())
    a.close("Gate1", "J15")
    r = a.route("Gate1", "Gate3")
    assert r["found"] and r["path"][1:4] == ["SecurityRoom", "SafalStore", "Tapri"]
    a.reopen("Gate1", "J15"); a.close("Gate1", "J16")
    r = a.route("Gate1", "Gate3")
    assert r["found"] and "J101" in r["path"]


def test_multi_stop_reports_an_unreachable_stop_instead_of_crashing():
    a = no_indoor(fresh())
    a.close("Gate1", "J15"); a.close("Gate1", "J16")     # Gate1 cut off from everything
    r = a.multi_stop("Gate1", {"Library": 30, "Admin": 30})
    assert r["found"] is False and "reason" in r


def test_accessible_route_avoids_stepped_paths():
    a = no_indoor(fresh()); g = a.graph
    a.set_accessible("J4", "J6", False)
    assert uses_road(a.route("Gate1", "Gate3")["path"], "J4", "J6")
    r = a.route("Gate1", "Gate3", accessible=True)
    assert not uses_road(r["path"], "J4", "J6") and r["found"]
    for x, y in zip(r["path"], r["path"][1:]):
        assert g.edge(x, y).accessible


def test_cautious_never_has_a_worse_worst_case():
    a = fresh()
    a.set_uncertain("J6", "J11", 1.2, 4.0)
    for s, t in [("Gate1", "Library"), ("Gate2", "Library"), ("Gate1", "Admin"), ("AC2", "GuestHouse")]:
        n, k = a.route(s, t), a.route(s, t, cautious=True)
        assert k["worst_minutes"] <= n["worst_minutes"] + 1e-9


def test_entrance_can_be_reached_from_the_road_beside_it():
    a = fresh()
    r = a.route("Gate3", "AC4_W")
    assert r["found"] and r["path"][-2].startswith("J1") and r["meters"] < 300      # steps off the ring road beside AC4_W


def test_indoor_links_join_the_entrances_of_a_building():
    a = fresh(); g = a.graph
    for b, ents in g.buildings.items():
        for x in ents:
            for y in ents:
                if x != y:
                    assert any(e.to == y for e in g.adj[x]), (b, x, y)


def test_j6_sits_on_the_j4_j11_road():
    g = fresh().graph
    chain = c.SNAP_CHAINS.get(("J4", "J6")) or ["J4", "J6"]
    assert abs(sum(g.edge(u, v).distance_m for u, v in zip(chain, chain[1:])) + g.edge("J6", "J11").distance_m - 187.7) < 3
    assert chain[-1] == "J6" and len(g.adj["J6"]) >= 3                                       # J6 is a real junction on that road


def _brute_force(a, start, stops, dwell=0.0, end=None):
    """Try every visiting order (and entrance choice) the slow, obvious way."""
    g = a.graph; best = math.inf
    names = [s for s in stops if s != end]
    for start_node in g.resolve(start):
        for order in itertools.permutations(names):
            seq = list(order) + ([end] if end else [])
            def go(node, i, t):
                nonlocal best
                if i == len(seq):
                    best = min(best, t); return
                p = seq[i]
                for goal in g.resolve(p):                      # any entrance of that building
                    path, cost, _ = c.best_first(g, [node], [goal])
                    if path and t + cost <= stops[p] + 1e-9:
                        go(goal, i + 1, t + cost + (dwell if i < len(seq) - 1 else 0))
            go(start_node, 0, 0.0)
    return best


def test_csp_finds_the_fastest_feasible_order():
    a = fresh()
    cases = [("Gate1", {"Library": 12, "Admin": 20, "AC1": 26, "HostelA": 45}, None),
             ("Gate2", {"Library": 30, "AC1": 30, "Gate3": 45}, "Gate3"),
             ("Tapri", {"Admin": 15, "HostelB": 20, "GuestHouse": 30}, None)]
    for start, stops, end in cases:
        r = a.multi_stop(start, stops, end=end)
        assert r["found"] and r["method"] == "csp" and r["all_deadlines_met"]
        assert abs(r["plan"][-1]["arrive"] - _brute_force(a, start, stops, end=end)) < 0.02, (start, stops)
        if end:
            assert r["plan"][-1]["stop"] == end


def test_csp_respects_dwell_time():
    a = fresh()
    r0 = a.multi_stop("Gate1", {"Library": 40, "AC1": 40, "Admin": 40})
    r5 = a.multi_stop("Gate1", {"Library": 40, "AC1": 40, "Admin": 40}, dwell=5)
    assert r5["plan"][-1]["arrive"] >= r0["plan"][-1]["arrive"] + 10 - 1e-6   # 2 stops to dwell at before the last


def test_impossible_deadlines_fall_back_to_annealing():
    a = fresh()
    r = a.multi_stop("Gate1", {"Library": 4, "Admin": 20, "AC1": 26})
    assert r["found"] and r["method"] == "anneal" and not r["all_deadlines_met"]
    assert sorted(p["stop"] for p in r["plan"]) == ["AC1", "Admin", "Library"]
    assert r["total_lateness"] > 0


def test_legs_chain_together_at_the_entrance_you_arrived_by():
    a = fresh()
    r = a.multi_stop("Gate1", {"Library": 30, "AC1": 30, "Admin": 30})
    for leg, nxt in zip(r["legs"], r["legs"][1:]):
        assert leg[-1] == nxt[0]


def test_multi_stop_works_with_each_algorithm():
    a = fresh()
    stops = {"Library": 30, "AC1": 30, "Gate3": 45}
    r = {alg: a.multi_stop("Gate1", stops, end="Gate3", algorithm=alg) for alg in c.ALGORITHMS}
    assert all(x["found"] and x["plan"][-1]["stop"] == "Gate3" for x in r.values())
    assert abs(r["astar"]["plan"][-1]["arrive"] - r["ucs"]["plan"][-1]["arrive"]) < 1e-6   # both optimal
    assert r["bfs"]["plan"][-1]["arrive"] >= r["astar"]["plan"][-1]["arrive"] - 1e-6
    assert r["astar"]["expanded"] <= r["ucs"]["expanded"]                                    # A* looks at fewer places


if __name__ == "__main__":
    tests = [(n, f) for n, f in sorted(globals().items()) if n.startswith("test_")]
    for name, fn in tests:
        fn(); print("ok  ", name)
    print(f"\nall {len(tests)} tests passed")
