"""
app.py -- Google-Maps-style campus navigator (Streamlit front end for campus_navigator.py)

Run:   python -m streamlit run app.py
Needs: python -m pip install --upgrade streamlit matplotlib pandas
Keep this file in the SAME folder as campus_navigator.py. The map comes from there, so when you
replace NODES/EDGES in campus_navigator.py this app shows your real campus.
"""

import math
import re

import matplotlib.pyplot as plt
import streamlit as st

# Pressing "Run" in an editor (or `python app.py`) starts the script without Streamlit's server, which
# only prints warnings. Detect that and relaunch properly with `streamlit run`.
from streamlit.runtime.scriptrunner import get_script_run_ctx

if get_script_run_ctx(suppress_warning=True) is None:
    import subprocess
    import sys
    print("Starting the app with Streamlit... (it will open in your browser; press Ctrl+C here to stop)")
    sys.exit(subprocess.call([sys.executable, "-m", "streamlit", "run", __file__]))

try:
    import folium
    import streamlit.components.v1 as components
    from campus_geo import GEO, ROAD_SHAPES
    SATELLITE_OK = True
except ImportError as _err:              # satellite view needs: pip install folium
    SATELLITE_OK = False
    SATELLITE_ERROR = str(_err)

from campus_navigator import INDOOR_PAIRS, SNAP_CHAINS, NavigatorAgent, cautious_cost, path_minutes, typical_cost

st.set_page_config(page_title="SAU Campus Navigator", page_icon="🧭", layout="wide")

st.html("""
<style>
@import url('https://fonts.googleapis.com/css2?family=Poppins:wght@400;600;700;800&display=swap');
html, body, [class*="css"], .stApp { font-family: 'Poppins', system-ui, sans-serif; }
.stApp { background: radial-gradient(1200px 600px at 8% -10%, #1b2a6b 0%, transparent 60%),
                     radial-gradient(900px 500px at 100% 0%, #5b1a6b 0%, transparent 55%), #0b1020; color: #e8ecff; }
header[data-testid="stHeader"] { background: transparent; }
.block-container { padding-top: 1.2rem; max-width: 1500px; }
.hero { position: relative; overflow: hidden; border-radius: 24px; padding: 28px 34px; margin-bottom: 18px;
        background: linear-gradient(120deg, rgba(77,163,255,.22), rgba(176,89,255,.22) 55%, rgba(255,77,141,.20));
        border: 1px solid rgba(255,255,255,.14); box-shadow: 0 20px 60px rgba(0,0,0,.45); backdrop-filter: blur(10px); }
.hero:before { content:""; position:absolute; inset:-40%; background: conic-gradient(from 0deg, transparent, rgba(77,163,255,.18), transparent 30%);
               animation: spin 14s linear infinite; }
@keyframes spin { to { transform: rotate(360deg); } }
.hero > * { position: relative; }
.hero h1 { margin:0; font-weight:800; font-size: 2.3rem; letter-spacing:-.5px;
           background: linear-gradient(90deg,#8ec5ff,#d9a7ff 50%,#ff9ac4); -webkit-background-clip:text; background-clip:text; color:transparent; }
.hero p { margin:.3rem 0 14px; color:#c5cdf5; font-size:1rem; }
.chips span { display:inline-block; margin:0 8px 6px 0; padding:6px 14px; border-radius:999px; font-size:.82rem; font-weight:600;
              background: rgba(255,255,255,.08); border:1px solid rgba(255,255,255,.16); color:#e8ecff; }
.chips b { color:#fff; }
/* inputs */
div[data-baseweb="select"] > div, .stNumberInput input { background: rgba(255,255,255,.07) !important; border: 1px solid rgba(255,255,255,.18) !important;
        border-radius: 14px !important; color:#fff !important; }
label, .stCheckbox, .stRadio label { color:#d6dcff !important; }
.stButton > button { border-radius: 14px; border:1px solid rgba(255,255,255,.2); background: rgba(255,255,255,.08); color:#fff; transition: all .2s; }
.stButton > button:hover { transform: translateY(-2px); border-color:#8ec5ff; box-shadow: 0 8px 22px rgba(77,163,255,.35); }
.stButton > button[kind="primary"] { background: linear-gradient(90deg,#3b82f6,#a855f7 55%,#ec4899); border:none; font-weight:700; font-size:1.1rem;
        padding:.7rem 1rem; box-shadow: 0 10px 30px rgba(168,85,247,.45); animation: glow 2.4s ease-in-out infinite; }
@keyframes glow { 50% { box-shadow: 0 10px 42px rgba(236,72,153,.65); } }
details { background: rgba(255,255,255,.05) !important; border:1px solid rgba(255,255,255,.12) !important; border-radius:16px !important; }
hr { border-color: rgba(255,255,255,.12); }
/* podium cards */
.podium { display:grid; grid-template-columns: repeat(3,1fr); gap:12px; margin:6px 0 12px; }
.card { position:relative; border-radius:20px; padding:16px 16px 14px; background: rgba(255,255,255,.07); border:1px solid rgba(255,255,255,.14);
        box-shadow: 0 12px 30px rgba(0,0,0,.35); animation: rise .6s both; overflow:hidden; }
.card:before { content:""; position:absolute; left:0; top:0; right:0; height:5px; background: var(--c); }
.card.first { background: linear-gradient(160deg, rgba(255,215,0,.16), rgba(255,255,255,.06)); border-color: rgba(255,215,0,.55);
              box-shadow: 0 14px 40px rgba(255,200,0,.25); }
@keyframes rise { from { opacity:0; transform: translateY(18px);} to { opacity:1; transform:none; } }
.card .rank { font-size:1.5rem; }
.card .nm { font-weight:700; font-size:1rem; color: var(--c); margin:2px 0 6px; line-height:1.2; }
.card .big { font-size:2rem; font-weight:800; color:#fff; line-height:1; }
.card .big small { font-size:.85rem; font-weight:600; color:#aab4e8; }
.card .sub { color:#b9c2f0; font-size:.8rem; margin-top:6px; }
.pill { display:inline-block; margin-top:8px; padding:3px 11px; border-radius:999px; font-size:.75rem; font-weight:700; }
.pill.best { background:#ffd54a; color:#3b2f00; } .pill.slow { background: rgba(255,77,100,.2); color:#ff9aa8; border:1px solid rgba(255,77,100,.5); }
.pill.same { background: rgba(45,224,138,.18); color:#7dffc0; border:1px solid rgba(45,224,138,.5); }
.bar { height:6px; border-radius:6px; background: rgba(255,255,255,.12); margin-top:10px; overflow:hidden; }
.bar i { display:block; height:100%; background: var(--c); border-radius:6px; }
.empty { text-align:center; padding:26px 14px; border-radius:20px; border:1px dashed rgba(255,255,255,.28); color:#c5cdf5; background: rgba(255,255,255,.04); }
.empty .ic { font-size:2.4rem; animation: bob 2.2s ease-in-out infinite; display:inline-block; } @keyframes bob { 50% { transform: translateY(-8px);} }
.legend span { margin-right:14px; font-weight:600; }
iframe { border-radius: 20px; }
[data-testid="stImage"] img, [data-testid="stPyplotRootElement"] { border-radius: 20px; }
@media (max-width: 900px) { .podium { grid-template-columns: 1fr; } .hero h1 { font-size:1.6rem; } }
</style>
""")

agent = NavigatorAgent()           # rebuilt on every interaction; closures re-applied below
graph = agent.graph
entrances = {n for ents in graph.buildings.values() for n in ents}
is_junction = lambda n: re.fullmatch(r"J\d+", n) is not None
# What a person can pick: buildings (reached through any entrance), gates, shops, open spaces. Not junctions.
places = sorted(graph.buildings) + sorted(n for n in graph.nodes if n not in entrances and not is_junction(n))
paths = sorted({tuple(sorted((a, e.to))) for a, es in graph.adj.items() for e in es})
# Only the real roads can be closed (junction/gate to junction/gate), not the short entrance links.
is_road_node = lambda n: is_junction(n) or n.startswith("Gate")
road_paths = [p for p in paths if is_road_node(p[0]) and is_road_node(p[1])]
is_hidden = lambda n: is_junction(n) and int(n[1:]) >= 100        # J101 and up: where an entrance meets a road
# A road with entrance points on it is several small pieces; people think of it as one road (J1 - J4).
piece_of = {tuple(sorted((u, v))): key for key, chain in SNAP_CHAINS.items() for u, v in zip(chain, chain[1:])}
road_keys = sorted({piece_of.get(p, p) for p in road_paths})
path_label = {k: f"{k[0]} - {k[1]}" for k in road_keys}
label_to_path = {v: k for k, v in path_label.items()}


def road_length(key):
    chain = SNAP_CHAINS.get(key) or [key[0], key[1]]
    return sum(graph.edge(u, v).distance_m for u, v in zip(chain, chain[1:]))


def where(place):
    """x, y of a place on the map. A building sits at the middle of its entrances."""
    nodes = graph.resolve(place)
    return (sum(graph.nodes[n][0] for n in nodes) / len(nodes), sum(graph.nodes[n][1] for n in nodes) / len(nodes))


def pick_default(name, fallback_index):
    return name if name in places else places[min(fallback_index, len(places) - 1)]


# ------------------------------------------------------------------ state: the list of stops
if "stops" not in st.session_state:
    st.session_state.stops = [0]
    st.session_state.next_id = 1
    st.session_state["stop_0"] = pick_default("Admin", 2)
st.session_state.setdefault("from_place", pick_default("Gate1", 0))
st.session_state.setdefault("to_place", pick_default("GuestHouse", 1))


def add_stop():
    i = st.session_state.next_id
    st.session_state.next_id += 1
    st.session_state.stops.append(i)


def remove_stop(i):
    if i in st.session_state.stops:          # a double click can ask twice; the second time it is already gone
        st.session_state.stops.remove(i)
    st.session_state.pop(f"stop_{i}", None)
    st.session_state.pop(f"dl_{i}", None)


def swap_ends():
    st.session_state.from_place, st.session_state.to_place = (
        st.session_state.to_place, st.session_state.from_place)


# ------------------------------------------------------------------ map drawing
def draw_map(routes, markers):
    """routes: [(legs, colour, width)], drawn in order (widest first). markers: [((x, y), label, colour)]."""
    fig, ax = plt.subplots(figsize=(7, 9))
    for a, b in paths:
        e = graph.edge(a, b)
        (x1, y1), (x2, y2) = graph.nodes[a], graph.nodes[b]
        road = (a, b) in road_paths
        if frozenset((a, b)) in INDOOR_PAIRS:
            if e.closed:
                continue
            style = dict(color="#b388ff", linestyle=(0, (2, 2)), linewidth=1.2)
        elif e.closed:
            style = dict(color="#d4470d", linestyle="--", linewidth=2.5)
        elif not e.accessible:
            style = dict(color="#b8960c", linestyle=":", linewidth=3)
        elif road:
            style = dict(color="#b6c0c9", linestyle="-", linewidth=2.5)
        else:
            style = dict(color="#dfe4e8", linestyle="-", linewidth=1)
        ax.plot([x1, x2], [y1, y2], zorder=1, **style)
    for z, (legs, colour, width) in enumerate(routes):
        for leg in legs:
            ax.plot([graph.nodes[n][0] for n in leg], [graph.nodes[n][1] for n in leg],
                    color=colour, linewidth=width, zorder=2 + z, solid_capstyle="round", solid_joinstyle="round")
    for n, (x, y) in graph.nodes.items():
        if n in entrances:
            ax.scatter([x], [y], s=10, color="#0b8a8a", zorder=3)
        elif is_hidden(n):
            continue
        elif is_junction(n):
            ax.scatter([x], [y], s=22, color="#5f6b76", zorder=3)
            ax.annotate(n, (x, y), textcoords="offset points", xytext=(4, 4), fontsize=6.5, color="#7a858f")
        else:
            ax.scatter([x], [y], s=34, color="#111" if n.startswith("Gate") else "#8e24aa", zorder=3,
                       marker="^" if n.startswith("Gate") else "o")
            ax.annotate(n, (x, y), textcoords="offset points", xytext=(6, -12), fontsize=8, color="#3c4650")
    for b in graph.buildings:
        x, y = where(b)
        ax.annotate(b, (x, y), ha="center", va="center", fontsize=8, fontweight="bold", color="#9a6700", zorder=4)
    for (x, y), label, colour in markers:
        ax.annotate(label, (x, y), textcoords="offset points", xytext=(0, 0), ha="center", va="center",
                    fontsize=10, fontweight="bold", color="white", zorder=5,
                    bbox=dict(boxstyle="circle,pad=0.35", fc=colour, ec="white", lw=1.5))
    ax.set_aspect("equal")
    ax.axis("off")
    fig.tight_layout()
    return fig


def edge_line(a, b):
    """Lat/lon points for walking from node a to node b: the real road shape if we have it, else a straight line."""
    pts = ROAD_SHAPES.get("|".join(sorted((a, b))))
    if not pts:
        return [GEO[a], GEO[b]]
    d = lambda p, q: (p[0] - q[0]) ** 2 + (p[1] - q[1]) ** 2
    return pts if d(pts[0], GEO[a]) + d(pts[-1], GEO[b]) <= d(pts[0], GEO[b]) + d(pts[-1], GEO[a]) else pts[::-1]


def draw_satellite(routes, markers):
    """routes: [(legs, colour, width)], drawn in order (widest first). markers: [((lat, lon), label, colour)].
    Needs internet for the satellite tiles."""
    lats, lons = [p[0] for p in GEO.values()], [p[1] for p in GEO.values()]
    m = folium.Map(location=[sum(lats) / len(lats), sum(lons) / len(lons)], zoom_start=17, tiles=None,
                   control_scale=True, max_zoom=20)
    folium.TileLayer(
        "https://server.arcgisonline.com/ArcGIS/rest/services/World_Imagery/MapServer/tile/{z}/{y}/{x}",
        attr="Imagery: Esri, Maxar, Earthstar Geographics", name="Satellite", max_native_zoom=19, max_zoom=20).add_to(m)
    folium.TileLayer("OpenStreetMap", name="Street map", max_zoom=20).add_to(m)
    for a, b in paths:                                                # all paths: roads in white, links faint
        e = graph.edge(a, b)
        road = (a, b) in road_paths
        if frozenset((a, b)) in INDOOR_PAIRS:
            if e.closed:
                continue
            kw = dict(color="#d1b3ff", weight=2, dash_array="3 6", opacity=0.9)
        elif e.closed:
            kw = dict(color="#ff6d00", weight=5, dash_array="8 8", opacity=1)
        elif not e.accessible:
            kw = dict(color="#ffd600", weight=5, dash_array="2 8", opacity=1)
        elif road:
            kw = dict(color="#ffffff", weight=3, opacity=0.75)
        else:
            kw = dict(color="#ffffff", weight=1, opacity=0.35)
        folium.PolyLine(edge_line(a, b), **kw).add_to(m)
    for legs, colour, width in routes:                                # your routes, on top (widest first)
        for leg in legs:
            line = []
            for a, b in zip(leg, leg[1:]):
                if frozenset((a, b)) in INDOOR_PAIRS:                 # through a building: dashed
                    if len(line) > 1:
                        folium.PolyLine(line, color=colour, weight=width, opacity=0.95).add_to(m)
                    line = []
                    folium.PolyLine([GEO[a], GEO[b]], color=colour, weight=width, opacity=0.95,
                                    dash_array="2 10", tooltip="through the building").add_to(m)
                else:
                    line += edge_line(a, b)
            if len(line) > 1:
                folium.PolyLine(line, color=colour, weight=width, opacity=0.95).add_to(m)
    for n, (la, lo) in GEO.items():
        if n in entrances:
            folium.CircleMarker((la, lo), radius=3, color="#00bfa5", fill=True, fill_opacity=1, weight=1,
                                tooltip=n).add_to(m)
        elif is_hidden(n):
            continue
        elif is_junction(n):
            folium.CircleMarker((la, lo), radius=4, color="#cfd8dc", fill=True, fill_color="#455a64",
                                fill_opacity=1, weight=1, tooltip=n).add_to(m)
        else:
            folium.CircleMarker((la, lo), radius=6, color="white", fill=True,
                                fill_color="#111" if n.startswith("Gate") else "#8e24aa", fill_opacity=1,
                                weight=2, tooltip=n).add_to(m)
    for b, ents in graph.buildings.items():                           # building names at the middle of their entrances
        la = sum(GEO[e][0] for e in ents) / len(ents)
        lo = sum(GEO[e][1] for e in ents) / len(ents)
        folium.Marker((la, lo), icon=folium.DivIcon(
            icon_size=(90, 16), icon_anchor=(45, 8),
            html=f'<div style="font:700 11px sans-serif;color:#fff;text-align:center;'
                 f'text-shadow:0 0 3px #000,0 0 3px #000">{b}</div>')).add_to(m)
    for (la, lo), label, colour in markers:
        folium.Marker((la, lo), icon=folium.DivIcon(
            icon_size=(26, 26), icon_anchor=(13, 13),
            html=f'<div style="width:26px;height:26px;border-radius:50%;background:{colour};color:#fff;'
                 f'border:2px solid #fff;font:700 13px/22px sans-serif;text-align:center">{label}</div>')).add_to(m)
    m.fit_bounds([[min(lats), min(lons)], [max(lats), max(lons)]])
    folium.LayerControl(collapsed=True).add_to(m)
    return m


def leg_steps(path, cost_fn):
    """Turn one path into 'Walk X m to Y' lines."""
    lines = []
    for a, b in zip(path, path[1:]):
        e = graph.edge(a, b)
        note = "" if e.accessible else " (stairs)"
        if e.max_time - e.min_time > 0.6 * e.base_time:
            note += f" (unpredictable: {e.min_time:.1f}-{e.max_time:.1f} min)"
        if frozenset((a, b)) in INDOOR_PAIRS:
            note += " (through the building)"
        lines.append(f"{a} → {b}: {e.distance_m:g} m, {cost_fn(e):.1f} min{note}")
    return lines


# ------------------------------------------------------------------ page layout
st.html(f"""
<div class="hero"><h1>🧭 SAU Campus Navigator</h1>
<p style="margin-bottom:0">Pick a start and a destination. A*, UCS and BFS race each other, and the best route wins.</p>
</div>
""")
panel, mapcol = st.columns([1, 1.35], gap="large")

with panel:
    # ---- From / stops / To, laid out like a directions panel
    c_from, c_swap = st.columns([6, 1], vertical_alignment="bottom")
    start = c_from.selectbox("From", places, key="from_place")
    c_swap.button("⇅", on_click=swap_ends, help="Swap start and destination")

    stop_values, stop_ids = [], list(st.session_state.stops)
    use_deadlines = st.session_state.get("use_deadlines", False)
    for i in stop_ids:
        c_sel, c_dl, c_x = st.columns([5, 2, 1] if use_deadlines else [6, 0.01, 1], vertical_alignment="bottom")
        val = c_sel.selectbox(f"Stop {stop_ids.index(i) + 1}", places, index=None,
                              placeholder="Choose a stop", key=f"stop_{i}")
        if use_deadlines:
            c_dl.number_input("By (min)", min_value=0.0, step=1.0, value=20.0, key=f"dl_{i}")
        c_x.button("✕", key=f"rm_{i}", on_click=remove_stop, args=(i,), help="Remove this stop")
        stop_values.append((i, val))

    st.button("＋ Add stop", on_click=add_stop)
    end = st.selectbox("To", places, key="to_place")
    if use_deadlines:
        end_deadline = st.number_input("Arrive at destination by (min)", min_value=0.0, step=1.0, value=30.0,
                                       key="dl_end")
    else:
        end_deadline = math.inf

    # ---- options
    o1, o2 = st.columns(2)
    accessible = o1.checkbox("Accessible route", help="Avoids paths marked as having steps (set them under More options).")
    cautious = o2.checkbox("Allow extra time", help="Plans with the slow end of each path's walking time.")
    optimise = st.checkbox("Best order for my stops", value=True,
                           help="On: the planner picks the quickest order to visit your stops, then ends at "
                                "the destination. Off: stops are visited in the order you listed them.")
    through = st.checkbox("Walk through buildings", value=True,
                          help="On: a route may enter a building by one entrance and leave by another when that is "
                               "quicker (shown dashed). Off: routes stay on the outside paths.")
    with st.expander("More options"):
        st.checkbox("Set arrive-by times (deadlines)", key="use_deadlines")
        dwell = st.number_input("Minutes spent at each stop", min_value=0.0, value=0.0, step=1.0)
        closed = st.multiselect("Closed paths", list(path_label.values()),
                                help="Paths blocked right now (maintenance, event).")
        stepped = st.multiselect("Paths with steps", list(path_label.values()),
                                 help="Not wheelchair-friendly. 'Accessible route' will avoid these.")
        crowded = st.multiselect("Crowded / unpredictable paths", list(path_label.values()),
                                 help="Walking time can swing from 0.8x to 2.5x. 'Allow extra time' plans for the slow end.")
    if not through:
        for pair in INDOOR_PAIRS:
            agent.close(*sorted(pair))
    for label in closed:
        agent.close(*label_to_path[label])
    for label in stepped:
        agent.set_accessible(*label_to_path[label], False)
    for label in crowded:
        typical = road_length(label_to_path[label]) / 90.0
        agent.set_uncertain(*label_to_path[label], typical * 0.8, typical * 2.5)

# ------------------------------------------------------------------ plan the trip with all three algorithms
stops = []
for i, val in stop_values:
    if val and val not in stops and val not in (start, end):
        stops.append(val)
deadline_of = {}
for i, val in stop_values:
    if val in stops and use_deadlines:
        deadline_of[val] = float(st.session_state.get(f"dl_{i}", math.inf))

cost_fn = cautious_cost if cautious else typical_cost
ALG = {"astar": ("A*", "#1a73e8", 4), "ucs": ("Uniform-cost search (UCS)", "#00c853", 8), "bfs": ("BFS", "#ff4081", 12)}


def plan_trip(alg):
    """Plan the whole trip using one algorithm for every walking leg. Returns a result dict."""
    res = {"alg": alg, "name": ALG[alg][0], "ok": False, "error": None, "legs": [], "rows": [], "status": None,
           "headline": None, "minutes": math.inf, "meters": 0.0, "expanded": 0, "lateness": 0.0}
    if not stops:
        r = agent.route(start, end, accessible, cautious, alg)
        if not r["found"]:
            res["error"] = r["reason"]
            return res
        res.update(legs=[r["path"]], rows=[{"place": end, "arrive": r["minutes"],
                                            "deadline": end_deadline if use_deadlines else None}],
                   expanded=r["expanded"], minutes=r["minutes"])
        if use_deadlines and r["minutes"] > end_deadline:
            res["lateness"] = r["minutes"] - end_deadline
    elif optimise:
        wanted = {s_: deadline_of.get(s_, math.inf) for s_ in stops}
        wanted[end] = end_deadline
        plan = agent.multi_stop(start, wanted, accessible, cautious, dwell, end=end, algorithm=alg)
        if not plan["found"]:
            res["error"] = plan["reason"]
            return res
        res.update(legs=plan["legs"], expanded=plan["expanded"], minutes=plan["plan"][-1]["arrive"],
                   lateness=plan["total_lateness"],
                   rows=[{"place": p["stop"], "arrive": p["arrive"], "risky": p["at_risk"],
                          "deadline": None if math.isinf(p["deadline"]) else p["deadline"]} for p in plan["plan"]])
        if plan["method"] == "csp":
            res["headline"] = "Quickest order that meets every deadline (CSP search)" if use_deadlines \
                else "Quickest order for your stops (CSP search)"
        else:
            res["status"] = "No order can meet every deadline. This is the order with the least total lateness."
            res["headline"] = "Best effort (simulated annealing)"
    else:
        seq, t = [start] + stops + [end], 0.0
        for a_, b_ in zip(seq, seq[1:]):
            r = agent.route(a_, b_, accessible, cautious, alg)
            if not r["found"]:
                res["error"] = f"No walkable route from {a_} to {b_}."
                return res
            res["legs"].append(r["path"])
            res["expanded"] += r["expanded"]
            t += path_minutes(graph, r["path"], cost_fn)
            dl = deadline_of.get(b_) if b_ != end else (end_deadline if use_deadlines else None)
            res["rows"].append({"place": b_, "arrive": round(t, 2), "deadline": dl})
            if dl is not None and t > dl:
                res["lateness"] += t - dl
            if b_ != end:
                t += dwell
        res["minutes"] = res["rows"][-1]["arrive"]
        res["headline"] = "Stops in the order you listed"
    res["meters"] = sum(graph.edge(a_, b_).distance_m for leg in res["legs"] for a_, b_ in zip(leg, leg[1:]))
    res["ok"] = True
    return res


def show_result(res):
    """Leg-by-leg directions for one algorithm's plan."""
    if res["headline"]:
        st.caption(res["headline"])
    if res["status"]:
        st.warning(res["status"])
    order = [start] + [r["place"] for r in res["rows"]]
    late_any = False
    for k, row in enumerate(res["rows"]):
        a_, b_ = order[k], order[k + 1]
        leg_min = path_minutes(graph, res["legs"][k], cost_fn)
        tag = ""
        if row["deadline"] is not None:
            if row["arrive"] > row["deadline"] + 1e-9:
                tag = f" · :red[late by {row['arrive'] - row['deadline']:.1f} min]"
                late_any = True
            else:
                tag = f" · :green[on time (by {row['deadline']:g})]"
                if row.get("risky"):
                    tag += " · :orange[risky if you walk slowly]"
        st.markdown(f"**{chr(65 + k)} → {chr(65 + k + 1)}**  {a_} to {b_} · {leg_min:.1f} min · "
                    f"arrive at {row['arrive']:.1f} min{tag}")
        for line in leg_steps(res["legs"][k], cost_fn):
            st.write("• " + line)
    if use_deadlines and not late_any and not res["status"]:
        st.success("Every deadline is met.")


# ------------------------------------------------------------------ results
results, ranked, error, highlight = {}, [], None, "All three"
with panel:
    st.button("Find routes", type="primary", use_container_width=True, key="find_routes",
              on_click=lambda: st.session_state.update(find=True))
    st.divider()
    if not st.session_state.get("find"):
        st.html('<div class="empty"><span class="ic">🗺️</span><br><b>Ready when you are</b><br>'
                    'Choose where you start, any stops and your destination, then press <b>Find routes</b>.<br>'
                    'You will see what A*, UCS and BFS each found, best first.</div>')
    elif start == end and not stops:
        error = "Start and destination are the same place."
        st.error(error)
    else:
        results = {alg: plan_trip(alg) for alg in ALG}
        failed = [r for r in results.values() if not r["ok"]]
        if failed:
            error = failed[0]["error"]
            st.error(error)
        else:
            # best first: fewest late minutes, then fastest, then fewest places examined
            order_pref = {"astar": 0, "ucs": 1, "bfs": 2}
            ranked = sorted(results.values(), key=lambda r: (round(r["lateness"], 2), round(r["minutes"], 2),
                                                             r["expanded"], order_pref[r["alg"]]))
            best = ranked[0]
            import pandas as pd
            table = []
            for k, r in enumerate(ranked, 1):
                gap = r["minutes"] - best["minutes"]
                table.append({"Rank": k, "Algorithm": r["name"], "Time (min)": round(r["minutes"], 2),
                              "Walking (m)": round(r["meters"]), "Places examined": r["expanded"],
                              "Compared with best": "best" if k == 1 else (f"+{gap:.2f} min slower" if gap > 0.004
                                                                      else "same time")})
            st.markdown("#### 🏁 Results, best first")
            medals = ["🥇", "🥈", "🥉"]
            worst = max(r["minutes"] for r in ranked) or 1
            cards = ""
            for k, r in enumerate(ranked):
                gap = r["minutes"] - best["minutes"]
                pill = '<span class="pill best">BEST</span>' if k == 0 else (
                    f'<span class="pill slow">+{gap:.2f} min slower</span>' if gap > 0.004 else '<span class="pill same">same time</span>')
                cards += (f'<div class="card {"first" if k == 0 else ""}" style="--c:{ALG[r["alg"]][1]};animation-delay:{k * .12}s">'
                          f'<div class="rank">{medals[k]}</div><div class="nm">{r["name"]}</div>'
                          f'<div class="big">{r["minutes"]:.1f}<small> min</small></div>'
                          f'<div class="sub">{r["meters"]:.0f} m walking<br>{r["expanded"]} places examined</div>{pill}'
                          f'<div class="bar"><i style="width:{100 * r["minutes"] / worst:.0f}%"></i></div></div>')
            st.html(f'<div class="podium">{cards}</div>')
            tied = [r["name"] for r in ranked if abs(r["minutes"] - best["minutes"]) <= 0.004
                    and abs(r["lateness"] - best["lateness"]) < 0.01]
            if len(tied) == 3:
                st.caption(f"All three found a {best['minutes']:.1f} min plan. {best['name']} examined the fewest places "
                           f"({best['expanded']}). BFS is not guaranteed to find the fastest route; here it happened to.")
            elif len(tied) > 1:
                st.caption(f"{' and '.join(tied)} found the fastest plan ({best['minutes']:.1f} min); "
                           f"{best['name']} examined the fewest places ({best['expanded']}). The other was slower.")
            else:
                st.caption(f"{best['name']} gave the best result ({best['minutes']:.1f} min); the others were slower.")
            highlight = st.radio("Show on the map", ["All three"] + [r["name"] for r in ranked], horizontal=True)
            for k, r in enumerate(ranked, 1):
                gap = r["minutes"] - best["minutes"]
                badge = "best" if k == 1 else (f"{gap:+.1f} min" if gap > 0.004 else "same time")
                with st.expander(f"**#{k} {r['name']}** · {r['minutes']:.1f} min · {r['meters']:.0f} m · "
                                 f"examined {r['expanded']} places · {badge}", expanded=(k == 1)):
                    show_result(r)
            with st.expander("What do these three do?"):
                st.markdown("- **BFS** takes the route with the fewest hops. It ignores how long each path is, so it can be slower.\n"
                            "- **Uniform-cost search** always expands the place with the least walking time so far, so its route is the fastest.\n"
                            "- **A\\*** does the same but also leans toward the goal using straight-line distance, so it "
                            "finds the same fastest route while examining fewer places.")

with mapcol:
    views = ["Satellite", "Schematic"] if SATELLITE_OK else ["Schematic"]
    view = st.radio("Map view", views, horizontal=True, label_visibility="collapsed")
    if not SATELLITE_OK:
        import sys
        st.error("**Satellite view is switched off** because a package is missing "
                 f"({SATELLITE_ERROR}). Stop the app (Ctrl+C), run the line below, then start the app again:")
        st.code(f'"{sys.executable}" -m pip install folium', language="bash")
    routes_to_draw, marker_source = [], None
    if ranked:
        if highlight == "All three":
            drawn = sorted(ranked, key=lambda r: -ALG[r["alg"]][2])       # widest (BFS) first, thinnest (A*) on top
            marker_source = ranked[0]
        else:
            drawn = [r for r in ranked if r["name"] == highlight]
            marker_source = drawn[0]
        routes_to_draw = [(r["legs"], ALG[r["alg"]][1], ALG[r["alg"]][2] + (3 if highlight != "All three" else 0))
                          for r in drawn]
    route_nodes = []                                                  # (node, label, colour): where you stand
    if marker_source:
        ends = [marker_source["legs"][0][0]] + [leg[-1] for leg in marker_source["legs"]]
        for k, node in enumerate(ends):
            colour = "#188038" if k == 0 else "#d93025" if k == len(ends) - 1 else "#3c4043"
            route_nodes.append((node, chr(65 + k), colour))
    legend = "".join(f'<span style="color:{c}">●</span> {n}' for n, c, _ in ALG.values() for n in [n.split(" (")[0]] if True)
    if view == "Satellite":
        if route_nodes:
            mk = [(GEO[n], lab, col) for n, lab, col in route_nodes]
        else:
            ents = graph.resolve(start)
            mk = [((sum(GEO[e][0] for e in ents) / len(ents), sum(GEO[e][1] for e in ents) / len(ents)), "A", "#188038")]
        _page = draw_satellite(routes_to_draw, mk).get_root().render()
        if hasattr(st, "iframe"):                                # newer Streamlit
            st.iframe(_page, height=700)
        else:
            components.html(_page, height=700)
        st.caption("Satellite imagery loads from the internet. Orange dashed: closed path. Yellow dotted: stairs only. "
                   "Teal dots: building entrances. Use the layers button for a street map.")
    else:
        mk = [(graph.nodes[n], lab, col) for n, lab, col in route_nodes] or [(where(start), "A", "#188038")]
        st.pyplot(draw_map(routes_to_draw, mk))
        st.caption("Orange dashed: closed path. Yellow dotted: stairs only. Small teal dots: building entrances.")
    if routes_to_draw:
        st.html('<div class="legend">' + legend + '</div><small style="color:#aab4e8">Where routes overlap you see the '
                    'colours side by side; the thinnest line (A*) sits on top.</small>')
    for p in graph.check():
        st.warning("Map check: " + p)
