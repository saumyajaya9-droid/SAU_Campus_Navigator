# Figures 1, 4, 5 and 6 of the report.
# Put this file in the same folder as campus_navigator.py and run:  python make_report_figures.py
import heapq
import matplotlib.pyplot as plt
import campus_navigator as cn

agent = cn.NavigatorAgent()
g = agent.graph                      # the campus graph
BLUE, ORANGE, RED, GREEN = "#1f77b4", "#ff7f0e", "#d62728", "#2ca02c"

# =============== Figure 1: campus graph with gates and building entrances ===============
plt.figure(figsize=(6, 5.4))
for edge in cn.EDGES:                                    # draw every path
    x1, y1 = cn.NODES[edge[0]]
    x2, y2 = cn.NODES[edge[1]]
    plt.plot([x1, x2], [y1, y2], color="#9ca3af", linewidth=1)

entrances = [n for names in cn.BUILDINGS.values() for n in names]
for name, (x, y) in cn.NODES.items():
    if name in entrances:
        plt.scatter(x, y, s=26, color=BLUE, zorder=3)                       # building entrance
    elif name.startswith("Gate"):
        plt.scatter(x, y, s=70, marker="s", color=RED, zorder=4)            # gate
        plt.annotate(name, (x, y), xytext=(4, 4), textcoords="offset points", fontsize=8)
    else:
        plt.scatter(x, y, s=6, color="#6b7280", zorder=2)                   # junction / other place

for building, names in cn.BUILDINGS.items():             # label each building at the middle of its entrances
    mx = sum(cn.NODES[n][0] for n in names) / len(names)
    my = sum(cn.NODES[n][1] for n in names) / len(names)
    plt.annotate(building, (mx, my), xytext=(0, 10), textcoords="offset points", fontsize=7.5,
                 ha="center", fontweight="bold", bbox=dict(boxstyle="round", fc="white", ec="none", alpha=0.85))
plt.gca().set_aspect("equal")
plt.title("Campus graph: 80 nodes, 105 paths", fontweight="bold")
plt.xlabel("metres east of Gate 1")
plt.ylabel("metres north of Gate 1")
plt.tight_layout()
plt.savefig("fig1_campus_graph.png", dpi=200)
plt.close()

# =============== Figure 4: places examined by UCS and A* (same trip) ===============
def search_trace(start, goal, use_heuristic):
    """Same idea as best_first() in campus_navigator.py, but it also returns the places examined."""
    goals = set(g.resolve(goal))
    h = (lambda n: g.heuristic_to_any(n, goals)) if use_heuristic else (lambda n: 0)
    frontier = [(h(s), 0, s) for s in g.resolve(start)]
    heapq.heapify(frontier)
    best_cost = {s: 0 for s in g.resolve(start)}
    came_from = {s: None for s in g.resolve(start)}
    examined = []
    while frontier:
        _, cost, node = heapq.heappop(frontier)
        if cost > best_cost.get(node, 1e18):
            continue
        examined.append(node)
        if node in goals:                                   # rebuild the route
            route = []
            while node is not None:
                route.append(node)
                node = came_from[node]
            return examined, route[::-1]
        for e in g.neighbors(node):
            new_cost = cost + cn.typical_cost(e)
            if new_cost < best_cost.get(e.to, 1e18):
                best_cost[e.to] = new_cost
                came_from[e.to] = node
                heapq.heappush(frontier, (new_cost + h(e.to), new_cost, e.to))

def bfs_trace(start, goal):
    """Breadth-first search: same idea as bfs() in campus_navigator.py, but it also returns the places examined."""
    from collections import deque
    goals = set(g.resolve(goal))
    queue = deque(g.resolve(start))
    came_from = {s: None for s in g.resolve(start)}
    examined = []
    while queue:
        node = queue.popleft()
        examined.append(node)
        if node in goals:
            route = []
            while node is not None:
                route.append(node)
                node = came_from[node]
            return examined, route[::-1]
        for e in g.neighbors(node):
            if e.to not in came_from:
                came_from[e.to] = node
                queue.append(e.to)

fig, axes = plt.subplots(1, 3, figsize=(12.5, 4.6))
for ax, use_h, title, color in ((axes[0], None, "BFS", RED), (axes[1], False, "UCS", ORANGE), (axes[2], True, "A*", BLUE)):
    examined, route = bfs_trace("AC2", "HostelA") if use_h is None else search_trace("AC2", "HostelA", use_h)
    for edge in cn.EDGES:
        (x1, y1), (x2, y2) = cn.NODES[edge[0]], cn.NODES[edge[1]]
        ax.plot([x1, x2], [y1, y2], color="#e5e7eb", linewidth=1)
    ax.scatter([cn.NODES[n][0] for n in examined], [cn.NODES[n][1] for n in examined], s=26, color=color, alpha=0.8, zorder=3)
    ax.plot([cn.NODES[n][0] for n in route], [cn.NODES[n][1] for n in route], color="black", linewidth=2.2, zorder=4)
    ax.scatter(*cn.NODES[route[0]], s=90, marker="s", color="green", zorder=5)                    # start
    ax.scatter(*cn.NODES[route[-1]], s=120, marker="*", color="gold", edgecolor="black", zorder=5)  # goal
    minutes = sum(g.edge(a, b).distance_m for a, b in zip(route, route[1:]))
    ax.set_title(f"{title}: {len(examined)} places examined\nroute length {minutes:.0f} m", fontweight="bold", fontsize=10)
    ax.set_aspect("equal")
    ax.set_xticks([])
    ax.set_yticks([])
fig.suptitle("AC2 to HostelA: dots = places examined, black line = route found")
plt.tight_layout()
plt.savefig("fig4_search_comparison.png", dpi=200)
plt.close()

# =============== Figure 5: mean places examined and mean time over all 182 pairs ===============
buildings = list(cn.BUILDINGS)
places = {"astar": [], "ucs": [], "bfs": []}
minutes = {"astar": [], "ucs": [], "bfs": []}
for a in buildings:
    for b in buildings:
        if a != b:
            for algo in places:
                r = agent.route(a, b, algorithm=algo)
                places[algo].append(r["expanded"])
                minutes[algo].append(r["minutes"])
mean = lambda v: sum(v) / len(v)

fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(8, 3.4))
for ax, data, title in ((ax1, places, "Places examined (mean)"), (ax2, minutes, "Walking time of route (mean, minutes)")):
    values = [mean(data[k]) for k in ("astar", "ucs", "bfs")]
    bars = ax.bar(["A*", "UCS", "BFS"], values, color=[BLUE, ORANGE, RED], width=0.6)
    ax.bar_label(bars, fmt="%.1f", padding=2)               # number on top of each bar
    ax.set_ylim(0, max(values) * 1.18)
    ax.set_title(title, fontweight="bold")
    ax.spines[["top", "right"]].set_visible(False)
plt.tight_layout()
plt.savefig("fig5_bars.png", dpi=200)
plt.close()

# =============== Figure 6: multi-stop plan against deadlines ===============
result = agent.multi_stop("Gate1", {"Library": 12, "Admin": 20, "AC1": 26, "HostelA": 45})
plan = result["plan"]                                        # list of stops in visiting order
colors = [BLUE, GREEN, "#9467bd", ORANGE]
plt.figure(figsize=(6.6, 3))
previous = 0
for i, stop in enumerate(plan):
    y = len(plan) - 1 - i                                    # first stop at the top
    plt.barh(y, stop["arrive"] - previous, left=previous, color=colors[i], height=0.5)      # walking leg
    plt.plot([stop["deadline"]] * 2, [y - 0.38, y + 0.38], color=RED, linewidth=2.2)         # deadline mark
    plt.text(stop["arrive"] + 0.4, y, f"arrive {stop['arrive']:.1f}", va="center", fontsize=8.5)
    previous = stop["arrive"]
plt.yticks(range(len(plan)), [s["stop"] for s in plan][::-1])
plt.xlabel("minutes from Gate 1")
plt.xlim(0, 50)
plt.title("CSP plan: every stop is reached before its deadline (red = deadline)", fontweight="bold", fontsize=10)
plt.gca().spines[["top", "right"]].set_visible(False)
plt.tight_layout()
plt.savefig("fig6_timeline.png", dpi=200)
plt.close()

print("Done. Saved fig1_campus_graph.png, fig4_search_comparison.png, fig5_bars.png, fig6_timeline.png")
