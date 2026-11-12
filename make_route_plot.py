# Draws the BFS, UCS and A* routes for two trips on the campus graph, side by side.
# Put this file next to campus_navigator.py and run:  python make_route_plot.py
import math
import matplotlib.pyplot as plt
import campus_navigator as cn

agent = cn.NavigatorAgent()
STYLE = {"bfs": ("BFS", "#ff4081"), "ucs": ("UCS", "#00a152"), "astar": ("A*", "#1a73e8")}
SHIFT = {"bfs": -5, "ucs": 0, "astar": 5}     # metres: each line is moved sideways so overlapping routes stay visible

def shifted(points, d):
    """Move a line sideways by d metres (perpendicular to the direction of travel)."""
    out = []
    for i, (x, y) in enumerate(points):
        a = points[max(i - 1, 0)]
        b = points[min(i + 1, len(points) - 1)]
        dx, dy = b[0] - a[0], b[1] - a[1]
        length = math.hypot(dx, dy) or 1
        out.append((x - dy / length * d, y + dx / length * d))
    return out

trips = [("AC2", "HostelA"), ("AC3", "Library")]
for start, goal in trips:
    fig, ax = plt.subplots(figsize=(9, 8.6))
    for e in cn.EDGES:                                       # grey campus paths
        (x1, y1), (x2, y2) = cn.NODES[e[0]], cn.NODES[e[1]]
        ax.plot([x1, x2], [y1, y2], color="#d1d5db", linewidth=1, zorder=1)
    labels = []
    for algo in ("bfs", "ucs", "astar"):
        r = agent.route(start, goal, algorithm=algo)
        points = [cn.NODES[n] for n in r["path"]]
        name, colour = STYLE[algo]
        xs, ys = zip(*shifted(points, SHIFT[algo]))
        ax.plot(xs, ys, color=colour, linewidth=2.6, zorder=3)
        labels.append(plt.Line2D([0], [0], color=colour, linewidth=3,
                                 label=f"{name}: {r['minutes']:.1f} min, {r['meters']:.0f} m, {r['expanded']} places examined"))
    # pointers (dots) and names for every node
    entrances = [n for names in cn.BUILDINGS.values() for n in names]
    for name, (x, y) in cn.NODES.items():
        if name.startswith("Gate"):
            colour, marker, size = "#d62728", "s", 60          # gates: red squares
        elif name in entrances:
            colour, marker, size = "#1f77b4", "o", 28          # building entrances: blue dots
        elif name.startswith("J"):
            colour, marker, size = "#6b7280", "o", 14          # road junctions: grey dots
        else:
            colour, marker, size = "#7b1fa2", "D", 24          # other places: purple diamonds
        ax.scatter(x, y, s=size, color=colour, marker=marker, zorder=4)
        ax.annotate(name, (x, y), xytext=(3, 3), textcoords="offset points", fontsize=6.5, color="#111827", zorder=6)
    ax.scatter(*cn.NODES[agent.route(start, goal)["path"][0]], s=110, color="green", edgecolor="white", zorder=5)
    ax.scatter(*cn.NODES[agent.route(start, goal)["path"][-1]], s=110, color="red", edgecolor="white", zorder=5)
    ax.set_title(f"{start} to {goal}", fontweight="bold")
    kinds = [plt.Line2D([0], [0], marker="s", color="w", markerfacecolor="#d62728", markersize=8, label="Gate"),
             plt.Line2D([0], [0], marker="o", color="w", markerfacecolor="#1f77b4", markersize=7, label="Building entrance"),
             plt.Line2D([0], [0], marker="o", color="w", markerfacecolor="#6b7280", markersize=5, label="Road junction (J..)"),
             plt.Line2D([0], [0], marker="D", color="w", markerfacecolor="#7b1fa2", markersize=6, label="Other place")]
    ax.legend(handles=labels + kinds, loc="upper right", fontsize=8, frameon=True)
    ax.set_aspect("equal")
    ax.set_xticks([]); ax.set_yticks([])
    plt.title(f"Routes found by BFS, UCS and A*: {start} to {goal}", fontweight="bold")
    plt.tight_layout()
    plt.savefig(f"routes_{start}_{goal}.png", dpi=220)
    plt.close()
    print("Saved", f"routes_{start}_{goal}.png")
