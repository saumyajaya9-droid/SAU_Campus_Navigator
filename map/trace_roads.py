"""Step 1: clean road mask from the satellite screenshot (Google's grey road lines), skeleton, pixel<->GPS map."""
import cv2, json, math, numpy as np, sys
from skimage.morphology import skeletonize
sys.path.insert(0, "/home/claude")
from campus_geo import GEO
S = "/tmp/claude-0/-home-claude/87d7c525-6fa4-5412-b0ec-77f6fe7d9c58/scratchpad/"
im = cv2.imread(S + "img/sat.png")
k = 111320 * math.cos(math.radians(28.485))
def metres(n): return np.array([GEO[n][1] * k, GEO[n][0] * 110540])
# similarity transform (rotation+scale+shift) from Gate1 -> pixel (880,1190) and Gate3 -> pixel (130,185); image y is down
g1, g3 = metres("Gate1"), metres("Gate3"); p1, p3 = np.array([880., 1190.]), np.array([130., 185.])
zg = complex(*(g3 - g1)); zp = complex(p3[0] - p1[0], -(p3[1] - p1[1]))        # flip y so both are y-up
q = zp / zg
def to_pix(lat, lon):
    m = np.array([lon * k, lat * 110540]) - g1; z = complex(*m) * q
    return np.array([p1[0] + z.real, p1[1] - z.imag])
def to_geo(x, y):
    z = complex(x - p1[0], -(y - p1[1])) / q; m = np.array([z.real, z.imag]) + g1
    return (float(m[1] / 110540), float(m[0] / k))
print("scale px per metre %.3f" % abs(q))
v_ = im.mean(2); sat_ = im.max(2).astype(int) - im.min(2).astype(int)
road = ((sat_ < 6) & (v_ > 124) & (v_ < 154)).astype(np.uint8)
dots = ((np.abs(im.astype(int)[:, :, 0] - im[:, :, 1]) <= 4) & (im.max(2) > 100) & (im.max(2) < 150)).astype(np.uint8)
m = cv2.morphologyEx(road, cv2.MORPH_CLOSE, cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (13, 13)))
n, lab, st, _ = cv2.connectedComponentsWithStats(m)
keep = np.zeros_like(m)
for i in range(1, n):
    if st[i, cv2.CC_STAT_AREA] > 600: keep[lab == i] = 1
sk = skeletonize(keep > 0).astype(np.uint8)
cv2.imwrite(S + "out/road_mask.png", keep * 255); cv2.imwrite(S + "out/road_skel.png", sk * 255)
vis = im.copy(); vis[sk > 0] = (0, 255, 255)
cv2.imwrite("/home/claude/skel_check.png", vis)
print("skeleton px", int(sk.sum()))

dt = cv2.distanceTransform((1 - sk).astype(np.uint8), cv2.DIST_L2, 3)
for n in sorted(GEO):
    if n.startswith("J") or n.startswith("Gate"):
        x, y = to_pix(*GEO[n]); print(n, "dist to road px %.0f (%.0f m)" % (dt[int(y), int(x)], dt[int(y), int(x)] / abs(q)))

# systematic offset check: vector from each junction's projected pixel to nearest skeleton pixel
ys, xs = np.nonzero(sk); from scipy.spatial import cKDTree; tree = cKDTree(np.c_[xs, ys])
vec = []
for n in sorted(GEO):
    if n.startswith("J") or n.startswith("Gate"):
        x, y = to_pix(*GEO[n]); d, i = tree.query([x, y]); vec.append((n, round(xs[i] - x), round(ys[i] - y)))
print(vec); a = np.array([[v[1], v[2]] for v in vec]); print("median shift px", np.median(a, 0))
