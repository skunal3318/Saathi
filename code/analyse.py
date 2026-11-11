"""Analyse SAATHI stair runs (C1, C2) and produce the proposal figure + metrics."""
import sys, json
sys.path.insert(0, sys.argv[1])
from saathi_zupt import *
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib import font_manager

DATA, OUT = sys.argv[2], sys.argv[3]
TRUTH = 16 * 6.5 * 0.0254  # 16 risers x 6.5 in
LAND = 8 * 6.5 * 0.0254    # mid-flight landing after 8 risers
BLUE, ORANGE = "#2a78d6", "#eb6834"
INK, INK2, GRID = "#222222", "#5f5e5a", "#e2e1dc"


def segments(mask, t, min_s):
    out, k = [], 0
    while k < len(mask):
        if mask[k]:
            j = k
            while j < len(mask) and mask[j]:
                j += 1
            if t[j - 1] - t[k] >= min_s:
                out.append((k, j))
            k = j
        else:
            k += 1
    return out


def analyse(run, gamma=3e4):
    t, f, w, b = load_run(f"{DATA}/{run}")
    ka, ks = find_start(t, w, f)
    t, f, w = t[ka:], f[ka:], w[ka:]
    z, _ = glrt_detector(f, w, gamma=gamma)
    pos, _ = run_ins(t, f, w, z)
    long_ = segments(z, t, 3.0)
    start = long_[0]
    top = max(long_[1:], key=lambda s: np.median(pos[s[0]:s[1], 2]))
    land_up = [s for s in long_ if start[1] <= s[0] < top[0]]
    # arrival = last foot-flat period >= 0.8 s; later, shorter stances are the phone being unstrapped
    arrive = segments(z, t, 0.8)[-1]
    p0 = pos[start[0]:start[1]].mean(0)
    p1 = pos[arrive[0]:arrive[1]].mean(0)
    stance = [s for s in segments(z, t, 0.0) if s[0] <= arrive[0]]
    mids = [(a + c) // 2 for a, c in stance]
    bt, bp = b.seconds_elapsed.to_numpy(), b.pressure.to_numpy()
    p_ref = np.median(bp[(bt >= t[start[0]]) & (bt <= t[start[1] - 1])])
    balt = 44330.0 * (1 - (bp / p_ref) ** (1 / 5.255))
    in_top = (bt >= t[top[0]]) & (bt <= t[top[1] - 1])
    m = dict(
        run=run.upper(),
        ins_top_m=float(np.median(pos[top[0]:top[1], 2])),
        ins_landing_m=float(np.median(pos[land_up[0][0]:land_up[0][1], 2])) if land_up else None,
        baro_top_m=float(np.median(balt[in_top])),
        baro_return_m=float(balt[bt <= t[arrive[1] - 1]][-1]),
        closing_err_3d_m=float(np.linalg.norm(p1 - p0)),
        closing_err_horiz_m=float(np.linalg.norm((p1 - p0)[:2])),
        closing_err_vert_m=float((p1 - p0)[2]),
        path_m=float(np.sum(np.linalg.norm(np.diff(pos[mids], axis=0), axis=1))),
        arrival_t_s=float(t[arrive[0]] - t[start[1]]),
        strides=count_steps(z),
    )
    return m, dict(arr=arrive[1], t=t - t[start[1]], pos=pos, bt=bt - t[start[1]], balt=balt, p0=p0, p1=p1, start=start)


res, tr = {}, {}
for r in ["c1", "c2"]:
    res[r], tr[r] = analyse(r)
res["truth"] = dict(top_m=TRUTH, landing_m=LAND, note="nominal: 16 risers x 6.5 in")
res["sensitivity_ins_top_m"] = {f"gamma={g:.0e}": {r: round(analyse(r, g)[0]["ins_top_m"], 2) for r in ["c1", "c2"]} for g in [1e4, 3e4, 1e5]}
json.dump(res, open(f"{OUT}/metrics.json", "w"), indent=1)
print(json.dumps(res, indent=1))

# ---------------------------------------------------------------- figure
for fam in ["Carlito", "Calibri"]:
    if any(fam in f.name for f in font_manager.fontManager.ttflist):
        plt.rcParams["font.family"] = fam
plt.rcParams.update({"font.size": 9, "axes.edgecolor": INK2, "axes.labelcolor": INK2,
                     "xtick.color": INK2, "ytick.color": INK2, "axes.linewidth": 0.8})
fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(7.2, 2.7), gridspec_kw={"width_ratios": [1, 1.55]})

# (a) top-down view, C2
d = tr["c2"]
k0 = d["start"][0]
ka = d["arr"]
ax1.plot(d["pos"][k0:ka, 0], d["pos"][k0:ka, 1], color=BLUE, lw=1.6, solid_capstyle="round")
ax1.plot(*d["p0"][:2], "o", ms=7, color=INK, mec="white", mew=1.5, zorder=5)
ax1.plot(*d["p1"][:2], "s", ms=7, color=BLUE, mec="white", mew=1.5, zorder=5)
ax1.annotate("start", d["p0"][:2], xytext=(-8, 6), textcoords="offset points", ha="right", color=INK, fontsize=8)
ax1.annotate("end", d["p1"][:2], xytext=(8, -3), textcoords="offset points", ha="left", va="center", color=INK, fontsize=8)
ax1.set_aspect("equal", adjustable="datalim")
ax1.set_xlabel("x (m)"); ax1.set_ylabel("y (m)")
ax1.set_title("(a) Run C2, top view", fontsize=9, color=INK, loc="left")
ax1.grid(color=GRID, lw=0.6); ax1.set_axisbelow(True)
for s in ["top", "right"]:
    ax1.spines[s].set_visible(False)

# (b) height vs time, C2: foot INS vs barometer vs truth
ax2.axhline(TRUTH, color=INK2, lw=1, ls=(0, (4, 3)))
ax2.text(32, TRUTH - 0.1, "1st floor\n(measured 2.64 m)", ha="center", va="top", fontsize=7.5, color=INK2, linespacing=1.1)
ax2.axhline(0, color=GRID, lw=0.8)
m = (d["t"] >= -3) & (np.arange(len(d["t"])) < d["arr"])
ax2.plot(d["t"][m], d["pos"][m, 2], color=BLUE, lw=1.6, label="Foot INS (ZUPT)")
mb = (d["bt"] >= -3) & (d["bt"] <= d["t"][d["arr"] - 1])
ax2.plot(d["bt"][mb], d["balt"][mb], color=ORANGE, lw=2, marker="o", ms=3.5, mec="white", mew=0.8,
         label="Barometer")
ax2.set_xlabel("time from start of walk (s)"); ax2.set_ylabel("height (m)")
ax2.set_title("(b) Run C2, height: up one floor and back", fontsize=9, color=INK, loc="left")
ax2.grid(axis="y", color=GRID, lw=0.6); ax2.set_axisbelow(True)
ax2.legend(frameon=False, loc="upper left", fontsize=8)
for s in ["top", "right"]:
    ax2.spines[s].set_visible(False)
fig.tight_layout()
fig.savefig(f"{OUT}/saathi_prelim.png", dpi=220)
