"""Demo figures for README and docs/index.html from results/demo/*.csv (written by kinmp_demo_dump).

The planned joint paths are only re-posed in MuJoCo's Python bindings for the picture (the C++ FK matches
MuJoCo to 1e-15 m on this model); nothing is planned or simulated again here.
Run: MUJOCO_GL=egl uv run --with mujoco --with matplotlib --with pandas --with pillow python scripts/make_demo_figures.py
"""
import os
import pathlib

os.environ.setdefault("MUJOCO_GL", "egl")
import matplotlib  # noqa: E402
matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import mujoco  # noqa: E402
import numpy as np  # noqa: E402
import pandas as pd  # noqa: E402
from PIL import Image, ImageDraw  # noqa: E402

R = pathlib.Path(__file__).resolve().parents[1]
OUT = R / "docs" / "assets"
OUT.mkdir(parents=True, exist_ok=True)
plt.rcParams.update({"font.family": "DejaVu Sans", "font.size": 10, "axes.spines.top": False, "axes.spines.right": False,
                     "axes.edgecolor": "#737373", "axes.labelcolor": "#3d3d3d", "xtick.color": "#737373", "ytick.color": "#737373"})
BOXES = [((0.45, -0.10, 0.0), (0.60, 0.10, 0.70)), ((0.30, -0.60, 0.80), (0.70, 0.60, 0.85))]  # scene.hpp (floor omitted)

# ---- Figure 1: IK over the 1,000 targets per chain
ik = pd.read_csv(R / "results" / "demo" / "ik_targets.csv")
fig, ax = plt.subplots(1, 2, figsize=(10, 3.4), dpi=150)
for k, (name, g) in enumerate(ik.groupby("chain", sort=False)):
    col = ["#0a0a0a", "#a3a3a3"][k]
    att = g[g.ok == 1].attempts.value_counts().reindex(range(1, 11), fill_value=0)
    ax[0].bar(np.arange(1, 11) + (k - 0.5) * 0.38, att.values, width=0.38, color=col,
              label=f"{name}: {int(g.ok.sum())} / 1000 solved")
    it = np.sort(g[g.ok == 1].iters.values)
    ax[1].plot(it, np.arange(1, len(it) + 1) / len(g), color=col, lw=1.4, label=name)
ax[0].set_xticks(range(1, 11)); ax[0].set_xlabel("attempts needed (1 = the given seed, then random restarts)")
ax[0].set_ylabel("targets"); ax[0].legend(frameon=False, fontsize=8.5)
ax[0].set_title("IK on 1,000 reachable targets per chain", loc="left", fontsize=10)
ax[1].set_xscale("log"); ax[1].set_xlabel("total DLS iterations (log)"); ax[1].set_ylabel("share of targets solved")
ax[1].set_ylim(0, 1); ax[1].legend(frameon=False, fontsize=8.5, loc="lower right")
ax[1].set_title("Convergence: share solved within n iterations", loc="left", fontsize=10)
fig.tight_layout(); fig.savefig(OUT / "demo-ik.png", facecolor="white"); plt.close(fig)

# ---- Figure 2: one blocked query, raw vs shortcut vs shortcut + CHOMP
p = pd.read_csv(R / "results" / "demo" / "paths.csv")
STY = {"raw": ("#c8c8c8", 1.4, "RRT-Connect raw"), "shortcut": ("#737373", 1.4, "shortcut"),
       "shortcut_chomp": ("#0a0a0a", 1.8, "shortcut + CHOMP")}
def length(g):
    q = g[[f"q{j}" for j in range(1, 8)]].values
    return np.linalg.norm(np.diff(q, axis=0), axis=1).sum()
fig, ax = plt.subplots(1, 3, figsize=(11, 3.5), dpi=150)
for v, g in p.groupby("variant", sort=False):
    c, lw, lab = STY[v]
    for a, j in zip(ax[:2], (1, 2)):
        a.plot(g.i, g[f"q{j}"], color=c, lw=lw, label=lab)
    ax[2].plot(g.x, g.y, color=c, lw=lw, label=f"{lab}")
for a, j in zip(ax[:2], (1, 2)):
    a.set_xlabel("waypoint (resampled to 50)"); a.set_ylabel(f"joint {j} (rad)")
    a.set_title(f"Joint {j} along the path", loc="left", fontsize=10)
ax[0].legend(frameon=False, fontsize=8)
(x0, y0, _), (x1, y1, _) = BOXES[0]
ax[2].add_patch(plt.Rectangle((x0, y0), x1 - x0, y1 - y0, color="#e5e5e5", zorder=0))
ax[2].text((x0 + x1) / 2, (y0 + y1) / 2, "pillar", ha="center", va="center", fontsize=8, color="#555555")
ax[2].set_aspect("equal"); ax[2].set_xlabel("x (m)"); ax[2].set_ylabel("y (m)")
ax[2].set_title("Tool path seen from above", loc="left", fontsize=10)
fig.tight_layout(); fig.savefig(OUT / "demo-paths.png", facecolor="white"); plt.close(fig)
print({v: round(length(g), 3) for v, g in p.groupby("variant", sort=False)}, "(resampled lengths)")

# ---- Figure 3: the shortcut + CHOMP path re-posed in MuJoCo, rendered headless
spec = mujoco.MjSpec.from_file(str(R / "assets" / "franka_emika_panda" / "panda_nohand.xml"))
spec.add_texture(name="sky", type=mujoco.mjtTexture.mjTEXTURE_SKYBOX, builtin=mujoco.mjtBuiltin.mjBUILTIN_GRADIENT,
                 rgb1=[1, 1, 1], rgb2=[0.92, 0.92, 0.92], width=64, height=64)
spec.visual.headlight.ambient = [0.45, 0.45, 0.45]
spec.visual.headlight.diffuse = [0.5, 0.5, 0.5]
w = spec.worldbody
w.add_geom(type=mujoco.mjtGeom.mjGEOM_PLANE, size=[1.5, 1.5, 0.05], rgba=[0.9, 0.9, 0.9, 1])
for lo, hi in BOXES:
    lo, hi = np.array(lo), np.array(hi)
    w.add_geom(type=mujoco.mjtGeom.mjGEOM_BOX, pos=(lo + hi) / 2, size=(hi - lo) / 2, rgba=[0.35, 0.35, 0.38, 0.55],
               contype=0, conaffinity=0)
w.add_light(pos=[0.8, -0.6, 2.2], dir=[-0.35, 0.25, -1], diffuse=[0.8, 0.8, 0.8], castshadow=True)
m = spec.compile(); d = mujoco.MjData(m)
ren = mujoco.Renderer(m, 360, 480)
cam = mujoco.MjvCamera(); cam.lookat[:] = [0.35, 0.0, 0.45]; cam.distance, cam.azimuth, cam.elevation = 2.0, 210, -22
g = p[p.variant == "shortcut_chomp"]
Q = g[[f"q{j}" for j in range(1, 8)]].values
dense = np.concatenate([np.linspace(Q[i], Q[i + 1], 3, endpoint=False) for i in range(len(Q) - 1)] + [Q[-1:]])
trail, frames = [], []
for k, q in enumerate(dense):
    d.qpos[:7] = q
    mujoco.mj_kinematics(m, d)
    trail.append(d.site("attachment_site").xpos.copy())
    ren.update_scene(d, cam)
    for pt in trail[::2]:
        if ren.scene.ngeom >= ren.scene.maxgeom: break
        mujoco.mjv_initGeom(ren.scene.geoms[ren.scene.ngeom], mujoco.mjtGeom.mjGEOM_SPHERE, [0.008, 0, 0], pt,
                            np.eye(3).flatten(), np.array([0.05, 0.05, 0.05, 1], np.float32))
        ren.scene.ngeom += 1
    img = Image.fromarray(ren.render())
    ImageDraw.Draw(img).rectangle([0, 0, 480, 22], fill=(255, 255, 255))
    ImageDraw.Draw(img).text((8, 5), f"shortcut + CHOMP path, waypoint {k * 50 // len(dense) + 1} / 50   (pillar + shelf)",
                             fill=(10, 10, 10))
    frames.append(img)
frames += [frames[-1]] * 12
pal = [f.quantize(colors=64, method=Image.Quantize.MEDIANCUT) for f in frames]
pal[0].save(OUT / "demo-panda-path.gif", save_all=True, append_images=pal[1:], duration=60, loop=0, optimize=True)
picks = [frames[i] for i in np.linspace(0, len(dense) - 1, 4).astype(int)]
strip = Image.new("RGB", (480 * 4, 360), "white")
for i, f in enumerate(picks):
    strip.paste(f, (480 * i, 0))
strip.save(OUT / "demo-panda-strip.png", optimize=True)
print(len(frames), "frames")
