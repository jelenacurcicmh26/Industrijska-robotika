"""Render the GIFs and stills in docs/media.

    python tools/record_media.py

Offscreen MuJoCo rendering, so no viewer window opens. The camera matches the
one the interactive scripts set up, so the clips look like what you see when
you run them yourself.
"""

import os
import sys

import mujoco
import numpy as np
from PIL import Image, ImageDraw, ImageFont

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import _sim                          # noqa: E402
import step3_computed_torque as s3   # noqa: E402

MEDIA = os.path.join("docs", "media")

W, H = 330, 300          # per-arm frame size
EVERY = 25               # record one frame every N physics steps -> 20 fps
FPS = 20

CAM = dict(azimuth=150, elevation=-14, distance=1.55, lookat=[0.05, 0.0, 0.40])


FLOOR_RGBA = (0.34, 0.37, 0.42, 1.0)   # mid grey-blue: white arm reads against it


def simplify_scene(model):
    """Flatten the background so the GIFs compress.

    The gradient skybox and the checkered, reflective floor look good in the
    viewer and are ruinous in a 64-colour palette: they dither into noise that
    defeats GIF frame-differencing. Flattening them cuts the files by ~15x.
    The floor keeps a mid tone, otherwise a white arm sits on a white floor.
    """
    for mat_id in range(model.nmat):
        model.mat_reflectance[mat_id] = 0.0

    floor_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, "floor")
    if floor_id >= 0:
        model.geom_matid[floor_id] = -1        # drop the checker material
        model.geom_rgba[floor_id] = FLOOR_RGBA
    return model


def _font(size):
    for p in (r"C:\Windows\Fonts\segoeui.ttf", r"C:\Windows\Fonts\arial.ttf"):
        if os.path.exists(p):
            return ImageFont.truetype(p, size)
    return ImageFont.load_default()


def make_camera(**overrides):
    spec = dict(CAM, **overrides)
    cam = mujoco.MjvCamera()
    cam.type = mujoco.mjtCamera.mjCAMERA_FREE
    cam.azimuth = spec["azimuth"]
    cam.elevation = spec["elevation"]
    cam.distance = spec["distance"]
    cam.lookat[:] = spec["lookat"]
    return cam


def renderer_for(sim, w=W, h=H):
    # The offscreen framebuffer defaults to 640x480. Rather than edit the
    # vendored model XML, grow it in the loaded model before the renderer
    # allocates its context.
    sim.model.vis.global_.offwidth = max(sim.model.vis.global_.offwidth, w)
    sim.model.vis.global_.offheight = max(sim.model.vis.global_.offheight, h)
    simplify_scene(sim.model)
    r = mujoco.Renderer(sim.model, height=h, width=w)
    r.scene.flags[mujoco.mjtRndFlag.mjRND_SKYBOX] = 0
    r.scene.flags[mujoco.mjtRndFlag.mjRND_REFLECTION] = 0
    return r


def grab(renderer, sim, cam):
    renderer.update_scene(sim.data, cam)
    return renderer.render()


def caption(img_arr, lines, sub_color=(206, 206, 200)):
    """Burn a caption into the top-left, over a scrim so it stays readable
    whatever the arm happens to be in front of."""
    img = Image.fromarray(img_arr).convert("RGB")
    height = sum(21 if big else 17 for _, big in lines) + 14

    scrim = Image.new("RGBA", img.size, (0, 0, 0, 0))
    ImageDraw.Draw(scrim).rectangle([0, 0, img.size[0], height], fill=(12, 12, 14, 190))
    img = Image.alpha_composite(img.convert("RGBA"), scrim).convert("RGB")

    d = ImageDraw.Draw(img)
    y = 7
    for text, big in lines:
        d.text((11, y), text, font=_font(15 if big else 12),
               fill=(255, 255, 255) if big else sub_color)
        y += 21 if big else 17
    return np.asarray(img)


def write_gif(path, frames, fps=FPS, colors=64, hold_first=0, hold_last=0):
    """hold_first / hold_last are extra milliseconds on the end frames.

    They have to be expressed as frame durations rather than as repeated
    frames: Pillow collapses identical consecutive frames, so a "pause" built
    by duplicating a frame silently disappears.
    """
    # One shared palette from the first frame, and disposal=1 so Pillow can
    # delta-encode against the previous frame instead of repainting all of it.
    base = Image.fromarray(frames[0]).convert(
        "P", palette=Image.ADAPTIVE, colors=colors)
    imgs = [base] + [Image.fromarray(f).quantize(palette=base, dither=Image.NONE)
                     for f in frames[1:]]

    step = int(1000 / fps)
    durations = [step] * len(imgs)
    durations[0] += hold_first
    durations[-1] += hold_last

    imgs[0].save(path, save_all=True, append_images=imgs[1:],
                 duration=durations, loop=0, optimize=True, disposal=1)
    mb = os.path.getsize(path) / 1e6
    print(f"  wrote {path}  ({len(frames)} frames, {mb:.1f} MB)")


# ── Clips ─────────────────────────────────────────────────────────────────────

def still_home():
    sim = _sim.ArmSim(s3.MODE_CT_FULL)
    r = renderer_for(sim, 900, 700)
    img = grab(r, sim, make_camera())
    path = os.path.join(MEDIA, "home-pose.png")
    Image.fromarray(img).save(path)
    r.close()
    print(f"  wrote {path}")


# Raised, half-extended start for the step-1 clip. From the home pose the arm
# only crumples in on itself; from here the tool drops ~0.87 m, which actually
# reads as a collapse.
COLLAPSE_START = np.array([0.0, -0.9, 0.0, 1.2, 0.0, 0.0, 0.0])


def gif_gravity_collapse(seconds=3.4):
    """Korak 1: zero torque, the arm falls."""
    sim = _sim.ArmSim(s3.MODE_CT_FULL, gravity_only=True)
    sim.data.qpos[sim.qpos_ids] = COLLAPSE_START
    mujoco.mj_forward(sim.model, sim.data)

    r = renderer_for(sim, 520, 470)
    cam = make_camera(elevation=-15, distance=2.25, lookat=[0.05, 0.0, 0.58])
    lines = [("Step 1 - no control torque", True),
             ("tau = 0, the arm falls under gravity", False)]

    frames = [caption(grab(r, sim, cam), lines)]
    for i in range(int(seconds / sim.dt)):
        sim.step()
        if i % EVERY == 0:
            frames.append(caption(grab(r, sim, cam), lines))

    r.close()
    # Pause on the raised pose, and again on the heap, so the loop reads.
    write_gif(os.path.join(MEDIA, "step1-gravity.gif"), frames,
              hold_first=900, hold_last=900)


def gif_hero(laps=1):
    """Korak 3 doing its job, for the top of the README."""
    sim = _sim.ArmSim(s3.MODE_CT_FULL)
    r, cam = renderer_for(sim, 620, 480), make_camera()
    frames = []
    n = int(laps * _sim.LAP_SECONDS / sim.dt)
    for i in range(n):
        e, _, _ = sim.step()
        if i % EVERY == 0:
            err = np.degrees(np.max(np.abs(e)))
            frames.append(caption(grab(r, sim, cam),
                                  [("Inverse dynamics control", True),
                                   (f"worst joint error {err:5.3f} deg", False)]))
    r.close()
    write_gif(os.path.join(MEDIA, "hero.gif"), frames)


def gif_comparison(laps=1):
    """Three controllers, same trajectory, side by side."""
    modes = [s3.MODE_PD, s3.MODE_CT_RIGID, s3.MODE_CT_FULL]
    titles = ["PD + gravity", "Computed torque", "Computed torque"]
    subs = ["step 2", "rigid body only", "full model"]

    sims = [_sim.ArmSim(m) for m in modes]
    rends = [renderer_for(s) for s in sims]
    cam = make_camera()
    frames = []

    n = int(laps * _sim.LAP_SECONDS / sims[0].dt)
    for i in range(n):
        errs = [sim.step()[0] for sim in sims]
        if i % EVERY:
            continue
        tiles = []
        for sim, r, t, sub, e in zip(sims, rends, titles, subs, errs):
            img = grab(r, sim, cam)
            err = np.degrees(np.max(np.abs(e)))
            tiles.append(caption(img, [(t, True), (sub, False),
                                       (f"error {err:6.3f} deg", False)]))
        frames.append(np.hstack(tiles))

    for r in rends:
        r.close()
    write_gif(os.path.join(MEDIA, "comparison.gif"), frames)


def main():
    os.makedirs(MEDIA, exist_ok=True)
    print("Rendering media (no window should open)...")
    still_home()
    gif_gravity_collapse()
    gif_hero()
    gif_comparison()


if __name__ == "__main__":
    main()
