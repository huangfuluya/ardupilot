# -*- coding: utf-8 -*-
"""Render the cooperative-lift SITL run as an MP4 animation.

The aircraft trajectories come from sim_log.csv (SITL).  SITL has no tether
physics, so the slung payload is reconstructed offline: a point mass on a
10 m rigid rod hanging from the midpoint of the two aircraft, integrated with
Verlet + distance constraint, which reproduces the pendulum response to the
logged formation motion.

Rendering uses a hand-rolled 3-D -> 2-D projection on a plain 2-D axes, which
is ~100x faster than mplot3d and gives full control over the overlays.
"""
import argparse
import csv
import math
import os

import matplotlib
matplotlib.use("Agg")
import matplotlib.animation as animation
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.patches import Polygon

import imageio_ffmpeg

HERE = os.path.dirname(os.path.abspath(__file__))
ROD_L = 10.0
G = 9.81

BG = "#0d1117"
FG = "#e6edf3"
MUTED = "#7d8ea3"
GRID = "#1b2735"
LEAD_C = "#4da3ff"
FOLL_C = "#ff9f43"
LOAD_C = "#3ddc97"


# ------------------------------------------------------------------ payload
def simulate_payload(lead_ned, foll_ned, t):
    """Payload on a 10 m line under the formation midpoint.

    It starts resting on the ground; the line stays slack until the aircraft
    have climbed more than ROD_L above it, then the payload is lifted off and
    starts to swing like a pendulum.
    """
    a = 0.5 * (lead_ned + foll_ned)

    dt = 0.01
    tt = np.arange(t[0], t[-1] + dt, dt)
    axx = np.interp(tt, t, a[:, 0])
    ayy = np.interp(tt, t, a[:, 1])
    azz = np.interp(tt, t, a[:, 2])

    pos = np.array([axx[0], ayy[0], 0.0])      # resting on the ground (NED z=0)
    vel = np.zeros(3)
    prev = pos.copy()
    out = np.zeros((len(tt), 3))
    grav = np.array([0.0, 0.0, G])

    for i in range(len(tt)):
        anchor = np.array([axx[i], ayy[i], azz[i]])
        vel = vel + grav * dt
        vel *= 0.999
        pos = pos + vel * dt

        # taut line: payload cannot be further than ROD_L from the anchor
        r = pos - anchor
        d = np.linalg.norm(r)
        if d > ROD_L and d > 1e-6:
            pos = anchor + r * (ROD_L / d)

        # ground contact
        if pos[2] > 0.0:
            pos[2] = 0.0
            if vel[2] > 0.0:
                vel[2] = 0.0
            vel[0] *= 0.85
            vel[1] *= 0.85

        vel = (pos - prev) / dt
        prev = pos.copy()
        out[i] = pos

    return np.column_stack([np.interp(t, tt, out[:, 0]),
                            np.interp(t, tt, out[:, 1]),
                            np.interp(t, tt, out[:, 2])])


# ------------------------------------------------------------------ projection
class Camera:
    def __init__(self, az_deg, el_deg):
        self.az = math.radians(az_deg)
        self.el = math.radians(el_deg)
        self.right = np.array([-math.sin(self.az), math.cos(self.az), 0.0])
        self.up = np.array([-math.cos(self.az) * math.sin(self.el),
                            -math.sin(self.az) * math.sin(self.el),
                            math.cos(self.el)])
        self.fwd = np.array([math.cos(self.az) * math.cos(self.el),
                             math.sin(self.az) * math.cos(self.el),
                             math.sin(self.el)])

    def project(self, pts):
        pts = np.asarray(pts, dtype=float)
        return np.stack([pts @ self.right, pts @ self.up], axis=-1)


def rot_body(yaw, pitch, roll):
    """Body (x fwd, y right, z up) -> world ENU, heading measured from north."""
    cy, sy = math.cos(yaw), math.sin(yaw)
    right = np.array([cy, -sy, 0.0])
    fwd = np.array([sy, cy, 0.0])
    up = np.array([0.0, 0.0, 1.0])
    R = np.column_stack([fwd, right, up])
    # pitch about body right axis, roll about body forward axis
    cp, sp = math.cos(pitch), math.sin(pitch)
    cr, sr = math.cos(roll), math.sin(roll)
    Ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    Rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    return R @ Ry @ Rx


def quad_points(center, R, arm=1.05):
    pts = []
    for sx in (1, -1):
        for sy in (1, -1):
            pts.append(center + R @ np.array([sx * arm, sy * arm, 0.0]))
    return np.array(pts)


def rotor_circle(center, R, r=0.30, n=12):
    th = np.linspace(0, 2 * math.pi, n, endpoint=False)
    local = np.stack([r * np.cos(th), r * np.sin(th), np.zeros_like(th)], axis=1)
    return center + (R @ local.T).T


def load_log(path):
    with open(path, newline="", encoding="utf-8") as f:
        rows = list(csv.DictReader(f))
    if not rows:
        raise SystemExit("empty log")
    return {k: np.array([float(r[k]) for r in rows]) for k in rows[0]}


# ------------------------------------------------------------------ main
def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--log", default=os.path.join(HERE, "sim_log.csv"))
    ap.add_argument("--fps", type=int, default=20)
    ap.add_argument("--step", type=float, default=0.1, help="sim seconds per frame")
    ap.add_argument("--out", default=os.path.join(HERE, "clift_sim.mp4"))
    ap.add_argument("--summary", default=os.path.join(HERE, "clift_summary.png"))
    args = ap.parse_args()

    d = load_log(args.log)
    t = d["t"]
    lead_ned = np.column_stack([d["l_n"], d["l_e"], d["l_d"]])
    foll_ned = np.column_stack([d["f_n"], d["f_e"], d["f_d"]])

    # prefer the payload simulated by SITL (dual cooperative-lift model);
    # fall back to the offline reconstruction if it is not available
    sim_payload_path = os.path.join(HERE, "sim_payload.csv")
    if os.path.exists(sim_payload_path):
        pd = load_log(sim_payload_path)
        payload_ned = np.column_stack([
            np.interp(t, pd["t"], pd["n"]),
            np.interp(t, pd["t"], pd["e"]),
            np.interp(t, pd["t"], pd["d"]),
        ])
        payload_source = "payload simulated by SITL (two-line model)"
    else:
        payload_ned = simulate_payload(lead_ned, foll_ned, t)
        payload_source = "payload reconstructed offline (10 m rod)"

    to_enu = np.array([[0, 1, 0], [1, 0, 0], [0, 0, -1]], dtype=float)
    lead = lead_ned @ to_enu.T
    foll = foll_ned @ to_enu.T
    payload = payload_ned @ to_enu.T

    dt = np.median(np.diff(t))
    frames = np.arange(0, len(t), max(1, int(round(args.step / dt))))

    # ---- figure / artists (created once, updated per frame)
    fig = plt.figure(figsize=(12.8, 7.2), dpi=100)
    fig.patch.set_facecolor(BG)
    ax = fig.add_axes([0.0, 0.0, 1.0, 1.0])
    ax.set_facecolor(BG)
    ax.set_xticks([])
    ax.set_yticks([])
    for sp in ax.spines.values():
        sp.set_visible(False)

    cam0 = Camera(-62, 20)
    # fixed zoom that fits the widest subject extent over the whole run
    max_dx = max_dy = 0.0
    zoom = [16.0]          # smoothed half-width, metres

    text_time = fig.text(0.025, 0.945, "", color=FG, fontsize=15, weight="bold")
    text_stat = fig.text(0.025, 0.895, "", color=MUTED, fontsize=10.5)
    fig.text(0.025, 0.045,
             "blue: lead    orange: follower    green: slung payload    "
             f"aircraft drawn 2× scale    {payload_source}",
             color=MUTED, fontsize=9)

    # static ground grid (redrawn every frame because the camera follows)
    NGRID = 9
    grid_lines = [ax.plot([], [], color=GRID, lw=0.8, zorder=1)[0] for _ in range(2 * NGRID)]

    trail_lead, = ax.plot([], [], color=LEAD_C, lw=1.6, alpha=0.75, zorder=3)
    trail_foll, = ax.plot([], [], color=FOLL_C, lw=1.6, alpha=0.75, zorder=3)
    trail_load, = ax.plot([], [], color=LOAD_C, lw=1.3, alpha=0.7, zorder=3)
    teth1, = ax.plot([], [], color="#c9d6e3", lw=1.5, alpha=0.9, zorder=5)
    teth2, = ax.plot([], [], color="#c9d6e3", lw=1.5, alpha=0.9, zorder=5)
    load_mark, = ax.plot([], [], marker="s", ms=13, color=LOAD_C,
                         markeredgecolor=BG, zorder=8)
    load_shadow, = ax.plot([], [], marker="o", ms=9, color="#12301f", zorder=2)
    load_drop, = ax.plot([], [], color=LOAD_C, lw=0.9, ls="--", alpha=0.35, zorder=2)

    quad_art = []
    for color in (LEAD_C, FOLL_C):
        arms = [ax.plot([], [], color=color, lw=2.0, zorder=6)[0] for _ in range(4)]
        rotors = [Polygon(np.zeros((12, 2)), closed=True, facecolor="none",
                          edgecolor=color, lw=1.0, zorder=7) for _ in range(4)]
        for rp in rotors:
            ax.add_patch(rp)
        nose, = ax.plot([], [], color=color, lw=2.6, zorder=7)
        body, = ax.plot([], [], marker="o", ms=6, color=color, zorder=7)
        quad_art.append((arms, rotors, nose, body))

    def update(k):
        i = frames[k]
        cam = Camera(-62, 20 + 4 * math.sin(0.02 * i))

        # camera follows the formation midpoint
        cxw = 0.5 * (lead[i, 0] + foll[i, 0])
        cyw = 0.5 * (lead[i, 1] + foll[i, 1])
        # centre on the projected bounding box of the whole subject and keep
        # it filling the frame with a smoothed zoom (so take-off is visible too)
        pts = cam.project(np.vstack([lead[i], foll[i], payload[i]]))
        pcx = 0.5 * (pts[:, 0].min() + pts[:, 0].max())
        pcy = 0.5 * (pts[:, 1].min() + pts[:, 1].max()) + 0.5
        span_x = pts[:, 0].max() - pts[:, 0].min()
        span_y = pts[:, 1].max() - pts[:, 1].min()
        target = max(span_x / 2.0, span_y * 16.0 / 9.0 / 2.0) * 1.55
        target = max(10.0, min(30.0, target))
        zoom[0] += 0.06 * (target - zoom[0])
        half_w = zoom[0]
        half_h = half_w * 9.0 / 16.0
        ax.set_xlim(pcx - half_w, pcx + half_w)
        ax.set_ylim(pcy - half_h, pcy + half_h)

        # ground grid around the camera centre
        span = 40.0
        xs = np.linspace(math.floor((cxw - span) / 10) * 10, cxw + span, NGRID)
        ys = np.linspace(math.floor((cyw - span) / 10) * 10, cyw + span, NGRID)
        gi = 0
        for gx in xs:
            q = cam.project(np.array([[gx, cyw - span, 0], [gx, cyw + span, 0]]))
            grid_lines[gi].set_data(q[:, 0], q[:, 1]); gi += 1
        for gy in ys:
            q = cam.project(np.array([[cxw - span, gy, 0], [cxw + span, gy, 0]]))
            grid_lines[gi].set_data(q[:, 0], q[:, 1]); gi += 1

        j = max(0, i - int(4.0 / dt))
        seg = slice(j, i + 1)
        p = cam.project(lead[seg])
        trail_lead.set_data(p[:, 0], p[:, 1])
        p = cam.project(foll[seg])
        trail_foll.set_data(p[:, 0], p[:, 1])
        p = cam.project(payload[seg])
        trail_load.set_data(p[:, 0], p[:, 1])

        pl = cam.project(payload[i:i + 1])[0]
        load_mark.set_data([pl[0]], [pl[1]])
        sh = cam.project(np.array([[payload[i, 0], payload[i, 1], 0.0]]))[0]
        load_shadow.set_data([sh[0]], [sh[1]])
        load_drop.set_data([pl[0], sh[0]], [pl[1], sh[1]])

        for (pos, yaw, pitch, roll, art) in (
                (lead[i], d["l_yaw"][i], d["l_pitch"][i], d["l_roll"][i], quad_art[0]),
                (foll[i], d["f_yaw"][i], d["f_pitch"][i], d["f_roll"][i], quad_art[1])):
            R = rot_body(float(yaw), float(pitch), float(roll))
            centers = quad_points(pos, R)
            cup = cam.project(np.vstack([pos, centers]))[0:1], cam.project(np.vstack([pos, centers]))[1:]
            body2d, arms2d = cup
            arms, rotors, nose, body = art
            body.set_data([body2d[0, 0]], [body2d[0, 1]])
            for a, c in zip(arms, arms2d):
                a.set_data([body2d[0, 0], c[0]], [body2d[0, 1], c[1]])
            for rp, c3 in zip(rotors, centers):
                circ = rotor_circle(c3, R)
                rp.set_xy(cam.project(circ))
            n3 = pos + R @ np.array([0.95, 0.0, 0.0])
            n2 = cam.project(np.array([pos, n3]))
            nose.set_data(n2[:, 0], n2[:, 1])

        a2 = cam.project(np.array([lead[i], payload[i]]))
        teth1.set_data(a2[:, 0], a2[:, 1])
        a2 = cam.project(np.array([foll[i], payload[i]]))
        teth2.set_data(a2[:, 0], a2[:, 1])

        speed = math.hypot(d["l_vn"][i], d["l_ve"][i])
        dz = lead[i, 2] - foll[i, 2]
        dist = math.hypot(lead[i, 0] - foll[i, 0], lead[i, 1] - foll[i, 1])
        text_time.set_text(f"Cooperative Lift · 2× ArduCopter SITL      t = {t[i]:5.1f} s")
        text_stat.set_text(f"lead speed {speed:4.1f} m/s    "
                           f"formation tilt {dz:+5.2f} m    spacing {dist:5.1f} m    "
                           f"payload {abs(payload[i, 2] - 0.5 * (lead[i, 2] + foll[i, 2])):4.1f} m below")
        return []

    print(f"rendering {len(frames)} frames ...")
    plt.rcParams["animation.ffmpeg_path"] = imageio_ffmpeg.get_ffmpeg_exe()
    ani = animation.FuncAnimation(fig, update, frames=len(frames),
                                  interval=1000 / args.fps, blit=False)
    writer = animation.FFMpegWriter(fps=args.fps, bitrate=4000,
                                    extra_args=["-vcodec", "libx264", "-pix_fmt", "yuv420p"])
    ani.save(args.out, writer=writer, dpi=100, savefig_kwargs={"facecolor": BG})
    print("saved", args.out)

    # ---------------- summary figure
    fig2, axs = plt.subplots(2, 2, figsize=(12, 7), dpi=110)
    axs[0, 0].plot(lead[:, 0], lead[:, 1], color="#1f6feb", lw=1.4, label="lead")
    axs[0, 0].plot(foll[:, 0], foll[:, 1], color="#e8890c", lw=1.4, label="follower")
    axs[0, 0].plot(payload[:, 0], payload[:, 1], color="#1a9e6f", lw=0.9, label="payload")
    axs[0, 0].set_title("Top view (East–North)")
    axs[0, 0].set_xlabel("East (m)")
    axs[0, 0].set_ylabel("North (m)")
    axs[0, 0].axis("equal")
    axs[0, 0].legend(fontsize=8)
    axs[0, 0].grid(alpha=0.3)

    axs[0, 1].plot(t, lead[:, 2], color="#1f6feb", label="lead")
    axs[0, 1].plot(t, foll[:, 2], color="#e8890c", label="follower")
    axs[0, 1].plot(t, payload[:, 2], color="#1a9e6f", lw=0.9, label="payload")
    axs[0, 1].set_title("Altitude")
    axs[0, 1].set_xlabel("t (s)")
    axs[0, 1].set_ylabel("m")
    axs[0, 1].legend(fontsize=8)
    axs[0, 1].grid(alpha=0.3)

    axs[1, 0].plot(t, lead[:, 2] - foll[:, 2], color="#7b3fe4")
    axs[1, 0].set_title("Acceleration lean: lead − follower altitude")
    axs[1, 0].set_xlabel("t (s)")
    axs[1, 0].set_ylabel("m")
    axs[1, 0].grid(alpha=0.3)

    axs[1, 1].plot(t, np.hypot(lead[:, 0] - foll[:, 0], lead[:, 1] - foll[:, 1]),
                   color="#c0392b")
    axs[1, 1].set_title("Horizontal spacing")
    axs[1, 1].set_xlabel("t (s)")
    axs[1, 1].set_ylabel("m")
    axs[1, 1].grid(alpha=0.3)
    fig2.tight_layout()
    fig2.savefig(args.summary, dpi=110)
    print("saved", args.summary)


if __name__ == "__main__":
    main()




