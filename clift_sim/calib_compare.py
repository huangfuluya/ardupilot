# -*- coding: utf-8 -*-
"""Calibrate the offline payload model against the SITL slung-payload model."""
import glob
import math
import os

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from pymavlink import mavutil

import slung_model

HERE = os.path.dirname(os.path.abspath(__file__))


def find_bin():
    files = sorted(glob.glob(os.path.join(HERE, "run0", "logs", "*.BIN")))
    return files[-1] if files else None


def load(path):
    mlog = mavutil.mavlink_connection(path)
    slup, pos = [], []
    while True:
        m = mlog.recv_match(type=["SLUP", "POS"])
        if m is None:
            break
        if m.get_type() == "SLUP":
            slup.append((m.TimeUS * 1e-6, m.Land, m.Tens,
                         m.PN, m.PE, m.PD, m.VN, m.VE, m.VD))
        else:
            pos.append((m.TimeUS * 1e-6, m.Lat, m.Lng, m.RelHomeAlt))
    st = np.array([r[0] for r in slup])
    tens = np.array([r[2] for r in slup])
    p_rel = np.array([[r[3], r[4], r[5]] for r in slup])
    pt = np.array([r[0] for r in pos])
    lat0, lon0 = pos[0][1], pos[0][2]
    veh = np.column_stack([
        (np.array([r[1] for r in pos]) - lat0) * 111320.0,
        (np.array([r[2] for r in pos]) - lon0) * 111320.0 * math.cos(math.radians(lat0)),
        -np.array([r[3] for r in pos]),
    ])
    return st, tens, p_rel, pt, veh


def main():
    path = find_bin()
    print("log:", path)
    st, tens, p_rel, pt, veh = load(path)
    if len(st) < 10:
        raise SystemExit("no SLUP data")

    veh_vel = np.column_stack([np.gradient(veh[:, k], pt) for k in range(3)])
    air = (tens > 0.2) & (st > 20.0)
    idx = np.where(air)[0]
    print(f"SLUP {len(st)} samples, airborne window {st[idx[0]]:.1f}..{st[idx[-1]]:.1f}s")

    results = []
    for drag in (0.0, 0.5, 1.0, 2.0):
        tt, p, v, ratio = slung_model.simulate(
            veh, veh_vel, pt, line_length=10.0, weight_kg=2.0,
            drag_coef=drag, dt=0.0025)
        anchor = np.column_stack([np.interp(tt, pt, veh[:, k]) for k in range(3)])
        rel_tt = p - anchor
        rel = np.column_stack([np.interp(st[idx], tt, rel_tt[:, k]) for k in range(3)])
        err = rel - p_rel[idx]
        rmse = float(np.sqrt(np.mean(err ** 2)))
        results.append((rmse, drag, rel, err))
    results.sort(key=lambda r: r[0])
    print("\nRMSE (m)  drag")
    for rmse, drag, _, _ in results:
        print(f"  {rmse:6.3f}   {drag:.1f}")

    rmse, drag, rel, err = results[0]
    per_axis = np.sqrt(np.mean(err ** 2, axis=0))
    print(f"\nbest drag {drag}: RMSE {rmse:.3f} m  "
          f"(N {per_axis[0]:.3f}  E {per_axis[1]:.3f}  D {per_axis[2]:.3f})")
    print(f"truth swing  N {np.ptp(p_rel[idx,0]):.2f}  E {np.ptp(p_rel[idx,1]):.2f}  "
          f"D {np.ptp(p_rel[idx,2]):.2f} m")

    fig, axs = plt.subplots(3, 1, figsize=(11, 8), sharex=True, dpi=110)
    labels = ("North (m)", "East (m)", "Down (m)")
    for k in range(3):
        axs[k].plot(st[idx], p_rel[idx, k], color="#1f6feb", lw=1.6, label="SITL SLUP")
        axs[k].plot(st[idx], rel[:, k], color="#e8890c", lw=1.4, ls="--",
                    label=f"offline model (drag {drag})")
        axs[k].set_ylabel(labels[k])
        axs[k].grid(alpha=0.3)
        axs[k].legend(fontsize=8, loc="upper right")
    axs[0].set_title("Slung payload: SITL model vs offline port (payload relative to vehicle)")
    axs[-1].set_xlabel("time since boot (s)")
    fig.tight_layout()
    fig.savefig(os.path.join(HERE, "calib_compare.png"), dpi=110)
    print("saved calib_compare.png")

    with open(os.path.join(HERE, "calib_result.txt"), "w", encoding="utf-8") as f:
        f.write(f"log {path}\nSLUP samples {len(st)} airborne {len(idx)}\n")
        f.write(f"best drag {drag} RMSE {rmse:.3f} m\n")
        f.write(f"per axis RMSE N {per_axis[0]:.3f} E {per_axis[1]:.3f} D {per_axis[2]:.3f}\n")
        for r, d, _, _ in sorted(results, key=lambda r: r[1]):
            f.write(f"  drag {d:.1f} RMSE {r:.3f}\n")


if __name__ == "__main__":
    main()
