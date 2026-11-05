# -*- coding: utf-8 -*-
"""Run the two-aircraft survey a few times per configuration and compare."""
import csv
import os
import shutil
import subprocess
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = r"C:\Users\86913\Desktop\mh\ardupilot"
PY = sys.executable

CONFIGS = [
    ("baseline", [], []),
    ("active", ["--damp-lead", "0.4"], ["--damp-p", "0.15", "--damp-d", "0.15"]),
]
REPEATS = 3


def metrics(path):
    rows = list(csv.DictReader(open(path, encoding="utf-8")))
    t = np.array([float(r["t"]) for r in rows])
    la = np.array([-float(r["l_d"]) for r in rows])
    fa = np.array([-float(r["f_d"]) for r in rows])
    lv = np.array([-float(r["l_vd"]) for r in rows])
    fv = np.array([-float(r["f_vd"]) for r in rows])
    sel = (t > 22) & (t < 80)
    tilt = la[sel] - fa[sel]
    return (tilt.std(), tilt.max() - tilt.min(), la[sel].std(), fa[sel].std(),
            (lv[sel] - fv[sel]).std())


def main():
    out = []
    for name, lead_args, foll_args in CONFIGS:
        for i in range(REPEATS):
            shutil.rmtree(os.path.join(HERE, "run0"), ignore_errors=True)
            shutil.rmtree(os.path.join(HERE, "run1"), ignore_errors=True)
            cmd = [PY, os.path.join(HERE, "run_sim.py"), "--repo", REPO] + lead_args + foll_args
            with open(os.path.join(HERE, f"sweep_{name}{i}.log"), "w", encoding="utf-8") as f:
                rc = subprocess.call(cmd, stdout=f, stderr=subprocess.STDOUT, cwd=HERE)
            path = os.path.join(HERE, "sim_log.csv")
            if rc != 0 or not os.path.exists(path):
                print(f"{name}{i}: run failed rc={rc}")
                continue
            shutil.copy(path, os.path.join(HERE, f"sweep_{name}{i}.csv"))
            m = metrics(path)
            print(f"{name}{i}: tilt std {m[0]:.2f} p2p {m[1]:.2f} lead {m[2]:.2f} "
                  f"foll {m[3]:.2f} relvel {m[4]:.2f}", flush=True)
            out.append((name, i) + m)
    with open(os.path.join(HERE, "sweep_results.csv"), "w", newline="", encoding="utf-8") as f:
        w = csv.writer(f)
        w.writerow(["config", "run", "tilt_std", "tilt_p2p", "lead_std", "foll_std", "relvel_std"])
        w.writerows(out)
    print("done")


if __name__ == "__main__":
    main()
