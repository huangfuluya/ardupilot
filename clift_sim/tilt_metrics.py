"""Baseline metrics for the formation tilt oscillation."""
import csv
import sys

import numpy as np

path = sys.argv[1] if len(sys.argv) > 1 else "sim_log.csv"
rows = list(csv.DictReader(open(path, encoding="utf-8")))
t = np.array([float(r["t"]) for r in rows])
la = np.array([-float(r["l_d"]) for r in rows])
fa = np.array([-float(r["f_d"]) for r in rows])
lv = np.array([-float(r["l_vd"]) for r in rows])
fv = np.array([-float(r["f_vd"]) for r in rows])

sel = (t > 22) & (t < 80)
d = la[sel] - fa[sel]
print(f"survey samples {sel.sum()}")
print(f"tilt mean {d.mean():+.2f} m  std {d.std():.2f}  p2p {d.max()-d.min():.2f}")
print(f"rel vertical speed std {np.std(lv[sel]-fv[sel]):.2f} m/s")
# dominant period via zero crossings of the detrended signal
x = d - d.mean()
sign = np.sign(x)
zc = np.where(np.diff(sign) != 0)[0]
if len(zc) > 2:
    dt = np.mean(np.diff(zc)) * 2 * np.median(np.diff(t[sel]))
    print(f"zero-crossing period ~{dt:.1f} s  (crossings {len(zc)})")
