"""Mode shape of the tilt oscillation: who is moving?"""
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
print(f"lead alt   mean {la[sel].mean():.2f} std {la[sel].std():.2f}")
print(f"foll alt   mean {fa[sel].mean():.2f} std {fa[sel].std():.2f}")
print(f"tilt       mean {(la[sel]-fa[sel]).mean():+.2f} std {(la[sel]-fa[sel]).std():.2f}")
print(f"corr(lead,foll) {np.corrcoef(la[sel], fa[sel])[0,1]:+.2f}")
print(f"lead vd std {lv[sel].std():.2f}  foll vd std {fv[sel].std():.2f}  corr {np.corrcoef(lv[sel], fv[sel])[0,1]:+.2f}")
