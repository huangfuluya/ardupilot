import csv

import numpy as np

log = {}
with open("sim_log.csv", encoding="utf-8") as f:
    for r in csv.DictReader(f):
        log.setdefault("t", []).append(float(r["t"]))
        for k in ("l_n", "l_e", "l_d", "f_n", "f_e", "f_d"):
            log.setdefault(k, []).append(float(r[k]))
pay = {"t": [], "n": [], "e": [], "d": [], "tens": []}
with open("sim_payload.csv", encoding="utf-8") as f:
    for r in csv.DictReader(f):
        pay["t"].append(float(r["t"]))
        for k in ("n", "e", "d"):
            pay[k].append(float(r[k]))
        pay["tens"].append(float(r["tens"]))

t = np.array(log["t"])
lead = np.column_stack([log["l_n"], log["l_e"], log["l_d"]])
foll = np.column_stack([log["f_n"], log["f_e"], log["f_d"]])
pt = np.array(pay["t"])
p = np.column_stack([np.interp(t, pt, pay[k]) for k in ("n", "e", "d")])

d1 = np.linalg.norm(p - lead, axis=1)
d2 = np.linalg.norm(p - foll, axis=1)
alt = -p[:, 2]
print("payload alt  min %.1f max %.1f" % (alt.min(), alt.max()))
print("dist to lead  min %.2f max %.2f" % (d1.min(), d1.max()))
print("dist to foll  min %.2f max %.2f" % (d2.min(), d2.max()))
print("payload below midpoint: min %.1f max %.1f" %
      ((-p[:, 2] - (-(lead[:, 2] + foll[:, 2]) / 2)).min(),
       (-p[:, 2] - (-(lead[:, 2] + foll[:, 2]) / 2)).max()))
print("\nt   lead_alt  foll_alt  pay_alt  d1   d2   tens")
for i in range(0, len(t), 100):
    print(f"{t[i]:5.1f} {-lead[i,2]:7.1f} {-foll[i,2]:8.1f} {-p[i,2]:8.1f} {d1[i]:5.1f} {d2[i]:5.1f}")
