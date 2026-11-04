import csv
import math

rows = list(csv.DictReader(open("sim_log.csv", encoding="utf-8")))
print("rows", len(rows), "t", rows[0]["t"], rows[-1]["t"])
prev = None
for r in rows:
    m = (r["l_mode"], r["f_mode"])
    if m != prev:
        print(f"t={float(r['t']):6.1f}  lead mode {r['l_mode']}  follow mode {r['f_mode']}")
        prev = m
# distance stats
d = [math.hypot(float(r["l_n"]) - float(r["f_n"]), float(r["l_e"]) - float(r["f_e"])) for r in rows]
print("spacing min/mean/max", round(min(d), 1), round(sum(d) / len(d), 1), round(max(d), 1))
for i in range(0, len(rows), 150):
    r = rows[i]
    print(f"t={float(r['t']):6.1f} lead=({float(r['l_n']):6.1f},{float(r['l_e']):6.1f}) "
          f"foll=({float(r['f_n']):6.1f},{float(r['f_e']):6.1f}) dist={d[i]:5.1f}")
