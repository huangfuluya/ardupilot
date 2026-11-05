# -*- coding: utf-8 -*-
"""Calibration run: one Copter with the SITL slung-payload model enabled.

Flies a take-off plus a circular survey so the payload swings, and records the
vehicle trajectory.  The SITL model writes its own SLUP log into the dataflash
BIN, which calibrate_slung.py compares against the offline pendulum.
"""
import csv
import math
import os
import subprocess
import time

import run_sim as rs

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = r"C:\Users\86913\Desktop\mh\ardupilot"
PORT = 14550


def main():
    subprocess.run(["taskkill", "/F", "/IM", "arducopter.exe"],
                   stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    time.sleep(1)

    run_dir = rs.prep_run_dir(REPO, 0, os.path.join(HERE, "clift.lua"), with_script=False)
    proc = rs.start_sitl(REPO, 0, PORT, rs.LEAD_HOME, run_dir,
                         os.path.join(HERE, "sitl_slup.parm"))
    rows = []
    try:
        conn = rs.connect(PORT, 1)
        rs.request_streams(conn)
        time.sleep(2)

        rs.set_mode(conn, 4)
        assert rs.arm(conn, timeout=120), "failed to arm"
        rs.takeoff(conn, 15.0)

        t_start = time.time()
        state = {}
        next_sp = 0.0
        airborne_t = None
        while True:
            now = time.time() - t_start
            if now > 150:
                break
            for _ in range(80):
                try:
                    m = conn.recv_match(blocking=False)
                except (ConnectionResetError, OSError):
                    break
                if m is None:
                    break
                tp = m.get_type()
                if tp == "LOCAL_POSITION_NED":
                    state.update(n=m.x, e=m.y, d=m.z, vn=m.vx, ve=m.vy, vd=m.vz)
                elif tp == "ATTITUDE":
                    state.update(roll=m.roll, pitch=m.pitch, yaw=m.yaw)

            alt = -state.get("d", 0.0)
            if airborne_t is None and alt > 13.0:
                airborne_t = now
                rs.log("at 20 m, starting survey")
            if airborne_t is not None:
                mt = now - airborne_t
                if mt < 40.0:
                    if now >= next_sp:
                        th = 2 * math.pi * mt / 30.0
                        rs.send_pos_ned(conn, 18 * math.sin(th), 18 * (1 - math.cos(th)), -15.0)
                        next_sp = now + 0.1
                elif mt >= 40.0 and mt < 42.0:
                    rs.set_mode(conn, 9)          # LAND
                elif mt > 60.0:
                    break

            if all(k in state for k in ("n", "e", "d")):
                rows.append([round(now, 3), state["n"], state["e"], state["d"],
                             state.get("vn", 0), state.get("ve", 0), state.get("vd", 0),
                             state.get("roll", 0), state.get("pitch", 0), state.get("yaw", 0)])
            time.sleep(0.05)

    finally:
        proc.terminate()
        time.sleep(1)
        if proc.poll() is None:
            proc.kill()

    out = os.path.join(HERE, "calib_vehicle.csv")
    with open(out, "w", newline="", encoding="utf-8") as f:
        w = csv.writer(f)
        w.writerow(["t", "n", "e", "d", "vn", "ve", "vd", "roll", "pitch", "yaw"])
        w.writerows(rows)
    rs.log(f"wrote {out} ({len(rows)} rows)")
    bins = sorted(os.listdir(os.path.join(run_dir, "logs")))
    rs.log("dataflash logs: " + ", ".join(bins))


if __name__ == "__main__":
    main()
