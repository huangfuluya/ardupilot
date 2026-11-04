# -*- coding: utf-8 -*-
"""Two-aircraft cooperative-lift SITL run.

Launches two ArduCopter SITL instances (lead + follower), loads clift.lua on
both, relays FOLLOW_TARGET from the lead to the follower, flies a scripted
path and records a CSV for the animation.

Usage:
    python run_sim.py --repo C:\\Users\\86913\\Desktop\\mh\\ardupilot

Outputs (in this directory):
    sim_log.csv     20 Hz state log for both aircraft
    run0/ run1/     SITL working dirs (APM/scripts, logs)
"""
import argparse
import csv
import math
import os
import shutil
import subprocess
import sys
import time

from pymavlink import mavutil

HERE = os.path.dirname(os.path.abspath(__file__))
CYGWIN_BIN = r"C:\cygwin64\bin"

LEAD_PORT = 14550
FOLL_PORT = 14560
LEAD_HOME = (-35.363262, 149.165237, 584.0, 0.0)
FOLL_HOME = (-35.363262, 149.165397, 584.0, 0.0)      # ~16 m east of the lead

PARAMS_LEAD = {
    "MAV_SYSID": 1,
    "CLIFT_ENABLE": 1, "CLIFT_ROLE": 0, "CLIFT_D": 5.0,
    "CLIFT_LEAN": 1, "CLIFT_BAL": 0, "CLIFT_MAV_CHAN": 0,
    "WP_ACCEL": 2.0, "WP_JERK": 2.0,
    "PSC_NE_JERK": 4.0, "PSC_D_JERK": 2.0,
}
PARAMS_FOLL = {
    "MAV_SYSID": 2,
    "CLIFT_ENABLE": 1, "CLIFT_ROLE": 1, "CLIFT_D": -5.0,
    "CLIFT_LEAN": 1, "CLIFT_BAL": 1, "CLIFT_BAL_P": 0.5, "CLIFT_BAL_D": 0.05,
    "CLIFT_GOSSIP": 1, "CLIFT_R2": 50.0, "CLIFT_R2_ACT": 0,
    "FOLL_ENABLE": 1, "FOLL_SYSID": 1, "FOLL_OFS_TYPE": 1,
    "FOLL_OFS_X": -10.0, "FOLL_OFS_Y": 0.0, "FOLL_OFS_Z": 0.0,
    "FOLL_ACCEL_NE": 4.0, "FOLL_JERK_NE": 8.0,
    "FOLL_ACCEL_D": 4.0, "FOLL_JERK_D": 4.0, "FOLL_TIMEOUT": 10.0,
    "WP_ACCEL": 2.0, "WP_JERK": 2.0,
}


def home_delta_ned():
    """Follower home expressed in the lead's local NED frame (metres)."""
    dlat = FOLL_HOME[0] - LEAD_HOME[0]
    dlon = FOLL_HOME[1] - LEAD_HOME[1]
    north = dlat * 111320.0
    east = dlon * 111320.0 * math.cos(math.radians(LEAD_HOME[0]))
    return north, east


def log(msg):
    print(f"[{time.strftime('%H:%M:%S')}] {msg}", flush=True)


def prep_run_dir(repo, idx, script_src):
    run_dir = os.path.join(HERE, f"run{idx}")
    scripts = os.path.join(run_dir, "scripts")
    os.makedirs(scripts, exist_ok=True)
    shutil.copy(script_src, os.path.join(scripts, "clift.lua"))
    dst_mod = os.path.join(scripts, "modules", "MAVLink")
    src_mod = os.path.join(repo, "libraries", "AP_Scripting", "modules", "MAVLink")
    if not os.path.isdir(dst_mod):
        shutil.copytree(src_mod, dst_mod)
    return run_dir


def start_sitl(repo, idx, port, home, run_dir, defaults):
    bin_path = os.path.join(repo, "build", "sitl", "bin", "arducopter.exe")

    env = dict(os.environ)
    env["PATH"] = CYGWIN_BIN + os.pathsep + env.get("PATH", "")
    cmd = [
        bin_path,
        "--model", "quad",
        "-I", str(idx),
        "--home", "{:.6f},{:.6f},{:.1f},{:.1f}".format(*home),
        "--serial0", f"udpclient:127.0.0.1:{port}",
        "--speedup", "1",
    ]
    if os.path.exists(defaults):
        cmd += ["--defaults", defaults]
    outp = open(os.path.join(run_dir, "sitl.log"), "w", encoding="utf-8", errors="replace")
    log("start " + " ".join(cmd))
    return subprocess.Popen(cmd, cwd=run_dir, env=env, stdout=outp, stderr=subprocess.STDOUT)


def connect(port, sysid, timeout=60):
    conn = mavutil.mavlink_connection(f"udpin:127.0.0.1:{port}", source_system=250 + sysid)
    t0 = time.time()
    while time.time() - t0 < timeout:
        hb = conn.recv_match(type="HEARTBEAT", blocking=True, timeout=2)
        if hb is not None and hb.get_srcSystem() == sysid:
            conn.target_system = sysid
            conn.target_component = 1
            log(f"connected sysid={sysid} on {port}")
            return conn
    raise RuntimeError(f"no heartbeat from sysid {sysid} on {port}")


def wait_script_loaded(conn, timeout=10):
    """Wait for the 'clift.lua loaded' STATUSTEXT so CLIFT_* params exist."""
    t0 = time.time()
    while time.time() - t0 < timeout:
        m = conn.recv_match(type="STATUSTEXT", blocking=True, timeout=2)
        if m is not None and "clift.lua loaded" in m.text:
            log("script loaded")
            return True
    log("WARNING: script load message not seen, continuing")
    return False


def request_streams(conn):
    """Ask for the messages the logger needs at a usable rate."""
    for msg_id, hz in ((30, 20), (32, 20), (33, 10), (24, 10)):
        conn.mav.command_long_send(
            conn.target_system, conn.target_component,
            mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, 0,
            msg_id, int(1e6 / hz), 0, 0, 0, 0, 0)


def set_params(conn, params, settle=0.4):
    for name, value in params.items():
        conn.mav.param_set_send(conn.target_system, conn.target_component,
                                name.encode(), float(value),
                                mavutil.mavlink.MAV_PARAM_TYPE_REAL32)
    time.sleep(settle)
    log(f"set {len(params)} params")


def set_mode(conn, mode):
    conn.mav.set_mode_send(conn.target_system,
                           mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
                           mode)


def arm(conn, timeout=90):
    """Keep asking to arm until the pre-arm checks pass."""
    t0 = time.time()
    last_cmd = 0.0
    while time.time() - t0 < timeout:
        if time.time() - last_cmd > 5:
            conn.mav.command_long_send(conn.target_system, conn.target_component,
                                       mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0,
                                       1, 0, 0, 0, 0, 0, 0)
            last_cmd = time.time()
        m = conn.recv_match(type=["HEARTBEAT", "STATUSTEXT"], blocking=True, timeout=1)
        if m is None:
            continue
        if m.get_type() == "HEARTBEAT":
            if m.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED:
                return True
        else:
            txt = m.text
            if "PreArm" in txt or "Arm" in txt:
                log(f"  arm check: {txt}")
    return False


def takeoff(conn, alt):
    conn.mav.command_long_send(conn.target_system, conn.target_component,
                               mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0,
                               0, 0, 0, 0, 0, 0, alt)


def send_pos_ned(conn, north, east, down):
    """Position-only setpoint in local NED (relative to EKF origin)."""
    conn.mav.set_position_target_local_ned_send(
        0, conn.target_system, conn.target_component,
        mavutil.mavlink.MAV_FRAME_LOCAL_NED, 0b0000111111111000,
        north, east, down, 0, 0, 0, 0, 0, 0, 0, 0)


def smoothstep(t):
    t = min(max(t, 0.0), 1.0)
    return t * t * (3 - 2 * t)


def path_at(t):
    """Smooth circular survey: hover, then a 20 m-radius circle.

    A circle keeps the lateral acceleration continuous, so the formation
    holds together and the acceleration-lean offset rotates around the ring.
    """
    if t < 8:
        return 0.0, 0.0
    if t > 66:
        t = 66
    th = 2 * math.pi * (t - 8) / 30.0
    R = 18.0
    return R * math.sin(th), R * (1 - math.cos(th))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--repo", required=True)
    ap.add_argument("--no-launch", action="store_true")
    ap.add_argument("--duration", type=float, default=70.0)
    args = ap.parse_args()

    repo = os.path.abspath(args.repo)
    script_src = os.path.join(HERE, "clift.lua")
    run_dirs = [prep_run_dir(repo, i, script_src) for i in (0, 1)]

    procs = []
    if not args.no_launch:
        procs.append(start_sitl(repo, 0, LEAD_PORT, LEAD_HOME, run_dirs[0], os.path.join(HERE, "sitl_lead.parm")))
        procs.append(start_sitl(repo, 1, FOLL_PORT, FOLL_HOME, run_dirs[1], os.path.join(HERE, "sitl_follower.parm")))

    lead = follow = None
    try:
        lead = connect(LEAD_PORT, 1)
        follow = connect(FOLL_PORT, 2)

        time.sleep(2)
        wait_script_loaded(lead)
        wait_script_loaded(follow)
        request_streams(lead)
        request_streams(follow)
        set_params(lead, PARAMS_LEAD)
        set_params(follow, PARAMS_FOLL)
        time.sleep(3)

        # ---- arm + takeoff
        for name, conn in (("lead", lead), ("follower", follow)):
            set_mode(conn, 4)          # GUIDED
        time.sleep(1)
        assert arm(lead), "lead failed to arm"
        assert arm(follow), "follower failed to arm"
        log("armed both")
        takeoff(lead, 20.0)
        takeoff(follow, 20.0)

        relay_count = [0]

        def relay_and_pump(seconds):
            """Forward FOLLOW_TARGET lead->follower and drain both links."""
            t_end = time.time() + seconds
            alts = {}
            while time.time() < t_end:
                m = lead.recv_match(type="FOLLOW_TARGET", blocking=False)
                while m is not None:
                    try:
                        follow.write(m.get_msgbuf())
                        relay_count[0] += 1
                    except Exception:
                        pass
                    m = lead.recv_match(type="FOLLOW_TARGET", blocking=False)
                for conn, sid in ((lead, 1), (follow, 2)):
                    mm = conn.recv_match(type=["GLOBAL_POSITION_INT", "HEARTBEAT"],
                                         blocking=False)
                    while mm is not None:
                        if mm.get_type() == "GLOBAL_POSITION_INT":
                            alts[sid] = mm.relative_alt
                        mm = conn.recv_match(type=["GLOBAL_POSITION_INT", "HEARTBEAT"],
                                             blocking=False)
                time.sleep(0.02)
            return alts

        t0 = time.time()
        while time.time() - t0 < 40:
            alts = relay_and_pump(0.5)
            if alts.get(1, 0) > 8000 and alts.get(2, 0) > 8000:
                break
        log("both above 8 m")
        relay_and_pump(3.0)            # give the follower a few valid targets first
        log("follower -> FOLLOW")
        set_mode(follow, 23)

        # ---- main loop
        t_start = time.time()
        rows = []
        last_log = 0.0
        relayed = 0
        next_setpoint = 0.0
        state = {1: {}, 2: {}}
        HN, HE = home_delta_ned()
        log(f"follower home in lead frame: north {HN:+.1f} east {HE:+.1f} m")

        while True:
            now = time.time() - t_start
            if now > args.duration:
                break

            # ---- drain every message once, then dispatch (recv_match with a
            #      type filter would discard the non-matching messages)
            for conn, sid in ((lead, 1), (follow, 2)):
                for _ in range(60):
                    m = conn.recv_match(blocking=False)
                    if m is None:
                        break
                    tp = m.get_type()
                    if tp == "FOLLOW_TARGET":
                        if sid == 1:
                            try:
                                follow.write(m.get_msgbuf())
                                relayed += 1
                            except Exception:
                                pass
                    elif tp == "LOCAL_POSITION_NED":
                        state[sid].update(n=m.x, e=m.y, d=m.z,
                                          vn=m.vx, ve=m.vy, vd=m.vz)
                    elif tp == "ATTITUDE":
                        state[sid].update(roll=m.roll, pitch=m.pitch, yaw=m.yaw)
                    elif tp == "HEARTBEAT":
                        state[sid]["armed"] = bool(m.base_mode & 128)
                        state[sid]["mode"] = m.custom_mode

            # feed the lead a moving setpoint
            if now >= next_setpoint:
                n, e = path_at(now)
                send_pos_ned(lead, n, e, -20.0)
                next_setpoint = now + 0.1

            if all(k in state[1] for k in ("n", "e", "d")) and \
               all(k in state[2] for k in ("n", "e", "d")):
                # each vehicle's LOCAL_POSITION_NED is relative to its own EKF
                # origin, so shift the follower into the lead's frame
                fn = state[2]["n"] + HN
                fe = state[2]["e"] + HE
                rows.append([
                    round(now, 3),
                    state[1]["n"], state[1]["e"], state[1]["d"],
                    state[1].get("vn", 0), state[1].get("ve", 0), state[1].get("vd", 0),
                    state[1].get("roll", 0), state[1].get("pitch", 0), state[1].get("yaw", 0),
                    state[1].get("mode", -1),
                    fn, fe, state[2]["d"],
                    state[2].get("vn", 0), state[2].get("ve", 0), state[2].get("vd", 0),
                    state[2].get("roll", 0), state[2].get("pitch", 0), state[2].get("yaw", 0),
                    state[2].get("mode", -1),
                ])

            if now - last_log > 5:
                last_log = now
                log(f"t={now:5.1f}s lead=({state[1].get('n',0):6.1f},{state[1].get('e',0):6.1f}) "
                    f"foll=({state[2].get('n',0):6.1f},{state[2].get('e',0):6.1f}) "
                    f"relayed={relay_count[0] + relayed}")

            time.sleep(0.05)

        # ---- land
        log("path done, landing")
        set_mode(lead, 9)
        set_mode(follow, 9)
        time.sleep(8)

        cols = ["t",
                "l_n", "l_e", "l_d", "l_vn", "l_ve", "l_vd", "l_roll", "l_pitch", "l_yaw", "l_mode",
                "f_n", "f_e", "f_d", "f_vn", "f_ve", "f_vd", "f_roll", "f_pitch", "f_yaw", "f_mode"]
        out_csv = os.path.join(HERE, "sim_log.csv")
        with open(out_csv, "w", newline="", encoding="utf-8") as f:
            w = csv.writer(f)
            w.writerow(cols)
            w.writerows(rows)
        log(f"wrote {out_csv} ({len(rows)} rows), relayed {relay_count[0] + relayed} FOLLOW_TARGET msgs")

    finally:
        for p in procs:
            p.terminate()
        time.sleep(1)
        for p in procs:
            if p.poll() is None:
                p.kill()


if __name__ == "__main__":
    sys.exit(main())







