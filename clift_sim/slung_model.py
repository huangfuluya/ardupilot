"""Python port of the SITL slung-payload model (SIM_SlungPayload.cpp).

The C++ model integrates the payload with a semi-implicit Euler step and
computes a tension force from: gravity component along the line, velocity
difference (2 s time constant), drag component and the centripetal term.
Ported here so the offline reconstruction and the SITL simulation use exactly
the same dynamics; the same formulation is then extended to two lines for the
cooperative-lift model.
"""
import numpy as np

G = 9.81
RHO = 1.225
AREA = 0.07          # 30 cm sphere
VEL_TO_ACCEL_TC = 2.0


def _proj(v, n):
    return np.dot(v, n) * n


def simulate(anchor_pos, anchor_vel, t, line_length=10.0, weight_kg=1.0,
             drag_coef=1.0, dt=0.0025, init_state=None):
    """Integrate the payload.

    anchor_pos/vel: (N,3) arrays in NED sampled at t; payload position is in
    the same NED frame.  Returns (tt, payload_pos, payload_vel, tension_ratio).
    """
    tt = np.arange(t[0], t[-1], dt)
    ap = np.column_stack([np.interp(tt, t, anchor_pos[:, k]) for k in range(3)])
    av = np.column_stack([np.interp(tt, t, anchor_vel[:, k]) for k in range(3)])

    if init_state is None:
        p = ap[0].copy()
        v = np.zeros(3)
    else:
        p, v = init_state[0].copy(), init_state[1].copy()

    gravity = np.array([0.0, 0.0, G]) * weight_kg
    landed = True
    acc = np.zeros(3)
    out_p = np.zeros((len(tt), 3))
    out_v = np.zeros((len(tt), 3))
    out_t = np.zeros(len(tt))

    for i in range(len(tt)):
        # integrate the previous step's acceleration
        v = v + acc * dt
        p = p + v * dt

        # ground: with no terrain in SITL the payload rests at the anchor's
        # home altitude (z = 0 in NED); the vehicle climbs, z becomes negative
        if p[2] > 0.0:
            landed = True
            p[2] = 0.0
            v[2] = min(v[2], 0.0)
            v[0] = 0.0
            v[1] = 0.0
            acc[:] = 0.0

        pv = ap[i] - p
        d = float(np.linalg.norm(pv))
        # the line pulls the payload off the ground once the vehicle is more
        # than ~1 m above it and the line is taut
        if landed and pv[2] < -1.0 and d >= line_length - 0.05:
            landed = False
        elif landed and p[2] < -0.01:
            landed = False

        tension = np.zeros(3)
        ratio = 0.0
        if d > 1e-6:
            pv_norm = pv / d
            if d > line_length:
                pv = pv * (line_length / d)
                p = ap[i] - pv
            ratio = float(np.clip(1.0 - (line_length - d) * 10.0, 0.0, 1.0))
            if ratio > 0.0:
                if pv_norm[2] < 0.0:                      # vehicle above payload
                    tension += -_proj(gravity, pv_norm)
                vdiff = av[i] - v
                vdiff_p = _proj(vdiff, pv_norm)
                if np.dot(vdiff_p, pv_norm) > 0.0 and np.linalg.norm(vdiff_p) > 0:
                    tension += vdiff_p / VEL_TO_ACCEL_TC * weight_kg
                vpar = _proj(v, pv_norm)
                vperp = v - vpar
                tension += pv_norm * (np.dot(vperp, vperp) * weight_kg / line_length)
                tension *= ratio

        # drag on the payload
        drag = np.zeros(3)
        if drag_coef > 0 and not landed and np.linalg.norm(v) > 1e-6:
            f = 0.5 * drag_coef * RHO * np.dot(v, v) * AREA
            drag = -v / np.linalg.norm(v) * f

        # drag component along the line also loads the tether
        if ratio > 0.0 and np.linalg.norm(drag) > 0:
            tension += -_proj(drag, pv / np.linalg.norm(pv)) * ratio

        acc = (gravity + drag + tension) / weight_kg
        if landed:
            acc[2] = min(acc[2], 0.0)

        out_p[i] = p
        out_v[i] = v
        out_t[i] = ratio

    return tt, out_p, out_v, out_t


def payload_relative(tt, p, anchor_tt, anchor_pos):
    """Payload position relative to the vehicle, resampled onto tt."""
    a = np.column_stack([np.interp(tt, anchor_tt, anchor_pos[:, k]) for k in range(3)])
    return p - a
