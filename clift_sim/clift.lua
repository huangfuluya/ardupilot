-- clift.lua — Cooperative Lift coordination script (two-aircraft demo)
--
-- Runs on BOTH aircraft. CLIFT_ROLE selects the behaviour:
--   ROLE = 0 (lead)     : broadcast FOLLOW_TARGET at 10 Hz, carrying this
--                         aircraft's consumed charge in custom_state, and
--                         apply the acceleration lean to its own station.
--   ROLE = 1 (follower) : receive the lead's FOLLOW_TARGET, apply the
--                         acceleration lean + energy balancing to its own
--                         station, and monitor the R/2 deviation.
--
-- Geometry (2 aircraft):
--   The formation centre C is the midpoint of the two stations. CLIFT_D is
--   this aircraft's signed distance from C along the LEAD's heading
--   (+ = ahead of C). For a pair 10 m apart: lead CLIFT_D = +5, follower
--   CLIFT_D = -5 (swap the signs if the follower is the one in front).
--
-- Offset law (vertical only, along the effective gravity direction):
--   lean_down   = (d . a) / g          d = station vector from C, a = target accel
--   balance_down= -(P*(mean-own Ah) + D*(mean-own A))   clamped, + = climb
--   total_down  = lean_down + balance_down - CLIFT_TRIM
--
-- References (stock ArduPilot master APIs only):
--   poscontrol:set_posvelaccel_offset / get_accel_target
--   follow:get_target_heading_deg / get_target_location_and_velocity_ofs
--   mavlink:init / register_rx_msgid / receive_chan / send_chan
--
-- CAUTION: offsets expire 3 s after the last set_posvelaccel_offset() call,
--          so this script must keep running. If it stops, the aircraft
--          silently returns to its nominal path.

local MAV_SEVERITY = {EMERGENCY=0, ALERT=1, CRITICAL=2, ERROR=3, WARNING=4, NOTICE=5, INFO=6, DEBUG=7}

local UPDATE_MS        = 100      -- 10 Hz
local GOSSIP_TIMEOUT_MS= 3000     -- lead battery considered stale after this
local R2_HOLD_MS       = 500      -- deviation must persist this long
local G                 = 9.81
local COPTER_MODE_AUTO   = 3
local COPTER_MODE_FOLLOW = 23
local COPTER_MODE_GUIDED = 4
local FOLLOW_TARGET_ID = 144

-- ---------------------------------------------------------------- parameters
local PARAM_TABLE_KEY = 185
local PARAM_TABLE_PREFIX = "CLIFT_"
assert(param:add_table(PARAM_TABLE_KEY, PARAM_TABLE_PREFIX, 17), 'clift: could not add param table')

local function add_param(name, idx, default_value)
    assert(param:add_param(PARAM_TABLE_KEY, idx, name, default_value),
           string.format('clift: could not add param %s%s', PARAM_TABLE_PREFIX, name))
    return Parameter(PARAM_TABLE_PREFIX .. name)
end

-- @Param: ENABLE  master switch
local ENABLE   = add_param("ENABLE",    1, 0)
-- @Param: ROLE    0 = lead, 1 = follower
local ROLE     = add_param("ROLE",      2, 1)
-- @Param: D       station distance from swarm centre along lead heading (m, + = ahead)
local D_M      = add_param("D",         3, -5.0)
-- @Param: LEAN    apply the acceleration lean
local LEAN     = add_param("LEAN",      4, 1)
-- @Param: BAL     energy balancing (follower only)
local BAL      = add_param("BAL",       5, 0)
-- @Param: BAL_P   balance P gain (m per Ah of consumed-charge error)
local BAL_P    = add_param("BAL_P",     6, 0.5)
-- @Param: BAL_D   balance D gain (m per A of instantaneous-current error)
local BAL_D    = add_param("BAL_D",     7, 0.05)
-- @Param: BAL_MAX balance offset clamp (m)
local BAL_MAX  = add_param("BAL_MAX",   8, 2.0)
-- @Param: TRIM    manual vertical trim (m, + = climb)
local TRIM     = add_param("TRIM",      9, 0.0)
-- @Param: R2      deviation monitor threshold (m)
local R2       = add_param("R2",       10, 2.5)
-- @Param: R2_ACT  0 = warn only, 1 = warn and force GUIDED
local R2_ACT   = add_param("R2_ACT",   11, 0)
-- @Param: GOSSIP  receive the lead's battery state
local GOSSIP   = add_param("GOSSIP",   12, 0)
-- @Param: MAV_CHAN MAVLink channel used to send FOLLOW_TARGET (lead only)
local MAV_CHAN = add_param("MAV_CHAN", 13, 0)
-- @Param: GOSSIP_OFS artificial charge offset added to the reported battery
--        state (mAh), for testing the balance loop on the bench
local GOSSIP_OFS = add_param("GOSSIP_OFS", 14, 0)
-- @Param: DAMP_P  formation levelling gain (m per m of altitude difference
--        to the peer), 0 = off.  The lower aircraft carries more load, so it
--        climbs to shed load and pull the formation level.
local DAMP_P   = add_param("DAMP_P",   15, 0.3)
-- @Param: DAMP_D  formation levelling damping (m per m/s of relative
--        vertical speed)
local DAMP_D   = add_param("DAMP_D",   16, 0.3)
-- @Param: DAMP_MAX levelling offset clamp (m)
local DAMP_MAX = add_param("DAMP_MAX", 17, 1.0)

-- ---------------------------------------------------------------- mavlink
-- NOTE: deliberately no require("MAVLink/mavlink_msgs") here. Parsing that
-- module at load time exceeds the default SCR_VM_I_COUNT budget on some
-- builds and blocks arming with "exceeded time limit". Both the FOLLOW_TARGET
-- payload (string.pack) and custom_state (string.unpack) are handled directly.
local FOLLOW_TARGET_CAPABILITIES = {POS=2^0, VEL=2^1, ACCEL=2^2, ATT_RATES=2^3}

-- ---------------------------------------------------------------- state
local offsets_active = false
local last_gossip_ms = 0
local last_gossip_text_ms = 0
local last_status_ms = 0
local peer_consumed_mah = nil
local peer_current_a = nil
local peer_vd = nil
local peer_alt = nil
local r2_start_ms = 0
local r2_warned = false
local rx_ready = false

-- ---------------------------------------------------------------- helpers
local function is_follower()
    return ROLE:get() >= 1
end

-- initialise the scripting MAVLink RX buffer once (both roles receive the
-- peer's state, the lead uses it for the relative-velocity damper)
local function ensure_rx()
    if rx_ready then
        return
    end
    mavlink:init(5, 1)
    mavlink:register_rx_msgid(FOLLOW_TARGET_ID)
    rx_ready = true
end

-- the lead only flies its offsets in AUTO/GUIDED; RTL/LAND must be left alone
local function lead_mode_ok()
    local mode = vehicle:get_mode()
    return mode == COPTER_MODE_AUTO or mode == COPTER_MODE_GUIDED
end

local function heading_rad()
    if is_follower() then
        local hdg = follow:get_target_heading_deg()
        if hdg == nil then
            return nil
        end
        return math.rad(hdg)
    end
    return ahrs:get_yaw_rad()
end

local function send_follow_target()
    local curr_loc = ahrs:get_location()
    if curr_loc == nil then
        return
    end
    local capabilities = FOLLOW_TARGET_CAPABILITIES.POS

    local vel_target_NED = poscontrol:get_vel_target()
    if vel_target_NED ~= nil then
        capabilities = capabilities + FOLLOW_TARGET_CAPABILITIES.VEL
    else
        vel_target_NED = Vector3f()
    end

    local accel_target_NED = poscontrol:get_accel_target()
    if accel_target_NED ~= nil then
        capabilities = capabilities + FOLLOW_TARGET_CAPABILITIES.ACCEL
    else
        accel_target_NED = Vector3f()
    end

    local attitude_quat = ahrs:get_quaternion()
    local curr_rot_rate = ahrs:get_gyro()
    if attitude_quat ~= nil and curr_rot_rate ~= nil then
        capabilities = capabilities + FOLLOW_TARGET_CAPABILITIES.ATT_RATES
    else
        attitude_quat = Quaternion()
        curr_rot_rate = Vector3f()
    end
    local curr_rot_rate_NED = ahrs:body_to_earth(curr_rot_rate)

    -- custom_state carries this vehicle's energy state plus its vertical
    -- velocity, so the peer can balance energy and damp the formation's
    -- vertical oscillation without a second telemetry stream.
    -- NOTE: AP_Scripting's Lua uses 32 bit integers, so the value must stay
    -- below 2^31:  consumed_mAh (15 bits) | current_A (8 bits) | v_d cm/s+128 (8 bits)
    local consumed_mah = math.floor((battery:consumed_mah(0) or 0) + GOSSIP_OFS:get())
    if consumed_mah > 32767 then consumed_mah = 32767 end
    local current_a = math.floor(battery:current_amps(0) or 0)
    if current_a > 255 then current_a = 255 elseif current_a < 0 then current_a = 0 end
    local vel_ned = ahrs:get_velocity_NED()
    local vd_cms = 0
    if vel_ned ~= nil then
        vd_cms = math.floor(vel_ned:z() * 100)
        if vd_cms > 127 then vd_cms = 127 elseif vd_cms < -128 then vd_cms = -128 end
    end
    local custom_state = consumed_mah * 65536 + current_a * 256 + (vd_cms + 128)

    local msg = {}
    msg.timestamp = millis():toint()
    msg.est_capabilities = capabilities
    msg.lat = curr_loc:lat()
    msg.lon = curr_loc:lng()
    msg.alt = curr_loc:alt() * 0.01
    msg.vel = {vel_target_NED:x(), vel_target_NED:y(), vel_target_NED:z()}
    msg.acc = {accel_target_NED:x(), accel_target_NED:y(), accel_target_NED:z()}
    msg.attitude_q = {attitude_quat:q1(), attitude_quat:q2(), attitude_quat:q3(), attitude_quat:q4()}
    msg.rates = {curr_rot_rate_NED:x(), curr_rot_rate_NED:y(), curr_rot_rate_NED:z()}
    msg.position_cov = {0, 0, 0}
    msg.custom_state = custom_state

    -- pack FOLLOW_TARGET by hand: pulling in the MAVLink Lua module costs more
    -- VM instructions at load time than the default scripting budget allows
    local payload = string.pack("<I8I8i4i4" .. string.rep("f", 17) .. "B",
        msg.timestamp, custom_state, msg.lat, msg.lon,
        msg.alt,
        msg.vel[1], msg.vel[2], msg.vel[3],
        msg.acc[1], msg.acc[2], msg.acc[3],
        msg.attitude_q[1], msg.attitude_q[2], msg.attitude_q[3], msg.attitude_q[4],
        msg.rates[1], msg.rates[2], msg.rates[3],
        msg.position_cov[1], msg.position_cov[2], msg.position_cov[3],
        capabilities)
    mavlink:send_chan(MAV_CHAN:get(), FOLLOW_TARGET_ID, payload)
end

-- extract custom_state (and the peer's altitude) from a raw mavlink_message_t
-- without the generic Lua decoder: the decoder loops over every field and can
-- blow the scripting instruction budget when messages arrive in bursts
local function gossip_custom_state(msg)
    local magic = string.byte(msg, 3)
    if magic ~= 0xFD and magic ~= 0xFE then
        return nil, nil
    end
    if string.unpack("<I3", msg, 10) ~= FOLLOW_TARGET_ID then
        return nil, nil
    end
    -- FOLLOW_TARGET payload starts at byte 13: timestamp (u64), custom_state
    -- (u64), lat (i32), lon (i32), alt (f32) at payload offset 24
    return string.unpack("<I8", msg, 21), string.unpack("<f", msg, 37)
end

local function receive_gossip()
    -- bounded drain: the scripting VM has an instruction budget per call, so
    -- never loop until the RX buffer is empty (a burst can trip the limit)
    for _ = 1, 4 do
        local msg = mavlink:receive_chan()
        if msg == nil then
            return
        end
        local cs, alt = gossip_custom_state(msg)
        if cs ~= nil and cs > 0 then
            local now = millis()
            -- integer maths only (Lua ints are 32 bit in AP_Scripting)
            peer_vd = (cs % 256 - 128) / 100.0
            peer_current_a = (cs // 256) % 256
            peer_consumed_mah = cs // 65536
            peer_alt = alt
            last_gossip_ms = now
            if now - last_gossip_text_ms > 5000 then
                last_gossip_text_ms = now
                gcs:send_text(6, string.format("clift gossip: peer %.0f mAh %.0f A %+.2f m/s",
                                               peer_consumed_mah, peer_current_a, peer_vd))
            end
        end
    end
end

local function clear_offsets()
    if offsets_active then
        poscontrol:set_posvelaccel_offset(Vector3f(), Vector3f(), Vector3f())
        offsets_active = false
        gcs:send_text(MAV_SEVERITY.INFO, "clift: offsets cleared")
    end
end

-- ---------------------------------------------------------------- main loop
local function update()
    if ENABLE:get() <= 0 then
        clear_offsets()
        return update, 1000
    end

    -- the lead broadcasts its state; the follower consumes it for the energy
    -- balance and the relative-velocity damper
    if not is_follower() then
        send_follow_target()
    else
        ensure_rx()
        receive_gossip()
    end

    -- only fly the offsets while actually flying and not taking off/landing.
    -- NOTE: a vehicle switched from GUIDED take-off into FOLLOW keeps the
    -- take-off state machine "running", so is_taking_off() stays true forever
    -- and would block the offsets; ignore the flag once in FOLLOW mode.
    local taking_off = vehicle:is_taking_off()
    if is_follower() and vehicle:get_mode() == COPTER_MODE_FOLLOW then
        taking_off = false
    end
    if not arming:is_armed() or not vehicle:get_likely_flying()
       or taking_off or vehicle:is_landing() then
        clear_offsets()
        return update, UPDATE_MS
    end

    -- follower must actually be following something
    if is_follower() then
        if not follow:have_target() or vehicle:get_mode() ~= COPTER_MODE_FOLLOW then
            clear_offsets()
            return update, UPDATE_MS
        end
    elseif not lead_mode_ok() then
        clear_offsets()
        return update, UPDATE_MS
    end

    local hdg = heading_rad()
    local accel = poscontrol:get_accel_target()
    if hdg == nil or accel == nil then
        return update, UPDATE_MS
    end

    -- station vector from the formation centre along the lead heading
    local d = D_M:get()
    local dN = d * math.cos(hdg)
    local dE = d * math.sin(hdg)

    -- 1) acceleration lean: leading stations descend when accelerating forward
    local lean_down = 0.0
    if LEAN:get() > 0 then
        lean_down = (dN * accel:x() + dE * accel:y()) / G
    end

    -- 2) energy balance (follower): more energy than the pair's mean -> climb
    local balance_down = 0.0
    local fresh = peer_consumed_mah ~= nil and (millis() - last_gossip_ms) < GOSSIP_TIMEOUT_MS
    if is_follower() and BAL:get() > 0 and GOSSIP:get() > 0 and fresh then
        local own_mah = battery:consumed_mah(0) or 0
        local own_a = battery:current_amps(0) or 0
        local mean_mah = 0.5 * (own_mah + peer_consumed_mah)
        local mean_a = 0.5 * (own_a + peer_current_a)
        local m = BAL_P:get() * ((mean_mah - own_mah) / 1000.0) + BAL_D:get() * (mean_a - own_a)
        m = math.max(-BAL_MAX:get(), math.min(BAL_MAX:get(), m))
        balance_down = -m          -- positive m means "climb and take more load"
    end

    -- 3) active load balance: damp the formation's vertical oscillation.
    -- Every aircraft damps its own vertical velocity (this kills the common
    -- bobbing that the tether/payload resonance drives through both position
    -- controllers), and optionally the formation is levelled against the
    -- peer's altitude (the lower aircraft carries more load).
    local damp_down = 0.0
    local own_vel = ahrs:get_velocity_NED()
    if DAMP_D:get() > 0 and own_vel ~= nil then
        damp_down = -DAMP_D:get() * own_vel:z()
    end
    if is_follower() and DAMP_P:get() > 0 and fresh and peer_alt ~= nil then
        local own_loc = ahrs:get_location()
        if own_loc ~= nil then
            local dalt = own_loc:alt() * 0.01 - peer_alt      -- + = this one higher
            damp_down = damp_down + DAMP_P:get() * dalt
        end
    end
    damp_down = math.max(-DAMP_MAX:get(), math.min(DAMP_MAX:get(), damp_down))

    -- 4) apply as a pure vertical offset (NED: +Z is down)
    local total_down = lean_down + balance_down + damp_down - TRIM:get()
    local offset = Vector3f()
    offset:z(total_down)
    if poscontrol:set_posvelaccel_offset(offset, Vector3f(), Vector3f()) then
        offsets_active = true
    end

    -- periodic status so the cooperative loop is visible in the GCS/logs
    local now_ms = millis()
    if now_ms - last_status_ms > 5000 then
        last_status_ms = now_ms
        if is_follower() and fresh then
            gcs:send_text(6, string.format("clift: %.0f/%.0fmAh %.0f/%.0fA L%+.2f B%+.2f D%+.2f",
                battery:consumed_mah(0) or 0, peer_consumed_mah,
                battery:current_amps(0) or 0, peer_current_a,
                -lean_down, -balance_down, -damp_down))
        else
            gcs:send_text(6, string.format("clift: lead L%+.2f D%+.2f", -lean_down, -damp_down))
        end
    end

    -- 4) R/2 deviation monitor (follower): target vs actual position
    if is_follower() then
        local target_loc = follow:get_target_location_and_velocity_ofs()
        local own_loc = ahrs:get_location()
        if target_loc ~= nil and own_loc ~= nil then
            local dv = own_loc:get_distance_NED(target_loc)
            local err = math.sqrt(dv:x() * dv:x() + dv:y() * dv:y() + dv:z() * dv:z())
            if err > R2:get() then
                if r2_start_ms == 0 then
                    r2_start_ms = millis()
                elseif (millis() - r2_start_ms) > R2_HOLD_MS and not r2_warned then
                    r2_warned = true
                    gcs:send_text(MAV_SEVERITY.CRITICAL,
                        string.format("clift: R/2 exceeded %.1fm", err))
                    if R2_ACT:get() >= 1 then
                        clear_offsets()
                        vehicle:set_mode(COPTER_MODE_GUIDED)
                    end
                end
            else
                if r2_warned then
                    gcs:send_text(MAV_SEVERITY.INFO, "clift: R/2 recovered")
                end
                r2_start_ms = 0
                r2_warned = false
            end
        end
    end

    return update, UPDATE_MS
end

gcs:send_text(MAV_SEVERITY.INFO, "clift.lua loaded")

return update()

