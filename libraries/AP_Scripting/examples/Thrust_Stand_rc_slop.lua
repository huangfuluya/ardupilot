-- move a servo in a sinisoidal fashion
local max_time_s = 30.0   --斜坡持续时间
local back_flag = true      --是否反向
local SERVO_FUNCTION = 94  --输出通道功能号
local sw = rc:find_channel_for_option(300)  --触发开关通道功能号
local RC_over = rc:get_channel(4)
local overide_rc_chan = 4
local PWM_min = 1000
local PWM_max = 2000


local e_stop = rc:find_channel_for_option(31)
local rc_trig_flag = false


local last_loop_time = 0
local trig_time_stamp = 0
function update() -- this is the loop which periodically runs
    -- 先检查是否有通道功能被设置为script1和rc的功能被设置成script1
    local armed_flag = false
    local e_stop_flag = false

    local over_time_flag = false
    local error_type = 0
    local over_time = 0
    local overide_pwm = rc:get_pwm(overide_rc_chan)
    if arming:is_armed() then
        -- if armed then motors are spinning
        armed_flag = true
    end
    -- 如果e_stop为空
    if e_stop then
        -- if E-stop switch is setup
        if e_stop:get_aux_switch_pos() == 2 then
            -- E-stop on, motors stopped
            e_stop_flag = true
        end
    end

    if sw then
        if rc_trig_flag == false and sw:get_aux_switch_pos() == 1 then
            -- E-stop on, motors stopped
            rc_trig_flag = true
            trig_time_stamp = millis():tofloat()*0.001
            gcs:send_text(0, "begin output")
            -- 打印trig_time_stamp
            -- gcs:send_text(0, string.format("trig_time_stamp=%.2f", (trig_time_stamp)))
            over_time_flag = false -- reset over_time_flag
        elseif sw:get_aux_switch_pos() < 1 then
            rc_trig_flag = false
            trig_time_stamp = 0
            over_time_flag = false -- reset over_time_flag
        end
    else
        error_type = 1
    end

    if armed_flag == true and e_stop_flag == false and rc_trig_flag == true and over_time_flag == false then
        over_time = (millis():tofloat()*0.001 - trig_time_stamp)
        -- gcs:send_text(0, string.format("millis=%.2f", millis():tofloat()*0.001))
        -- gcs:send_text(0, string.format("trig_time_stamp=%.2f", trig_time_stamp))
        -- gcs:send_text(0, string.format("over_time=%.2f", over_time))
        if back_flag == false then
            if over_time >= max_time_s then
                over_time = max_time_s
                over_time_flag = true
            end
            overide_pwm = math.floor((over_time / max_time_s) * (PWM_max - PWM_min) + PWM_min)
            RC_over:set_override(overide_pwm)
            -- SRV_Channels:set_output_norm(SERVO_FUNCTION, (over_time / max_time_s*2.0-1.0))
            
        end

        if back_flag == true then
            if over_time >= 2 * max_time_s then
                over_time = 2 * max_time_s
                over_time_flag = true
            end
            if over_time >= max_time_s and over_time <= 2 * max_time_s then
                -- SRV_Channels:set_output_norm(SERVO_FUNCTION, ((2.0 * max_time_s - over_time) / max_time_s*2.0-1.0))
                overide_pwm = math.floor(((2.0 * max_time_s - over_time) / max_time_s) * (PWM_max - PWM_min) + PWM_min)
            else
                -- SRV_Channels:set_output_norm(SERVO_FUNCTION, (over_time / max_time_s*2-1))
                overide_pwm = math.floor((over_time / max_time_s) * (PWM_max - PWM_min) + PWM_min)
                -- SRV_Channels:set_output_pwm_chan_timeout(SERVO_FUNCTION, 1500, 100)
            end
            RC_over:set_override(overide_pwm)
        end
    else
        SRV_Channels:set_output_norm(SERVO_FUNCTION, -1.0)
    end
    if millis():tofloat()*0.001-last_loop_time>8 then
        last_loop_time = millis():tofloat()*0.001
        if error_type==1 then
            gcs:send_text(0, "error_type=1")
        else
            gcs:send_text(0, "script running")
            -- 打印armed_flag
            -- gcs:send_text(0, string.format("armed_flag=%s", tostring(armed_flag)))
            -- gcs:send_text(0, string.format("e_stop_flag=%s", tostring(e_stop_flag)))
            -- gcs:send_text(0, string.format("rc_trig_flag=%s", tostring(rc_trig_flag)))
            -- gcs:send_text(0, string.format("over_time_flag=%s", tostring(over_time_flag)))
            -- gcs:send_text(0, string.format("over_time=%d", over_time))
        end
    end
    return update, 10 -- reschedules the loop at 100Hz
end

return update() -- run immediately before starting to reschedule
