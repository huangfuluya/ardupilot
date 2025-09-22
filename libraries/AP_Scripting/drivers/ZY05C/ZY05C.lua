-- ZY05C.lua
-- write by huangluya at 2025-09-05

-- Load CAN driver with a buffer size of 20
local driver = CAN:get_device(20)
local init_done = false
local SERVO_ID_LIST = {0x11,0x12} -- 0~63,即：0x00~0x3F
local GCS_ID_LIST = {0x3F,0x3E} -- 0~63,即：0x00~0x3F
local SERVO_FUNCTION_LIST = {141,140}
local SERVO_ANGLE_MAX = {120,120}  -- 伺服最大角度，单位度，最大120度


-- ================= Helper functions for CAN Extended IDs =================
-- Highest bit (bit31) used as Extended Frame Flag by underlying driver.
local EXT_FLAG = uint32_t(1) << 31            -- 0x80000000
local EFF_MASK = uint32_t(0x1FFFFFFF)         -- 29-bit mask

-- Build an extended frame id from a full 29-bit numeric value (already composed)
local function make_ext_id(full29)
    return EXT_FLAG | uint32_t(full29)
end

-- Build extended id from high16 + low16 pieces (for readability in this spec)
local function make_ext_id_hilo(high16, low16)
    return EXT_FLAG | (uint32_t(high16) << 16) | uint32_t(low16)
end

-- Compose pattern 0xPPSSXXXX (P=prefix byte, S=servo id byte, XXXX=suffix 16 bits)
local function compose_29(prefix_byte, servo_id_byte, suffix16)
    return (uint32_t(prefix_byte) << 24) | (uint32_t(servo_id_byte) << 16) | uint32_t(suffix16)
end

-- Extract 29-bit effective id (remove flags)
local function eff(id)
    return id & EFF_MASK
end

-- Safe hex string for uint32_t userdata or plain number
local function u32_hex(u)
    local n = tonumber(u)
    if n then
        return string.format("0x%X", n)
    end
    -- Fallback: try tostring (may already be hex or decimal depending on binding)
    return tostring(u)
end
-- local servo_id = uint32_t(0x11)
-- local gcs_id = uint32_t(0x3F)

-- send the id set command
function set_id(servo_id)
    msg = CANFrame()

    -- Extended frame, node ID 0x143F4401
    msg:id(make_ext_id_hilo(0x143F, 0x4401))
    -- 先传低字节，再传高字节
    msg:data(0, 0x01) --0x01:设定设备ID值 0x02：读取产品当前ID值
    msg:data(1, 0x00)
    msg:data(2, servo_id)
    msg:data(3, 0x00)
    msg:data(4, 0x3F)
    msg:data(5, 0x00)
    msg:data(6, 0x00)
    msg:data(7, 0x00)

    msg:dlc(8)

    driver:write_frame(msg, uint32_t(10000))
end

function send_check_cmd()
    msg = CANFrame()
    msg:id(make_ext_id_hilo(0x143F, 0x4402))
    msg:dlc(8)
    driver:write_frame(msg, uint32_t(10000))
end

function send_angle(gcs_id, angle)
    msg = CANFrame()
    msg:id(make_ext_id_hilo(0x1400 | uint32_t(gcs_id), 0x4405))
    -- 先传低字节，再传高字节
    -- 角度值可能是浮点，需转整数并限制在 0~0xFFFF
    local a = angle
    if type(a) == 'number' then
        a = math.floor(a + 0.5) -- 四舍五入
        if a < 0 then a = 0 elseif a > 0xFFFF then a = 0xFFFF end
    end
    msg:data(0, a & 0xFF)
    msg:data(1, (a >> 8) & 0xFF)
    msg:data(2, 0x00)
    msg:data(3, 0x00)
    msg:data(4, 0x00)
    msg:data(5, 0x00)
    msg:data(6, 0x00)
    msg:data(7, 0x00)

    msg:dlc(8)

    driver:write_frame(msg, uint32_t(10000))
end

function receive_set_ack()
    -- Read a message from the buffer
    frame = driver:read_frame()
    if not frame then
        return false
    end

    local raw_id = frame:id()
    local eff_id = eff(raw_id)
    -- Debug print raw + effective
    gcs:send_text(0, string.format("recv raw=%s eff=%s", u32_hex(raw_id), u32_hex(eff_id)))

    -- Expected ack id 0x1411FC01 (29-bit value)
    if eff_id == uint32_t(0x1411FC01) then
        gcs:send_text(0, "Received set-id ack")
        if frame:data(0) == 0x01 then
            gcs:send_text(0, "Set ID success")
            return true
        else
            gcs:send_text(0, "Set ID failed")
        end
    end
    return false
end

function receive_check_ack(servo_id)
    frame = driver:read_frame()
    if not frame then
        return false
    end

    local eff_id = eff(frame:id())
    local expected = compose_29(0x14, servo_id, 0xFC02)
    if eff_id == expected then
        -- gcs:send_text(0, "Self-check ack id=" .. u32_hex(eff_id))
        if frame:data(0) == 0 then
            gcs:send_text(0, "Self-check passed")
            return true
        else
            -- Decode bit flags
            local b0 = frame:data(0)
            local b1 = frame:data(1)
            if b0 & 0x01 == 1 then gcs:send_text(0, "Self-check BIT error") end
            if (b0 >> 1) & 0x01 == 1 then gcs:send_text(0, "Self-check IO error") end
            if (b0 >> 2) & 0x01 == 1 then gcs:send_text(0, "Self-check CAN error") end
            if (b0 >> 3) & 0x01 == 1 then gcs:send_text(0, "Self-check AD error") end
            if (b0 >> 5) & 0x01 == 1 then gcs:send_text(0, "Self-check Timer error") end
            if (b0 >> 6) & 0x01 == 1 then gcs:send_text(0, "Self-check Control Voltage error") end
            if (b0 >> 7) & 0x01 == 1 then gcs:send_text(0, "Self-check Bus Voltage error") end
            if (b1 >> 0) & 0x01 == 1 then gcs:send_text(0, "Self-check Initial Position error") end
            if (b1 >> 1) & 0x01 == 1 then gcs:send_text(0, "Self-check Initial Current error") end
            return true
        end
    end
    return false
end

-- send the id set command
function receive_angle_ack(servo_id)
    frame = driver:read_frame()
    if not frame then
        return
    end

    local eff_id = eff(frame:id())
    local cmd_ack = compose_29(0x14, servo_id, 0xFC05)
    local angle_resp = compose_29(0x14, servo_id, 0xFC06)

    if eff_id == cmd_ack then
        -- gcs:send_text(0, "Angle cmd ack id=" .. u32_hex(eff_id))
        local angle = (frame:data(0) << 8) | frame:data(1)
        local current_angle = (frame:data(2) << 8) | frame:data(3)
        local current = (frame:data(4) << 8) | frame:data(5)
        local status = (frame:data(6) << 8) | frame:data(7)
        -- gcs:send_text(0, string.format("Angle cmd=0x%X curr=0x%X I=0x%X status=0x%X", angle, current_angle, current, status))
    elseif eff_id == angle_resp then
        local bus_voltage = (frame:data(0) << 8) | frame:data(1)
        -- gcs:send_text(0, string.format("Bus voltage=0x%X", bus_voltage))
    end
end

function init(servo_id)
    local flag_set = false
    local flag_check = false
    local time_stamps = millis()

    set_id(servo_id)
    while not flag_set and millis() - time_stamps < 1000 do
        -- wait for the ack
        flag_set = receive_set_ack()
    end

    send_check_cmd()
    time_stamps = millis()
    while not flag_check and millis() - time_stamps < 1000 do
        -- wait for the ack
        flag_check = receive_check_ack(servo_id)
    end

    if flag_set and flag_check then
        init_done = true
        gcs:send_text(0, "Init done")
    end
end

function update()
    -- if not init_done then
    -- -- don't init the servo at the real flight, unless you want to change the ID
    --     init(SERVO_ID_LIST)
    --     return update, 10000
    -- end
    -- 读取第一个通道的输出值

    -- local current_position = SRV_Channels:get_output_scaled(0)
    -- SRV_Channels:get_output_pwm(0)
    len = #SERVO_ID_LIST
    for i, servo_id in ipairs(SERVO_ID_LIST) do

        current_position = (SRV_Channels:get_output_pwm(SERVO_FUNCTION_LIST[i])-1000)/10 * SERVO_ANGLE_MAX[i]
        -- gcs:send_text(0, string.format("SRV_Channels:get_output_scaled=%f", SRV_Channels:get_output_pwm(140)))
        -- gcs:send_text(0, string.format("idx:%d;position=%f", i, current_position))
        send_angle(GCS_ID_LIST[i], current_position)
        -- 读取可能的角度反馈帧（防止缓冲堆积）
        for j = 1, 2 do
            receive_angle_ack(servo_id)
        end
    end
    return update, 10
end

return update(), 1000
