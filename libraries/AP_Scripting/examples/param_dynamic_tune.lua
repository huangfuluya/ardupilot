--  用于动态的修改参数值
--  指定映射通道，并指定最小最大的范围

local scripting_rc_1 = rc:find_channel_for_option(300)
local param_name1 = 'BF_D0SP'
local min_value1 = -0.5
local max_value1 = 0.5

local scripting_rc_2 = rc:find_channel_for_option(301)
local param_name2 = 'BF_ABAS'
local min_value2 = 0.2
local max_value2 = 1.2

function update() -- this is the loop which periodically runs

  -- get and print all the scripting parameters
    local value = param:get('SCR_ENABLE')
    if scripting_rc_1 then
        -- gcs:send_text(0, "Scripting in 1:" .. tostring(scripting_rc_1:norm_input()))
        param:set(param_name1, scripting_rc_1:norm_input() * (max_value1 - min_value1) + min_value1)
        gcs:send_text(0, "Set " .. param_name1 .. " to " .. tostring(scripting_rc_1:norm_input() * (max_value1 - min_value1) + min_value1))
    end
    if scripting_rc_2 then
        -- gcs:send_text(0, "Scripting in 2:" .. tostring(scripting_rc_2:norm_input()))
        param:set(param_name2, scripting_rc_2:norm_input() * (max_value2 - min_value2) + min_value2)
        gcs:send_text(0, "Set " .. param_name2 .. " to " .. tostring(scripting_rc_2:norm_input() * (max_value2 - min_value2) + min_value2))
    end

  return update, 2000 -- reschedules the loop
end

return update() -- run immediately before starting to reschedule
