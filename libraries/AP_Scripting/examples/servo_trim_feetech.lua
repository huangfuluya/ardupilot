
-- you need set a rcX_options to 300. before pluging the servo, you should trim the servo to the middle position.
-- this script can run lonely.
local K_SCRIPTING1 = 300
local servo_trim_channel = rc:find_channel_for_option(300)
local chan_num = 6
feth_chan = param:get('SERVO_FETH_CHAN')
if not feth_chan then
  gcs:send_text(6, 'get SERVO_FETH_CHAN failed')
end
function send_trim_pwm(begain_chan,chan_num,trim_value,timeout)
  for i = begain_chan,chan_num do
    SRV_Channels:set_output_pwm_chan_timeout(i, trim_value, timeout)
  end
end

function update()
    if not servo_trim_channel then -- 判断是否有相应的设置，没有的话就直接让舵机归中
        send_trim_pwm(feth_chan,chan_num,1500,1200)
    elseif servo_trim_channel:get_aux_switch_pos()==0 then
        send_trim_pwm(feth_chan,chan_num,1500,1200)
    end

    return update, 1000
end

return update, 1000
