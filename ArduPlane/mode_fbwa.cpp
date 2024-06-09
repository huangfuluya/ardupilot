#include "mode.h"
#include "Plane.h"

void ModeFBWA::update()
{
    // set nav_roll and nav_pitch using sticks
    plane.nav_roll_cd  = plane.channel_roll->norm_input() * plane.roll_limit_cd;
    plane.update_load_factor();
    float pitch_input = plane.channel_pitch->norm_input();
    if (pitch_input > 0) {
        plane.nav_pitch_cd = pitch_input * plane.aparm.pitch_limit_max*100;
    } else {
        plane.nav_pitch_cd = -(pitch_input * plane.pitch_limit_min*100);
    }
    plane.adjust_nav_pitch_throttle();
    plane.nav_pitch_cd = constrain_int32(plane.nav_pitch_cd, plane.pitch_limit_min*100, plane.aparm.pitch_limit_max.get()*100);
    if (plane.fly_inverted()) {
        plane.nav_pitch_cd = -plane.nav_pitch_cd;
    }
    if (plane.failsafe.rc_failsafe && plane.g.fs_action_short == FS_ACTION_SHORT_FBWA) {
        // FBWA failsafe glide
        plane.nav_roll_cd = 0;
        plane.nav_pitch_cd = 0;
        SRV_Channels::set_output_limit(SRV_Channel::k_throttle, SRV_Channel::Limit::MIN);
    }
    RC_Channel *chan = rc().find_channel_for_option(RC_Channel::AUX_FUNC::FBWA_TAILDRAGGER);
    if (chan != nullptr) {
        // check for the user enabling FBWA taildrag takeoff mode
        bool tdrag_mode = chan->get_aux_switch_pos() == RC_Channel::AuxSwitchPos::HIGH;
        if (tdrag_mode && !plane.auto_state.fbwa_tdrag_takeoff_mode) {
            if (plane.auto_state.highest_airspeed < plane.g.takeoff_tdrag_speed1) {
                plane.auto_state.fbwa_tdrag_takeoff_mode = true;
                plane.gcs().send_text(MAV_SEVERITY_WARNING, "FBWA tdrag mode");
            }
        }
    }

    if(g2.err_to_rate_enable){
        //获取侧滑角
        float beta = AP::ahrs().getSSA();
        float roll = AP::ahrs().get_roll();
        float yaw = AP::ahrs().get_yaw();
        float arspd_ms = AP::ahrs().groundspeed();      //没有空速的时候就用地速
        UNUSED_RESULT(AP::ahrs().airspeed_estimate(arspd_ms));
        update_yaw_rate_des(beta, roll, arspd_ms, yaw);
    }
}

void ModeFBWA::update_yaw_rate_des(float beta, float bank, float arspd_ms,float yaw)
{
//////////////////////////////// parameters //////////////////////////
    // const float beta_to_rate = 0.1f;//从侧滑角到偏航角速度的转换系数
    // const float rate_constrain = radians(45.0f);//偏航角速度的限制,deg/s^2
    // const float yaw_err_constrain = radians(30.0f);
    // const float err_to_rate = 0.1f;

    //从eeprom中读取参数，便于调试
    const float beta_to_rate = plane.g2.beta_to_rate;//从侧滑角到偏航角速度的转换系数
    const float rate_constrain = radians(plane.g2.rate_constrain_deg);//偏航角速度的限制,deg/s^2
    const float yaw_err_constrain = radians(plane.g2.yaw_err_constrain_deg);
    const float err_to_rate = plane.g2.err_to_rate;
//////////////////////////// end of parameters ///////////////////////////////////////////

    float yaw_rate_des = 0.0f;
    yaw_rate_des += GRAVITY_MSS/arspd_ms * tanf(bank);
    yaw_rate_des += beta_to_rate * beta;  //正负号待确定
    yaw_rate_des = constrain_float(yaw_rate_des, -rate_constrain, rate_constrain);

    //更新目标航向角度
    float this_time_ms = AP_HAL::millis();
    _yaw_des += yaw_rate_des * (this_time_ms - _last_time_ms) / 1000.0f; //dt默认为1/400
    _last_time_ms = this_time_ms;
    float yaw_err = wrap_PI(_yaw_des - yaw);
    yaw_err = constrain_float(yaw_err, -yaw_err_constrain, yaw_err_constrain);
    _yaw_des = yaw + yaw_err;
    yaw_rate_des = yaw_err * err_to_rate;
    _yaw_ctrl = yaw_rate_des / yaw_err_constrain;
    _yaw_ctrl = constrain_float(_yaw_ctrl, -1.0f, 1.0f);

    //debug
    mavlink_debug_vect_t msg;
    msg.x = _yaw_ctrl;
    msg.y = _yaw_des;
    msg.z = beta;
    memcpy(msg.name, "debug_msg", sizeof(msg.name));
    msg.time_usec = AP_HAL::millis() * 1000;
    // gcs().send_to_active_channels(MAVLINK_MSG_ID_DEBUG_VECT,(const char *)&msg);
    mavlink_msg_debug_vect_send_struct(MAVLINK_COMM_0, &msg);
}

bool ModeFBWA::_enter() 
{
    _last_time_ms = AP_HAL::millis();
    _yaw_des = 0.0f; //初始化时将其赋值为真实的航向角
    return true;
}; 

