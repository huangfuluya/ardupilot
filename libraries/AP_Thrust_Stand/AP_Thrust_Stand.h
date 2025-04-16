
#pragma once

 #ifndef AP_THRUST_STAND_ENABLED
    #define AP_THRUST_STAND_ENABLED 1
#endif
 #if AP_THRUST_STAND_ENABLED
 
 #include <AP_HAL/AP_HAL.h>
 #include <AP_Param/AP_Param.h>
//  #include "libmodbus/modbus-rtu.h"
 
 class AP_Thrust_Stand {
 public:
     AP_Thrust_Stand();
 
     /* Do not allow copies */
     CLASS_NO_COPY(AP_Thrust_Stand);
 
     static const struct AP_Param::GroupInfo var_info[];
    
     static AP_Thrust_Stand *get_singleton() { return _singleton; }

     void update();
 
 private:
 
     AP_HAL::UARTDriver *thrust_stand_uart;
 
     void init(void);

     void tick(void);

     void log_thrust_and_torque(void) const;

     AP_Int16 _rate;

     static AP_Thrust_Stand *_singleton;

     float _Fx;
     float _Fy;
     float _Fz;
     float _Mx;
     float _My;
     float _Mz;

     int8_t _port_num;
     uint32_t _baudrate;
     bool _setup_complete;

     bool check_uart(void);
 };

 namespace AP {
     AP_Thrust_Stand *thrust_stand();
 }  // namespace AP
 
 #endif  // AP_THRUST_STAND_ENABLED
 
 