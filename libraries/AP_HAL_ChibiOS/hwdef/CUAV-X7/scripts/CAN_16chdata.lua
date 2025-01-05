 -- 读取飞控状态和通道，发送自定义CAN消息
 -- 加载 CAN 驱动程序，使用脚本协议，缓冲区大小为5
 local driver = CAN:get_device(5)
 -- 初始化解锁变量
 local arm = 0
 -- 调用系统循环
 function update()

 
   msg = CANFrame()

   -- 获取16个通道PWM值
   local pwm1, pwm2, pwm3, pwm4 = rc:get_pwm(1), rc:get_pwm(2), rc:get_pwm(3), rc:get_pwm(4)
   local pwm5, pwm6, pwm7, pwm8 = rc:get_pwm(5), rc:get_pwm(6), rc:get_pwm(7), rc:get_pwm(8)
   local pwm9, pwm10, pwm11, pwm12 = rc:get_pwm(9), rc:get_pwm(10), rc:get_pwm(11), rc:get_pwm(12)
   local pwm13, pwm14, pwm15, pwm16 = rc:get_pwm(13), rc:get_pwm(14), rc:get_pwm(15), rc:get_pwm(16)
 
 
   local servoh1 = pwm1  >> 8     -- 提取高8位 
   local servol1 = pwm1  & 0xFF   -- 提取低8位
   local servoh2 = pwm2  >> 8      
   local servol2 = pwm2  & 0xFF   
   local servoh3 = pwm3  >> 8      
   local servol3 = pwm3  & 0xFF  
   local servoh4 = pwm4  >> 8      
   local servol4 = pwm4  & 0xFF 
   local servoh5 = pwm5  >> 8     
   local servol5 = pwm5  & 0xFF   
   local servoh6 = pwm6  >> 8      
   local servol6 = pwm6  & 0xFF   
   local servoh7 = pwm7  >> 8      
   local servol7 = pwm7  & 0xFF  
   local servoh8 = pwm8  >> 8      
   local servol8 = pwm8  & 0xFF 
   local servoh9 = pwm9  >> 8     
   local servol9 = pwm9  & 0xFF   
   local servoh10 = pwm10  >> 8      
   local servol10 = pwm10  & 0xFF   
   local servoh11 = pwm11  >> 8      
   local servol11 = pwm11  & 0xFF  
   local servoh12 = pwm12  >> 8      
   local servol12 = pwm12  & 0xFF 
   local servoh13 = pwm13  >> 8     
   local servol13 = pwm13  & 0xFF   
   local servoh14 = pwm14  >> 8      
   local servol14 = pwm14  & 0xFF   
   local servoh15 = pwm15 >> 8      
   local servol15 = pwm15  & 0xFF  
   local servoh16 = pwm16  >> 8      
   local servol16 = pwm16  & 0xFF 

   --第一帧数据ID=0x0A  发送飞控状态

   msg:id( uint32_t(10) )          

   local mode = vehicle:get_mode()    --获取飞行模式
   local gpsStatus = gps:status(0)    --获取RTK状态
   local gpsm = gps:num_sats(0)       --获取卫星数
    if arming:is_armed() then         --判断解锁状态
      arm = 0xAA
    else
     arm = 0xDD
    end

   msg:data(0,arm)                  --赋值解锁状态字节
   msg:data(1, mode)                --赋值飞行模式字节
   msg:data(2, gpsStatus)           --赋值RTK状态字节
   msg:data(4, gpsm)                --赋值卫星数量
   
   msg:dlc(8)                       --发送字节数

   driver:write_frame(msg, 10000)   --编写具有10000超时的帧

   --第二帧数据ID=0x0B  发送1~4通道PWM值
   msg:id( uint32_t(11) )         

   msg:data(0, servoh1)             
   msg:data(1, servol1)
   msg:data(2, servoh2)
   msg:data(3, servol2)
   msg:data(4, servoh3)
   msg:data(5, servol3)
   msg:data(6, servoh4)
   msg:data(7, servol4)

   msg:dlc(8)                     

   driver:write_frame(msg, 10000)

   --第三帧数据ID=0x0C  发送5~8通道PWM值

   msg:id( uint32_t(12) )

   msg:data(0, servoh5) 
   msg:data(1, servol5)
   msg:data(2, servoh6)
   msg:data(3, servol6)
   msg:data(4, servoh7)
   msg:data(5, servol7)
   msg:data(6, servoh8)
   msg:data(7, servol8)

   msg:dlc(8)      

   driver:write_frame(msg, 10000)

   --第四帧数据ID=0x0D  发送9~12通道PWM值

   msg:id( uint32_t(13) )
 
   msg:data(0, servoh9) 
   msg:data(1, servol9)
   msg:data(2, servoh10)
   msg:data(3, servol10)
   msg:data(4, servoh11)
   msg:data(5, servol11)
   msg:data(6, servoh12)
   msg:data(7, servol12)
 
   msg:dlc(8)      
 
   driver:write_frame(msg, 10000)

   --第五帧数据ID=0x0E  发送13~16通道PWM值

   msg:id( uint32_t(14) )
  
    msg:data(0, servoh13) 
    msg:data(1, servol13)
    msg:data(2, servoh14)
    msg:data(3, servol14)
    msg:data(4, servoh15)
    msg:data(5, servol15)
    msg:data(6, servoh16)
    msg:data(7, servol16)
  
    msg:dlc(8)      
  
    driver:write_frame(msg, 10000)
    gcs:send_text(0,string.format("CAN msg sent"))


  return update, 50    --延时20ms

end

return update()
