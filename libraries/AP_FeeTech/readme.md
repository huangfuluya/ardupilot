Here is the driver's code for feetech bus servo.
here is the param meaning 
SERVO_FETH_CHAN: this is the first chan sending to feetech servo, zero is the first. the total number of chans is six.At the same time, you also need to change these six feetech servos id from "SERVO_FETH_CHAN"+1 to "SERVO_FETH_CHAN"+6. it is a little wield, but it is the history problem about first is zero or one. 
SerialProtocol_FEETECH=49:需要设置串口功能为49，波特率需要根据舵机进行修改，有的是115200，有的是100 0000。
rcx_options:178：使用这个遥控器通道来控制舵机回中，我在遥控器上设置的与解锁相同的开关，这样可以在上锁的时候自动回中。
