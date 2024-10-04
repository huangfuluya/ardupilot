/*
 * SCSerial.h
 * 飞特串行舵机硬件接口层程序
 * 日期: 2022.3.29
 * 作者: 
 */
#pragma once
#include "AP_FeeTech_config.h"

#if AP_FEETECH_ENABLED

#include "SCS.h"
#include <AP_HAL/AP_HAL.h>
class SCSerial : public SCS
{
public:
	SCSerial();
	SCSerial(uint8_t End);
	SCSerial(uint8_t End, uint8_t Level);

protected:
	int writeSCS(unsigned char *nDat, int nLen) override;//输出nLen字节
	int readSCS(unsigned char *nDat, int nLen) override;//输入nLen字节
	int readSCS(unsigned char *nDat, int nLen, unsigned long TimeOut) override;
	int writeSCS(unsigned char bDat) override;//输出1字节
	void rFlushSCS() override;//
	void wFlushSCS() override;//
public:
	unsigned long IOTimeOut;//输入输出超时
	AP_HAL::UARTDriver *pSerial;//串口指针
	int Err;
public:
	virtual int getErr(){  return Err;  }
};

#endif // AP_FEETECH_ENABLED
