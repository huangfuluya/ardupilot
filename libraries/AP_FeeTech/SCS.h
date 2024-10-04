/*
 * SCS.h
 * 飞特串行舵机通信层协议程序
 * 日期: 2022.4.2
 * 作者: 
 */
#pragma once
#include "AP_FeeTech_config.h"

#if AP_FEETECH_ENABLED

#include "INST.h"
// using namespace AP_FEETECH;
class SCS{
public:
	SCS();
	SCS(uint8_t End);
	SCS(uint8_t End, uint8_t Level);
	int genWrite(uint8_t ID, uint8_t MemAddr, uint8_t *nDat, uint8_t nLen);//普通写指令
	int regWrite(uint8_t ID, uint8_t MemAddr, uint8_t *nDat, uint8_t nLen);//异步写指令
	int RegWriteAction(uint8_t ID = 0xfe);//异步写执行指令
	void syncWrite(uint8_t ID[], uint8_t IDN, uint8_t MemAddr, uint8_t *nDat, uint8_t nLen);//同步写指令
	int writeByte(uint8_t ID, uint8_t MemAddr, uint8_t bDat);//写1个字节
	int writeWord(uint8_t ID, uint8_t MemAddr, uint16_t wDat);//写2个字节
	int Read(uint8_t ID, uint8_t MemAddr, uint8_t *nData, uint8_t nLen);//读指令
	int readByte(uint8_t ID, uint8_t MemAddr);//读1个字节
	int readWord(uint8_t ID, uint8_t MemAddr);//读2个字节
	int Ping(uint8_t ID);//Ping指令
	int syncReadPacketTx(uint8_t ID[], uint8_t IDN, uint8_t MemAddr, uint8_t nLen);//同步读指令包发送
	int syncReadPacketRx(uint8_t ID, uint8_t *nDat);//同步读返回包解码，成功返回内存字节数，失败返回0
	int syncReadRxPacketToByte();//解码一个字节
	int syncReadRxPacketToWrod(uint8_t negBit=0);//解码两个字节，negBit为方向为，negBit=0表示无方向
	void syncReadBegin(uint8_t IDN, uint8_t rxLen, uint32_t TimeOut);//同步读开始
	void syncReadEnd();//同步读结束
	int Recovery(uint8_t ID);//恢复舵机参数为默认值
public:
	uint8_t _Level;//舵机返回等级
	uint8_t _End;//处理器大小端结构
	uint8_t Error;//舵机状态
	uint8_t syncReadRxPacketIndex;
	uint8_t syncReadRxPacketLen;
	uint8_t *syncReadRxPacket;
	uint8_t *syncReadRxBuff;
	uint16_t syncReadRxBuffLen;
	uint16_t syncReadRxBuffMax;
	uint32_t syncTimeOut;
protected:
	virtual int writeSCS(unsigned char *nDat, int nLen) = 0;
	virtual int readSCS(unsigned char *nDat, int nLen) = 0;
	virtual int readSCS(unsigned char *nDat, int nLen, unsigned long TimeOut) = 0;
	virtual int writeSCS(unsigned char bDat) = 0;
	virtual void rFlushSCS() = 0;
	virtual void wFlushSCS() = 0;
protected:
	void writeBuf(uint8_t ID, uint8_t MemAddr, uint8_t *nDat, uint8_t nLen, uint8_t Fun);
	void Host2SCS(uint8_t *DataL, uint8_t* DataH, uint16_t Data);//1个16位数拆分为2个8位数
	uint16_t	SCS2Host(uint8_t DataL, uint8_t DataH);//2个8位数组合为1个16位数
	int	Ack(uint8_t ID);//返回应答
	int checkHead();//帧头检测
};

#endif // AP_FEETECH_ENABLED