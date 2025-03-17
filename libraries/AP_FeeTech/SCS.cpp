/*
 * SCS.cpp
 * 飞特串行舵机通信层协议程序
 * 日期: 2022.4.2
 * 作者: 
 */

#include "AP_FeeTech_config.h"

#if AP_FEETECH_ENABLED

#include <stddef.h>
#include "SCS.h"

SCS::SCS()
{
	_Level = 1;//除广播指令所有指令返回应答
	Error = 0;
}

SCS::SCS(uint8_t End)
{
	_Level = 1;
	this->_End = End;
	Error = 0;
}

SCS::SCS(uint8_t End, uint8_t Level)
{
	this->_Level = Level;
	this->_End = End;
	Error = 0;
}

//1个16位数拆分为2个8位数
//DataL为低位，DataH为高位
void SCS::Host2SCS(uint8_t *DataL, uint8_t* DataH, uint16_t Data)
{
	if(_End){
		*DataL = (Data>>8);
		*DataH = (Data&0xff);
	}else{
		*DataH = (Data>>8);
		*DataL = (Data&0xff);
	}
}

//2个8位数组合为1个16位数
//DataL为低位，DataH为高位
uint16_t SCS::SCS2Host(uint8_t DataL, uint8_t DataH)
{
	uint16_t Data;
	if(_End){
		Data = DataL;
		Data<<=8;
		Data |= DataH;
	}else{
		Data = DataH;
		Data<<=8;
		Data |= DataL;
	}
	return Data;
}

void SCS::writeBuf(uint8_t ID, uint8_t MemAddr, uint8_t *nDat, uint8_t nLen, uint8_t Fun)
{
	uint8_t msgLen = 2;
	uint8_t bBuf[6];
	uint8_t CheckSum = 0;
	bBuf[0] = 0xff;
	bBuf[1] = 0xff;
	bBuf[2] = ID;
	bBuf[4] = Fun;
	if(nDat){
		msgLen += nLen + 1;
		bBuf[3] = msgLen;
		bBuf[5] = MemAddr;
		writeSCS(bBuf, 6);
		
	}else{
		bBuf[3] = msgLen;
		writeSCS(bBuf, 5);
	}
	CheckSum = ID + msgLen + Fun + MemAddr;
	uint8_t i = 0;
	if(nDat){
		for(i=0; i<nLen; i++){
			CheckSum += nDat[i];
		}
		writeSCS(nDat, nLen);
	}
	writeSCS(~CheckSum);
}

//普通写指令
//舵机ID，MemAddr内存表地址，写入数据，写入长度
int SCS::genWrite(uint8_t ID, uint8_t MemAddr, uint8_t *nDat, uint8_t nLen)
{
	rFlushSCS();
	writeBuf(ID, MemAddr, nDat, nLen, FEETECH_INST_WRITE);
	wFlushSCS();
	return Ack(ID);
}

//异步写指令
//舵机ID，MemAddr内存表地址，写入数据，写入长度
int SCS::regWrite(uint8_t ID, uint8_t MemAddr, uint8_t *nDat, uint8_t nLen)
{
	rFlushSCS();
	writeBuf(ID, MemAddr, nDat, nLen, FEETECH_INST_REG_WRITE);
	wFlushSCS();
	return Ack(ID);
}

//异步写执行指令
//舵机ID
int SCS::RegWriteAction(uint8_t ID)
{
	rFlushSCS();
	writeBuf(ID, 0, NULL, 0, FEETECH_INST_REG_ACTION);
	wFlushSCS();
	return Ack(ID);
}

//同步写指令
//舵机ID[]数组，IDN数组长度，MemAddr内存表地址，写入数据，写入长度
void SCS::syncWrite(uint8_t ID[], uint8_t IDN, uint8_t MemAddr, uint8_t *nDat, uint8_t nLen)
{
	rFlushSCS();
	uint8_t mesLen = ((nLen+1)*IDN+4);
	uint8_t Sum = 0;
	uint8_t bBuf[7];
	uint8_t msg_buf[100];
	uint8_t msg_len;
	bBuf[0] = 0xff;
	bBuf[1] = 0xff;
	bBuf[2] = 0xfe;
	bBuf[3] = mesLen;
	bBuf[4] = FEETECH_INST_SYNC_WRITE;
	bBuf[5] = MemAddr;
	bBuf[6] = nLen;
	// writeSCS(bBuf, 7);
	for (uint8_t i = 0; i < 7; i++){
		msg_buf[i] = bBuf[i];
	}
	msg_len = 7;

	Sum = 0xfe + mesLen + FEETECH_INST_SYNC_WRITE + MemAddr + nLen;
	uint8_t i, j;
	for(i=0; i<IDN; i++){
		// writeSCS(ID[i]);
		// writeSCS(nDat+i*nLen, nLen);
		msg_buf[msg_len++] = ID[i];
		for (j=0; j < nLen; j++){
			msg_buf[msg_len++] = nDat[i*nLen+j];
		}
		Sum += ID[i];
		for(j=0; j<nLen; j++){
			Sum += nDat[i*nLen+j];
		}
	}
	// writeSCS(~Sum);
	msg_buf[msg_len++] = ~Sum;
	writeSCS(msg_buf,msg_len);

	wFlushSCS();
}

int SCS::writeByte(uint8_t ID, uint8_t MemAddr, uint8_t bDat)
{
	rFlushSCS();
	writeBuf(ID, MemAddr, &bDat, 1, FEETECH_INST_WRITE);
	wFlushSCS();
	return Ack(ID);
}

int SCS::writeWord(uint8_t ID, uint8_t MemAddr, uint16_t wDat)
{
	uint8_t bBuf[2];
	Host2SCS(bBuf+0, bBuf+1, wDat);
	rFlushSCS();
	writeBuf(ID, MemAddr, bBuf, 2, FEETECH_INST_WRITE);
	wFlushSCS();
	return Ack(ID);
}

//读指令
//舵机ID，MemAddr内存表地址，返回数据nData，数据长度nLen
int SCS::Read(uint8_t ID, uint8_t MemAddr, uint8_t *nData, uint8_t nLen)
{
	rFlushSCS();
	writeBuf(ID, MemAddr, &nLen, 1, FEETECH_INST_READ);
	wFlushSCS();
	if(!checkHead()){
		return 0;
	}
	uint8_t bBuf[4];
	Error = 0;
	if(readSCS(bBuf, 3)!=3){
		return 0;
	}
	if(bBuf[0]!=ID && ID!=0xfe){
		return 0;
	}
	if(bBuf[1]!=(nLen+2)){
		return 0;
	}
	int Size = readSCS(nData, nLen);
	if(Size!=nLen){
		return 0;
	}
	if(readSCS(bBuf+3, 1)!=1){
		return 0;
	}
	uint8_t calSum = bBuf[0]+bBuf[1]+bBuf[2];
	uint8_t i;
	for(i=0; i<Size; i++){
		calSum += nData[i];
	}
	calSum = ~calSum;
	if(calSum!=bBuf[3]){
		return 0;
	}
	Error = bBuf[2];
	return Size;
}

//读1字节，超时返回-1
int SCS::readByte(uint8_t ID, uint8_t MemAddr)
{
	uint8_t bDat;
	int Size = Read(ID, MemAddr, &bDat, 1);
	if(Size!=1){
		return -1;
	}else{
		return bDat;
	}
}

//读2字节，超时返回-1
int SCS::readWord(uint8_t ID, uint8_t MemAddr)
{	
	uint8_t nDat[2];
	int Size;
	uint16_t wDat;
	Size = Read(ID, MemAddr, nDat, 2);
	if(Size!=2)
		return -1;
	wDat = SCS2Host(nDat[0], nDat[1]);
	return wDat;
}

//Ping指令，返回舵机ID，超时返回-1
int	SCS::Ping(uint8_t ID)
{
	rFlushSCS();
	writeBuf(ID, 0, NULL, 0, FEETECH_INST_PING);
	wFlushSCS();
	Error = 0;
	if(!checkHead()){
		return -1;
	}
	uint8_t bBuf[4];
	if(readSCS(bBuf, 4)!=4){
		return -1;
	}
	if(bBuf[0]!=ID && ID!=0xfe){
		return -1;
	}
	if(bBuf[1]!=2){
		return -1;
	}
	uint8_t calSum = ~(bBuf[0]+bBuf[1]+bBuf[2]);
	if(calSum!=bBuf[3]){
		return -1;			
	}
	Error = bBuf[2];
	return bBuf[0];
}

int SCS::checkHead()
{
	uint8_t bDat;
	uint8_t bBuf[] = {0, 0};
	uint8_t Cnt = 0;
	while(1){
		if(!readSCS(&bDat, 1)){
			return 0;
		}
		bBuf[1] = bBuf[0];
		bBuf[0] = bDat;
		if(bBuf[0]==0xff && bBuf[1]==0xff){
			break;
		}
		Cnt++;
		if(Cnt>10){
			return 0;
		}
	}
	return 1;
}

int	SCS::Ack(uint8_t ID)
{
	Error = 0;
	if(ID!=0xfe && _Level){
		if(!checkHead()){
			return 0;
		}
		uint8_t bBuf[4];
		if(readSCS(bBuf, 4)!=4){
			return 0;
		}
		if(bBuf[0]!=ID){
			return 0;
		}
		if(bBuf[1]!=2){
			return 0;
		}
		uint8_t calSum = ~(bBuf[0]+bBuf[1]+bBuf[2]);
		if(calSum!=bBuf[3]){
			return 0;
		}
		Error = bBuf[2];
	}
	return 1;
}

int	SCS::syncReadPacketTx(uint8_t ID[], uint8_t IDN, uint8_t MemAddr, uint8_t nLen)
{
	rFlushSCS();
	syncReadRxPacketLen = nLen;
	uint8_t checkSum = (4+0xfe)+IDN+MemAddr+nLen+FEETECH_INST_SYNC_READ;
	uint8_t i;
	writeSCS(0xff);
	writeSCS(0xff);
	writeSCS(0xfe);
	writeSCS(IDN+4);
	writeSCS(FEETECH_INST_SYNC_READ);
	writeSCS(MemAddr);
	writeSCS(nLen);
	for(i=0; i<IDN; i++){
		writeSCS(ID[i]);
		checkSum += ID[i];
	}
	checkSum = ~checkSum;
	writeSCS(checkSum);
	wFlushSCS();
	
	syncReadRxBuffLen = readSCS(syncReadRxBuff, syncReadRxBuffMax, syncTimeOut);
	return syncReadRxBuffLen;
}

void SCS::syncReadBegin(uint8_t IDN, uint8_t rxLen, uint32_t TimeOut)
{
	syncReadRxBuffMax = IDN*(rxLen+6);
	syncReadRxBuff = new uint8_t[syncReadRxBuffMax];
	syncTimeOut = TimeOut;
}

void SCS::syncReadEnd()
{
	if(syncReadRxBuff){
		delete syncReadRxBuff;
		syncReadRxBuff = NULL;
	}
}

int SCS::syncReadPacketRx(uint8_t ID, uint8_t *nDat)
{
	uint16_t syncReadRxBuffIndex = 0;
	syncReadRxPacket = nDat;
	syncReadRxPacketIndex = 0;
	while((syncReadRxBuffIndex+6+syncReadRxPacketLen)<=syncReadRxBuffLen){
		uint8_t bBuf[] = {0, 0, 0};
		uint8_t calSum = 0;
		while(syncReadRxBuffIndex<syncReadRxBuffLen){
			bBuf[0] = bBuf[1];
			bBuf[1] = bBuf[2];
			bBuf[2] = syncReadRxBuff[syncReadRxBuffIndex++];
			if(bBuf[0]==0xff && bBuf[1]==0xff && bBuf[2]!=0xff){
				break;
			}
		}
		if(bBuf[2]!=ID){
			continue;
		}
		if(syncReadRxBuff[syncReadRxBuffIndex++]!=(syncReadRxPacketLen+2)){
			continue;
		}
		Error = syncReadRxBuff[syncReadRxBuffIndex++];
		calSum = ID+(syncReadRxPacketLen+2)+Error;
		for(uint8_t i=0; i<syncReadRxPacketLen; i++){
			syncReadRxPacket[i] = syncReadRxBuff[syncReadRxBuffIndex++];
			calSum += syncReadRxPacket[i];
		}
		calSum = ~calSum;
		if(calSum!=syncReadRxBuff[syncReadRxBuffIndex++]){
			return 0;
		}
		return syncReadRxPacketLen;
	}
	return 0;
}

int SCS::syncReadRxPacketToByte()
{
	if(syncReadRxPacketIndex>=syncReadRxPacketLen){
		return -1;
	}
	return syncReadRxPacket[syncReadRxPacketIndex++];
}

int SCS::syncReadRxPacketToWrod(uint8_t negBit)
{
	if((syncReadRxPacketIndex+1)>=syncReadRxPacketLen){
		return -1;
	}
	int Word = SCS2Host(syncReadRxPacket[syncReadRxPacketIndex], syncReadRxPacket[syncReadRxPacketIndex+1]);
	syncReadRxPacketIndex += 2;
	if(negBit){
		if(Word&(1<<negBit)){
			Word = -(Word & ~(1<<negBit));
		}
	}
	return Word;
}

int SCS::Recovery(uint8_t ID)
{
	rFlushSCS();
	writeBuf(ID, 0, NULL, 0, FEETECH_INST_RECOVERY);
	wFlushSCS();
	return Ack(ID);
}

#endif // AP_FEETECH_ENABLED