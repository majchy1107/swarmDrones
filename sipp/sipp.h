/*
	Copyright © 2010 senseFly
	Author: Antoine Beyeler
	
	All rights reserved. Do not distribute.
*/

// SIPP = Simple Inter Processor Protocol


#ifndef __SF_SIPP_H__
#define __SF_SIPP_H__

#ifdef __cplusplus
extern "C" {
#endif

#ifdef SIPP_PREFIX
	#include SIPP_PREFIX
#endif


/*
* Protocol:
* 
* 	Header byte 	(0x5F)
* 	Packet ID 		(1 byte)
* 	Packet data		(n bytes)
* 	Checksum 		(2 bytes)
* 
*	Total size		n+4 bytes
*/


//! Header byte used for SIPP packets.
#define SIPP_HEADER_BYTE	0x5f


/*!
* Declare the structure of a protocol.
* 
* Example:
* 
* 	void MyStructReceived(unsigned char id, MyStruct* data);
* 	void MyOtherStructReceived(unsigned char id, MyOtherStruct* data);
* 
* 	SIPP_DECLARE(MyProtocol, sizeof(MyBiggestStruct),
* 		SIPP_PACKET(0x01, MyStruct, &MyStructReceived),
* 		SIPP_PACKET(0x02, MyOtherStruct, &MyOtherStructReceived)
* 	);
* 
* 
* 	void main()
*	{
* 		sipp_Setup(&MyProtocol, &sendDataFunction, &flushDataFunction);
* 	}
* 
* CAUTION: maxSize parameter MUST be large enough for the biggest packet to 
* be received, otherwise it will be dropped.
* 
* NOTE: since this macro declares global variables for buffer and protocol
* definition, it must be called from a .c file (and not a .h that may be part
* of several translation units).
*/
#define SIPP_DECLARE(name, maxSize, ...)									\
	unsigned char sipp_ ## name ## _bufferTag[maxSize];						\
	sipp_PacketDesc sipp_ ## name ## _packetDescTag[] = { __VA_ARGS__ };	\
	sipp_ProtocolData name = { sipp_ ## name ## _bufferTag, maxSize, 		\
		sipp_ ## name ## _packetDescTag,									\
		sizeof(sipp_ ## name ## _packetDescTag) / sizeof(sipp_ ## name ## _packetDescTag[0]) }

//! Declare a packet. To be used in conjunction with #SIPP_DECLARE
#define SIPP_PACKET(id, type, func) 	{ id, sizeof(type), (sipp_PacketIncomingFunc) func }



//! Prototype for the callback function called upon reception of a packet.
typedef void (*sipp_PacketIncomingFunc)(unsigned char id, void* packet);
//! Prototype for the function that SIPP calls to send data.
typedef void (*sipp_SendDataFunc)(unsigned char *, unsigned long);
//! Prototype for the function that SIPP calls to flush data.
typedef void (*sipp_FlushDataFunc)();


//! Structure to hold parameters of a given packet.
typedef struct
{
	unsigned char id;
	int size;
	sipp_PacketIncomingFunc func;
} sipp_PacketDesc;


//! Structure to hold buffers and packet description for the protocol.
typedef struct
{
	unsigned char* buffer;
	int bufferSize;
	sipp_PacketDesc* descs;
	int descCount;
} sipp_ProtocolData;



//! Initialize SIPP. Must be called first. Must only be called once.
void sipp_Setup(sipp_ProtocolData* protocol, sipp_SendDataFunc sendDataFunc, sipp_FlushDataFunc flushDataFunc);

//! Sends a packet. The structure passed must be the same than the one registered.
void sipp_SendPacket(unsigned char id, void* data);

//! Call for incoming data.
void sipp_ReceiveData(unsigned char* data, unsigned long len);

//! Call for incoming data (alternate form).
void sipp_ReceiveDataByte(unsigned char c);


#ifdef __cplusplus
}
#endif


#endif
