/*
	Copyright © 2010 senseFly
	Author: Antoine Beyeler
	
	All rights reserved. Do not distribute.
*/


#include "sipp.h"


#include <stdio.h>

#ifndef NULL
	#define NULL 0
#endif


sipp_ProtocolData* sipp_protocol = NULL;
sipp_SendDataFunc sipp_sendData = NULL;
sipp_FlushDataFunc sipp_flushData = NULL;

	// reception state machine
enum
{
	SIPP_HEADER,
	SIPP_ID,
	SIPP_DATA,
	SIPP_CHK1,
	SIPP_CHK2
} sipp_state = SIPP_HEADER;
int sipp_dataReceived;
sipp_PacketDesc* sipp_incomingPacket;

unsigned short sipp_dataChk;
	


sipp_PacketDesc* sipp_GetDesc(unsigned char id)
{
	if (!sipp_protocol)
		return NULL;
		
	int i;
	for (i = 0; i < sipp_protocol->descCount; i++)
	{
		if (sipp_protocol->descs[i].id == id)
			return &sipp_protocol->descs[i];
	}
	
	return NULL;
}

inline void sipp_ChecksumStep(unsigned char data, unsigned short* chk)
{
	*chk = (*chk << 1) | (*chk >> 15);

#ifdef __C30
	// this weird construction is to work around a bug in C30's optimizer
	unsigned char c = data;
	*chk ^= *(&c);
#else
	*chk ^= data;
#endif
}

unsigned short sipp_Checksum(unsigned char* data, int len)
{
	unsigned short chk = 0;
	while (len--)
		sipp_ChecksumStep(*data++, &chk);
	return chk;
}

void sipp_Setup(sipp_ProtocolData* protocol, sipp_SendDataFunc sendDataFunc, sipp_FlushDataFunc flushDataFunc)
{
	sipp_protocol = protocol;
	sipp_sendData = sendDataFunc;
	sipp_flushData = flushDataFunc;
}

void sipp_SendPacket(unsigned char id, void* data)
{
	sipp_PacketDesc* desc = sipp_GetDesc(id);
	if (desc == NULL)
		return;
	if (!sipp_sendData)
		return;
	
	// send header
	unsigned char header[2] = { 0x5F, id };
	sipp_sendData(header, 2);
	
	// send data
	sipp_sendData(data, desc->size);
	
	// compute checksum
	unsigned short chk = sipp_Checksum((unsigned char*) data, desc->size);
	sipp_sendData((unsigned char*) &chk, 2);
	
	// flush
	if (sipp_flushData)
		sipp_flushData();
}

void sipp_ReceiveData(unsigned char* data, unsigned long len)
{
	while (len--)
		sipp_ReceiveDataByte(*data++);
}

void sipp_ReceiveDataByte(unsigned char c)
{
	switch (sipp_state)
	{
		case SIPP_HEADER:
		{
			if (c == SIPP_HEADER_BYTE)
				sipp_state = SIPP_ID;
		}
		break;
		
		case SIPP_ID:
		{
			sipp_incomingPacket = sipp_GetDesc(c);
			if (sipp_incomingPacket)
			{
				sipp_dataReceived = 0;
				sipp_dataChk = 0;
				
				sipp_state = SIPP_DATA;
			}
			else
				sipp_state = SIPP_HEADER;
		}
		break;
		
		case SIPP_DATA:
		{
			if (sipp_dataReceived < sipp_protocol->bufferSize)
			{
				sipp_protocol->buffer[sipp_dataReceived++] = c;
				sipp_ChecksumStep(c, &sipp_dataChk);
				
				if (sipp_dataReceived == sipp_incomingPacket->size)
					sipp_state = SIPP_CHK1;
			}
			else
				sipp_state = SIPP_HEADER;
		}
		break;
		
		case SIPP_CHK1:
		{
			if ((sipp_dataChk & 0xFF) == c)
				sipp_state = SIPP_CHK2;
			else
				sipp_state = SIPP_HEADER;
		}
		break;
		
		case SIPP_CHK2:
		{
			if ((sipp_dataChk >> 8) == c)
			{
				if (sipp_incomingPacket->func)
					sipp_incomingPacket->func(sipp_incomingPacket->id, sipp_protocol->buffer);
			}
			
			sipp_state = SIPP_HEADER;
		}
		break;
		
		default:
		{
			// should never arise...
			sipp_state = SIPP_HEADER;
		}
		
	};
}
