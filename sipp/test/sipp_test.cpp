#include "../sipp.h"

#include <dashel/dashel.h>
#include <iostream>
#include <string>
#include <valarray>
#include <algorithm>
#include <time.h>


using namespace Dashel;

Stream* connectedStream = NULL;

class DashelWrapper : public Hub
{
public:
	virtual ~DashelWrapper() { }


protected:
	virtual void connectionCreated(Stream* stream)
	{
		// we connect a single stream at a time
		if (connectedStream)
			closeStream(stream);
		else connectedStream = stream;
	}

	virtual void connectionClosed(Stream* stream, bool)
	{
		if (stream == connectedStream)
			connectedStream = NULL;
	}

	virtual void incomingData(Stream *stream)
	{
		// we read one byte at a time, relying on the fact that dashel will repeatedly call
		// incomingData as long as data is still available
		unsigned char byte = stream->read<unsigned char>();
		// printf("\n-- Dashel rcv : %X",(unsigned char)byte);
		sipp_ReceiveDataByte(byte);
	}
};



extern "C" void sendData(unsigned char *data, unsigned long len)
{
	if (connectedStream)
		connectedStream->write(data, len);
}

extern "C" void flushData()
{
	if (connectedStream)
		connectedStream->flush();
}



typedef struct
{
	unsigned char payload;
}  __attribute((packed)) Payload1;

typedef struct
{
	long payload;
	float attrib;
}  __attribute((packed)) Payload2;


extern "C" void payload1Incoming(unsigned char id, Payload1* packet)
{
	std::cout << "Received payload 1 with id " << (int) id << " and data " << (int) packet->payload << "." << std::endl;
}

extern "C" void payload2Incoming(unsigned char id, Payload2* packet)
{
	std::cout << "Received payload 2 with with id " << (int) id << ", data " << packet->payload << " and " << packet->attrib << "." << std::endl;
}



SIPP_DECLARE(MyProtocol, sizeof(Payload2),
	SIPP_PACKET(0x01, Payload1, &payload1Incoming),
	SIPP_PACKET(0x02, Payload2, &payload2Incoming)
);





int main(int argc, char *argv[])
{
	std::string target = "tcpin:port=12345";

	if (argc > 1)
		target = argv[1];

	// create wrapper
	DashelWrapper wrapper;
	wrapper.connect(target);
	
	// setup SIPP
	sipp_Setup(&MyProtocol, &sendData, &flushData);


	bool packet1 = true;
	Payload1 payload1 = { 0 };
	Payload2 payload2 = { 0, 0.0 };

	while (true)
	{
		wrapper.step(1000);
		
		if (packet1)
		{
			payload1.payload++;
			sipp_SendPacket(0x01, &payload1);
		}
		else
		{
			payload2.payload++;
			payload2.attrib += 1.0;
			sipp_SendPacket(0x02, &payload2);
		}
		
		packet1 = !packet1;
	}

	return 0;
}
