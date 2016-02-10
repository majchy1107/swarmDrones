#ifndef __NETWORKNODE_H
#define __NETWORKNODE_H

#include <stdio.h>  
#include <string.h> 
#include <stdlib.h>
#include <unistd.h>
#include <errno.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <sys/time.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <arpa/inet.h>
#include <list>
#include <pthread.h>
#include <semaphore.h>

#include <stropts.h>
#include <netdb.h>
#include <termios.h>



#include "CommonConstants.h"
#include "SwarmHelper.h"



#define PORT 4950 
#define MAX_AGENT_NUM 20


typedef struct {
	float cost;
	int neighbor;
} BID;

struct helloMsg {

	// Original (standard) structure
	unsigned short int id;
	unsigned long int msgCounter;
	std::list<long int> msgTimes;
	//long int lon;
	//long int lat;
	//long int course;
	float lon;
	float lat;
	float robotLon;
	float robotLat;
	float course;
	float speed;
	float speedx;
	float speedy;
	clock_t messageTime;
	//unsigned short int auction_p;
	//unsigned short int auction;
	//int tokenList[MAX_NEIGH_NUM];
	//int tokenList_p[MAX_NEIGH_NUM];
	//BID bidList[MAX_NEIGH_NUM];
	//BID bidList_p[MAX_NEIGH_NUM];
	//int topology[TOTAL_ROBOT_NUM+1][TOTAL_ROBOT_NUM+1];
	//short int goalAlt;
	//short int metric;
};


typedef struct {
	float lon;
 	float lat; 
	float robotLon;
	float robotLat;
	float course;
	float speed;
	float speedx;
	float speedy;
	int id;
	//unsigned short int auction_p;
	//unsigned short int auction;
	//int tokenList[MAX_NEIGH_NUM];
	//int tokenList_p[MAX_NEIGH_NUM];
	//BID bidList[MAX_NEIGH_NUM];
	//BID bidList_p[MAX_NEIGH_NUM];
	//int topology[TOTAL_ROBOT_NUM+1][TOTAL_ROBOT_NUM+1];
} MSGSTATE;

static std::list <helloMsg> helloMsgs;

//void start_network(GAPI &gapi);
void start_network(MSGSTATE &msg);
void stop_network(void);
void emptyInputBuffer(void);
void *broadcastMessage(void *ptr);
void client_init(void);
void client_send(void);
void server_init(void);
void *server_recpt(void* ptr);
void close_server(void);
void close_client(void);
void *run_curl(void* ptr);
std::list <helloMsg> getInputMessages(double commtime);

#endif
