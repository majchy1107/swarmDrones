#include "NetworkNode.h"
#include <iostream>

//#include "globalVars.h"



int clsockfd;
struct sockaddr_in broad_addr; 
struct hostent *he;
int broadcast;
const char* addr;
int svsockfd;
struct sockaddr_in my_addr; 
struct sockaddr_in cl_addr; 
socklen_t addr_len;
fd_set readfds;
struct timeval tv;
int numbytes;
std::list <helloMsg> inputMessageBuffer;
sem_t wifi_sem;
sem_t rw_sem;

#define MSG_SAMPLE_TIME 0.1


void start_network(MSGSTATE &msg) {

	broadcast = 1;
	addr = "192.168.100.255";
	server_init();
	client_init();
	srand(time(NULL));
	pthread_t thread;
	pthread_create(&thread,NULL,server_recpt,NULL);
	pthread_t thread2;
	pthread_create(&thread2,NULL,broadcastMessage, (void*) &msg);
	pthread_t thread3;
	pthread_create(&thread3,NULL,run_curl,(void*) &msg);

	sem_init(&wifi_sem,10,1); //mutex
	sem_init(&rw_sem,10,1); // mutex for read/write
}

void stop_network() {

	close_server();
	close_client();

}

void server_init(void) {
	my_addr.sin_family = AF_INET;
	my_addr.sin_port = htons(PORT);
	my_addr.sin_addr.s_addr = INADDR_ANY;
	memset(my_addr.sin_zero, '\0', sizeof my_addr.sin_zero);

	if ((svsockfd = socket(AF_INET, SOCK_DGRAM, 0)) == -1) perror("socket");

	if (bind(svsockfd, (struct sockaddr *)&my_addr, sizeof my_addr) == -1) perror("bind");
}

void client_init(void) {
	broadcast = 1;
	int inputBuffer = 1;
	int outputBuffer = 1;
	int flag = 1;

	if ((clsockfd = socket(AF_INET, SOCK_DGRAM, 0)) == -1) perror("socket");
	if (setsockopt(clsockfd, SOL_SOCKET, SO_BROADCAST, (char *)&broadcast, sizeof broadcast) == -1) perror("setsockopt (SO_BROADCAST)");
	if (setsockopt(clsockfd, SOL_SOCKET, SO_SNDBUF, (char *)&outputBuffer,sizeof outputBuffer) == -1) perror("setsockopt (SO_BROADCAST)");
	if (setsockopt(clsockfd, SOL_SOCKET,SO_RCVBUF, (char *)&inputBuffer,sizeof inputBuffer) == -1) perror("setsockopt (SO_BROADCAST)");

	broad_addr.sin_family = AF_INET;
	broad_addr.sin_port = htons(PORT);
	broad_addr.sin_addr.s_addr = inet_addr("192.168.100.255");
	memset(broad_addr.sin_zero, '\0', sizeof broad_addr.sin_zero);

	// init message - see if it needs mutex!!!!
}

void *server_recpt(void* ptr) {

	FD_ZERO(&readfds);
	FD_SET(svsockfd,&readfds);
	tv.tv_sec = 0;
	tv.tv_usec = 0;
	int tokenListInit[MAX_AGENT_NUM]={};
	BID bidListInit[MAX_AGENT_NUM];
	int topologyInit[TOTAL_ROBOT_NUM][TOTAL_ROBOT_NUM]={{}};
	while(1) {

		FD_ZERO(&readfds);
		FD_SET(svsockfd,&readfds);
		tv.tv_sec = 0;
		tv.tv_usec = 0;
		if(select(svsockfd+1, &readfds, NULL, NULL, &tv)>0) {
			addr_len = sizeof cl_addr;
			struct helloMsg *recptMess=(helloMsg *)malloc(sizeof(struct helloMsg));
			recptMess->id = 0;
			recptMess->lon = 0;
			recptMess->lat = 0;
			recptMess->robotLon = 0;
			recptMess->robotLat = 0;
			recptMess->course = 0;
			recptMess->speed = 0;
			recptMess->speedx = 0;
			recptMess->speedy = 0;
			recptMess->messageTime=0;
                       
			sem_wait(&wifi_sem);

			if ((numbytes = recvfrom(svsockfd, recptMess, sizeof(*recptMess), 0, (struct sockaddr *)&cl_addr, &addr_len)) == -1) perror("recvfrom");

			bool idFound=false;

			for(std::list<helloMsg>::iterator i=inputMessageBuffer.begin(); i!=inputMessageBuffer.end(); i++) {

				if((*i).id == recptMess->id) {
					idFound = true;
					(*i).msgTimes.push_back(clock());
					(*i).lon = recptMess->lon;
					(*i).lat = recptMess->lat;
					(*i).robotLon = recptMess->robotLon;
					(*i).robotLat = recptMess->robotLat;
					(*i).course = recptMess->course;
					(*i).speed = recptMess->speed;
					(*i).speedx = recptMess->speedx;
					(*i).speedy = recptMess->speedy;
					(*i).messageTime = recptMess->messageTime;
					
				}
			}

			if(!idFound) {

				helloMsg hmt;
				hmt.id = recptMess->id;
				hmt.msgTimes.push_back(clock());
				hmt.lon = recptMess->lon;
				hmt.lat = recptMess->lat;
				hmt.robotLon = recptMess->robotLon;
				hmt.robotLat = recptMess->robotLat;
				hmt.course = recptMess->course;
				hmt.speed = recptMess->speed;
				hmt.speedx = recptMess->speedx;
				hmt.speedy = recptMess->speedy;
				hmt.messageTime = recptMess->messageTime;
				inputMessageBuffer.push_back(hmt);

			}
			free(recptMess);
			sem_post(&wifi_sem);

		}

	}

}

void *broadcastMessage(void *ptr) {

	time_t lastTime=clock();
	timeval t_current, t_previous, t_diff;
	double t_elapsed = 0; 
	gettimeofday(&t_current,NULL);
	gettimeofday(&t_previous, NULL);

	MSGSTATE *msg=(MSGSTATE*) ptr;
	while(1) {
		
		// send message every n seconds
                gettimeofday(&t_current,NULL);
                timersub(&t_current, &t_previous, &t_diff);
                t_elapsed = t_diff.tv_sec+t_diff.tv_usec/1000000.0;
		//std::cout<<t_elapsed<<std::endl;
                if (t_elapsed > MSG_SAMPLE_TIME) {
	                helloMsg *hmsg=(helloMsg *)malloc(sizeof(struct helloMsg));
                	gettimeofday(&t_previous,NULL);
			// Standard information broadcasted
			//while message is sent, primary thread cannot write to  variable va
			//sem_wait(&rw_sem);
			//HARCODED ID - REMOVE!!!!!!
			hmsg->id = msg->id;
			//set according to gapi interface
			hmsg->lon = msg->lon;
	        	hmsg->lat = msg->lat;
			hmsg->robotLon = msg->robotLon;
			hmsg->robotLat = msg->robotLat;
			hmsg->course = msg->course;
			hmsg->speed = msg->speed;
			hmsg->speedx = msg->speedx;
			hmsg->speedy = msg->speedy;
			hmsg->messageTime = clock();
			usleep(10); // sleep for 10 microseconds
			if ((numbytes=sendto(clsockfd, hmsg, sizeof(helloMsg), 0,(struct sockaddr *)&broad_addr, sizeof broad_addr)) == -1) perror("sendto");
			free(hmsg);
			//sem_post(&rw_sem);
		}
		
	}

}


std::list <helloMsg> getInputMessages(double commtime) {

	sem_wait(&wifi_sem);
	for(std::list<helloMsg>::iterator i=inputMessageBuffer.begin();i!=inputMessageBuffer.end();i++) {

		while((*i).msgTimes.size()>0) {

			if(((double)(clock()-(*i).msgTimes.front())/(double)CLOCKS_PER_SEC)>commtime) (*i).msgTimes.pop_front();
			else break;

		}

	}

	int counter = inputMessageBuffer.size();

	while(inputMessageBuffer.size()>0 && counter>0) {
		counter--;
		if(inputMessageBuffer.front().msgTimes.size()==0) inputMessageBuffer.pop_front();
		else {

			inputMessageBuffer.push_back(inputMessageBuffer.front());
			inputMessageBuffer.pop_front();
		}

	}
	sem_post(&wifi_sem);
	return inputMessageBuffer;

}

void emptyInputBuffer() {

	while(inputMessageBuffer.size()>0)
		inputMessageBuffer.pop_back();
}

void *run_curl(void *ptr) {

	// run system command to get olsrd data

	MSGSTATE *msg= (MSGSTATE*) ptr;
        timeval t_current, t_previous, t_diff;
        double t_elapsed = 0;
        gettimeofday(&t_current,NULL);
        gettimeofday(&t_previous, NULL);



	int neighbors[20]={};
	double lq[20]={};
	int num_neighbors = 0;
	double lq_av[20] = {};
	double lq_av_prev[20] = {};
	float alpha = 0.2;

	//char command[50];
	while(1) {
                gettimeofday(&t_current,NULL);
                timersub(&t_current, &t_previous, &t_diff);
                t_elapsed = t_diff.tv_sec+t_diff.tv_usec/1000000.0;
                //std::cout<<t_elapsed<<std::endl;

                if (t_elapsed > 1) {
			gettimeofday(&t_previous,NULL);

			/*if (( msg->id >= 1) && (msg->id <= 3)) {

				char* fileName_lq = new char[100];
				sprintf(fileName_lq,"/home/root/formation_simple/testolsrd%d.txt",msg->id);
				char command[50];
				sprintf(command,"curl localhost:2006/lq 2>/dev/null 1> testolsrd%d.txt &",msg->id);
				system(command);

				parseLQfile(fileName_lq, neighbors, lq, &num_neighbors);
				//std::cout<<"NUMBER OF NEIGHBORS: "<<num_neighbors<<std::endl;
				for (int i=1; i <= num_neighbors; i++) {
					//std::cout<<neighbors[i]<<std::endl;
		                        lq_av[neighbors[i]]=alpha * lq[i] + (1-alpha)*lq_av_prev[neighbors[i]];
		                        lq_av_prev[neighbors[i]]=lq_av[neighbors[i]];
					//std::cout << distanceD[neighbors[i]]<<" "<< bump_fcn(lq_av[neighbors[i]]/2,0.2)<<std::endl;
		        		//std::cout<<"LINKQ "<< timestamp <<" "<< i <<" "<<norm_ebee_id[neighbors[i]]<< " "  << lq[i] <<" "<<std::endl;
					std::cout<<"LINKQ "<< time(NULL) <<" "<< neighbors[i] <<" "<<0<< " "  << lq[i] <<" "<< lq_av[neighbors[i]]<<" "<<std::endl;
				}
				//for (int j=1; j<=TOTAL_ROBOT_NUM; j++) {
				//	std::cout<<mybidList_current[j].cost<< " ";
				//}
				std::cout<<std::endl;
			}*/
		}
	}

}

void close_server(void) {

	close(svsockfd);

}

void close_client(void) {

	close(clsockfd);

}
