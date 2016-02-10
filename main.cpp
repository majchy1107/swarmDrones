

#include "NetworkNode.h"

bool cmpNeighbors(const NEIGHBORS &n1, const NEIGHBORS &n2)
        { return n1.distance < n2.distance; }

#define SYNC 1
#define VP_SPEED_LIMIT 3

// Main
int main(int argc, char *argv[]) {

    	MSGSTATE msg;
    	//EBEESTATE ebee;

   	 // Initialization
	//serial port initialization
	std::string target_gapi = "ser:device=/dev/ttyO0; 115200";

	// take arguments from console
	int id = atoi(argv[1]);
	float distanceR = atof(argv[2]);


	//API interface instance
	GAPI gapi(target_gapi);
	time_t t = 0;
	time_t t_change = 0;
	bool toggle = false;



    	// init msg state
   	msg.course = 0;
    msg.lon = 0;
    msg.lat = 0;
    msg.speedx = 0;
  	msg.speedy = 0;
	msg.id = id;
	msg.robotLon = 0;
	msg.robotLat = 0;

	std::cout.precision(15);


	float distanceD[20] = {30, 30, 30, 30, 30, 30, 30, 30, 30, 30, 30}; // set up intitial distance to all agents
	float distanceD_prev[20] = {30, 30, 30, 30, 30, 30, 30, 30, 30, 30};
	float start_longitude = 46.567;
	float start_latitude =  6.523;
	float altitude = 70;


	switch ( id ) {

		case 1 :
			start_longitude = ATTRACTOR_POINT_LONGITUDE11;	
			start_latitude = ATTRACTOR_POINT_LATITUDE11;
			altitude = 60;
			break;


		case 2 :

			start_longitude = ATTRACTOR_POINT_LONGITUDE12;	
			start_latitude = ATTRACTOR_POINT_LATITUDE12;
			altitude = 70;
			break;

		case 3 :

			start_longitude = ATTRACTOR_POINT_LONGITUDE13;	
			start_latitude = ATTRACTOR_POINT_LATITUDE13;
			altitude = 80;
			break;

		case 4 :

			start_longitude = ATTRACTOR_POINT_LONGITUDE14;	
			start_latitude = ATTRACTOR_POINT_LATITUDE14;
			altitude = 90;
			break;

		default:


			start_longitude = ATTRACTOR_POINT_LONGITUDE13;	
			start_latitude = ATTRACTOR_POINT_LATITUDE13;
			altitude = 75;
	}	


	char* fileName_lq = new char[100];
	int neighbors[20]={};
	double lq[20]={};
	int num_neighbors = 0;
	double lq_av[20] = {};
	double lq_av_prev[20] = {};
	float alpha = 0.2;

	srand(time(NULL));

    msg.lon = start_longitude;
    msg.lat = start_latitude;

	start_network(msg);


	// INITIALIZATION OF VIRTUAL AGENT VARIABLES

	float output[2];
	point vp;
	point vp_neigh[MAX_NEIGH_NUM];

	point F,F_attr,v_align,F_mig;
	point navp;

	// initialization of virtual agent
	convertFromGPStoXY(start_longitude,start_latitude,output);
	vp.x = output[0];
	vp.y = output[1];
	vp.speedx = 0;
	vp.speedy = 0;
	vp.course = 0;

	// initialization of navigation agent
	convertFromGPStoXY(MIGRATION_POINT_LONGITUDE,MIGRATION_POINT_LATITUDE,output);
	navp.x = output[0];
	navp.y = output[1];
	navp.speedx = 0;
	navp.speedy = 0;

	// sample time, set to the average message time
	double Td = 0.05;
	double norm;

	//LGVF constants
	double K = 0.4;
	double beta = 0.1;
	double delta_ni_p = 0;
	double omega = 0;
	double v = 14;

	//SYNC variables
	double delta_theta = 0;
	double delta_v = 0;
	double K_sync = 1;
	int start_sync = 0;
	int stop_sync = 1;

	//set initial wpt - to make ebee work - TODO: check this waypoint
	gapi.setGWptLatitude(EBEE_INIT_LATITUDE);
    gapi.setGWptLongitude(EBEE_INIT_LONGITUDE);
    gapi.setGWptHeight(altitude);
    gapi.setGWptRadius(30);
    gapi.updateGWpt();

	bool prevEnableFlag = true;
	bool experiment_started = false;
	float distance_robot = 0;
	int msgs_count = 0;
	float distance_robot_sum = 0;
	float distance_robot_mean = 0;


	// INTERACTION neighborhood
	int neighbors_interaction[MAX_NEIGH_NUM]={};
	double norm_va_id[MAX_NEIGH_NUM]={};
	int num_int_neighbors = 0;
	double Rc_sigma[MAX_NEIGH_NUM]={};
	double Ri_sigma[MAX_NEIGH_NUM]={};
	double norm_sigma[MAX_NEIGH_NUM]={};
	point nij[MAX_NEIGH_NUM];
	double angle_va_id[MAX_NEIGH_NUM]={};

	// timer variables
	time_t  time_current = time(NULL);
	timeval t_current, t_previous, t_diff;
	double t_elapsed = 0;
	gettimeofday(&t_current,NULL);
	gettimeofday(&t_previous,NULL);

	// array for message timestamps
	double message_timestamp[20] = {0};
	double delta_theta_pom = 0;
 



	// Robot behaviour
	while(1) {



		gapi.step();


		if ((prevEnableFlag!=gapi.enableFlag())&&(gapi.enableFlag()==true))
		{

			gapi.setGWptLatitude(EBEE_INIT_LATITUDE);
    		gapi.setGWptLongitude(EBEE_INIT_LONGITUDE);
    		gapi.setGWptHeight(altitude);
    		gapi.setGWptRadius(30);
   			gapi.updateGWpt();


			//turnrate control - enable
			gapi.setAileronControlMode(GAPI_TURN_RATE_CONTROL);
			//altitude control - enable
			gapi.setElevatorControlMode(GAPI_ALTITUDE_CONTROL);
			//speed control - enable 
			gapi.setThrustControlMode(GAPI_SPEED_CONTROL);
			gapi.updateControl(); // updateControl();

			std::cout<<"Gumstix has control"<<experiment_started<<std::endl;
			prevEnableFlag = gapi.enableFlag();
			t_change = time(NULL);
			experiment_started = true;

		}
		else if ((prevEnableFlag!=gapi.enableFlag())&&(gapi.enableFlag()==false))
		{
			gapi.setAileronControlMode(GAPI_NO_CONTROL);
			gapi.setElevatorControlMode(GAPI_NO_CONTROL);
			gapi.setThrustControlMode(GAPI_NO_CONTROL);
			gapi.updateControl(); // updateControl() ??
			prevEnableFlag = gapi.enableFlag();
			// reinitialize target variables
			std::cout<<"Gumstix has no control"<< experiment_started<<std::endl;
			experiment_started = false;


		}

		if (gapi.enableFlag()) { //UNCOMMENT


		//************ READ MESSAGES *********************//
		int update_force_vector = 0;
		int update_VA_state = 0;
   		int new_message_received = 0;

		// norm measures for logging and formation algorithm
		double average_norm = 0;
		double average_norm_ebee = 0;
		int number_neighbors = 0;

		std::list<helloMsg> msgs = getInputMessages(0.001); //5

		F.x = 0;
		F.y = 0;
		v_align.x = 0;
		v_align.y = 0;
		delta_theta_pom = 0;

		// read message list
      		for(std::list<helloMsg>::iterator i=msgs.begin(); i != msgs.end(); i++) {

			// CALCULATE FORMATION ALGORITHM
               		if (((*i).id != id) && ((*i).id >= 1) && ((*i).id <= TOTAL_ROBOT_NUM)) { // change allowed id for larger swarms

					// take into account only new messages
					if ((*i).messageTime > message_timestamp[(*i).id]) {
						// when new message arrives from agent j, update force vectors
						update_VA_state = 1;
						update_force_vector = 1;
						new_message_received = 1;

						double tdiff = ((*i).messageTime - message_timestamp[(*i).id])/CLOCKS_PER_SEC;
						message_timestamp[(*i).id] = (*i).messageTime;

					}


					// take data from messages and save to va structure
					convertFromGPStoXY((*i).lon, (*i).lat, output);
					vp_neigh[(*i).id].x = output[0];
					vp_neigh[(*i).id].y = output[1];
					vp_neigh[(*i).id].speed = (*i).speed;
					vp_neigh[(*i).id].course =(*i).course;
					vp_neigh[(*i).id].speedx = (*i).speedx;
					vp_neigh[(*i).id].speedy = (*i).speedy;

					// calculate vector towards neughbors and euclidean distance
					double vector_angle=atan2(vp_neigh[(*i).id].y-vp.y,vp_neigh[(*i).id].x-vp.x);
					norm = euclid_norm(vp.x,vp.y,vp_neigh[(*i).id].x,vp_neigh[(*i).id].y);
					norm_va_id[(*i).id] = norm;
					angle_va_id[(*i).id] = vector_angle;

				    Rc_sigma[(*i).id] = sigma_norm(distanceR * 1.2, 0.1);
					Ri_sigma[(*i).id] = sigma_norm(distanceR, 0.1);
					norm_sigma[(*i).id] = sigma_norm(norm,0.1); 

					// calculate attr/rep force vector
					double vec_gradient_abs = sqrt(1+0.1*pow(norm,2));
					nij[(*i).id].x = (vp_neigh[(*i).id].x - vp.x)/vec_gradient_abs;
					nij[(*i).id].y = (vp_neigh[(*i).id].y - vp.y)/vec_gradient_abs;

					//update interaction topology
					for (int it=1; it<=TOTAL_ROBOT_NUM; it++) {

						if (norm_va_id[it] == 0) {
							neighbors_interaction[it]=0;
							
						 } 
						else {

							if (norm_va_id[it] <= distanceR*1.2) {
								neighbors_interaction[it]=1;
	
							}
							if (norm_va_id[it] > distanceR*1.2) {
		                        neighbors_interaction[it]=0;
		                                              
							}
						}
					}

					// check if agent has only one or no neighbors
					int neighbors_number = 0;
					std::vector<NEIGHBORS> va_neighbors;
					NEIGHBORS neighbor_aux;
					for (int it=1; it <= TOTAL_ROBOT_NUM; it++) {
						if (neighbors_interaction[it]==1) {
							neighbors_number++;
						}
						if ((it!=id) && (norm_va_id[it]!=0)) {
							neighbor_aux.distance = norm_va_id[it];
							neighbor_aux.id = it;
							va_neighbors.push_back(neighbor_aux);
						}
					}

					std::sort(va_neighbors.begin(),va_neighbors.end(), cmpNeighbors);

					if  ((neighbors_number == 0) && (va_neighbors.size() > 2)) {
						// add two closest neighbors to interaction list
						neighbors_interaction[va_neighbors.at(0).id]=1;
					}

					if ((neighbors_number == 1) && (va_neighbors.size() > 1)) {
						// add one closest neighbor to interaction list
						if (neighbors_interaction[va_neighbors.at(0).id]) {
							neighbors_interaction[va_neighbors.at(1).id]=1;
					
						} else {
							neighbors_interaction[va_neighbors.at(0).id]=1;
				
						}
					}


					// select neighbors within Rd*1.2, change this, use data from interaction neighborhood
					// *************** ATTR/REP FORCE *****************************//
					if (update_force_vector) {
						// removed distanceD[(*i).id]
						update_force_vector = 0;

						for (int it = 1; it <= TOTAL_ROBOT_NUM; it++) {
							if (neighbors_interaction[it]==1) {
				

								double Fi = bump_fcn(norm_sigma[it]/Rc_sigma[it],0.2) * sygmoid(norm_sigma[it],Ri_sigma[it],5,5);
								F.x = F.x + Fi * nij[it].x;
								F.y = F.y + Fi * nij[it].y;

		                                     		v_align.x = v_align.x + bump_fcn(norm_sigma[it]/Rc_sigma[it],0.2)*(vp_neigh[it].speedx-vp.speedx);
                                                		v_align.y = v_align.y + bump_fcn(norm_sigma[it]/Rc_sigma[it],0.2)*(vp_neigh[it].speedy-vp.speedy);
                                                		F.x = F.x + v_align.x;
                                                		F.y = F.y + v_align.y;
							}
						}
						//std::cout<<std::endl;
					}


					// ********* PHASE SYNCHRONIZATION ***********************

					if (new_message_received) { // we are syncing with all agents that we can communicate


						float angle = Angle(gapi.longitude(),gapi.latitude(),msg.lon,msg.lat);
		

						delta_theta = (*i).course - angle; //worked in hwl test

						//wrap the angle
						if (delta_theta > M_PI) {
							delta_theta = delta_theta-2*M_PI;
						}

						else {
							if (delta_theta <= -M_PI) {
								delta_theta = delta_theta + 2*M_PI;
							}
						}
	
						delta_theta_pom = delta_theta_pom + delta_theta;
			
						new_message_received = 0;
					}


				}// if ID
				
		} //for loop - messages




		// *********** UPDATE VA STATE ******************//

		if (update_VA_state) {


                        double nav_incr_x = -sygmoid(vp.x,navp.x,3,3)-sygmoid(vp.speedx,0,1,1);
			double nav_incr_y = -sygmoid(vp.y,navp.y,3,3)-sygmoid(vp.speedy,0,1,1);

			F.x = F.x + nav_incr_x;
                        F.y = F.y + nav_incr_y;




			vp.x = vp.x + vp.speedx * Td;
			vp.y = vp.y + vp.speedy * Td;

			vp.speedx = vp.speedx + F.x * Td;
			vp.speedy = vp.speedy + F.y * Td;

			vp.speed = sqrt(pow(vp.speedx,2)+pow(vp.speedy,2));
			vp.course = atan2(vp.speedy,vp.speedx);

			
			//delta_v = 1*delta_theta_pom;
			delta_v = sygmoid(delta_theta,0,2,2); //from emulator
			update_VA_state = 0;
			delta_theta_pom = 0; //from emulator 
			

		}

		// ************ UPDATE FLIGHT CONTROLLER *****************//

		// calculate controller output, send turn rate and speed to ebee eumulator/ ebee in reality

		
		if (SYNC == 1) {
			v = 14 + delta_v;
			if (v > 16) {
				v = 16;
			}

			if (v < 12) {
				v = 12;
			}
		}
		
		// FIELD TEST 
		point robot;
		convertFromGPStoXY(gapi.longitude(),gapi.latitude(),output); // in reality take this data from gapi
		robot.x = output[0];
		robot.y = output[1];
		robot.speedx =  gapi.airSpeed()*cos(-gapi.course()/360.0*2*M_PI+M_PI/2.0);
		robot.speedy = gapi.airSpeed()*sin(-gapi.course()/360.0*2*M_PI+M_PI/2.0);
		robot.speed = gapi.airSpeed();
		robot.course = -gapi.course()/360.0*2*M_PI+M_PI/2.0;

		double phi = -gapi.course()/360.0*2*M_PI+M_PI/2.0;



		if (vp.speed > VP_SPEED_LIMIT) {
			vp.speedx = VP_SPEED_LIMIT * cos(vp.course);
			vp.speedy = VP_SPEED_LIMIT * sin(vp.course);
		}



		omega = calculateTurnRate (robot, vp, phi, &delta_ni_p, 30, v, K, beta);


		gapi.setAileronCommand(-omega*(180/M_PI));
		gapi.setElevatorCommand(altitude);
		gapi.setThrustCommand(v);
		gapi.updateControl();


		gettimeofday(&t_current,NULL);
		timersub(&t_current, &t_previous, &t_diff);
		t_elapsed = t_diff.tv_sec+t_diff.tv_usec/1000000.0;

		convertFromXYtoGPS(vp.x,vp.y,output);
		float vAgent_lon = output[0];
		float vAgent_lat = output[1];


		//******************* LOGGING *************************//
		if (t_elapsed > DATA_SAMPLE_TIME) {
			gettimeofday(&t_previous,NULL);
			double timestamp = t_previous.tv_sec+t_previous.tv_usec/1000000.0;

			//convertFromGPStoXY(gapi.longitude(), gapi.latitude(), output);
			convertFromXYtoGPS(vp.x,vp.y,output);
std::cout<<"LOG"<<id<<" "<<timestamp<<" "<<gapi.longitude()<<" "<<gapi.latitude()<<" "<<output[0]<<" " <<output[1]<<" "<<phi<<" "<<norm<<" "<<v<< std::endl;

//std::cout<<"LOG"<<id<<" "<<timestamp<<" "<<ebee.lon<<" "<<ebee.lat<<" "<<output[0]<<" " <<output[1]<<" "<<ebee.v<<" "<<ebee.fi<<" "<<norm<<std::endl;
			time_current = time(NULL);

		
		}


		// *************** UPDATE MESSAGE DATA ********************//
		//sem_wait(&rw_sem);
		
		// TEST OUTDOOR
		msg.course = Angle(gapi.longitude(),gapi.latitude(),vAgent_lon,vAgent_lat);
		// FILL IN THE MESSAGE
		convertFromXYtoGPS(vp.x,vp.y,output);
		msg.lon = output[0];
		msg.lat = output[1];
		msg.speedx = vp.speedx;
		msg.speedy = vp.speedy;
		//va.course = robot.course;
		msg.id = id;
		msg.robotLon = gapi.longitude();
		msg.robotLat = gapi.latitude();
		//sem_post(&rw_sem);


	} // if (gapi.enableFlag())
	}//while

	// Exit the application
	return 0;

}


