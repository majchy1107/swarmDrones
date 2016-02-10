#include "SwarmHelper.h"


bool initWaypoints(int id, float *start_longitude, float *start_latitude  ) {


if ((id >= 1) && (id <= 10)) {
        if (id == 1) {
                *start_latitude = 46.524392;
                *start_longitude = 6.570561;
        }

        if (id == 2) {
                *start_longitude  = 6.570551;
                *start_latitude = 46.524378;
        }

        if (id == 3) {
                *start_longitude = 6.570575;
                *start_latitude = 46.524369;
        }

        if (id == 4) {
                *start_longitude = 6.570586;
                *start_latitude = 46.524387;
        }

        if (id == 5) {
                *start_longitude = 6.570561;
                *start_latitude = 46.524394;
        }

        if (id == 6) {
                *start_longitude = 6.570584;
                *start_latitude = 46.524368;
        }

        if (id == 7) {
                *start_longitude = 6.570546;
                *start_latitude = 46.524350;
        }

        if (id == 8) {
                *start_longitude = 6.570572;
                *start_latitude = 46.524356;
        }

        if (id == 9) {
                *start_longitude = 6.570521;
                *start_latitude = 46.524336;
        }

        if (id == 10) {
                *start_longitude = 6.570570;
                *start_latitude = 46.524327;
        }
        return 1;
} else {
        return 0;
}

}


void writeToFile(const char * fileName, int timestamp, int longitude, int latitude, int id) {

        // open file for writing
        FILE  *fileid;
        float data1 = 0.0;
        float data2 = 0.0;
        float data3 = 0.0;
        float data4 = 0.0;
        fileid = fopen(fileName,"a");
        fprintf(fileid,"%d\t%d\t%d\t%f\t%f\t%f\t%f\t%d\n",timestamp,longitude,latitude,data1,data2,data3,data4,id);
        fclose(fileid);
}


int parseLQfile(const char *fileName, int neighbors[], double linkQ[],int*  num_neighbors) {

        const int MAX_CHAR_LINE = 500;
        const int MAX_DATA_LINE = 20;
        const char* const DELIMITER = "\t";
        int line_count = 0;
        int neighbors_counter = 0;


        ifstream fin;
        fin.open(fileName);
        if (!fin.good()) {
                return 0;
        }

        while (!fin.eof()) {

                char buf[MAX_CHAR_LINE];
                fin.getline(buf, MAX_CHAR_LINE);

                int n = 0;

                const char* token[MAX_DATA_LINE] = {};

                token[0]=strtok(buf,DELIMITER);
                if (token[0]) {
                        for (n=1; n < MAX_DATA_LINE; n++) {
                                token[n] = strtok(0, DELIMITER);
                                if (!token[n]) break;
                        }
                line_count++;
		//std::cout<<line_count<<std::endl;

                }

                if ((line_count > 2) && (!fin.eof())) {
                        neighbors_counter ++;

                        std::string ip_string = token[0];
                        string ip_delimiter = ".";
                        size_t current;
                        size_t next = -1;
                        do {
                                current = next + 1;
                                next = ip_string.find_first_of(ip_delimiter, current);
                                //std::cout << current << std::endl;
                                if (current == 12) {
                                        neighbors[neighbors_counter] = atoi(ip_string.substr(current, next-current).c_str());
                                }
                        } while (next != std::string::npos);

                        linkQ[neighbors_counter] = atof(token[3])/(atof(token[2])*atof(token[1]));
                        //std::cout<<linkQ[neighbors_counter]<<" "<<neighbors[neighbors_counter]<<std::endl;
                }

        }

        *num_neighbors = neighbors_counter;
        fin.close();
}


void mergeTopologyMatrix(int myMatrix[TOTAL_ROBOT_NUM+1][TOTAL_ROBOT_NUM+1],int neighMatrix[TOTAL_ROBOT_NUM+1][TOTAL_ROBOT_NUM+1],int agents_num, int myId){

for (int i=1; i<=agents_num; i++){
        for (int j=1; j<=agents_num; j++) {
                if (i!=myId) {
                        myMatrix[i][j]=neighMatrix[i][j];
                }
        }
}

}

void calculateLaplacian(int mytopology[TOTAL_ROBOT_NUM+1][TOTAL_ROBOT_NUM+1], int myLaplacian[TOTAL_ROBOT_NUM+1][TOTAL_ROBOT_NUM+1],int agents_num) {

        for (int i=1; i<=agents_num; i++) {
                int sum_row = 0;
                for(int j=1; j<=agents_num; j++) {
                        sum_row+=mytopology[i][j];
                        myLaplacian[i][j]=-mytopology[i][j];
                }
                myLaplacian[i][i]=sum_row;
        }
}


bool isSafeNeighbor(int my_id, int neigh_id, int mytopology[TOTAL_ROBOT_NUM+1][TOTAL_ROBOT_NUM+1], int agents_num) {
        // calculate laplacian
        int myLaplacian[TOTAL_ROBOT_NUM+1][TOTAL_ROBOT_NUM+1]={{}};
        int mytopology_aux[TOTAL_ROBOT_NUM+1][TOTAL_ROBOT_NUM+1];

        for(int i=1; i<=agents_num; i++){  //REMOVE THIS CODE!!!!!
                for (int j=1; j<=agents_num; j++) {
                        mytopology_aux[i][j] = mytopology[i][j];
                }
        }

        //memcpy(mytopology_aux, mytopology, sizeof(mytopology));
        mytopology_aux[neigh_id][my_id]=0;
        mytopology_aux[my_id][neigh_id]=0;

        calculateLaplacian(mytopology_aux, myLaplacian, agents_num);
        TNT::Array2D<double> Laplacian(agents_num,agents_num);
        // REMOVE THIS CODE: convert from 2D array to TNT::Array2D
        for(int i=1; i<=agents_num; i++){
                for (int j=1; j<=agents_num; j++) {
                        Laplacian[i-1][j-1] = myLaplacian[i][j];
                }
        }
        JAMA::Eigenvalue<double> eigs(Laplacian);
        TNT::Array1D<double> eig_vals;
        eigs.getRealEigenvalues(eig_vals);

      std::cout<<"EIGENVALUES "<< my_id << " ";
      for (int i=0; i<agents_num; i++)
              std::cout<<eig_vals[i]<<" ";
              std::cout<<std::endl;
        if (eig_vals[1]<=0) 
                return 0;
        else
                return 1;

}


double bump_fcn( double x, double h) {

double f = 0;

if (( x>=0 ) && (x < h)) 
{
    
    f = 1;
    
}

if ((x >= h) && (x <= 1)) 
{
    
    f = 0.5*(1+cos(M_PI*(x-h)/(1-h)));
    
}

if ((x < 0) || (x > 1)) 
{
   
    f = 0;
    
}  


return f;
}


double sigma_norm(double norm, double e) {

    double res = 0;
    return res = (1/e)*(sqrt(1+e*pow(norm,2))-1);
}

double euclid_norm (double x1, double y1, double x2, double y2) {

    return sqrt(pow(x2-x1,2)+pow(y2-y1,2));

}




double Gvector_angle(double my_point[], double neigh_point[], double distance, double vector_nij[]) {
    
    
    double vec_gradient_abs = sqrt(1+0.1*pow(distance,2));
        double nij_x = (my_point[0]-neigh_point[0])/vec_gradient_abs;
    double nij_y = (my_point[1]-neigh_point[1])/vec_gradient_abs;

    return atan2(nij_y,nij_x);
}   


double sygmoid(double p,double pr,double a,double b)  {

    return 0.5*((((a+b)*(p-pr))/sqrt(1+pow((p-pr),2)))+(a-b));

}



// Calculates center of waypoint to follow from current Swinglet pos and course
void centerGWaypoint(float longitude, float latitude, float heading, float radius,int dir, float centerOutput[]) {

float xypos[2];
float x,y,phi,phi_c,x_c,y_c;
float GPS_c[2];

    convertFromGPStoXY(longitude,latitude,xypos);
    x=xypos[0];
    y=xypos[1];
    phi=-heading/360.0*2*M_PI+M_PI/2.0;
    //determine the center of waypoint, tangential to current course (+-pi/2)
    if (dir==1) {
        //clockwise direction
        phi_c=phi-M_PI/2;
    } else {
        //counterclockwise direction
        phi_c=phi+M_PI/2;
    }
    x_c=x+radius*cos(phi_c);
    y_c=y+radius*sin(phi_c);

    //output
    convertFromXYtoGPS(x_c,y_c,centerOutput);
}

// calculates the heading of a line that connects two GPS positions (center of a waypoint and current Swinglet position)

void WaypointHeading(float startLon, float startLat, float goalLon, float goalLat, float *heading, float *distance) {

float startX,startY,goalX,goalY;
float outputArray[2];

convertFromGPStoXY(startLon,startLat,outputArray);
startX=outputArray[0];  
startY=outputArray[1];

convertFromGPStoXY(goalLon,goalLat,outputArray);
goalX=outputArray[0];
goalY=outputArray[1];

*heading=atan2(goalY-startY,goalX-startX);

//prevent big angles
while (*heading > M_PI)
    *heading -= 2.0*M_PI;
while (*heading < -M_PI)
    *heading += 2.0*M_PI;


*distance=sqrt(pow(goalX-startX,2)+pow(goalY-startY,2));

}


// Swinglet distance from the line
void DistanceFromLine(float currentLon, float currentLat, float baseLon, float baseLat, float heading, float *error) {

    float dx,dy;
    long lonDiff = baseLon - currentLon;
    long latDiff = baseLat - currentLat;
    float outputArray[2];
    
    convertFromGPStoXY(lonDiff,latDiff,outputArray);
    dx = outputArray[0];
    dy = outputArray[1];
    
    //check this formula - could be a bottleneck

    *error = sinf(heading) * dx + cosf(heading) * dy;
    //*error = cosf(heading) * dx - sinf(heading) * dy;
    
    
}

//Turnrate to follow the line

float FollowLine(float *desiredTurnRate, float heading, float baseLon, float baseLat, GAPI &gapi) {

    float tempDheading;
    float error;
    
    DistanceFromLine (gapi.longitude(), gapi.latitude(), baseLon, baseLat, heading, &error);
    
    if (error > NAV_LINE_FOLLOW_TAU)
    {
        tempDheading = heading + NAV_LINE_ENTRY_ANGLE;
    }
    else if (error < -NAV_LINE_FOLLOW_TAU) 
    {
        tempDheading = heading - NAV_LINE_ENTRY_ANGLE;
    }
    else
    {
        error *= NAV_LINE_ENTRY_ANGLE/NAV_LINE_FOLLOW_TAU;
        tempDheading = heading + error;
    }
    
    if (tempDheading > M_PI) 
        tempDheading -= 2.0 * M_PI;
    else if (tempDheading < - M_PI) 
        tempDheading += 2.0 * M_PI;

    //add part with turnrate
    float course = -gapi.course()/360.0*2*M_PI+M_PI/2.0; //course in rad
    float headingDiff = tempDheading - course;
    float turnRate;
    // Normalize angle
    while (headingDiff > M_PI)
        headingDiff -= 2.0*M_PI;
    while (headingDiff < -M_PI)
        headingDiff += 2.0*M_PI;

    turnRate = *desiredTurnRate + K_HEADING * headingDiff;
    
    if (turnRate > MAX_TURNRATE)
        turnRate = MAX_TURNRATE;
    if (turnRate < -MAX_TURNRATE) 
        turnRate = -MAX_TURNRATE;

    return turnRate;

}


// CONVERSION FUNCTIONS 

// Function that convert GPS values (longitude and latitude) to meters - GAPI VERSION
void convertFromGPStoXY(float longitude, float latitude, float outputArray[]) {
    
    outputArray[0] = longitude * (M_PI / 180) * 6378000 * cos(((latitude) * (M_PI / 180))); 
    outputArray[1] = latitude * (M_PI / 180) * 6378000;
    
}

// Function that convert GPS values (longitude and latitude) to meters - GAPI VERSION
void convertFromXYtoGPS(float x, float y, float GPS[]) {
    
    GPS[1] = y / ((M_PI / 180.0) * 6378000); //latitude
    GPS[0] = x / ((M_PI / 180.0) * 6378000 * cos(GPS[1]*(M_PI / 180 )));
    
}

float calculateDistance (float wptLon1, float wptLat1, float wptLon2, float wptLat2) {
    return sqrt(pow((wptLat2 - wptLat1),2) + pow((wptLon2 - wptLon1),2));
}

int getNeighbour(int currIndex, int direction, int arraySize) {
    int nextIndex = currIndex + direction;
    if (nextIndex < 0) {
        nextIndex = nextIndex + arraySize;
    } else if (nextIndex >= arraySize) {
        nextIndex = 0;
    }
    return nextIndex;
}

waypoints convertInputArrayFromGPStoXY(float wptLon[], float wptLat[]) {
    waypoints wpts;
    wpts.longitudes = std::vector<float>();
    wpts.latitudes = std::vector<float>();

    int i = 0;
    for (i = 0; i < NUM_OF_WPTS; i++) {
        float converted[2];
        convertFromGPStoXY(wptLon[i], wptLat[i], converted);
        wpts.longitudes.push_back(converted[0]);
        wpts.latitudes.push_back(converted[1]);
    }
    return wpts;
}

waypoints convertListFromXYtoGPS(waypoints wpin) {
    waypoints wpts;
    wpts.longitudes = std::vector<float>();
    wpts.latitudes = std::vector<float>();
    wpts.wptDirections = wpin.wptDirections;

    int i = 0;
    for (i = 0; i < wpin.longitudes.size(); i++) {
        float converted[2];
        convertFromXYtoGPS(wpin.longitudes[i], wpin.latitudes[i], converted);
        wpts.longitudes.push_back(converted[0]);
        wpts.latitudes.push_back(converted[1]);
    }
    return wpts;
}

waypoints getWpts(float wptLon[], float wptLat[], float robotPos[], float rad) {

    waypoints wpts;
    wpts.longitudes = std::vector<float>();
    wpts.latitudes = std::vector<float>();
    wpts.wptDirections = std::vector<int>();
    
    //convert GPS to XY
    waypoints input = convertInputArrayFromGPStoXY(wptLon, wptLat);

    float pos[2];
    convertFromGPStoXY(robotPos[0], robotPos[1], pos);

    int i = 0;
    int clWpt = 0; //index of closest waypoint
    float minDist = 0;
    for (i = 0; i < NUM_OF_WPTS; i++) {
        if (i == 0) {
            minDist = calculateDistance(input.longitudes[i], input.latitudes[i], pos[0], pos[1]);
        } else {
            float dist = calculateDistance(input.longitudes[i], input.latitudes[i], pos[0], pos[1]);
            if (dist < minDist) {
                minDist = dist;
                clWpt = i;
            }
        }
    }
    
    //std::cout << "Closest to: " << clWpt << "\n";

    int ln = getNeighbour(clWpt, -1, NUM_OF_WPTS);
    int rn = getNeighbour(clWpt, 1, NUM_OF_WPTS);

    //get longer edge 
    float distLeft = calculateDistance(input.longitudes[clWpt], input.latitudes[clWpt], input.longitudes[ln], input.latitudes[ln]);
    float distRight = calculateDistance(input.longitudes[clWpt], input.latitudes[clWpt], input.longitudes[rn], input.latitudes[rn]);    
    
    int pair1[2];
    int pair2[2];

    //change the condition to get sweeping by longer or shorter edge
    if (distLeft < distRight) {
        pair1[0] = clWpt;
        pair1[1] = rn;
        pair2[0] = ln;
        pair2[1] = getNeighbour(ln, -1, NUM_OF_WPTS);
    } else {
        pair1[0] = clWpt;
        pair1[1] = ln;
        pair2[0] = rn;
        pair2[1] = getNeighbour(rn, 1, NUM_OF_WPTS);
    }

    //std::cout << pair1[0] << " "<< pair1[1] << " "<< pair2[0] << " "<< pair2[1];
    
    float angle1 = atan2(input.latitudes[pair1[1]] - input.latitudes[pair1[0]], input.longitudes[pair1[1]] - input.longitudes[pair1[0]]);
    float angle2 = atan2(input.latitudes[pair2[1]] - input.latitudes[pair2[0]], input.longitudes[pair2[1]] - input.longitudes[pair2[0]]);

    wpts.longitudes.push_back(input.longitudes[pair1[0]]);
    wpts.latitudes.push_back(input.latitudes[pair1[0]]);

    bool stopCond = false;
    float distance1 = calculateDistance(input.longitudes[pair1[0]], input.latitudes[pair1[0]], input.longitudes[pair1[1]], input.latitudes[pair1[1]]);
    float distance2 = calculateDistance(input.longitudes[pair2[0]], input.latitudes[pair2[0]], input.longitudes[pair2[1]],  input.latitudes[pair2[1]]);
    
    i = 0;
    while (!stopCond) {
        if ((i % 2) == 0) {
            //add wpt oposite to the starting edge
            float newLong = (2*rad*(i+1))*cos(angle2) + input.longitudes[pair2[0]];
            float newLat = (2*rad*(i+1))*sin(angle2) + input.latitudes[pair2[0]];

            float newDist = calculateDistance(newLong, newLat, input.longitudes[pair2[1]], input.latitudes[pair2[1]]);
            if (newDist > distance2) {
                stopCond = true;
            } else {
                distance2 = newDist;                
                wpts.longitudes.push_back(newLong);
                wpts.latitudes.push_back(newLat);
            }
            
        } else {
            float newLong = (2*rad*(i+1))*cos(angle1) + input.longitudes[pair1[0]];
            float newLat = (2*rad*(i+1))*sin(angle1) + input.latitudes[pair1[0]];

            float newDist = calculateDistance(newLong, newLat, input.longitudes[pair1[1]], input.latitudes[pair1[1]]);  
            if (newDist > distance1) {
                stopCond = true;
            } else {
                distance1 = newDist;                
                wpts.longitudes.push_back(newLong);
                wpts.latitudes.push_back(newLat);
            }
        }
        i++;
    }

    wpts.wptDirections.push_back(getFirstWPTDir(wpts));
    //std::cout << "Dist 1 " << calculateDistance(wpts.longitudes[1],wpts.latitudes[1],wpts.longitudes[3],wpts.latitudes[3]) << "\n";
    //std::cout << "Dist 2 " << calculateDistance(wpts.longitudes[0],wpts.latitudes[0],wpts.longitudes[2],wpts.latitudes[2]) << "\n";
    return convertListFromXYtoGPS(wpts);
}

int getFirstWPTDir (waypoints wpts) {
    /* returns the direction of first waypoint
        1 - clokwise
        0 - counter-clock wise
    */

    float ang1 = atan2((wpts.latitudes[1] - wpts.latitudes[0]), (wpts.longitudes[1] - wpts.longitudes[0]));
    float ang2 = atan2((wpts.latitudes[3] - wpts.latitudes[1]), (wpts.longitudes[3] - wpts.longitudes[1]));

    if (ang1 < 0) {
        ang1 += (2*M_PI);
    }

    if (ang2 < 0) {
        ang2 += (2*M_PI);
    }

//  std::cout << ang1 << " " << ang2 << "\n";

    if ((ang1 - ang2) > 0) {
        return 0; 
    } else {
        return 1;
    }
}

float rand_val( float a, float b) {
    return a + (b-a) * ( (float) rand() / (float) RAND_MAX);
}


float calculateTurnRate (point p, point pcp, double phi, double *delta_ni_p, double rd, double v, double K, double beta) {
    
    //calculate desired speed
    point vd;
    double r=sqrt(pow((p.x-pcp.x),2)+pow((p.y-pcp.y),2));
    
    // override beta
    //beta=v/(r*(pow(r,2)+pow(rd,2)));

    vd.speedx=-beta*((pow(r,2)-pow(rd,2))*(p.x-pcp.x)+2*r*rd*(p.y-pcp.y));
        vd.speedy=-beta*(-2*r*rd*(p.x-pcp.x)+(pow(r,2)-pow(rd,2))*(p.y-pcp.y)); 
    
    point vr;
    //vr.speedx=v*cos(phi)-pcp.speedx; //check orientation and position obtained from sensors
    //vr.speedy=v*sin(phi)-pcp.speedy; //match orientation and coord system

    vr.speedx = p.speedx-pcp.speedx;
    vr.speedy = p.speedy - pcp.speedy;
    //calculate ni

    double ni=0;
    ni=atan2(vr.speedy,vr.speedx);

    double vr_sq,k_ni;
    vr_sq=pow(v,2)+pow(pcp.speedx,2)+pow(pcp.speedy,2)-2*v*(pcp.speedx*cos(phi)+pcp.speedy*sin(phi));
    k_ni=(pow(v,2)-v*(pcp.speedx*cos(phi)+pcp.speedy*sin(phi)))/(vr_sq);

    double ni_d,vni_d=0;
    ni_d=atan2(vd.speedy,vd.speedx);
    vni_d=4*v*((rd*pow(r,2))/pow((pow(r,2)+pow(rd,2)),2));

    double delta_ni=0;
    delta_ni=ni-ni_d;

    //prevent discontinuities

    if ((delta_ni-*delta_ni_p)>M_PI)
        delta_ni=delta_ni-(2*M_PI);
    if ((delta_ni-*delta_ni_p)<-M_PI)
        delta_ni=delta_ni+(2*M_PI);

    //turn rate calculation
    double omega=-K*delta_ni+vni_d/k_ni;

    *delta_ni_p=delta_ni;

    //return turn rate
     return omega;
}

void DistanceAndAngle(float swingletLon, float swingletLat,float course, float baseLon, float baseLat,float *distance, float *angle) {
    
    float outputXYswing[2],swingletX,swingletY,swingletphi,distancephi,angle_switch,baseX,baseY;
    
    convertFromGPStoXY(swingletLon,swingletLat,outputXYswing);
    swingletX = outputXYswing[0];
    swingletY = outputXYswing[1];
    convertFromGPStoXY(baseLon,baseLat,outputXYswing);
    baseX=outputXYswing[0];
    baseY=outputXYswing[1];

    // swinglet distance from base

    *distance = sqrt(pow(baseX-swingletX,2)+pow(baseY-swingletY,2)); 
    
    //swinglet course
    
    swingletphi=-course/360.0*2*M_PI+M_PI/2.0;
    distancephi=atan2(baseY-swingletY,baseX-swingletX);
    
    
    angle_switch=swingletphi-distancephi;
    
    while (angle_switch > M_PI)
        angle_switch -= 2.0*M_PI;
    while (angle_switch < -M_PI)
        angle_switch += 2.0*M_PI;
    *angle=angle_switch;

}


float Distance(float wpt1Lon, float wpt1Lat, float wpt2Lon, float wpt2Lat) {

    float data1[2];
    float data2[2];
    float distance;


    //std::cout<<"gapi lon in distance fcn"<<wpt1Lon<<std::endl;
    //std::cout<<"gapi lat in distance fcn"<<wpt1Lat<<std::endl;
    //std::cout<<"planner lon in distance fcn"<<wpt2Lon<<std::endl;
    //std::cout<<"planner lat in distance fcn"<<wpt2Lat<<std::endl;

    convertFromGPStoXY(wpt1Lon,wpt1Lat,data1);
    convertFromGPStoXY(wpt2Lon,wpt2Lat,data2);
    distance = sqrt(pow((data1[0]-data2[0]),2)+pow((data1[1]-data2[1]),2));

    //std::cout<<"distance in Distance fcn"<<distance<<std::endl;
    return distance;

}

float Angle( float swinglet_lon, float swinglet_lat, float longitude, float latitude) {
    

    //convert from GPS to XY
    float data1[2];
    float data2[2];


    convertFromGPStoXY(longitude,latitude,data1);
    convertFromGPStoXY(swinglet_lon,swinglet_lat,data2);

    float angle = atan2(data1[1]-data2[1], data1[0]-data2[0]);


    while (angle > M_PI)
        angle -= 2.0*M_PI;
    while (angle < -M_PI)
        angle += 2.0*M_PI;
    
    return angle;
     
}





