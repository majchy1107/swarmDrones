#include <sstream>
#include <fstream>
#include <iostream>
#include <vector>
#include <algorithm>

#include <stdio.h>  
#include <string.h> 
#include <stdlib.h>
#include<stdio.h>
#include<time.h>
#include<math.h>

//#include "NetworkNode.h"
#include "gapi.h"
#include "SwarmHelper.h"
#include "tnt/jama_eig.h"
#include "CommonConstants.h"


#define NAV_LINE_FOLLOW_TAU 20.0
#define NAV_LINE_ENTRY_ANGLE M_PI/4.0
#define K_HEADING 0.5
#define MAX_TURNRATE (35/180.0)*M_PI
#define EPFL_HOME_LONGITUDE 6.5690000
#define EPFL_HOME_LATITUDE 46.523400
#define NUM_OF_WPTS 4

typedef struct {
        float distance;
        int id;
}NEIGHBORS;


typedef struct {
	std::vector<float> longitudes;
	std::vector<float> latitudes;
	std::vector<int> wptDirections;
} waypoints;

// 'Point' structure
struct point {
	
    double x; // X coordinate
    double y; // Y coordinate
    double speed;
    double speedx;
    double speedy;
    double course;
	
};


float rand_val( float a, float b);

float calculateTurnRate (point p, point pcp, double phi, double *delta_ni_p, double rd, double v, double K, double beta);

void centerGWaypoint(float longitude, float latitude, float heading, float radius,int dir, float centerOutput[]);

void WaypointHeading(float startLon, float startLat, float goalLon, float goalLat, float *heading, float *distance);

void DistanceFromLine(float currentLon, float currentLat, float baseLon, float baseLat, float heading, float *error);

float FollowLine(float *desiredTurnRate, float heading, float baseLon, float baseLat, GAPI &gapi); 

void convertFromGPStoXY(float longitude, float latitude, float outputArray[]);

void convertFromXYtoGPS(float x, float y, float GPS[]);

float calculateDistance (float wptLon1, float wptLat1, float wptLon2, float wptLat2);

int getNeighbour(int currIndex, int direction, int arraySize);

waypoints convertInputArrayFromGPStoXY(float wptLon[], float wptLat[]);

waypoints convertListFromXYtoGPS(waypoints wpin);

void DistanceAndAngle(float swingletLon, float swingletLat,float course, float baseLon, float baseLat,float *distance, float *angle);

waypoints getWpts(float wptLon[], float wptLat[], float robotPos[], float rad);

int getFirstWPTDir (waypoints wpts);

float Distance(float wpt1Lon, float wpt1Lat, float wpt2Lon, float wpt2Lat);

float Angle(float swinglet_lon, float swinglet_lat, float longitude, float latitude);

double bump_fcn(double x, double h);

double sigma_norm(double norm, double e);

double euclid_norm (double x1, double y1, double x2, double y2);

double Gvector_angle(double my_point[], double neigh_point[], double distance, double vector_nij[]);

double sygmoid (double p, double pr, double a, double b);

bool initWaypoints(int id, float *start_longitude, float *start_latitude  ) ;

void writeToFile(const char * fileName, int timestamp, int longitude, int latitude, int id); 

int parseLQfile(const char *fileName, int neighbors[], double linkQ[],int*  num_neighbors);

void mergeTopologyMatrix(int myMatrix[TOTAL_ROBOT_NUM+1][TOTAL_ROBOT_NUM+1],int neighMatrix[TOTAL_ROBOT_NUM+1][TOTAL_ROBOT_NUM+1],int agents_num, int myId);

void calculateLaplacian(int mytopology[TOTAL_ROBOT_NUM+1][TOTAL_ROBOT_NUM+1], int myLaplacian[TOTAL_ROBOT_NUM+1][TOTAL_ROBOT_NUM+1],int agents_num) ;

bool isSafeNeighbor(int my_id, int neigh_id, int mytopology[TOTAL_ROBOT_NUM+1][TOTAL_ROBOT_NUM+1], int agents_num) ;
