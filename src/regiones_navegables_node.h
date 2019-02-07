#include "ros/ros.h"
#include <iostream>
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/Odometry.h>
#include <vector>

#include <math.h>
#include <limits>
#include <fstream>



const double inf = std::numeric_limits<double>::infinity();
const double pi = 3.14159265;
const int UNKNOWN = 0;
const int YES = 1;
const int NO = -1;


/** Suscripciones */
ros::Subscriber laserSub;
ros::Subscriber odomSub;
ros::Subscriber ground_truthSub;
ros::Subscriber objectiveSub;


/** Funciones */ 
void regiones_navigables();
bool comprobar_navigable();
void escribir_ficheros();


/*************** VARIABLES PARTE REGIONES ***************/
// Número rayos del LIDAR
const int num_laser = 360;

// Distancia máxima a la que queremos que llegue el LIDAR
const float max_range_laser = 30.0; // metros

// Distancia que define cuándo hay una discontinuidad. Sugieren un valor igual o mayor al diámetro del robot.
const float rango_discontinuidad = 1.5; // metros


struct position{
	float x;
	float y;
	float theta;
};
struct position odom, pos_obj, ground_truth;

struct region_charac{
	float gap_1;
	float gap_2;
	bool obj_inside;
	float obj_ang_nearness;
	int navigable;
};

std::vector <float > v_laser, v_PND;
std::vector <int> v_i_discontinuities;
std::vector <region_charac> v_regions;