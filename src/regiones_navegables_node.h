#include "ros/ros.h"
#include <iostream>
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/Odometry.h>
#include <vector>

#include <math.h>
#include <limits>
#include <fstream>


#define ESCRIBIR_FICHEROS // Sirve para escribir o no ficheros con datos para matlab


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





/*************** VARIABLES PARTE REGIONES ***************/
// Número rayos del LIDAR
const int num_laser = 360;

// Distancia máxima a la que queremos que llegue el LIDAR
const float max_range_laser = 30.0; // metros

// Distancia que define cuándo hay una discontinuidad. Se toma como discontinuidad un hueco por el que puede entrar el robot
const float rango_discontinuidad = 1.5; // metros

// Radio de golpe posible por cada punto del lidar. Tengo un láser detectando, pero no sé cómo de voluminoso es el objeto impactado
const float radio_puntos_laser = 0.2; // metros


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
	int nearest_gap;
	float obj_ang_nearness;
	int navigable;
};

struct laserPoint{
	float dist;
	float angle;
	float x;
	float y;
};

std::vector <float > v_laser, v_PND;
std::vector <int> v_i_discontinuities;
std::vector <region_charac> v_regions;


/** Funciones */ 
void regiones_navigables();
bool comprobar_navigable(region_charac *, float, float);
void escribir_ficheros();
float fixAngle180Rad(float);
float fixAngle360Rad(float);