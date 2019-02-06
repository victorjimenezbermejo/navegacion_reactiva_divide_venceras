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


/** Suscripciones */
ros::Subscriber laserSub;


/** Funciones */ 
void escribir_ficheros();


/*************** VARIABLES PARTE REGIONES ***************/
// Número rayos del LIDAR
const int num_laser = 360;

// Distancia máxima a la que queremos que llegue el LIDAR
const float max_range = 30.0; // metros

// Distancia que define cuándo hay una discontinuidad. Sugieren un valor igual o mayor al diámetro del robot.
const float rango_discontinuidad = 1.5; // metros



std::vector <float > v_PND;
std::vector <int> v_i_discontinuities;
std::vector <std::pair<float, float> > v_regions;