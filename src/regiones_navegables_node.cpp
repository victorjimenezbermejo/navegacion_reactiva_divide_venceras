/**
Control reactivo para la evitación de obstáculos

Se divide en tres partes:
1. Identificar las regiones por las que se puede mover y elegir una
2. Decidir qué clase de situación es
3. Ejercer las acciones correspondientes a la situación elegida


1º IDENTIFICAR LAS REGIONES POR LAS QUE SE PUEDE MOVER Y ELEGIR UNA
Se divide en cinco partes:
1. Nearest diagram (ND)
2. Encontrar las discontinuidades del entorno.
3. Definir una región entre cada dos discontinuidades.
4. Filtrar regiones y elegir una.
5. Calcular si es navigable
x. Comprobar si hay objetos dentro de un área de seguridad


EXPLICACIÓN 1. Nearest diagram (ND)
Es como poner las medidas invertidas. Lo que está más cerca tiene más altura en el diagrama y algo que está tan lejos que no se 
alcanza a medir tiene distancia cero.
Puntos ND (PND) = máximo rango láser + rango discontinuidad - distancia medida

EXPLICACIÓN 3. Definir una región entre cada dos discontinuidades.
   Condiciones para ser una región:
   - Una región se define entre dos discontinuidades (angulos donde hay cambio de dist grande) denotadas por left (l) y right (r)
   - Una región debe estar definida por dos discontinuidad y NO puede contener otras discontinuidades dentro.
   - Las discontinuidades están en los extremos de este conjunto de ángulos.
   - Al menos una de las dos discontinuidades (l y r) debe ser creciente. Se comprueba si es creciente o decreciente de la siguiente manera:
     - a = dist.at(l-1) - dist.at(l) --> a > 0 = creciente, a < 0 = decreciente
     - a = dist.at(r) - dist.at(r-1) --> a > 0 = creciente, a < 0 = decreciente
   Esta última condición es la que va a hacer que solo elijamos aquellas zonas que nos permiten avanzar
   + Extra: sí puede haber dos regiones seguidas

**/

#include "regiones_navegables_node.h"


/** Bueno esto debería ser temporal. Básicamente para las pruebas */
void objectiveCallback(const nav_msgs::Odometry::ConstPtr msg_obj)
{
	pos_obj.x = msg_obj->pose.pose.position.x;
	pos_obj.y = msg_obj->pose.pose.position.y;
	pos_obj.theta = 2*atan2(msg_obj->pose.pose.orientation.z, msg_obj->pose.pose.orientation.w);

	printf("Objetivo: %f, %f, %f\n", pos_obj.x, pos_obj.y, pos_obj.theta*180.0/pi);
}


void laserCallback(const sensor_msgs::LaserScan::ConstPtr& msg_laser)
{
	v_laser = msg_laser->ranges;
}

void odomCallback(const nav_msgs::Odometry::ConstPtr& msg_odom)
{
	odom.x = msg_odom->pose.pose.position.x;
	odom.y = msg_odom->pose.pose.position.y;
	odom.theta = 2*atan2(msg_odom->pose.pose.orientation.z, msg_odom->pose.pose.orientation.w);

	//printf("Odom: %f, %f, %f\n", odom.x, odom.y, odom.theta*180.0/pi);
}

// En realidad lo uso porque publica todo el rato. Odom solo publica cuando me muevo. 
void ground_truthCallback(const nav_msgs::Odometry::ConstPtr msg_ground_truth)
{
	ground_truth.x = msg_ground_truth->pose.pose.position.x;
	ground_truth.y = msg_ground_truth->pose.pose.position.y;
	ground_truth.theta = 2*atan2(msg_ground_truth->pose.pose.orientation.z, msg_ground_truth->pose.pose.orientation.w);

	printf("Ground truth: %f, %f, %f\n", ground_truth.x, ground_truth.y, ground_truth.theta*180.0/pi);
}



int main(int argc, char **argv) {
	ros::init(argc, argv, "regiones_navegables_node"); // Inicializa un nuevo nodo llamado v_nav
	ros::NodeHandle nh;

	// Suscripciones
	laserSub = nh.subscribe("/Doris/scan", 1, laserCallback);
	odomSub = nh.subscribe("/Doris/odom", 1, odomCallback);
	ground_truthSub = nh.subscribe("/Doris/ground_truth/state", 1, ground_truthCallback);
	objectiveSub = nh.subscribe("punto_bjetivo", 1, objectiveCallback);

	v_PND.reserve(180);

	printf("Nodo lanzado\n");

	pos_obj.x = 10.0;
	pos_obj.y = 0.0;
	
	
	ros::Rate r(10);
	while (ros::ok())
	{
		regiones_navigables();
		ros::spinOnce();
		r.sleep();
	}


	return 0;
};




void regiones_navigables()
{
	float gap_1, gap_2, slope_1, slope_2;
	struct region_charac region;

	v_PND.clear();
	v_i_discontinuities.clear();
	v_regions.clear();



	/** 0. Calcular máxima distancia */
	float distance_objective = std::sqrt( std::pow(pos_obj.x - ground_truth.x, 2.0) + std::pow(pos_obj.y - ground_truth.y, 2.0) );
	float angle_objective = std::atan2(pos_obj.y - ground_truth.y, pos_obj.x - ground_truth.x) - ground_truth.theta; 

	float max_range = max_range_laser;
	if(distance_objective < max_range)
	{
		max_range = distance_objective;
	}

	printf("Distancia al objetivo %f, máximo rango %f\n", distance_objective, max_range);


	/** 1. Nearest diagram */
	// Doris toma medidas entre -90 y 90, cada 0.5 grados -> 360 medidas, 180º
	for(int i = 0; i < v_laser.size(); i++)
	{
		if(v_laser.at(i) > max_range)
		{ 
			v_PND.push_back(0.0);
		}
		else
		{
			v_PND.push_back(max_range + rango_discontinuidad - v_laser.at(i));
		}
	}


	/** 2. Encontrar las discontinuidades en el entorno */
	// Comparamos la distancia de cada ángulo con la del anterior. Si supera un rango -> discontinuidad	
	// El caso de 90 y -90 es especial. Si no se detecta nada -> sí lo cogemos como apertura de valle (discontinuidad)
	
	if(v_PND.front() == 0.0){
		v_i_discontinuities.push_back(0);
	}
	for(int i = 1; i < v_PND.size(); i++) // Ojito int i = 1 
	{
		if(std::abs( v_PND.at(i) - v_PND.at(i-1) ) > rango_discontinuidad)
		{
			// Ojito i no es el ángulo. La operación realizada cambia el vector de derecha a izquierda a ángulos entre -90 y 90
			v_i_discontinuities.push_back(i); 
			printf("Discontinuidad en el ángulo %f, con magnitud %f\n", 90.0 - (i) / 2.0, std::abs( v_PND.at(i) - v_PND.at(i-1) ));
		}
	}
	if(v_PND.back() == 0.0){
		v_i_discontinuities.push_back(v_PND.size() - 1);
	}

	printf("Número discontinuidades (contando con -90 y 90): %d\n\n", v_i_discontinuities.size());



	/** 3. Definir las regiones */
	float dist_1, dist_2;
	if(v_i_discontinuities.size() > 1)
	{
		// Al ir de uno en uno en orden ya cumplimos la condición de que no haya regiones intermedias y las dos estén en los extremos
		for(int i = 0; i < v_i_discontinuities.size()-1; i++) // .size() - 1
		{
			gap_1 = v_i_discontinuities.at(i);
			gap_2 = v_i_discontinuities.at(i+1);

			if(gap_1 == 0){
				slope_1 = inf;
			}
			else{
				slope_1 = v_PND.at(gap_1 - 1) - v_PND.at(gap_1);
			}
			
			slope_2 = v_PND.at(gap_2) - v_PND.at(gap_2 - 1);	
			
			// Si una de las dos es creciente aceptamos como región
			if( (slope_1) > 0 || (slope_2) > 0)
			{
				region.gap_1 = v_i_discontinuities.at(i);
				region.gap_2 = v_i_discontinuities.at(i+1);


				// Comprobar si el objetivo está dentro de esta región o calcular cómo de lejos está
				if( (angle_objective >= (90.0 - (region.gap_1) / 2.0) * pi/180.0) && (angle_objective <= (90.0 - (region.gap_2) / 2.0) * pi/ 180.0) ){
					region.obj_inside = true;
					region.obj_ang_nearness = 0.0;
				}
				else
				{
					region.obj_inside = false;

					dist_1 = angle_objective - ( (90.0 - (region.gap_1) / 2.0) * pi/180.0 );
					dist_2 = angle_objective - ( (90.0 - (region.gap_2) / 2.0) * pi/180.0 );
					if(std::abs(dist_1) < std::abs(dist_2)){
						region.obj_ang_nearness = dist_1; // Si la dist es negativa ya sabemos que es el gap 1. Si es positiva es el gap 2
					}
					else{
						region.obj_ang_nearness = dist_2; // Si la dist es negativa ya sabemos que es el gap 1. Si es positiva es el gap 2
					}
				}

				region.navigable = UNKNOWN;
				v_regions.push_back(region);
				printf("Región entre los ánuglos: %f y %f\n", 90.0 - (region.gap_1) / 2.0, 90.0 - (region.gap_2) / 2.0);

			}
		}
	}
	else
	{
		region.gap_1 = 0;
		region.gap_2 = num_laser - 1;
		region.obj_inside = true;
		region.obj_ang_nearness = 0.0;
		region.navigable = UNKNOWN; 

		v_regions.push_back(region);
	}

	printf("Número de regiones: %d\n\n", v_regions.size());


	/** 4. Elegir región 
		Se va a elegir aquella región que esté más cerca del objetivo, si lo contiene es el candidato ideal.
		Pero hay que comprobar si la región elegida es navigable, si no lo es habrá que elegir la siguiente mejor.
	*/
	int i_region = -1, i_closests = -1;
	// Bucle hasta encontrar una región navigable.
	do
	{
		i_region = -1;
		i_closests = -1;
		min_dist = inf;

		// Bucle que elige la región a explorar. 
		// Se elige en base a las que aún no se han explorado (no se sabe si son navigables)
		// Si alguna no explorada contiene al objetivo se elige esa. Si no aquella que presente la discontinuidad más cercana
		for(int i = 0; i < v_regions.size() && (i_region == -1); i++)
		{
			if(v_regions.at(i).navigable == UNKOWN)
			{
				if(v_regions.at(i).obj_inside){
					i_region = i;
				}
				else
				{
					if(std::abs(v_regions.at(i).obj_ang_nearness) < min_dist)
					{
						mind_dist = std::abs(v_regions.at(i).obj_ang_nearness);
						i_closests = i;
					}
				}

			}
		}

		i_region = i_closests;
	}
	while( !comprobar_navigable(v_regions.at(i_region), angle_objective, distance_objective) );

	escribir_ficheros();
}


/** Función que calcula si un punto es alcanzable o no*/
bool navigable comprobar_navigable(vector<region_charac> reg, float ang_obj, float dist_obj)
{
	printf("Analizando region entre los angulos: %f y %f\n", 90.0 - (reg.gap_1) / 2.0, 90.0 - (reg.gap_2) / 2.0);

	float x_obj, y_obj;
	
	if(reg.obj_inside)
	{
		x_obj = pos_obj.x;
		y_obj = pos_obj.y;
	}
	else
	{
		/** Si el objetivo no está dentro de la región hay que poner como objetivo de esta región aquel punto que esté más cercano al objetivo */
		// El punto más cercano se calcula como la intersección de la recta P y la recta Q
		// - P: recta que pasa por la posición del robot y el punto objetivo
		// - Q: recta perpendicular a P que pasa por el punto objetivo

		// Recta P
		float m_P = std::tan( (pos_obj.y - ground_truth.y) / (pos_obj.x - ground_truth.y) );
		float n_P = pos_obj.y - m_P * pos_obj.x;
		// Recta Q
		float m_Q = (-1) * (1 / m_P);
		float n_Q = pos_obj.y - n_P * pos_obj.x;
		// Nuevo punto objetivo
		x_obj = (n_Q - n_P) / (m_Q - m_P);
		y_obj = m_Q*x_obj + n_Q;

		/** Si cambia el punto objetivo hay que repetir el cálculo de la distancia y ángulo a él*/ 
		dist_obj = std::sqrt( std::pow(pos_obj.x - ground_truth.x, 2.0) + std::pow(pos_obj.y - ground_truth.y, 2.0) );
		ang_obj = std::atan2(pos_obj.y - ground_truth.y, pos_obj.x - ground_truth.x) - ground_truth.theta; 
	}


	/** 4.1 Eliminar puntos que no nos interesen de la región */
	


	return false;
}


void escribir_ficheros()
{
	// Distancias
	std::ofstream f_dist;
	f_dist.open("/home/vjimenez/catkin_ws/src/regiones_navegables/include/regiones_navegables/distancia.txt", std::ios::out); 
	if(f_dist.fail()) 
		std::cout<<"No se puedo abrir el archivo distancia"<<std::endl;

	for(unsigned int i = 0; i < v_PND.size(); i ++)
		f_dist<<v_PND.at(i)<<"\n";

	f_dist.close();


	// Discontinuidades
	std::ofstream f_discontinuities;
	f_discontinuities.open("/home/vjimenez/catkin_ws/src/regiones_navegables/include/regiones_navegables/discontinuidades.txt", std::ios::out); 

	if(f_discontinuities.fail()) 
		std::cout<<"No se puedo abrir el archivo discontinuidades"<<std::endl;

	for(unsigned int i = 0; i < v_i_discontinuities.size(); i ++)
		f_discontinuities<<v_i_discontinuities.at(i)<<"\n";

	f_discontinuities.close();

/*
	// Regiones
	std::ofstream f_regiones;
	f_regiones.open("/home/vjimenez/catkin_ws/src/regiones_navegables/include/regiones_navegables/regiones.txt", std::ios::out); 

	if(f_regiones.fail()) 
		std::cout<<"No se puedo abrir el archivo distancia"<<std::endl;

	for(unsigned int i = 0; i < v_regions.size(); i ++)
		f_regiones<<v_regions.at(i).gap_1<<" "<<v_regions.at(i).gap_2<<"\n";

	f_regiones.close();
	*/
}