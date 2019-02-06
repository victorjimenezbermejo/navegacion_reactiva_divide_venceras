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
5. Calcular si es navegable
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

void laserCallback(const sensor_msgs::LaserScan::ConstPtr& msg_laser)
{
	float gap_1, gap_2, slope_1, slope_2;
	std::pair<float, float> region;

	v_PND.clear();
	v_i_discontinuities.clear();
	v_regions.clear();

	/** 1. Nearest diagram */
	// Doris toma medidas entre -90 y 90, cada 0.5 grados -> 360 medidas, 180º
	for(int i = 0; i < msg_laser->ranges.size(); i++)
	{
		if(msg_laser->ranges.at(i) > max_range)
		{ 
			v_PND.push_back(0.0);
		}
		else
		{
			v_PND.push_back(max_range + rango_discontinuidad - msg_laser->ranges.at(i));
		}
	}

	/** 2. Encontrar las discontinuidades en el entorno */
	// Comparamos la distancia de cada ángulo con la del anterior. Si supera un rango -> discontinuidad	
	v_i_discontinuities.push_back(0);
	for(int i = 1; i < v_PND.size(); i++) // Ojito int i = 1 
	{
		if(std::abs( v_PND.at(i) - v_PND.at(i-1) ) > rango_discontinuidad)
		{
			// Ojito i no es el ángulo. La operación realizada cambia el vector de derecha a izquierda a ángulos entre -90 y 90
			v_i_discontinuities.push_back(i); 
			printf("Discontinuidad en el ángulo %f, con magnitud %f\n", 90.0 - (i) / 2.0, std::abs( v_PND.at(i) - v_PND.at(i-1) ));
		}
	}
	v_i_discontinuities.push_back(num_laser - 1);

	printf("Número discontinuidades (contando con -90 y 90): %d\n\n", v_i_discontinuities.size());



	/** 3. Definir las regiones */
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
			
			// Si una de las dos es creciente aceptamos
			if( (slope_1) > 0 || (slope_2) > 0)
			{
				region.first = v_i_discontinuities.at(i);
				region.second = v_i_discontinuities.at(i+1);

				v_regions.push_back(region);
				printf("Región entre los ánuglos: %f y %f\n", 90.0 - (region.first) / 2.0, 90.0 - (region.second) / 2.0);
			}
		}
	}

	printf("Número de regiones: %d\n\n", v_regions.size());

	escribir_ficheros();
}


int main(int argc, char **argv) {
	ros::init(argc, argv, "regiones_navegables_node"); // Inicializa un nuevo nodo llamado v_nav
	ros::NodeHandle nh;

	// Suscripciones
	laserSub = nh.subscribe("/Doris/scan", 1, laserCallback);


	v_PND.reserve(180);

	printf("Nodo lanzado\n");
	ros::spin();

	return 0;
};


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
		std::cout<<"No se puedo abrir el archivo distancia"<<std::endl;

	for(unsigned int i = 0; i < v_i_discontinuities.size(); i ++)
		f_discontinuities<<v_i_discontinuities.at(i)<<"\n";

	f_discontinuities.close();


	// Regiones
	std::ofstream f_regiones;
	f_regiones.open("/home/vjimenez/catkin_ws/src/regiones_navegables/include/regiones_navegables/regiones.txt", std::ios::out); 

	if(f_regiones.fail()) 
		std::cout<<"No se puedo abrir el archivo distancia"<<std::endl;

	for(unsigned int i = 0; i < v_regions.size(); i ++)
		f_regiones<<v_regions.at(i).first<<" "<<v_regions.at(i).second<<"\n";

	f_regiones.close();
}