/**
 * @author Nicolas Bredeche <nicolas.bredeche@upmc.fr>
 */


#ifndef VANILLAEEWORLDOBSERVER_H
#define VANILLAEEWORLDOBSERVER_H


#include "RoboroboMain/common.h"
#include "RoboroboMain/roborobo.h"

#include "Observers/WorldObserver.h"

#include "VanillaEE/include/parameter.h"
#include "VanillaEE/include/genome.h"

#include <vector>
#include <iostream>
#include <algorithm>
#include <limits>

class World;

class VanillaEEWorldObserver : public WorldObserver
{
	protected:
			parameter param;
			double average(std::vector<double> array);
			double variance (std::vector<double> array , double average);
			std::vector<double> sort (std::vector<double> array);
			double firstQuartil (std::vector<double> array);
			double secondQuartil (std::vector<double> array);
			double thirdQuartil (std::vector<double> array);
			double numberOfGenome (std:: vector<genome>array);
			void saveLogs();
			void saveAllLogs();
			void saveAllExperience();
			void updateExperiement();
			std::string name;
		
	public:
		VanillaEEWorldObserver( World *__world );
		~VanillaEEWorldObserver();
				
		void reset();
		void step();
		
};

#endif

