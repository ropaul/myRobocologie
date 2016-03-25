/**
 * @author Nicolas Bredeche <nicolas.bredeche@upmc.fr>
 * @author Yann Teddy Ropaul<yannteddy.ropaul@gmail.com>
 */



#ifndef VANILLAEECONTROLLER_H
#define VANILLAEECONTROLLER_H

#include "RoboroboMain/common.h"
#include "RoboroboMain/roborobo.h"
#include "Controllers/Controller.h"
#include "WorldModels/RobotWorldModel.h"
#include "VanillaEE/include/genome.h"
#include "VanillaEE/include/parameter.h"
#include <vector>
#include <iostream>



#define STEPTONEXTGENERATION 1000
#define FITNESSWINDOWSIZE 20
#define RESOURCESATBEGINNING 900
#define RESOURCESVALUE 100
#define MUTATIONALPHA 5
#define MUTATIONEPSILON 0.2
#define MUTATIONSIGMA 0.0001
#define TOURNAMENTSIZE 3
#define DEATHLENGTH 10
#define GAUSIENNE true




class VanillaEEController : public Controller
{
	public:
		//Initializes the variables
		VanillaEEController( RobotWorldModel *__wm );
		~VanillaEEController();
		
		void reset();
		void step();
		genome getGenome();
		parameter getParam();
		void setGenome(genome _genome);
       		void monitorSensory();
		void fillPool(genome _genome, double _fitness);
		double getFitness();
		double getPoolSize()const ;
		bool isAlive() ;
		double getDistance();

		

	private:
		parameter param;
		int ticks ;
		int resources ;
		int fitnessWindowSize;
		int wait ; // if any genome were capt , wait.
		double xOld;
		double yOld;
		double xNew;
		double yNew;
		double distanceTraveled;
		double vt;
		double vr ;
		//double alpha ;
		//double epsilon;
		double fitness;
		std::vector<double> fitnessWindow;
		std::vector<double> poolSizeWindow;
		int poolSize;
		std::vector<genome> genomePool ;
		std::vector<double> fitnessPool;
		genome myGenome;		
		void broadcastGenome();
		void updateFitness();
		void updateFitnessDist();
		void evaluation();

		void fillFitness(int size);
		void fillPoolSize(int size);
		void evolution(int time);
		genome tournament(int tournamentSize);
		int inPool(genome _genome);
		void resourcesManagement();
		void cleanThePool();
		void updateDistance();
		void updateDistanceV2();
		genome fitnessProportional(int tournamentSize);
		void  reactivation1();
		void  reactivation2();
		void  reactivation3();


		


};


#endif

