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
		parameter param;  // have all the param of the simulation
		int ticks ;  // count the number of iteration
		int resources ;  // the quantity of ressources , not use yet
		int fitnessWindowSize; // good question
		int wait ; // if any genome were capt , wait.
		double xOld;  // the old position in x-axes
		double yOld;  // the old position in y-axes
		double xNew; // the new postion
		double yNew; // the new position
		double distanceTraveled; // the distance traveled in ont tick
		double vt; // the velocity translational
		double vr ; // the velocity rotationnal

		//double alpha ;
		//double epsilon;
		double fitness;  // the fitness , may be replace by distanceTraveled

		std::vector<double> fitnessWindow; // sliding window for the fitness
		std::vector<double> poolSizeWindow; // sliding window for the match number
		int poolSize;
		std::vector<genome> genomePool ;  // the genome receive by other agents
		std::vector<double> fitnessPool;  // the fitness associate with the genome received.
		double Smax;  // the senssor value the who see the farrer
		genome myGenome;		// own genome
		void broadcastGenome();
		void updateFitness();
		void updateFitnessDist();
		void evaluation();

		void fillFitness(int size, double fitnessF);
		void fillPoolSize(int size);
		void evolution(int time);
		genome tournament(int tournamentSize);
		int inPool(genome _genome);
		void resourcesManagement();
		void cleanThePool();
		void cleanTheWindow(); // clean the fitnessWindow
		void updateDistance();
		void updateDistanceV2();
		genome fitnessProportional(int tournamentSize);
		void  reactivation1();
		void  reactivation2();
		void  reactivation3();


		


};


#endif

