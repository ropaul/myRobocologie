/*
 * parameter.h
 *@author Yann Teddy Ropaul<yannteddy.ropaul@gmail.com>
 */

#ifndef INCLUDE_PARAMETER_H_
#define INCLUDE_PARAMETER_H_

#include "RoboroboMain/common.h"
#include "RoboroboMain/roborobo.h"
#include "Controllers/Controller.h"
#include "WorldModels/RobotWorldModel.h"
#include <vector>
#include <iostream>
#include <algorithm>
#include <limits>
#include <map>

#define MINSTEPTONEXTGENERATION 1000
#define MINFITNESSWINDOWSIZE 10
#define MINRESOURCESATBEGINNING 900
#define MINRESOURCESVALUE 100
#define MINMUTATIONALPHA 5
#define MINMUTATIONEPSILON 0.2
#define MINMUTATIONSIGMA 0.0001
#define MINTOURNAMENTSIZE 5
#define MINDEATHLENGTH 2  // do not touch this , never																																								never
#define MINCHOICETOURNOI 1 // IF 1 IT IS A SIMPLE TOURNOI , 2 IT IS FITNESS PROPORTIONNAL TOURNOI

#define MAXSTEPTONEXTGENERATION 0
#define MAXFITNESSWINDOWSIZE 0
#define MAXRESOURCESATBEGINNING  0
#define MAXRESOURCESVALUE 0
#define MAXMUTATIONALPHA  0
#define MAXMUTATIONEPSILON  0
#define MAXMUTATIONSIGMA 0.001
#define MAXTOURNAMENTSIZE 2
#define MAXDEATHLENGTH 0
#define MAXCHOICETOURNOI 2

#define STEPSTEPTONEXTGENERATION 0
#define STEPFITNESSWINDOWSIZE 0
#define STEPRESOURCESATBEGINNING 1
#define STEPRESOURCESVALUE 0
#define STEPMUTATIONALPHA 0
#define STEPMUTATIONEPSILON 0
#define STEPMUTATIONSIGMA 0.005
#define STEPTOURNAMENTSIZE 0
#define STEPDEATHLENGTH  0
#define STEPCHOICETOURNOI 0

#define EVALUATESTEPTONEXTGENERATION false
#define EVALUATEFITNESSWINDOWSIZE false
#define EVALUATERESOURCESATBEGINNING false
#define EVALUATERESOURCESVALUE false
#define EVALUATEMUTATIONALPHA false
#define EVALUATEMUTATIONEPSILON false
#define EVALUATEMUTATIONSIGMA false
#define EVALUATETOURNAMENTSIZE false
#define EVALUATEDEATHLENGTH false
#define EVALUATECHOICETOURNOI false



#define GAUSIENNE true
#define STEPTONEXTEXPERIMENT 100
#define REACTIVATIONVALUE  1  // if 1, reactivate only if other genome from other agent, 2 reactivate with new genome , 3 never reactivate
#define FITNESSTYPE 2 // if 2 the fitness is the distance traveled , if 1 fitness is the number of reward
#define NBRUN  10





class parameter
{
	public:
		parameter();
		double getSteptonextgeneration() ;
		double getFitneswindowsize() ;
		double getResourcesatbegining() ;
		double getResourcesvalues() ;
		double getMutationalpha() ;
		double getMutationepsilon() ;
		double getMutationsigma() ;
		double getTournamentsize() ;
		double getDeathlength() ;
		bool getGausienne() ;
		int getSteptonextexperiment();
		int getReactivationValue();
		int getFitnessType();
		int getNbRun();
		int getChoiceTournoi();


		bool increaseSteptonextgeneration() ;
		bool increaseFitneswindowsize() ;
		bool increaseResourcesatbegining() ;
		bool increaseResourcesvalues() ;
		bool increaseMutationalpha() ;
		bool increaseMutationepsilon() ;
		bool increaseMutationsigma() ;
		bool increaseTournamentsize() ;
		bool increaseDeathlength() ;
		bool increaseChoiceTournoi();


		void reset();


		std::string toString();

	private:

		double steptonextgeneration ;
		double fitneswindowsize ;
		double resourcesatbegining ;
		double resourcesvalues ;
		double mutationalpha ;
		double mutationepsilon ;
		double mutationsigma ;
		double tournamentsize ;
		double deathlength ;
		bool gausienne ;
		int steptonextexperiment;
		int reactivationValue;
		int fitnessType;
		int nbrun;
		int choicetournoi;

};

#endif /* INCLUDE_PARAMETER_H_ */

