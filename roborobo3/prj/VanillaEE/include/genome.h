/**
 * @author Yann Teddy Ropaul<yannteddy.ropaul@gmail.com>
 */
 
#ifndef GENOME_H
#define GENOME_H 

#include "RoboroboMain/common.h"
#include "RoboroboMain/roborobo.h"
#include "Controllers/Controller.h"
#include "WorldModels/RobotWorldModel.h"
#include <vector>
#include <iostream>
#include <algorithm>
#include <limits>

class genome
{
	public:
		genome ();
		genome(int genomeSize, int id , int currentGeneration);
		genome ( const genome& copy) ;
		~genome();
		double evaluateOld( double enter[])const;
		double evaluateVelocityRotation( double enter[], int sizeArray)const;
		double evaluateVelocityTranslation( double enter[], int sizeArray)const;
		genome mutate(double alpha , double epsilon)const;
		genome mutateGauss(double sigma)const;
		bool equals(genome obj);
		int getSize()const;
		std::string toString();
		int getId()const;
		int getGeneration()const;
		bool sameAncestor(genome other)const;
		

		


	private:

		std::vector<double> gene ;
		int idOwner ;
		int generation;
		void setGene (int index, double value);	
		double getGene(int index);
		double evaluateLeft( double enter[], int sizeArray)const;
		double evaluateRight( double enter[], int sizeArray)const;
		
		
		




				
};


#endif

