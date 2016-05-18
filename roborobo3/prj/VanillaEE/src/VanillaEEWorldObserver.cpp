/**
 * @author Nicolas Bredeche <nicolas.bredeche@upmc.fr>
 */




#include "VanillaEE/include/VanillaEEWorldObserver.h"



#include "World/World.h"



#include <iostream>

#include <fstream>

#include <string>

#include <vector>

#include <limits.h>

#include <stdlib.h>

using namespace  std;

VanillaEEWorldObserver::VanillaEEWorldObserver( World *__world ) : WorldObserver( __world ) ,param(),turn(0),bestGenome(0)
{
	_world = __world;

	name = "VanillaEE/logs_" + param.toString()+".csv";

	ofstream fichier(name, ios::out | ios::trunc);  //déclaration du flux et ouverture du fichier



	if(fichier)  // si l'ouverture a réussi

	{
		fichier << "generations ; different number of genome ; average fitness ; variance Fitness ; first quartile Fitness ; median fitness ; "
				"third quartile fitness, average match ; variance match ;first quartile match ; median Match ; third quartile match ; agent alive; "
				"average genome's age; variance genome's age; first quartile genome's age; median genome's age ; third quartile genome's age ; yougest genome ;"
				"oldest genome ; average distance ; variance distance ; first quartiel distance ; sencond quartile distance ; third quartile distance"<< endl;

		fichier.close();  // on referme le fichier
	}
	else { // sinon
		cerr << "Erreur à l'ouverture !" << endl;
	}

	name = "logs_" + param.toString()+".csv";

	ofstream fichier1("VanillaEE/fitness_" +name, ios::out | ios::trunc);  //déclaration du flux et ouverture du fichier
	ofstream fichier2("VanillaEE/poolSize_"+name, ios::out | ios::trunc);  //déclaration du flux et ouverture du fichier
	ofstream fichier3("VanillaEE/genomeAge_"+name, ios::out | ios::trunc);  //déclaration du flux et ouverture du fichier
	ofstream fichier4("VanillaEE/distance_"+name, ios::out | ios::trunc);  //déclaration du flux et ouverture du fichier
	ofstream fichier5("VanillaEE/inactive_"+name, ios::out | ios::trunc);  //déclaration du flux et ouverture du fichier
	ofstream fichier6("VanillaEE/nbgenome_"+name, ios::out | ios::trunc);  //déclaration du flux et ouverture du fichier
	ofstream fichier7("VanillaEE/fitnessmax_" +name, ios::out | ios::trunc);  //déclaration du flux et ouverture du fichier
	ofstream fichier8("VanillaEE/poolSizemax_" +name, ios::out | ios::trunc);  //déclaration du flux et ouverture du fichier
	if(fichier1 && fichier2 && fichier3 && fichier4 && fichier5 && fichier6 )  // si l'ouverture a réussi

	{
		fichier1 << "fitness"<< endl;
		fichier2 << "Pool size"<< endl;
		fichier3 << "Genome Age"<< endl;
		fichier4 << "Distance Traveled"<< endl;
		fichier5 << "desactived Agents"<< endl;
		fichier6 << "nbgenome"<< endl;
		fichier7 << "fitness max"<< endl;
		fichier8 << "pool size max"<< endl;

		fichier1.close();  // on referme le fichier
		fichier2.close();  // on referme le fichier
		fichier3.close();  // on referme le fichier
		fichier4.close();  // on referme le fichier
		fichier5.close();  // on referme le fichier
		fichier6.close();  // on referme le fichier
		fichier7.close();  // on referme le fichier
		fichier8.close();  // on referme le fichier
	}
	else { // sinon
		cerr << "Erreur à l'ouverture !" << endl;
	}




}

VanillaEEWorldObserver::~VanillaEEWorldObserver()
{
	// nothing to do.
}

void VanillaEEWorldObserver::reset()
{
	// nothing to do.
}

void VanillaEEWorldObserver::step()
{
	//		int lap (param.getSteptonextgeneration());
	//
	//		if ( gWorld->getIterations() % lap == 0  && lap!=0)
	//		{
	//			//if ( gVerbose && gDisplayMode == 0 )
	//			//_world->resetWorld();
	//			//saveLogs();
	//			//saveAllLogs();
	//			saveAllExperience();
	//		}
	//
	//		// the changement of experiement is the number of generation
	//		if(gWorld->getIterations()!=0 && gWorld->getIterations() % (param.getSteptonextexperiment() * lap) ==0){
	//
	//			std::cout << name << endl;
	//
	//			_world->resetWorld();
	//
	//			//updateExperiement();
	//
	//		}

	//	saveAllLogs();
	//	saveLogs();
		saveAllExperience();
//	testDuPlusFort();

	//registerTrace();


}



void VanillaEEWorldObserver::registerTrace(){

	if ( gWorld->getIterations() ==20){

		initTrajectoriesMonitor();
		gTrajectoryMonitorMode = 0;


	}

	if (gWorld->getIterations() ==1020){
		SDL_Delay(PAUSE_COMMAND); // 1000ms delay
		saveTrajectoryImage();

	}


	int lap (param.getSteptonextgeneration());

	if ( gWorld->getIterations()!=0 && gWorld->getIterations() % (param.getSteptonextexperiment() * lap  -1001 ) ==0){

		initTrajectoriesMonitor();
		gTrajectoryMonitorMode = 0;


	}

	if (gWorld->getIterations()!=0 && gWorld->getIterations() % (param.getSteptonextexperiment() * lap  -1 ) ==0){
		SDL_Delay(PAUSE_COMMAND); // 200ms delay
		saveTrajectoryImage();

	}
}


// save datas.
void VanillaEEWorldObserver::saveLogs() {
	int lap (param.getSteptonextgeneration());

	if ( gWorld->getIterations() % lap == 0  && lap!=0)
	{
		std::cout << "Save logs\n";

		int lap (param.getSteptonextgeneration());

		vector<double> fitnessArray (0);
		vector<double> matchArray(0);
		vector < genome> genArray(0);
		vector <double>  distArray(0);

		double isalive (0);
		vector <double>generation(0);
		int youngest (INT_MAX);
		int oldest (0);

		for ( int i = 0 ; i != gWorld->getNbOfRobots() ; i++ )
		{
			//Robot *robot = (gWorld->getRobot(i));
			VanillaEEController *controller = ((VanillaEEController*)(gWorld->getRobot(i)->getController()));
			fitnessArray.push_back(controller->getFitness());
			matchArray.push_back(controller->getPoolSize());
			genArray.push_back(controller->getGenome());
			distArray.push_back(controller->getDistance());

			if(controller->isAlive()) isalive +=1 ;

			generation.push_back(controller->getGenome().getGeneration());

			youngest = min (youngest,controller->getGenome().getGeneration());
			oldest = max (oldest,controller->getGenome().getGeneration());

		}



		double averageFitness (average(fitnessArray));
		double varianceFitness (variance (fitnessArray,averageFitness));
		vector<double> fitnessSort (sort(fitnessArray));
		double firstQFitness (firstQuartil(fitnessSort));
		double secondQFitness (secondQuartil(fitnessSort));
		double thirdQFitness (thirdQuartil(fitnessSort));

		double averageMatch (average(matchArray));
		double varianceMatch (variance (matchArray,averageMatch));
		vector<double> matchSort (sort(matchArray));
		double firstQMatch (firstQuartil(matchSort));
		double secondQMatch (secondQuartil(matchSort));
		double thirdQMatch (thirdQuartil(matchSort));

		double averageGen (average(generation));
		double varianceGen (variance (generation,averageGen));
		vector<double> genSort (sort(generation));
		double firstQGen (firstQuartil(genSort));
		double secondQGen (secondQuartil(genSort));
		double thirdQGen (thirdQuartil(genSort));


		double averageDist (average(distArray));
		double varianceDist (variance (distArray,averageDist));
		vector<double> distSort (sort(distArray));
		double firstQDist (firstQuartil(distSort));
		double secondQDist (secondQuartil(distSort));
		double thirdQDist (thirdQuartil(distSort));


		double nbGenome = numberOfGenome(genArray);


		name = "VanillaEE/logs_" + param.toString()+".csv";

		ofstream fichier(name, ios::out |  ios::app);  //déclaration du flux et ouverture du fichier



		if(fichier)  // si l'ouverture a réussi

		{

			fichier << gWorld->getIterations() / lap << " ; " << nbGenome <<";"<< averageFitness<<";"<< varianceFitness <<";" <<firstQFitness <<";" <<secondQFitness <<";"
					<<thirdQFitness<<";"<< averageMatch <<";" << varianceMatch<<";" <<firstQMatch <<";" << secondQMatch <<";" <<thirdQMatch << ";"<< isalive<< ";"<<averageGen<< ";"
					<<varianceGen<< ";"<<firstQGen<< ";"<<secondQGen<< ";"<<thirdQGen<< ";"<<youngest<< ";"<<oldest<<";"<<averageDist << ";"<< varianceDist << ";"<< firstQDist<< ";"<<
					secondQDist << ";"<< thirdQDist <<endl;

			fichier.close();  // on referme le fichier

		}

		else{  // sinon

			cerr << "Erreur à l'ouverture !" << endl;

		}
	}

}







void VanillaEEWorldObserver::saveAllLogs() {
	int lap (param.getSteptonextgeneration());

	if ( gWorld->getIterations() % lap == 0  && lap!=0)
	{
		std::cout << "Save  ALL logs\n";

		name = "logs_" + param.toString()+".csv";

		ofstream fichier1("VanillaEE/fitness_" +name, ios::out |  ios::app);  //déclaration du flux et ouverture du fichier
		ofstream fichier2("VanillaEE/poolSize_"+name, ios::out |  ios::app);  //déclaration du flux et ouverture du fichier
		ofstream fichier3("VanillaEE/genomeAge_"+name, ios::out |  ios::app);  //déclaration du flux et ouverture du fichier
		ofstream fichier4("VanillaEE/distance_"+name, ios::out |  ios::app);  //déclaration du flux et ouverture du fichier



		int isalive(0);

		for ( int i = 0 ; i != gWorld->getNbOfRobots() ; i++ )
		{
			VanillaEEController *controller = ((VanillaEEController*)(gWorld->getRobot(i)->getController()));

			if(controller->isAlive()) isalive +=1 ;


		}




		if(fichier1 && fichier2 && fichier3 && fichier4 )  // si l'ouverture a réussi

		{

			for (int i =0 ; i < gWorld->getNbOfRobots() ; i ++) {
				VanillaEEController *controller = ((VanillaEEController*)(gWorld->getRobot(i)->getController()));
				fichier1  << controller->getFitness()<<"," ;
				fichier2 << controller->getPoolSize()<<",";
				fichier3 << controller->getGenome().getGeneration()<<"," ;
				fichier4 << controller->getDistance()<<",";

			}
			fichier1  <<endl;
			fichier2  <<endl;
			fichier3  <<endl;
			fichier4  <<endl;


			fichier1.close();  // on referme le fichier
			fichier2.close();  // on referme le fichier
			fichier3.close();  // celui la , non
			fichier4.close();  // on referme le fichier


		}

		else{  // sinon

			cerr << "Erreur à l'ouverture !" << endl;

		}
	}
	if(gWorld->getIterations()!=0 && gWorld->getIterations() % (param.getSteptonextexperiment() * lap) ==0){

		std::cout << name << endl;

		_world->resetWorld();

		updateExperiement();

	}
}




void VanillaEEWorldObserver::saveAllExperience(){




	int lap (param.getSteptonextgeneration());

	if ( turn < param.getNbRun() ){


		// The following code shows an example where every 100 iterations, robots are re-located to their initial positions, and parameters are randomly changed.
		//
		// REMOVE OR COMMENT THE FOLLOWING TO AVOID RESETTING POSITIONS EVERY 100 ITERATIONS
		//

		if ( gWorld->getIterations() % lap == 0  && gWorld->getIterations()!=0)
		{
			//if ( gVerbose && gDisplayMode == 0 )
			//_world->resetWorld();
			std::cout << "Save  experiences\n";
			cout <<gWorld->getIterations() / (param.getSteptonextexperiment() * lap) << endl;





			int isalive(0);
			int nbgenome(0);

			vector<double> fitnessArray (0);
			vector<double> matchArray(0);
			vector < genome> genArray(0);
			vector <double>  distArray(0);


			vector <double>generation(0);
			int youngest (INT_MAX);
			int oldest (0);

			for ( int i = 0 ; i != gWorld->getNbOfRobots() ; i++ )
			{
				//Robot *robot = (gWorld->getRobot(i));
				VanillaEEController *controller = ((VanillaEEController*)(gWorld->getRobot(i)->getController()));
				fitnessArray.push_back(controller->getFitness());
				matchArray.push_back(controller->getPoolSize());
				genArray.push_back(controller->getGenome());
				distArray.push_back(controller->getDistance());

				if(controller->isAlive()) isalive +=1 ;

				generation.push_back(controller->getGenome().getGeneration());

				youngest = min (youngest,controller->getGenome().getGeneration());
				oldest = max (oldest,controller->getGenome().getGeneration());


			}




			vector<double> fitnessSort (sort(fitnessArray));

			double secondQFitness (secondQuartil(fitnessSort));



			vector<double> matchSort (sort(matchArray));

			double secondQMatch (secondQuartil(matchSort));



			vector<double> genSort (sort(generation));

			double secondQGen (secondQuartil(genSort));




			vector<double> distSort (sort(distArray));

			double secondQDist (secondQuartil(distSort));



			nbgenome = numberOfGenome(genArray);









			name = "logs_" + param.toString()+".csv";

			ofstream fichier1("VanillaEE/fitness_" +name, ios::out |  ios::app);  //déclaration du flux et ouverture du fichier
			ofstream fichier2("VanillaEE/poolSize_"+name, ios::out |  ios::app);  //déclaration du flux et ouverture du fichier
			ofstream fichier3("VanillaEE/genomeAge_"+name, ios::out |  ios::app);  //déclaration du flux et ouverture du fichier
			ofstream fichier4("VanillaEE/distance_"+name, ios::out |  ios::app);  //déclaration du flux et ouverture du fichier
			ofstream fichier5("VanillaEE/inactive_"+name, ios::out |  ios::app);  //déclaration du flux et ouverture du fichier
			ofstream fichier6("VanillaEE/nbgenome_"+name, ios::out |  ios::app);  //déclaration du flux et ouverture du fichier
			ofstream fichier7("VanillaEE/fitnessmax_"+name, ios::out |  ios::app);  //déclaration du flux et ouverture du fichier
			ofstream fichier8("VanillaEE/poolSizemax_"+name, ios::out |  ios::app);  //déclaration du flux et ouverture du fichier



			if(fichier1 && fichier2 && fichier3 && fichier4 && fichier5)  // si l'ouverture a réussi

			{

				fichier1  << secondQFitness<<"," ;
				fichier2 << secondQMatch<<",";
				fichier3 << secondQGen<<"," ;
				fichier4 << secondQDist<<",";
				fichier5 << isalive<<",";
				fichier6 << nbgenome<<",";
				fichier7 << fitnessSort[gWorld->getNbOfRobots() - 1]<<",";
				fichier8 << matchSort[gWorld->getNbOfRobots() - 1]<<",";



				fichier1.close();  // on referme le fichier
				fichier2.close();  // on referme le fichier
				fichier4.close();  // on referme le fichier
				fichier5.close();  // on referme le fichier
				fichier6.close();  // on referme le fichier
				fichier7.close();  // on referme le fichier
				fichier8.close();  // on referme le fichier

			}

			else{  // sinon

				cerr << "Erreur à l'ouverture !" << endl;

			}


			if (gWorld->getIterations()!=0 && gWorld->getIterations() % (param.getSteptonextexperiment() * lap ) ==0){
				cout << "NOW YOU SEE THIS" << endl;
				turn ++;

				ofstream fichier1("VanillaEE/fitness_" +name, ios::out |  ios::app);  //déclaration du flux et ouverture du fichier
				ofstream fichier2("VanillaEE/poolSize_"+name, ios::out |  ios::app);  //déclaration du flux et ouverture du fichier
				ofstream fichier3("VanillaEE/genomeAge_"+name, ios::out |  ios::app);  //déclaration du flux et ouverture du fichier
				ofstream fichier4("VanillaEE/distance_"+name, ios::out |  ios::app);  //déclaration du flux et ouverture du fichier
				ofstream fichier5("VanillaEE/inactive_"+name, ios::out |  ios::app);  //déclaration du flux et ouverture du fichier
				ofstream fichier6("VanillaEE/nbgenome_"+name, ios::out |  ios::app);  //déclaration du flux et ouverture du fichier
				ofstream fichier7("VanillaEE/fitnessmax_"+name, ios::out |  ios::app);  //déclaration du flux et ouverture du fichier
				ofstream fichier8("VanillaEE/poolSizemax_"+name, ios::out |  ios::app);  //déclaration du flux et ouverture du fichier




				if(fichier1 && fichier2 && fichier3 &&fichier4 && fichier5 && fichier6 && fichier7)  // si l'ouverture a réussi

				{



					fichier1  <<endl;
					fichier2  <<endl;
					fichier3  <<endl;
					fichier4  <<endl;
					fichier5  <<endl;
					fichier6  <<endl;
					fichier7  <<endl;
					fichier8  <<endl;

					fichier1.close();  // on referme le fichier
					fichier2.close();  // on referme le fichier
					fichier4.close();  // on referme le fichier
					fichier5.close();  // on referme le fichier
					fichier6.close();  // on referme le fichier
					fichier7.close();  // on referme le fichier
					fichier8.close();  // on referme le fichier

				}

				else{  // sinon

					cerr << "Erreur à l'ouverture !" << endl;

				}


				//_world->resetWorld();

			}
			if(gWorld->getIterations()!=0 && gWorld->getIterations() % (param.getSteptonextexperiment() * lap) ==0){

				std::cout << name << endl;

				_world->resetWorld();

				//updateExperiement();

			}

		}

	}
}



void VanillaEEWorldObserver::updateExperiement(){

	bool changement = false ;

	if (EVALUATERESOURCESATBEGINNING && param.increaseResourcesatbegining()){
		for ( int i = 0 ; i != gWorld->getNbOfRobots() ; i++ )
		{
			//Robot *robot = (gWorld->getRobot(i));
			VanillaEEController *controller = ((VanillaEEController*)(gWorld->getRobot(i)->getController()));
			if (controller->getParam().increaseResourcesatbegining()){
				controller->reset();
				changement = true;
			}
			else {
				//stop the simulation, but how ?
			}

		}
	}
	if (EVALUATEFITNESSWINDOWSIZE && param.increaseFitneswindowsize()){
		for ( int i = 0 ; i != gWorld->getNbOfRobots() ; i++ )
		{
			//Robot *robot = (gWorld->getRobot(i));
			VanillaEEController *controller = ((VanillaEEController*)(gWorld->getRobot(i)->getController()));
			if (controller->getParam().increaseFitneswindowsize()){
				controller->reset();
				changement = true;
			}
			else {
				//stop the simulation, but how ?
			}

		}
	}
	if (EVALUATEMUTATIONALPHA  && param.increaseMutationalpha()){
		for ( int i = 0 ; i != gWorld->getNbOfRobots() ; i++ )
		{
			//Robot *robot = (gWorld->getRobot(i));
			VanillaEEController *controller = ((VanillaEEController*)(gWorld->getRobot(i)->getController()));
			if (controller->getParam().increaseMutationalpha()){
				controller->reset();
				changement = true;
			}
			else {
				//stop the simulation, but how ?
			}

		}
	}
	if (EVALUATEMUTATIONEPSILON  && param.increaseMutationepsilon()){
		for ( int i = 0 ; i != gWorld->getNbOfRobots() ; i++ )
		{
			//Robot *robot = (gWorld->getRobot(i));
			VanillaEEController *controller = ((VanillaEEController*)(gWorld->getRobot(i)->getController()));
			if (controller->getParam().increaseMutationepsilon()){
				controller->reset();
				changement = true;
			}
			else {
				//stop the simulation, but how ?
			}

		}
	}
	if (EVALUATEMUTATIONSIGMA  && param.increaseMutationsigma()){
		for ( int i = 0 ; i != gWorld->getNbOfRobots() ; i++ )
		{
			//Robot *robot = (gWorld->getRobot(i));
			VanillaEEController *controller = ((VanillaEEController*)(gWorld->getRobot(i)->getController()));
			if (controller->getParam().increaseMutationsigma()){
				controller->reset();
				changement = true;
			}
			else {
				//stop the simulation, but how ?
			}

		}
	}


	if (EVALUATEDEATHLENGTH  && param.increaseDeathlength()){
		for ( int i = 0 ; i != gWorld->getNbOfRobots() ; i++ )
		{
			//Robot *robot = (gWorld->getRobot(i));
			VanillaEEController *controller = ((VanillaEEController*)(gWorld->getRobot(i)->getController()));
			if (controller->getParam().increaseDeathlength()){
				controller->reset();
				changement = true;
			}
			else {
				//stop the simulation, but how ?
			}

		}
	}

	if (EVALUATERESOURCESVALUE  && param.increaseResourcesvalues()){
		for ( int i = 0 ; i != gWorld->getNbOfRobots() ; i++ )
		{
			//Robot *robot = (gWorld->getRobot(i));
			VanillaEEController *controller = ((VanillaEEController*)(gWorld->getRobot(i)->getController()));
			if (controller->getParam().increaseResourcesvalues()){
				controller->reset();
				changement = true;
			}
			else {
				//stop the simulation, but how ?
			}

		}
	}


	if (EVALUATESTEPTONEXTGENERATION  && param.increaseSteptonextgeneration()){
		for ( int i = 0 ; i != gWorld->getNbOfRobots() ; i++ )
		{
			//Robot *robot = (gWorld->getRobot(i));
			VanillaEEController *controller = ((VanillaEEController*)(gWorld->getRobot(i)->getController()));
			if (controller->getParam().increaseSteptonextgeneration()){
				controller->reset();
				changement = true;
			}
			else {
				//stop the simulation, but how ?
			}

		}
	}

	if (EVALUATETOURNAMENTSIZE  && param.increaseTournamentsize()){
		for ( int i = 0 ; i != gWorld->getNbOfRobots() ; i++ )
		{
			//Robot *robot = (gWorld->getRobot(i));
			VanillaEEController *controller = ((VanillaEEController*)(gWorld->getRobot(i)->getController()));
			if (controller->getParam().increaseTournamentsize()){
				controller->reset();
				changement = true;
			}
			else {
				//stop the simulation, but how ?
			}

		}
	}


	if(changement){


		name = "VanillaEE/logs_" + param.toString()+".csv";

		ofstream fichier(name, ios::out | ios::trunc);  //déclaration du flux et ouverture du fichier



		if(fichier)  // si l'ouverture a réussi

		{
			fichier << "generations ; different number of genome ; average fitness ; variance Fitness ; first quartile Fitness ; median fitness ; "
					"third quartile fitness, average match ; variance match ;first quartile match ; median Match ; third quartile match ; agent alive; "
					"average genome's age; variance genome's age; first quartile genome's age; median genome's age ; third quartile genome's age ; yougest genome ;"
					"oldest genome "<< endl;

			fichier.close();  // on referme le fichier
		}
		else { // sinon
			cerr << "Erreur à l'ouverture !" << endl;
		}

		name = "logs_" + param.toString()+".csv";

		ofstream fichier1("VanillaEE/fitness_" +name, ios::out | ios::trunc);  //déclaration du flux et ouverture du fichier
		ofstream fichier2("VanillaEE/poolSize_"+name, ios::out | ios::trunc);  //déclaration du flux et ouverture du fichier
		ofstream fichier3("VanillaEE/genomeAge_"+name, ios::out | ios::trunc);  //déclaration du flux et ouverture du fichier
		ofstream fichier4("VanillaEE/distance_"+name, ios::out | ios::trunc);  //déclaration du flux et ouverture du fichier

		if(fichier1 && fichier2 && fichier3 && fichier4  )  // si l'ouverture a réussi

		{
			fichier1 << "fitness"<< endl;
			fichier2 << "Pool size"<< endl;
			fichier3 << "Genome Age"<< endl;
			fichier4 << "Distance Traveled"<< endl;

			fichier1.close();  // on referme le fichier
			fichier2.close();  // on referme le fichier
			fichier3.close();  // on referme le fichier
			fichier4.close();  // on referme le fichier
		}
		else { // sinon
			cerr << "Erreur à l'ouverture !" << endl;
		}
	}

}

// for a vector of double , give the average
double VanillaEEWorldObserver::average(vector<double> array){
	double result (0);
	for (int i = 0 ;  i < gWorld->getNbOfRobots() ; i++){
		result += array[i];
	}
	return (result/ gWorld->getNbOfRobots());
}

// for a vector , give the variance (nned the average to work)
double VanillaEEWorldObserver::variance (vector<double> array , double average){

	double result (0);
	for (int i = 0 ;  i < gWorld->getNbOfRobots() ; i++){
		result += pow((array[i] - average),2);
	}
	return (result/ gWorld->getNbOfRobots());
}

// for a vector of double , sort the vector with a insert sort
vector<double> VanillaEEWorldObserver::sort (vector<double> array){

	vector<double> result (0);
	for (int i = 0 ;  i < (int) array.size() ; i++){
		result.push_back( array[i]);
	}
	for (int i = (int) array.size() ;  i >0  ; i--){
		for (int j = 0 ; j < i ; j++){
			if (result[j] >result[j+1]){
				double temp (result[j]);
				result[j] = result[j+1];
				result[j+1] = temp;
			}
		}
	}
	return result;
}

// WARNING the enter array must be sort !! Give the first quartile
double VanillaEEWorldObserver::firstQuartil (vector<double> array){

	return array[int((1.0/4) *  (int)array.size())];
}


// WARNING the enter array must be sort !! Give the second quartile
double VanillaEEWorldObserver::secondQuartil (vector<double> array){

	return array[int((1.0/2) *  (int)array.size())];
}



// WARNING the enter array must be sort !! Give the third quartile
double VanillaEEWorldObserver::thirdQuartil (vector<double> array){

	return array[int((3.0/4) *  (int)array.size())];
}



//give the nomber of different genome it exist in the simulation.
double VanillaEEWorldObserver::numberOfGenome(vector<genome>array){
	vector<genome> temp (0);
	for (int i = 0 ;i <(int) array.size() ; i ++){
		bool exist = false;
		for (int j =0; j < (int)temp.size(); j ++){
			if (temp[j].sameAncestor (array[i])) exist = true;
		}
		if (exist==false) temp.push_back(array[i]);
	}
	return temp.size();
}






void VanillaEEWorldObserver::testDuPlusFort(){

	int lap (param.getSteptonextgeneration());

	if ( turn < param.getNbRun() ){

		if ( gWorld->getIterations() / lap *1.0 == 10.0  && lap!=0 && gWorld->getIterations() % lap == 0 )
		{

			vector<VanillaEEController*> arrayController(0);

			for ( int i = 0 ; i != gWorld->getNbOfRobots() ; i++ )
			{
				VanillaEEController *controller = ((VanillaEEController*)(gWorld->getRobot(i)->getController()));


				arrayController.push_back(controller);

			}


			vector<genome> arraysort = sortController(arrayController);

			for (int i = 0 ; i < 100 ; i ++){
				bestGenome.push_back(arraysort[(int)arraysort.size() -i -1]);
			}


		}




		if ( gWorld->getIterations() / lap >= 10  && gWorld->getIterations() % lap == 0  && lap!=0 && gWorld->getIterations()!=0)
		{





			if ( gWorld->getIterations() % lap == 0  && gWorld->getIterations()!=0)
			{
				//if ( gVerbose && gDisplayMode == 0 )
				//_world->resetWorld();
				std::cout << "Save  experiences\n";






				int isalive(0);
				int nbgenome(0);

				vector<double> fitnessArray (0);
				vector<double> matchArray(0);
				vector < genome> genArray(0);
				vector <double>  distArray(0);


				vector <double>generation(0);


				for ( int i = 0 ; i != gWorld->getNbOfRobots() ; i++ )
				{
					//Robot *robot = (gWorld->getRobot(i));
					VanillaEEController *controller = ((VanillaEEController*)(gWorld->getRobot(i)->getController()));

					if (isIn(bestGenome, controller->getGenome())){

						fitnessArray.push_back(controller->getFitness());
						matchArray.push_back(controller->getPoolSize());
						genArray.push_back(controller->getGenome());
						distArray.push_back(controller->getDistance());

						if(controller->isAlive()) isalive +=1 ;

						generation.push_back(controller->getGenome().getGeneration());


					}

				}

				double secondQFitness (0);

				double secondQMatch (0);

				double secondQGen (0);

				double secondQDist (0);

				vector<double> fitnessSort (0);

				vector<double> matchSort (0);

				if ((int) fitnessArray.size() >0){


					vector<double> fitnessSort =sort(fitnessArray);

					secondQFitness =secondQuartil(fitnessSort);



					vector<double> matchSort =sort(matchArray);

					secondQMatch =secondQuartil(matchSort);



					vector<double> genSort (sort(generation));

					secondQGen =secondQuartil(genSort);




					vector<double> distSort (sort(distArray));

					secondQDist =secondQuartil(distSort);



					nbgenome = numberOfGenome(genArray);







				}

				name = "logs_" + param.toString()+".csv";

				ofstream fichier1("VanillaEE/fitness_" +name, ios::out |  ios::app);  //déclaration du flux et ouverture du fichier
				ofstream fichier2("VanillaEE/poolSize_"+name, ios::out |  ios::app);  //déclaration du flux et ouverture du fichier
				ofstream fichier3("VanillaEE/genomeAge_"+name, ios::out |  ios::app);  //déclaration du flux et ouverture du fichier
				ofstream fichier4("VanillaEE/distance_"+name, ios::out |  ios::app);  //déclaration du flux et ouverture du fichier
				ofstream fichier5("VanillaEE/inactive_"+name, ios::out |  ios::app);  //déclaration du flux et ouverture du fichier
				ofstream fichier6("VanillaEE/nbgenome_"+name, ios::out |  ios::app);  //déclaration du flux et ouverture du fichier
				ofstream fichier7("VanillaEE/fitnessmax_"+name, ios::out |  ios::app);  //déclaration du flux et ouverture du fichier
				ofstream fichier8("VanillaEE/poolSizemax_"+name, ios::out |  ios::app);  //déclaration du flux et ouverture du fichier



				if(fichier1 && fichier2 && fichier3 && fichier4 && fichier5)  // si l'ouverture a réussi

				{

					fichier1  << secondQFitness<<"," ;
					fichier2 << secondQMatch<<",";
					fichier3 << secondQGen<<"," ;
					fichier4 << secondQDist<<",";
					fichier5 << isalive<<",";
					fichier6 << nbgenome<<",";



					if (fitnessSort.size()>0){
						fichier7 << fitnessSort[max((int)(fitnessSort.size() - 1),0)]<<",";
						fichier8 << matchSort[max((int)(matchSort.size() - 1),0)]<<",";
					}

					if (fitnessSort.size()==0){
											fichier7 << 0<<",";
											fichier8 << 0<<",";
										}


					fichier1.close();  // on referme le fichier
					fichier2.close();  // on referme le fichier
					fichier4.close();  // on referme le fichier
					fichier5.close();  // on referme le fichier
					fichier6.close();  // on referme le fichier
					fichier7.close();  // on referme le fichier
					fichier8.close();  // on referme le fichier

				}

				else{  // sinon

					cerr << "Erreur à l'ouverture !" << endl;

				}

				if (gWorld->getIterations()!=0 && gWorld->getIterations() % (param.getSteptonextexperiment() * lap ) ==0){

					turn ++;

					ofstream fichier1("VanillaEE/fitness_" +name, ios::out |  ios::app);  //déclaration du flux et ouverture du fichier
					ofstream fichier2("VanillaEE/poolSize_"+name, ios::out |  ios::app);  //déclaration du flux et ouverture du fichier
					ofstream fichier3("VanillaEE/genomeAge_"+name, ios::out |  ios::app);  //déclaration du flux et ouverture du fichier
					ofstream fichier4("VanillaEE/distance_"+name, ios::out |  ios::app);  //déclaration du flux et ouverture du fichier
					ofstream fichier5("VanillaEE/inactive_"+name, ios::out |  ios::app);  //déclaration du flux et ouverture du fichier
					ofstream fichier6("VanillaEE/nbgenome_"+name, ios::out |  ios::app);  //déclaration du flux et ouverture du fichier
					ofstream fichier7("VanillaEE/fitnessmax_"+name, ios::out |  ios::app);  //déclaration du flux et ouverture du fichier
					ofstream fichier8("VanillaEE/poolSizemax_"+name, ios::out |  ios::app);  //déclaration du flux et ouverture du fichier




					if(fichier1 && fichier2 && fichier3 &&fichier4 && fichier5 && fichier6 && fichier7)  // si l'ouverture a réussi

					{



						fichier1  <<endl;
						fichier2  <<endl;
						fichier3  <<endl;
						fichier4  <<endl;
						fichier5  <<endl;
						fichier6  <<endl;
						fichier7  <<endl;
						fichier8  <<endl;

						fichier1.close();  // on referme le fichier
						fichier2.close();  // on referme le fichier
						fichier4.close();  // on referme le fichier
						fichier5.close();  // on referme le fichier
						fichier6.close();  // on referme le fichier
						fichier7.close();  // on referme le fichier
						fichier8.close();  // on referme le fichier

					}

					else{  // sinon

						cerr << "Erreur à l'ouverture !" << endl;

					}


					//_world->resetWorld();

				}
				if(gWorld->getIterations()!=0 && gWorld->getIterations() % (param.getSteptonextexperiment() * lap) ==0){


					_world->resetWorld();

					//updateExperiement();

				}

			}

		}
	}
}

vector<genome> VanillaEEWorldObserver::sortController(vector<VanillaEEController*> array){
	vector<genome> result (0);

	int j (0);
	for  (int i =0 ; i < (int)array.size(); i++ ){
		double max (0);
		int maxController (0);
		for  ( j =i ; j < (int)array.size(); j++ ){
			if (array[j]->getFitness() > max){
				max = array[i]->getFitness();
				maxController = j;
			}
		}
		result.push_back(array[maxController]->getGenome());
		array.erase(array.begin() +maxController);
	}

	return result;

}


bool VanillaEEWorldObserver::isIn(vector<genome> array, genome candidat){


	for (int i =0 ; i < (int)array.size(); i++){

		if (array[i].equals(candidat)) return true;
	}

	return false;

}

