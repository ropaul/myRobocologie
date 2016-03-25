/**
 * @author Nicolas Bredeche <nicolas.bredeche@upmc.fr>
 * @author yann teddy Ropaul<yannteddy.ropaul@gmail.com>
 */


#include "VanillaEE/include/VanillaEEController.h"
#include "World/World.h"

#include <limits.h>

using namespace std;


//variables to changes :
// tournament variables
//mutation variables (gauss or binomiales)
//the time to the next generation
// the size of fitnessWindow 
// the number of resources at beginning
// the reward give as resources

VanillaEEController::VanillaEEController( RobotWorldModel *__wm ) : Controller ( __wm ),param(), ticks(1),resources(param.getResourcesatbegining()),
		fitnessWindowSize(1),wait(0)/*,alpha(5),epsilon(0.5)*/,fitness(0),poolSize(0) ,genomePool(0), myGenome(_wm->_cameraSensorsNb *3*2 +2,_wm->getId(), 1 )
{
	poolSize= 0;
	xOld = __wm->getXReal();
	yOld = __wm->getYReal();
	xNew = __wm->getXReal();
	yNew = __wm->getYReal();
	updateDistance();


} 

VanillaEEController::~VanillaEEController()
{
	// nothing to do.
}

void VanillaEEController::reset()
{

	for (int i = 0 ; i < (int)fitnessWindow.size(); i++){
		fitnessWindow.pop_back();
		poolSizeWindow.pop_back();

	}
	cleanThePool();

	ticks =1;
	resources=param.getResourcesatbegining();
	fitnessWindowSize=1;
	wait=0;
	fitness=0;
	poolSize=0;

	setGenome(*(new genome(_wm->_cameraSensorsNb *3*2+2, _wm->getId(),1 )));


}

void VanillaEEController::step()
{

	ticks ++; // increasse the ticks of the robots (use for the mutation)

	//wait is in case the agent meet nobody and can't evolve to the next generation. So we stop him
	// and the agent have enough resources to continue
	if (wait ==0 /*&&  resources*/){


		//EVALUATION OF THE SITUATION
		evaluation();

		switch(param.getFitnessType()){
		case 1 :
			updateFitness();
			updateDistanceV2();
			break;
		case 2:
		default :
			updateFitnessDist();
			break;
		}

		//resourcesManagement();


		//COMMUNICATION WITH THE OTHERS (NEAR THE AGENT)
		broadcastGenome();

	}


	// MUTATION (WHEN THE TIME COME)
	evolution(param.getSteptonextgeneration());

}




genome VanillaEEController::getGenome()
{
	return myGenome;
}

void VanillaEEController::setGenome(genome  _genome)
{
	myGenome = _genome;
}




//function to the evaluation step
// this function look the sensor and evaluate the genome. then it give the deplacement and rotation velocity of the agent.
void VanillaEEController:: evaluation() {



	double enter [ _wm->_cameraSensorsNb *3]; // array who detect robot, reward and other


	//enter is the vector of the value of the sensor
	// double enter [ _wm->_cameraSensorsNb *3]; // array who detect robot, reward and other
	for( int i = 0 ; i < _wm->_cameraSensorsNb; i++){
		enter[i+_wm->_cameraSensorsNb*2] = 0;
		enter[i+_wm->_cameraSensorsNb] = 0;
		enter[i] = 0;
		int targetIndex = _wm->getObjectIdFromCameraSensor(i);

		if ( targetIndex >= gRobotIndexStartOffset && targetIndex < gRobotIndexStartOffset+gNumberOfRobots  ) {  // robot
			enter[i] = (double)(( (double)gSensorRange - _wm->getCameraSensorValue(i,SENSOR_DISTANCEVALUE))/ (double)gSensorRange);
		}
		else if (targetIndex >= gPhysicalObjectIndexStartOffset && targetIndex < gPhysicalObjectIndexStartOffset + (int)gPhysicalObjects.size()){ // reward
			enter[i +_wm->_cameraSensorsNb ] = (double)(( (double)gSensorRange - _wm->getCameraSensorValue(i,SENSOR_DISTANCEVALUE))/ (double)gSensorRange);
		}
		else{ // other
			enter[i+  2* _wm->_cameraSensorsNb ] = (double)(( (double)gSensorRange - _wm->getCameraSensorValue(i,SENSOR_DISTANCEVALUE))/ (double)gSensorRange);
		}
	}

	/*
  for (int i = 0 ; i < _wm->_cameraSensorsNb *3; i++) {
    cout << enter[i] << "|" ;
  }
  cout << endl;*/


	// rotation

	double vr (myGenome.evaluateVelocityRotation(enter, _wm->_cameraSensorsNb *3 )); // evaluate give a number between[-5,5] which will be the velocity


	if (std::isnan(vr)){
		cout << "ERROR!!!!!"<< endl;
		for (int i = 0 ; i < _wm->_cameraSensorsNb *3; i++) {
			cout << enter[i] << "|" ;
		}
		cout << endl;
		cout << myGenome.toString();
	}

	if ( vr < -5){
		_wm-> _desiredRotationalVelocity= -5;
	}
	else if ( vr > 5){
		_wm-> _desiredRotationalVelocity =5;
	}
	else {
		_wm->_desiredRotationalVelocity = vr;
	}

	//go forward

	vt = myGenome.evaluateVelocityTranslation(enter, _wm->_cameraSensorsNb * 3);// + 1 - ( (double)gSensorRange - ((_wm->getCameraSensorValue(2,SENSOR_DISTANCEVALUE)+_wm->getCameraSensorValue(3,SENSOR_DISTANCEVALUE))/2) )  / (double)gSensorRange;

	//cout <<"vt="<< vt<< endl;


	if (std::isnan(vt)){
		cout << "ERROR!!!!!"<< endl;
		for (int i = 0 ; i < _wm->_cameraSensorsNb *3; i++) {
			cout << enter[i] << "|" ;
		}
		cout << endl;
		cout << myGenome.toString();
	}

	if ( vt < -1){
		_wm->_desiredTranslationalValue= -1;
	}
	else if ( vt >1){
		_wm->_desiredTranslationalValue =1;
	}
	else {
		_wm->_desiredTranslationalValue = vt;
	}


}



//function to the evolution step
// time is the time of steps to wait before mutation
void VanillaEEController:: evolution(int time) {


	if(ticks% time == 0){
		updateDistanceV2(); // update the traveled distance
		//cout << "END OF GENERATION FOR THE AGENT " << _wm->getId() << endl;
		//ticks = 0 ;
		//cout << "pool size= "<< poolSize<< endl;


		// At the beginning, we let the fitness be evaluate long enough to be significant
		if (fitnessWindowSize < param.getFitneswindowsize()) {
			cout << "FILL THE FITNESS " << _wm->getId() << endl;
			fitnessWindowSize ++;
			fillPoolSize(param.getFitneswindowsize());
			fillFitness( param.getFitneswindowsize());
			fitness =0;
			cleanThePool();
			if(fitnessWindowSize == param.getFitneswindowsize()){
				cout << "end of the beginning evaluation of the fitness"<< endl ;
			}
		}

		else {

			// MUTATION TIME
			cout << "END OF GENERATION FOR THE AGENT " << _wm->getId() << endl;
			fillPoolSize( param.getFitneswindowsize()); // update the variable pool size
			fillFitness( param.getFitneswindowsize());// update his own fitness


			//choice of the tournament style

			genome child = genome();

			switch ( param.getChoiceTournoi()){

			case 1 :
				child = tournament(param.getTournamentsize()); // use the tournament algorithm to choose the next generation genome
				break;
			case 2 :
			default :
				child = fitnessProportional(param.getTournamentsize()); // use the fitness proportinal algorithm to choose the next generation genome
				break;
			}



			if(wait == 0 &&  myGenome.equals(child)){ // if the agent meet nobody , he will die alone
				wait = DEATHLENGTH ;
				resources = 0;
				fitness = 0;
				resourcesManagement(); // stop the agent
				return;
			}
			//
			if(wait == 0 &&  myGenome.equals(child) == false){
				if (GAUSIENNE == true){
					setGenome( child.mutateGauss(param.getMutationsigma()));
				}
				else {
					setGenome( child.mutate(param.getMutationalpha(), param.getMutationepsilon()));
				}
				cleanThePool(); // clean the differents pool
			}


		}
		if (wait != 0){
			wait --; // the wait will end in two generation
			if (wait == 0){

				switch(param.getReactivationValue()){
				case 1:
					reactivation1();
					break;
				case 2:

					reactivation2();
					break;
				case 3:
				default:

					reactivation3();
					break;
				}
				resources = RESOURCESATBEGINNING ;
			}

		}
		else {
			resources = param.getResourcesatbegining() ;
		}
		fitness =0 ; // new generation , new fitness (even if dead)
		distanceTraveled = 0;
	}
}


// type of reactionvation of the agent.
// In this, only whenan other agent give him his genome, this agent will be reactivate with the genome of the other agent
void VanillaEEController::reactivation1(){
	genome child = genome();

	switch ( param.getChoiceTournoi()){

	case 1 :
		child = tournament(param.getTournamentsize()); // use the tournament algorithm to choose the next generation genome
		break;
	case 2 :
	default :
		child = fitnessProportional(param.getTournamentsize()); // use the fitness proportinal algorithm to choose the next generation genome
		break;
	}
	if(  myGenome.equals(child) ==false){
		if (GAUSIENNE == true){
			setGenome( child.mutateGauss(param.getMutationsigma()));
		}
		else {
			setGenome( child.mutate(param.getMutationalpha(), param.getMutationepsilon()));
		}
		cleanThePool(); // clean the differents pool
	}

}



// type of reactionvation of the agent.
// in this, the genome is a new genome , generate randomly.
void VanillaEEController::reactivation2(){
	setGenome(*(new genome(_wm->_cameraSensorsNb *3*2+2, _wm->getId(),ticks/ param.getSteptonextgeneration() )));//new agent !
}

// type of reactionvation of the agent.
// in this, the agent stay dead
void VanillaEEController::reactivation3(){
	wait = INT_MAX;
}

// use to spread the own agent's genome from other agent he see with his sensor
void VanillaEEController::broadcastGenome()
{


	for( int i = 0 ; i < _wm->_cameraSensorsNb ; i++)
	{
		int targetIndex = _wm->getObjectIdFromCameraSensor(i);

		if ( targetIndex >= gRobotIndexStartOffset && targetIndex < gRobotIndexStartOffset+gNumberOfRobots )   // sensor ray bumped into a robot : communication is possible
		{
			targetIndex = targetIndex - gRobotIndexStartOffset; // convert image registering index into robot id.

			VanillaEEController* targetRobotController = dynamic_cast<VanillaEEController*>(gWorld->getRobot(targetIndex)->getController());

			if ( ! targetRobotController )
			{
				std::cerr << "Error from robot " << _wm->getId() << " : the observer of robot " << targetIndex << " is not compatible." << std::endl;
				exit(-1);
			}
			else {

				targetRobotController->fillPool(myGenome,fitness);
			}
		}
	}
}





//a simple tournament selection
// tournamentSize must be between 1 and sizePool
genome VanillaEEController::tournament(int tournamentSize){
	// if pool is empty, return by default the agent's genome
	if(poolSize == 0){
		return getGenome();
	}
	//if tounamentSize is under 1, is not possible to tournament
	if (tournamentSize < 1){
		std::cerr << "error from robot" << _wm->getId() << " the choice of robot goes wrong"<< endl;
		exit(-1);
	}
	//if 1, it is a random choice
	if( tournamentSize == 1){
		return genomePool[ (int) rand() % poolSize];
	}
	int tempSize (tournamentSize);
	std::vector<genome> tournament (0);
	std::vector<double> fitnesstournament(0);
	// if not enough genome , it work anyway
	if (tournamentSize > poolSize){
		tempSize = poolSize;
	}
	int tournamentSize2 (tempSize); // save the size for later
	while (tempSize > 0){ // take randomly tempSize genomes to tournament
		int randValue ( (int) rand() % poolSize);
		tournament.push_back(genomePool[randValue]);
		fitnesstournament.push_back(fitnessPool[randValue]);
		genomePool.erase(genomePool.begin()+randValue); // erase the genome which is choose for the tournament (the others will be erase later)
		fitnessPool.erase(fitnessPool.begin()+randValue);
		tempSize -= 1;
		poolSize -=1; // the genomePool size decrease too
	}
	double max (0);
	int indexMax(0);
	// choose the best genome between those we peek randomly
	for (int j =0 ; j < tournamentSize2 ; j++){
		if (max < fitnesstournament[j]){
			indexMax = j;
			max = fitnesstournament[j];
		}
	}
	return tournament[ indexMax] ;
}




//tournament selection which use  the fitness proportional
// tournamentSize must be between 1 and sizePool
genome VanillaEEController::fitnessProportional(int tournamentSize){
	// if pool is empty, return by default the agent's genome
	if(poolSize == 0){
		return getGenome();
	}
	//if tounamentSize is under 1, is not possible to tournament
	if (tournamentSize < 1){
		std::cerr << "error from robot" << _wm->getId() << " the choice of robot goes wrong"<< endl;
		exit(-1);
	}
	//if 1, it is a random choice
	if( tournamentSize == 1){
		return genomePool[ (int) rand() % poolSize];
	}
	int tempSize (tournamentSize);
	std::vector<genome> tournament (0);
	std::vector<double> fitnesstournament(0);
	// if not enough genome , it work anyway
	if (tournamentSize > poolSize){
		tempSize = poolSize;
	}
	int tournamentSize2 (tempSize); // save the size for later
	while (tempSize > 0){ // take randomly tempSize genomes to tournament
		int randValue ( (int) rand() % poolSize);
		tournament.push_back(genomePool[randValue]);
		fitnesstournament.push_back(fitnessPool[randValue]);
		genomePool.erase(genomePool.begin()+randValue); // erase the genome which is choose for the tournament (the others will be erase later)
		fitnessPool.erase(fitnessPool.begin()+randValue);
		tempSize -= 1;
		poolSize -=1; // the genomePool size decrease too
	}

	// SAME THING THAN TOURNAMENT ^^^^^^^^^^^^^^^^^^^^^^^^
	// BEGINNING OF THE FITNESS PROPORTIONNAL  vvvvvvvvvvv

	double max (0);
	int indexMax(0);

	double invocationDuRandom((rand()%1000) / 1000.0); // just a random number between [0,1] (for the fitness proportional)

	for (int k = 1 ; k <=tournamentSize  ; k++){
		max =0;
		indexMax=0;
		// choose the best genome between those we peek randomly
		for (int j =0 ; j < fitnesstournament.size(); j++){
			if (max < fitnesstournament[j]){
				indexMax = j;
				max = fitnesstournament[j];
			}
		}

		if (invocationDuRandom < 1/ ( pow (2,k)) || k+1 > tournamentSize) {
			return tournament[ indexMax] ;
		}
		fitnesstournament.erase (fitnesstournament.begin() + indexMax);
		tournament.erase ( tournament.begin() + indexMax);
	}

	return tournament[ indexMax] ;
}




//test if the _genome is in the genomePool
int  VanillaEEController::inPool(genome _genome){
	for (int i =0 ; i < poolSize ; i ++){
		if( _genome.equals(genomePool[i])){
			return i;
		}

	}
	return -1;
}




// put the _genome and the fitness  in the Pool
void VanillaEEController:: fillPool (genome _genome,double _fitness){

	if(  inPool(_genome)== -1){
		genomePool.push_back(_genome);
		fitnessPool.push_back(_fitness);
		poolSize+=1;
	}
	else{ //update of the fitness value
		fitnessPool[inPool(_genome)] = _fitness;
	}
}


void VanillaEEController:: cleanThePool(){
	//erase all the genome for the next génération
	while (poolSize > 0){
		genomePool.pop_back();
		fitnessPool.pop_back();
		poolSize -= 1;
	}

}

double VanillaEEController::getPoolSize()const{
	double result (0);
	int fsize(poolSizeWindow.size());
	if (fsize == 0) return 0;
	for (int i =0 ; i < fsize ; i ++ ) {
		result += poolSizeWindow[i];
	}

	return (double) ( result / fsize);
}


//to fill the poolSizeWindow
void VanillaEEController::fillPoolSize(int size){
	while (fitnessWindowSize > size ) {
		poolSizeWindow.erase(poolSizeWindow.begin());
	}
	poolSizeWindow.push_back(poolSize);
}


double VanillaEEController:: getFitness(){
	double result (0);
	int fsize(fitnessWindow.size());
	if (fsize == 0) return 0;
	for (int i =0 ; i < fsize ; i ++ ) {
		result += fitnessWindow[i];
	}

	return (double) ( result / fsize);
}

// when meet an other robot which is already meet, we must update the fitness of this robots in the fitnessPool
void VanillaEEController::updateFitness(){


	// through floor sensor
	int targetIndex = _wm->getGroundSensorValue();
	if(targetIndex >= gPhysicalObjectIndexStartOffset && targetIndex < gPhysicalObjectIndexStartOffset + (int)gPhysicalObjects.size() )   // ground sensor is upon a physical object (OR: on a place marked with this physical object footprint, cf. groundsensorvalues image)
	{
		fitness++;
		resources += param.getResourcesvalues();
	} 

}

void VanillaEEController::updateFitnessDist(){

	updateDistanceV2()	;
	fitness = getDistance();



}

//to fill the fitnessWindow 
void VanillaEEController::fillFitness(int size){
	while (fitnessWindowSize > size ) {
		fitnessWindow.erase(fitnessWindow.begin());
		fitnessWindowSize -- ;
	}
	fitnessWindow.push_back(fitness);
}




// manage the resources
void VanillaEEController:: resourcesManagement() {
	if (resources){
		resources --;
	}
	else {
		_wm->_desiredTranslationalValue = 0 ;
		_wm-> _desiredRotationalVelocity = 0;
	}

}

bool VanillaEEController:: isAlive(){
	if (wait) return false;
	return true;
}


parameter VanillaEEController::getParam() {
	return param;

}


double VanillaEEController::getDistance(){
	return distanceTraveled;

}


void VanillaEEController::updateDistance(){
	xNew = _wm->getXReal();
	yNew = _wm->getYReal();
	distanceTraveled =  sqrt( pow(xNew - xOld , 2 ) + pow(yNew - yOld , 2 ) );
	xOld = _wm->getXReal();
	yOld = _wm->getYReal();

}


void VanillaEEController::updateDistanceV2(){

	distanceTraveled += (vt * ( 1- vr) * (1 - _wm->_cameraSensorsNb))  ;




}




