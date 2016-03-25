/**
 * @author yann teddy Ropaul<yannteddy.ropaul@gmail.com>
 */


#include "VanillaEE/include/parameter.h"

using namespace std;




parameter::parameter() 
{




steptonextgeneration = MINSTEPTONEXTGENERATION;
fitneswindowsize =MINFITNESSWINDOWSIZE;
resourcesatbegining= MINRESOURCESATBEGINNING;
resourcesvalues =MINRESOURCESVALUE;
mutationalpha =MINMUTATIONALPHA;
mutationepsilon= MINMUTATIONEPSILON;
mutationsigma =MINMUTATIONSIGMA;
tournamentsize= MINTOURNAMENTSIZE;
deathlength =MINDEATHLENGTH;



gausienne =GAUSIENNE;
steptonextexperiment = STEPTONEXTEXPERIMENT;
reactivationValue = REACTIVATIONVALUE;
fitnessType = FITNESSTYPE;
nbrun=NBRUN;
choicetournoi = MINCHOICETOURNOI;

}

 void parameter:: reset(){

	 steptonextgeneration = MINSTEPTONEXTGENERATION;
	 fitneswindowsize =MINFITNESSWINDOWSIZE;
	 resourcesatbegining= MINRESOURCESATBEGINNING;
	 resourcesvalues =MINRESOURCESVALUE;
	 mutationalpha =MINMUTATIONALPHA;
	 mutationepsilon= MINMUTATIONEPSILON;
	 mutationsigma =MINMUTATIONSIGMA;
	 tournamentsize= MINTOURNAMENTSIZE;
	 deathlength =MINDEATHLENGTH;



	 gausienne =GAUSIENNE;
	 steptonextexperiment = STEPTONEXTEXPERIMENT;
	 reactivationValue = REACTIVATIONVALUE;
	 fitnessType = FITNESSTYPE;
	 nbrun=NBRUN;
 }


double parameter::getSteptonextgeneration(){
return steptonextgeneration;

}


double parameter::getFitneswindowsize() {
	return  fitneswindowsize;

}


double parameter::getResourcesatbegining() {
 return resourcesatbegining;

}


double parameter::getResourcesvalues() {
	return resourcesvalues;

}


double parameter::getMutationalpha() {
	return mutationalpha;

}


double parameter::getMutationepsilon() {
	return mutationepsilon;

}


double parameter::getMutationsigma() {
return mutationsigma;

}


double parameter::getTournamentsize() {
return tournamentsize;

}


double parameter::getDeathlength() {
	return deathlength;
}


bool parameter::getGausienne() {
return gausienne;

}

int parameter::getSteptonextexperiment() {
	return steptonextexperiment;
}


int parameter::getFitnessType(){
	return fitnessType;
}


int parameter::getReactivationValue(){
	return reactivationValue;
}


int parameter::getNbRun(){
	return nbrun;
}


int parameter::getChoiceTournoi(){
	return choicetournoi;
}

bool parameter::increaseSteptonextgeneration() {
	if (steptonextgeneration + STEPSTEPTONEXTGENERATION <=MAXSTEPTONEXTGENERATION){
					steptonextgeneration += STEPSTEPTONEXTGENERATION;
					return true ;
				}
				return false;

}


bool parameter::increaseFitneswindowsize() {
	if (fitneswindowsize + STEPFITNESSWINDOWSIZE <=MAXFITNESSWINDOWSIZE){
					fitneswindowsize += STEPFITNESSWINDOWSIZE;
					return true ;
				}
				return false;

}


bool parameter::increaseResourcesatbegining() {
	if (resourcesvalues + STEPRESOURCESVALUE <=MAXRESOURCESVALUE){
				resourcesvalues += STEPRESOURCESVALUE;
				return true ;
			}
			return false;

}


bool parameter::increaseResourcesvalues() {
	if (resourcesvalues + STEPRESOURCESVALUE <=MAXRESOURCESVALUE){
				resourcesvalues += STEPRESOURCESVALUE;
				return true ;
			}
			return false;

}


bool parameter::increaseMutationalpha() {
	if (mutationalpha + STEPMUTATIONALPHA <=MAXMUTATIONALPHA){
				mutationalpha += STEPMUTATIONALPHA;
				return true ;
			}
			return false;

}


bool parameter::increaseMutationepsilon() {
	if (mutationepsilon + STEPMUTATIONEPSILON <=MAXMUTATIONEPSILON){
				mutationepsilon += STEPMUTATIONEPSILON;
				return true ;
			}
			return false;

}


bool parameter::increaseMutationsigma() {
	if (mutationsigma + STEPMUTATIONSIGMA <=MAXMUTATIONSIGMA){
			mutationsigma += STEPMUTATIONSIGMA;
			return true ;
		}
		return false;

}


bool parameter::increaseTournamentsize() {
	if (tournamentsize + STEPTOURNAMENTSIZE <=MAXTOURNAMENTSIZE){
			tournamentsize += STEPTOURNAMENTSIZE;
			return true ;
		}
		return false;

}


bool parameter::increaseDeathlength() {
	if (deathlength + STEPDEATHLENGTH <=MAXDEATHLENGTH){
		deathlength += STEPDEATHLENGTH;
		return true ;
	}
	return false;
}


bool parameter::increaseChoiceTournoi() {
	if (choicetournoi + STEPCHOICETOURNOI <=MAXCHOICETOURNOI){
		choicetournoi += STEPCHOICETOURNOI;
		return true ;
	}
	return false;
}


std::string parameter::toString(){
	std::string s ( "");
	ostringstream strs;
	    strs <<  steptonextgeneration <<"_"<< fitneswindowsize <<"_"<< resourcesatbegining <<"_"<< resourcesvalues <<"_"<< mutationalpha <<"_"
	    		<< mutationepsilon <<"_"<< mutationsigma <<"_"<< tournamentsize <<"_"
	    << deathlength <<"_"<< gausienne <<"_"<< steptonextexperiment;
	    s += strs.str();
	    return s;
}

