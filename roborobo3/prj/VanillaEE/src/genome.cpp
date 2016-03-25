/**
 * @author yann teddy Ropaul<yannteddy.ropaul@gmail.com>
 */


#include "VanillaEE/include/genome.h"

using namespace std;



//Create an empty genome
genome::genome() : gene(0),idOwner(0), generation(0)
{
}

// Gene size MUST BE twice the number of your sensor +2 (left or right)
genome::genome(int genomeSize,int id,int currentGeneration ): gene (genomeSize), idOwner(id), generation(currentGeneration)
{


  //double temp [genomeSize];
  //size = genomeSize;
  for(int i =0 ; i < genomeSize  ; i++) {
    // have a value between [-1, 1]
    //gene.push_back( (double) (rand()%2000/1000.0) -1 );
    gene[i] =  (double) (rand()%2000/1000.0) -1;
  }
 
}

genome::genome (const genome& copy): gene (copy.getSize()),idOwner(copy.getId()),generation(copy.getGeneration())
 {
   int size =copy.getSize() ;
 
 
  for(int i =0 ; i <size ; i++) {
    double temp (copy.gene[i]);
    gene[i]=( temp );
  }
}	

genome::~genome()
{
	// nothing to do.
}


//(V1.0) evaluate give the rotation velocity for the agent
//input : a array of the vvalue of the sensor
//output : a value between [-5,5]
double genome::evaluateVelocityRotation(double enter[], int sizeArray)const{
  
  /* double sumGene ( 0);
  double result (0);
  // cout << getSize()  << endl;
  for( int i =0 ; i < getSize();i++){
    if (i <sizeArray){
      sumGene+= gene[i];
      result += enter[i] * gene[i] - (enter[i] * gene[i+sizeArray]);
    }
    else {
      sumGene+= 1;
      result += gene[i] -  gene[i+sizeArray];
      
    }	
  } 
  // we normalize the result and then make the value between [-5,5] *nb sensor
  */
  return evaluateRight(enter,sizeArray)    ;
}

double genome::evaluateVelocityTranslation(double enter[], int sizeArray)const{


  return evaluateLeft(enter,sizeArray)  ;
}


//(V1.0) evaluate give the rotation velocity for the agent
//input : a array of the value of the sensor
//output : a value between [-5,5]
double genome::evaluateLeft(double enter[], int sizeArray)const{
  
  double result (0);
  // cout << getSize()  << endl;
  for( int i =0 ; i < getSize()/2;i++){
    if (i <sizeArray){
      result += enter[i] * gene[i];
    }
    else {
      result += gene[i] -  gene[i+sizeArray];
      
    }	
  } 
  return result; // /getSize() ;
}

//(V1.0) evaluate give the rotation velocity for the agent
//input : a array of the vvalue of the sensor
//output : a value between [-5,5]
double genome::evaluateRight(double enter[], int sizeArray)const{

  double result (0);
  // cout << getSize()  << endl;
  for( int i =0 ; i < getSize()/2;i++){
    if (i <sizeArray){
    
      result +=  (enter[i] * gene[i+sizeArray]);
    }
    else {
    
      result +=   gene[i+sizeArray];
      
    }	
  } 
  return result;//  /getSize() ;
}


//(V1.0) evaluate give the rotation velocity for the agent
//input : a array of the vvalue of the sensor
//output : a value between [-5,5]
double genome::evaluateOld(double enter[])const{
  
  double sumGene ( 0);
  double result (0);
  for( int i =0 ; i <(int)sizeof(enter);i++){
    if (i <(int) sizeof(enter)){
      sumGene+= gene[i];
      result += enter[i] * gene[i];
    }
    else {
      sumGene+= 1;
      result += gene[i];
    }	
  } 
  // we normalize the result and then make the value between [-5,5] *nb sensor
  return(double) (result) * 5  ;
}


// use to mutate the genome
// input :alpha (the proportion of mutation, between 0 and size of gene), epsilon (the rate of mutation, between [0,1])
// output : a mutate genome
genome genome::mutate(double alpha , double epsilon)const {
  genome copy(*this);
  int size (gene.size());
  for (int i = 0 ; i < size;i ++) {
    int random =(int)  rand() % size;
    double mutant (copy.getGene(random) + - epsilon +  (double)((rand()%1000)/1000.0)*2* epsilon); // mutate value;
    mutant = std::fmin (mutant , 1);
    mutant = std::fmax (mutant, -1);  // mutant is between [-1,1]
      copy.setGene(random, mutant);
  }
  return copy; 
}

// mute the genome with a gaussian distribution
genome genome::mutateGauss(double sigma) const{
  genome copy (*this);
  std::default_random_engine generator;
  
  int size (gene.size());
  for (int i = 0 ; i < size ; i++) {
    int random =(int)  rand()  % size;
     double mu (copy.getGene(i));

    std::normal_distribution<double> distribution(mu,sigma);
  
   
    double mutant (distribution(generator));
   
    //    cout << mutant <<"|";
    mutant = std::fmin (mutant , 1);
    mutant = std::fmax (mutant, -1);  // mutant is between [-1,1]
    copy.setGene(random, mutant);
  }
  // cout << endl;
  return copy;
}

//
void genome::setGene(int index, double value) {
  gene[index] = value;
}
  
double genome::getGene(int index){
  return gene[index];
}

int genome:: getSize()const{
  return gene.size();
}

bool genome::equals(genome obj){
  int size( getSize());
  if ( size == obj.getSize() ){
    for (int i =0 ; i < size; i++) {
      if( getGene(i) != obj.getGene(i))
	return false;
    }
    return true;
  }
  return false;
}
    

string genome::toString(){
  string s (" ");
  cout.precision(3);
  for (int i =0 ; i < getSize(); i++){
    // cout << fixed << getGene(i)<<"|";
    ostringstream strs;
    strs << getGene(i)<<"|";
    s += strs.str();
  }
  // cout << endl;
  return s ;
}

int genome::getId()const{
	return idOwner;
}


int genome::getGeneration()const{
	return generation;
}
  
bool genome::sameAncestor(genome other)const{
	if (idOwner == other.getId() && generation == other.getGeneration()) return true;
	return false;
}
