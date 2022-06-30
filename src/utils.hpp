#ifndef UTILITIES
#define UTILITIES

#include <random>
#include <iostream>
#include <chrono>
#include <fstream>

#define SIMULATION_CONSTANT_TIME 0.2
#define SIMULATION_CONSTANT_TIME_SQUARED 0.04
#define MAX_ACCELERATION 1
#define LONGITUDINAL_ACTIONS 5
#define LATERAL_ACTIONS 3
#define MIN_DESIRED_SPEED 25
#define MAX_DESIRED_SPEED 35
#define ROAD_WIDTH 10.2
#define ROAD_SAFETY_GAP 0.2 
#define GAMMA 0.6

/*Factored value MCTS hyperparameters*/
#define MAX_FVMCTS_DEPTH 3
#define FVMCTS_GAMMA 0.95
#define SIMULATIONS_FROM_ROOT 12
#define DISTANCE_FOR_CREATING_EDGE 35
#define EXPLORATION_TERM 0.5
#define ALPHA 1
#define BETA 0


/*Max Plus hyperparameters */
#define maxPlusMessagePassingPerRound 2
#define maxPlusMCSimulationRounds 2


const std::vector<double> longitudinalAccelerationValues{ -2,-1,0,1,3 };
const std::vector<double> lateralAccelerationValues{ 0,-1,1 };
const int availableActions = 15;

class Car {
private:
	double position_x, position_y;
	double velocity_x, velocity_y;
	double length, width;
	double desired_speed;
	int carNumber;

public:
	Car() = default;

	Car(int no) {
		carNumber = no;
	}

	void setPosition(double x, double y) {
		position_x = x;
		position_y = y;
	}
	void setVelocity(double vx, double vy) {
		velocity_x = vx;
		velocity_y = vy;
	}

	void setDimensions(double l, double w) {
		length = l;
		width = w;
	}
	void setDesiredSpeed(double s) {
		desired_speed = s;
	}

	double getPositionX() {
		return position_x;
	}
	double getPositionY() {
		return position_y;
	}
	double getVelocityX() {
		return velocity_x;
	}
	double getVelocityY() {
		return velocity_y;
	}

	double getLength() {
		return length;
	}
	double getWidth() {
		return width;
	}
	double getDesiredSpeed() {
		return desired_speed;
	}

	int getCarNumber() {
		return carNumber;
	}

	// Check if the cars has a collision with another given car
	bool isCrashed(Car c);

	void print(std::ostream& stream) {
		stream << "Position:(" << position_x << "," << position_y << ")\nAcceleration:(" << velocity_x << "," << velocity_y << ")\n";
	}
};

// Class for generating random actions for monte carlo simulations , based on a distribution
class lateralDistribution {
private:
	double velocity;
public:
	lateralDistribution(double velocity) {
		this->velocity = velocity;
	}

	int generateNext() {
		return rand()%LATERAL_ACTIONS;
	}
};

class longitudinalDistribution {
private:
	double velocity;
	double desiredSpeed;
public:
	longitudinalDistribution(double velocity, double desiredSpeed) {
		this->velocity = velocity;
		this->desiredSpeed = desiredSpeed;
	}

	int generateNext(){
		return rand()%LONGITUDINAL_ACTIONS;
	}
};


class timestampStatistics {
public:
	double timestamp;
	int cars;
	int collisions;
	int carsOutOfBounds;
	double sumOfDifferencesFromDesiredSpeed;

	timestampStatistics(double timestamp, int cars, int collisions, int carsOutOfBounds, double sumOfDifferencesFromDesiredSpeed) {
		this->timestamp = timestamp;
		this->cars = cars;
		this->collisions = collisions;
		this->carsOutOfBounds = carsOutOfBounds;
		this->sumOfDifferencesFromDesiredSpeed = sumOfDifferencesFromDesiredSpeed;
	}

	double getAverageDifferenceFromDesiredSpeed() {
		return (sumOfDifferencesFromDesiredSpeed / cars);
	}
};


class edge {
private:
	int carINumber;
	int carJNumber;

	/*Vector representing the joint temporary utility of all the actions of the agent i,for all available actions of the agent j.
	It is different than mji. It is used in the MaxPlus algorithm.
	*
	* First index is for the car with the car1Number number.
	* Second index for the car with the car2Number number.
	*/
	std::vector<std::vector<float>> mij;

	/*Vector representing the joint temporary utility of all the actions of the agent j,for all available actions of the agent i.
	It is different than mij. It is used in the MaxPlus algorithm.
	*
	* First index is for the car with the car1Number number.
	* Second index for the car with the car2Number number.
	*/
	std::vector<std::vector<float>> mji;


	/*Vector representing all possible couples of actions between the two agents and their value
	*
	* First index is for the car with the car1Number number.
	* Second index for the car with the car2Number number.
	*/
	std::vector<std::vector<float>> Qij;

	/*Times each combination of actions has been selected
	* Indexing is similar to Qij.
	*/
	std::vector<std::vector<int>> Nij;

	/* Variable storing the current best actions for the agents pointing to this edge.
	* The purpose for dong so, is to avoid looping in the coordinated graph to take the aj when
	* the algorithm is in the message passing phase.
	*/
	float bestActionValueI;
	float bestActionValueJ;
	int bestActionIndexI;
	int bestActionIndexJ;



public:
	edge(int carINumber, int carJNumber) {
		this->carINumber = carINumber;
		this->carJNumber = carJNumber;

		// Calculate all available actions aj for agent j.
		this->mij = std::vector<std::vector<float>>(availableActions, std::vector<float>(availableActions, 0.0));
		this->mji = std::vector<std::vector<float>>(availableActions, std::vector<float>(availableActions, 0));
		this->Qij = std::vector<std::vector<float>>(availableActions, std::vector<float>(availableActions, 0));
		this->Nij = std::vector<std::vector<int>>(availableActions, std::vector<int>(availableActions, 0));


	}

	std::vector<std::vector<float>> getMij() {
		return this->mij;
	}

	void setMij(int indexi, int indexj, float value) {
		this->mij[indexi][indexj] = value;
	}

	std::vector<std::vector<float>> getMji() {
		return this->mji;
	}

	void setMji(int indexi, int indexj, float value) {
		//std::cout << "SETTING mji for " << indexi << ", " << indexj << "  is " << value << std::endl;
		this->mji[indexi][indexj] = value;
	}



	void initialiseMij() {
		for (int i = 0; i < availableActions; i++) {
			for (int j = 0; j < availableActions; j++) {
				this->mij[i][j] = 0;
			}
		}
	}

	void initialiseMji() {
		for (int i = 0; i < availableActions; i++) {
			for (int j = 0; j < availableActions; j++) {
				mji[i][j] = 0;
			}
		}
	}



	std::vector<std::vector<float>> getQij() {
		return Qij;
	}

	void setQij(int indexI, int indexJ, float value) {
		Qij[indexI][indexJ] = value;
	}

	void setNij(int indexI, int indexJ, int value) {
		Nij[indexI][indexJ] = value;
	}

	std::vector<std::vector<int>> getNij() {
		return Nij;
	}

	void setCarNumberI(int carINumber) {
		this->carINumber = carINumber;
	}

	int getCarNumberI() {
		return carINumber;
	}

	void setCarNumberJ(int carJNumber) {
		this->carJNumber = carJNumber;
	}

	int getCarNumberJ() {
		return carJNumber;
	}

	void setBestActionValueI(float value) {
		this->bestActionValueI = value;
	}

	float getBestActionValueI() {
		return bestActionValueI;
	}

	void setBestActionValueJ(float value) {
		this->bestActionValueJ = value;
	}

	float getBestActionValueJ() {
		return bestActionValueJ;
	}



	void setBestActionIndexI(int index) {
		this->bestActionIndexI = index;
	}

	float getBestActionIndexI() {
		return bestActionIndexI;
	}
	void setBestActionIndexJ(int index) {
		this->bestActionIndexJ = index;
	}

	float getBestActionIndexJ() {
		return bestActionIndexJ;
	}

	float getBestM_ifIgetsActionJ(int index) {
		float max = -1000;
		for (std::vector<float> row : mij) {
			if (row[index] > max) {
				max = row[index];
			}
		}
		return max;
	}

	float getBestM_ifJgetsActionI(int index) {
		float max = -1000;
		for (std::vector<float> row : mji) {
			//std::cout << "For action " << index << " best responce: " << row[index] << std::endl;
			if (row[index] > max) {
				max = row[index];
			}
		}
		return max;
	}

	float getMijRowSum(int indexRow) {
		float totalSum = 0;
		for (int i = 0; i < availableActions; i++) {
			totalSum = totalSum + mij[indexRow][i];
		}
		return totalSum;
	}

	float getMjiRowSum(int indexRow) {
		float totalSum = 0;
		for (int i = 0; i < availableActions; i++) {
			totalSum = totalSum + mji[indexRow][i];
		}
		return totalSum;
	}
};


class node {
private:
	Car car;

	int temporaryBestActionIndex;// The best action for an agent at a given moment im maxPlus
	float temporaryBestActionValue; // The score q,of the action above

	int bestAction; // The best action of an agent after the simulation phase
	std::vector<float> actionValues; // The best action value of an agent after the simulation phase


	int N;//The number N, of all actions this node has taken
	std::vector<float> Q; // The quality of each possible move
	std::vector<int> Ni; // The times each possible move has been selected 
	std::vector<edge*> adjacencyList; // Edges of the graph

public:
	node(Car car) {
		this->car = car;
		this->N = 0;
		this->Q = std::vector<float>(availableActions, 1);//Initialize quality of move
		this->Ni = std::vector<int>(availableActions, 1);
		this->temporaryBestActionIndex = 2;//zero acceleration
		this->temporaryBestActionValue = 0;
		this->actionValues = std::vector<float>(availableActions, 0.0);
		this->bestAction = 2;//zero acceleration
	}

	void setAdjacencyList(std::vector<edge*> list) {
		this->adjacencyList = list;
	}

	void addEdge(edge* e) {
		this->adjacencyList.push_back(e);
	}

	int getCarNo() {
		return car.getCarNumber();
	}

	Car getCar() {
		return car;
	}

	std::vector<float> getQ() {
		return Q;
	}

	void setQ(int index, float q) {
		this->Q[index] = q;
	}

	std::vector<int> getNi() {
		return Ni;
	}

	void setNi(int index, int N) {
		this->Ni[index] = N;
	}

	std::vector<edge*> getAdjacencyList() {
		return adjacencyList;
	}

	void setTemporaryBestActionIndex(int actionIndex) {
		temporaryBestActionIndex = actionIndex;
	}

	int getTemporaryBestActionIndex() {
		return temporaryBestActionIndex;
	}

	void setTemporaryBestActionValue(float value) {
		temporaryBestActionValue = value;
	}

	float getTemporaryBestActionValue() {
		return temporaryBestActionValue;
	}


	void setBestActionIndex(int actionIndex) {
		bestAction = actionIndex;
	}

	int getBestActionIndex() {
		return bestAction;
	}

	void setActionValue(int actionIndex, float value) {
		actionValues[actionIndex] = value;
	}

	std::vector<float> getBestActionValues() {
		return actionValues;
	}

	float getBestActionValue(int index) {
		return actionValues[index];
	}


	void setN(int n) {
		this->N = n;
	}

	int getN() {
		return N;
	}

	int getBestAction() {
		int action = 0;
		float actionValue = 0;
		//std::cout << "///NEW CAR////" << std::endl;
		for (int f = 0; f < actionValues.size(); f++) {
			//std::cout << "Action " << f << " , value: " << actionValues[f] << std::endl;

			if (actionValues[f] > actionValue) {
				actionValue = actionValues[f];
				action = f;
			}
		}
		return action;
	}

	void setEdge(int index, edge* e) {
		this->adjacencyList[index] = e;
	}

};

double differenceFromDesiredSpeed(double desiredSpeed, double actualSpeed);

/*Function that accumulates the values of a float vector
*
*/
float vectorSumFloat(std::vector<float> v);

/*Function that accumulates the values of an integer vector
*
*/
int vectorSumInt(std::vector<std::vector<int>> v);

int vectorSum(std::vector<int> v);

Car applyDynamics(Car c, int indexActionX, int indexActionY);


float imediateReward(Car c, Car c_star, std::vector<Car> adjacencyList);

float betweenCarsReward(Car c, Car c1);


node getNodeByCarNumber(int carNumber);

extern std::vector<node> maxPlusGraph;



#endif