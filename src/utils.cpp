#ifndef UTILSCPP
#define UTILSCPP
#include "utils.hpp"


std::vector<node> maxPlusGraph{};

Car applyDynamics(Car c, int indexActionX, int indexActionY) {

	int accelX = longitudinalAccelerationValues[indexActionX];
	int accelY = lateralAccelerationValues[indexActionY];

	double oldVelocityX = c.getVelocityX();
	double oldVelocityY = c.getVelocityY();

	c.setVelocity(oldVelocityX + accelX * SIMULATION_CONSTANT_TIME, oldVelocityY + accelY * SIMULATION_CONSTANT_TIME);
	c.setPosition(c.getPositionX() + oldVelocityX * SIMULATION_CONSTANT_TIME + (1 / 2) * accelX * SIMULATION_CONSTANT_TIME_SQUARED, c.getPositionY() + oldVelocityY * SIMULATION_CONSTANT_TIME + (1 / 2) * accelY * SIMULATION_CONSTANT_TIME_SQUARED);

	return c;
}

float imediateReward(Car c, Car c_star, std::vector<Car> adjacencyList) {

	float newDistanceFromDesiredSpeed = abs(c_star.getDesiredSpeed() - c_star.getVelocityX());
	float oldDistanceFromDesiredSpeed = abs(c.getDesiredSpeed() - c.getVelocityX());

	float score = oldDistanceFromDesiredSpeed - newDistanceFromDesiredSpeed;

	float res = (1000 / newDistanceFromDesiredSpeed);// +score;//elegxos gia 0
	//std::cout << "Res=" << res << std::endl;
	// Check if a collision in eminent
	float collision = 0;

	// Check for the adjacency list if necessary 
	/*
	@@

	*/
	for (Car c2 : adjacencyList) {
		collision = collision + betweenCarsReward(c, c2);
	}

	//ALPHA favours reaching the desired speed and BETA favours avoiding collisions
	return ALPHA * res - BETA * collision;
}

/* Function that computes the reward based on the likelihood of collision betweeen two cars*/
float betweenCarsReward(Car c, Car c1) {
	float distanceX = abs(c.getPositionX() - c1.getPositionX());
	float distanceY = abs(c.getPositionY() - c1.getPositionY());

	float score = (20 / distanceX) + (10 / distanceY);

	return 0.1 * score;
}


/*Function that accumulates the values of a float vector
*
*/
float vectorSumFloat(std::vector<float> v) {
	float s = 0;
	for (float f : v) {
		s = s + f;
	}
	return s;
}

/*Function that accumulates the values of an integer vector
*
*/
int vectorSumInt(std::vector<std::vector<int>> v) {
	int s = 0;
	for (std::vector<int> vi : v) {
		for (int i : vi) {
			s = s + i;
		}
	}
	return s;
}


int vectorSum(std::vector<int> v) {
	int s = 0;
	for (int i : v) {
		s = s + i;
	}
	return s;
}

node getNodeByCarNumber(int carNumber) {
	for (node n : maxPlusGraph) {
		if (n.getCar().getCarNumber() == carNumber) {
			return n;
		}
	}
}

#endif

