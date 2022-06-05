#include <stdio.h>
#include <stdlib.h>
#include <math.h>


#define DEFINE_VARIABLES
#ifdef __unix__
#include "LaneFree_linux.h"
#elif defined(WIN32)
#include <LaneFree_win.h>
#include <sstream>
#include <fstream>
#endif

#include "utils.hpp"
#include "Controller.h"
#include "FactoredValueMCTS.hpp"

#define MIN_DESIRED_SPEED 25
#define MAX_DESIRED_SPEED 35
#define ROAD_WIDTH 10.2
#define ROAD_SAFETY_GAP 0.2 
#define CARS_IN_SIMULATION 200
#define RUNNING_AS_INDEPENDENT_AGENTS 0
#define FVMCTS_LIMIT 10

std::vector<timestampStatistics> stats{};

std::ofstream myFile;// ("C:\Users\pgian\Documents\stats.csv");

Car createCarFromSumo(int index, NumericalID* myids) {
	Car c = Car(index);

	c.setPosition(get_position_x(myids[index]), get_position_y(myids[index]));

	c.setVelocity(get_speed_x(myids[index]), get_speed_y(myids[index]));

	c.setDimensions(get_veh_length(myids[index]), get_veh_width(myids[index]));

	c.setDesiredSpeed(get_desired_speed(myids[index]));

	return c;
}


void simulation_initialize() {

	//initialize srand with the same seed as sumo
	//srand(get_seed());
	srand(time(0));

	//insert 20 vehicles
	int n_init = 0;

	double x_incr = 25, y_incr = 2.5, vx_incr = 5;
	double x_val = x_incr, y_val = y_incr, vx_val = vx_incr, vy_val = 0;
	int virtual_lanes = 3;
	double width = 10;


	char veh_name[40];
	//route_id and type_id should be defined in the scenario we are running
	char route_id[20] = "route0";
	char type_id[20] = "1";
	NumericalID v_id;
	for (int i = 0; i < n_init; i++) {

		sprintf(type_id, "%d", i % 8 + 1);
		sprintf(veh_name, "%s_plugin_%d", type_id, (i + 1));

		v_id = insert_new_vehicle(veh_name, route_id, type_id, x_val, y_val, vx_val, vy_val, 0);
		printf("%s inserted\n", veh_name);
		y_val = y_val + y_incr;
		if (i % virtual_lanes == (virtual_lanes - 1)) {
			x_val += x_incr;
			if (vx_val < 35) {
				vx_val += vx_incr;
			}

			y_val = y_incr;
		}

	}



}

void simulation_step() {
	NumericalID* myids = get_lane_free_ids();
	NumericalID n_myids = get_lane_free_ids_size();

	// Allocate memory for statistics in this timestamp
	if (stats.begin() == stats.end() || stats.back().timestamp != get_current_time_step()) {
		timestampStatistics st = timestampStatistics(get_current_time_step(), n_myids, 0, 0, 0);
		stats.push_back(st);
	}

	int MCTSCars = 1;
	int visibilityTarget = 80;
	double pos_x, pos_y, speed, speed_y, des_speed, ux, uy, length, width, TS = get_time_step_length();
	int t = get_current_time_step();
	int i, j;
	char* vname;

	//printf("timestep:%d\n", n_myids);

	//PART TWO: Scalable tree search
	// Initiate statistics for the nodes and the edges of the coordination graph that will be created.
	if (RUNNING_AS_INDEPENDENT_AGENTS == 0) {
		std::cout << "--------------------------------------------------------" << std::endl;
		laneFreeGlobalState state = laneFreeGlobalState();
		for (int j = 0; j < n_myids; j++) {
			Car car = createCarFromSumo(j, myids);
			state.addCar(car);
		}

		if (state.getNumberOfCarsInRoads() > 1) {
			factoredValueMCTS mcts = factoredValueMCTS();
			mcts.main(state);

			for (node nod : maxPlusGraph) {
				int actionIndex = nod.getBestAction();
				std::div_t action = std::div(actionIndex, longitudinalAccelerationValues.size());
				if (nod.getAdjacencyList().size() > 0) {
					std::cout << "Action taken from " << nod.getCar().getCarNumber() << " is " << actionIndex << std::endl;
					std::cout << "Desired speed = " << nod.getCar().getDesiredSpeed() << ", actual speed=" << nod.getCar().getVelocityX() << std::endl;
					apply_acceleration(myids[nod.getCar().getCarNumber()], longitudinalAccelerationValues.at(action.rem), lateralAccelerationValues[action.quot]);
				}
				else {
					apply_acceleration(myids[nod.getCar().getCarNumber()], longitudinalAccelerationValues[3], lateralAccelerationValues[0]);
				}
			}
			maxPlusGraph.clear();
		}
		std::cout << "////////////////" << std::endl;
	}

	/* Attempt for creating a tree and pick an action based on MCTS algortihm.Each cars acts indpendenlty and treats the traffic as
	* part of the environment.
	*
	* PART ONE:
	* Create
	*
	* */
	if (RUNNING_AS_INDEPENDENT_AGENTS == 1) {

		// Create a an initial root state for a car to pick an action
		//laneFreeState root = laneFreeState();
		/*if (n_myids > vehicle_num_to_control) {
			pos_x = get_position_x(myids[vehicle_num_to_control]);
			pos_y = get_position_y(myids[vehicle_num_to_control]);
			speed_y = get_speed_y(myids[vehicle_num_to_control]);
			speed = get_speed_x(myids[vehicle_num_to_control]);
			des_speed = get_desired_speed(myids[vehicle_num_to_control]);
			length = get_veh_length(myids[vehicle_num_to_control]);
			width = get_veh_width(myids[vehicle_num_to_control]);
			root.setAgentState(pos_x, pos_y, speed, speed_y,des_speed,length,width);
			//printf("Vehicle name: %s\n", get_vehicle_name(myids[vehicle_num_to_control]));
			/*int i = 0;
			for (i = 0; i < n_myids; i++) {
				printf("Vehicle name = %s\nVehicle position=%d\nVehicle coordinates = %.2f %.2f\n", get_vehicle_name(myids[i]),i,get_position_x(myids[i]),get_position_y(myids[i]));
			}
		}
		else {
			return;
		}*/

		// TODO
		// Area in meters in front of the car, where other moving cars are anticipated.If a car is more far then this distance,it does not 
		// matter to the agent.
		// double om = 10;


		/*
		* Every MCTSCars cars, we have one car following the MCTS algorithm
		*/

		//1.Create controlled vehicles names
		/*std::vector<std::string> MCTSvehNames;
		for (int k = 0; k < CARS_IN_SIMULATION; k=k+MCTSCars) {
			std::string str(std::string("normal_flow."));
			std::ostringstream ss;
			ss << k;
			str.append(ss.str());
			MCTSvehNames.push_back(str);
		}*/

		//2.Iterate through current vehicles in the road
		for (i = 0; i < n_myids; i++) {

			/*
			* Check if the car is controlled by the MCTS algrorithm
			*/
			/*int exists = 0;
			for (std::string carName : MCTSvehNames) {
				if (carName.compare(get_vehicle_name(myids[i])) == 0) {
					exists = 1;
				}
			}
			if (exists == 0) {
				continue;
			}*/

			/*
			* Create root of the MCTS tree
			*/
			laneFreeState root = laneFreeState();

			/*
			* Create root state for the join state tree
			*/
			laneFreeGlobalState sroot = laneFreeGlobalState();
			for (j = 0; j < n_myids; j++) {
				Car car = createCarFromSumo(j, myids);
				sroot.addCar(car);
			}


			//Take other cars in the road into consideration

			for (j = 0; j < n_myids; j++) {

				//std::string str2(get_vehicle_name(myids[j]));
				if (j == i) {
					Car agent = createCarFromSumo(j, myids);
					root.setAgentState(agent);
					continue;
				}

				if (abs(get_position_x(myids[i]) - get_global_position_x(myids[j])) > visibilityTarget) {
					continue;
				}

				Car car = createCarFromSumo(j, myids);
				root.addCar(car);
			}


			//std::cout << "Cars anticipated== " << root.getNumberOfCarsInRoads() << std::endl;
			/* After creating the initial state,run the MCTS algorithm and get the best action*/
			carAction next = MCTSInstance::calculateAction(root);

			//std::cout << "ACTION PICKED: " << next.getLongitudinalAccelerationValue() << ", AND " << next.getLateralAccelerationValue() << " for agent " << ss.str() << std::endl;

			/* Apply the best action to the controlled car*/
			/* 1. Check if the car would exceed road limits if this action applies to it.
			   2. Change the lateral acceleration if necessary
			   3. Apply the corrected acceleration to the vehicle
			 */
			double newVelocity = root.getControlledCar().getVelocityY() + get_time_step_length() * next.getLateralAccelerationValue(); // v = u + at
			double dLeft = (ROAD_WIDTH - ROAD_SAFETY_GAP - (root.getControlledCar().getWidth() / 2)) - root.getControlledCar().getPositionY(); // d(t)=boundary - vehicle_position
			double dRight = (root.getControlledCar().getWidth() / 2) + ROAD_SAFETY_GAP - root.getControlledCar().getPositionY();

			//std::cout << "New velocity = " << newVelocity << std::endl;
			//std::cout << "D = " << dLeft << std::endl;

			if (newVelocity >= sqrt(2 * MAX_ACCELERATION * abs(dLeft)) && newVelocity > 0) {
				/*std::cout << "Current velocity --> " << root.getControlledCar().getVelocityY() << std::endl;
				std::cout << "Out of bounds LEFT --> " << root.getControlledCar().getPositionY() << " Car-> "<< i << std::endl;
				std::cout << "Velocity --> " << newVelocity << std::endl;
				std::cout << "d(t) = " << abs(dLeft) << std::endl;
				std::cout << "Applied = " << next.getLateralAccelerationValue() << std::endl;
				std::cout << "To apply = " << root.getControlledCar().getVelocityY() / SIMULATION_CONSTANT_TIME << std::endl << "---------------" << std::endl;*/
				// Cancel out MCTS and apply acceleration that keeps the cars inbound
				//next.setLateralAccelerationValue(-1 - (root.getControlledCar().getVelocityY() / SIMULATION_CONSTANT_TIME));
			}

			if (abs(newVelocity) >= sqrt(2 * MAX_ACCELERATION * abs(dRight)) && newVelocity < 0) {
				/*std::cout << "Current velocity --> " << root.getControlledCar().getVelocityY() << std::endl;
				std::cout << "Out of bounds RIGHT --> " << root.getControlledCar().getPositionY() << " Car-> " << i << std::endl;
				std::cout << "Velocity --> " << newVelocity << std::endl;
				std::cout << "d(t) = " << abs(dRight) << std::endl;
				std::cout << "Applied = " << next.getLateralAccelerationValue() << std::endl;
				std::cout << "To apply = " << root.getControlledCar().getVelocityY() / SIMULATION_CONSTANT_TIME << std::endl << "---------------" << std::endl;*/
				// Cancel out MCTS and apply acceleration that keeps the cars inbound
				//next.setLateralAccelerationValue(-(root.getControlledCar().getVelocityY() / SIMULATION_CONSTANT_TIME) + 1);


			}
			//std::cout << next.getLongitudinalAccelerationValue() << "," << next.getLateralAccelerationValue() << std::endl;

			if (next.getLongitudinalAccelerationValue() == NULL) {
				std::cout << "GITCHA!" << std::endl;
				//std::cout << next.getLongitudinalAccelerationValue() << std::endl;
				continue;
			}

			if (next.getLateralAccelerationValue() == NULL) {
				std::cout << "GITCHA v22!" << std::endl;
				//std::cout << next.getLateralAccelerationValue() << std::endl;
				continue;
			}

			//std::cout << next.getLongitudinalAccelerationValue() << std::endl;
			//std::cout << next.getLateralAccelerationValue() << std::endl;

			apply_acceleration(myids[i], next.getLongitudinalAccelerationValue(), next.getLateralAccelerationValue());
			//apply_acceleration(myids[i], -0.3, 0);

		}


	}





	/*
	* APPENDIX AUXILLIARY CODE:
	* STATISTICS
	*/

	double totalDifferenceFromDesiredSpeed = 0;
	for (j = 0; j < n_myids; j++) {
		totalDifferenceFromDesiredSpeed = totalDifferenceFromDesiredSpeed + differenceFromDesiredSpeed(get_speed_x(myids[j]), get_desired_speed(myids[j]));
	}
	stats.back().sumOfDifferencesFromDesiredSpeed = totalDifferenceFromDesiredSpeed;

	//std::cout << "SPEED ->> " << stats.back().sumOfDifferencesFromDesiredSpeed << " CARS --> " << stats.back().cars;
	//std::cout << "DIFFERENCE --> " << stats.back().getAverageDifferenceFromDesiredSpeed() << std::endl;


	NumericalID* detector_ids = get_detectors_ids();
	int* detector_values = get_detectors_values();

	NumericalID detectors_size = get_detectors_size();

	char* detector_name;
	for (j = 0; j < detectors_size; j++) {
		detector_name = get_detector_name(detector_ids[j]);
		printf("detector:%s count:%d\n", detector_name, detector_values[j]);
	}

	//Check the density per road per segment
	NumericalID* myedges = get_all_edges();
	NumericalID n_myedges = get_all_edges_size();
	int* density_per_edge;
	int size;
	double segment_length = 100; //in meters
	for (i = 0; i < n_myedges; i++) {
		density_per_edge = get_density_per_segment_per_edge(myedges[i], segment_length);
		if (density_per_edge != NULL) {
			size = get_density_per_segment_per_edge_size(myedges[i], segment_length);
			//printf("Edge id %lld\nDensity per segment:", myedges[i]);
			//for (j = 0; j < size; j++) {
			//	printf("%d,", density_per_edge[j]);
			//}
			//printf("\n");


		}

	}

	//For larger networks, you may control vehicles based on the road edge they are in
	NumericalID* ids_in_edge;
	NumericalID n_edge_ids;

	double vx, accel;

	for (i = 0; i < n_myedges; i++) {
		//printf("edge id: %lld\n", myedges[i]);
		n_edge_ids = get_all_ids_in_edge_size(myedges[i]);
		length = get_edge_length(myedges[i]);
		width = get_edge_width(myedges[i]);
		if (n_edge_ids > 0) {
			//vehicles are ordered
			ids_in_edge = get_all_ids_in_edge(myedges[i]);
			//printf("Vehicles in edge with id %lld:", myedges[i]);
			for (j = 0; j < n_edge_ids; j++) {
				vname = get_vehicle_name(ids_in_edge[j]);
				//printf("%s\t",vname);
			}
			//printf("\n\n");
		}
	}



}


void simulation_finalize() {
	/*
	* WRITING STATISTICS TO FILE
	*/
	myFile.open("stats.csv");
	myFile << "Collisions,out_of_bounds,speedDiff" << std::endl;

	for (timestampStatistics ts : stats) {

		myFile << ts.collisions << "," << ts.carsOutOfBounds << "," << ts.getAverageDifferenceFromDesiredSpeed() << std::endl;
	}
	myFile.close();
	return;
}


void event_vehicle_enter(NumericalID veh_id) {

	//set_desired_speed(veh_id, rand()%(MAX_DESIRED_SPEED - MIN_DESIRED_SPEED + 1) + MIN_DESIRED_SPEED);
	set_desired_speed(veh_id, (double)(rand() % (int)(MAX_DESIRED_SPEED - MIN_DESIRED_SPEED + 1) + MIN_DESIRED_SPEED));
	//char* vname1 = get_vehicle_name(veh_id);
	// printf("Vehicle %s entered with speed %f.\n",vname1,get_speed_x(veh_id));

	//make the vehicles emulate a ring road scenario
	set_circular_movement(veh_id, false);

}

void event_vehicle_exit(NumericalID veh_id) {
	char* vname1 = get_vehicle_name(veh_id);
	printf("Vehicle %s exited at time %.2f, at pos:%f.\n", vname1, get_current_time_step() * get_time_step_length(), get_position_x(veh_id));

}

void event_vehicles_collide(NumericalID veh_id1, NumericalID veh_id2) {
	char vname1[40];
	sprintf(vname1, "%s", get_vehicle_name(veh_id1));

	char* vname2 = get_vehicle_name(veh_id2);
	printf("Collision between %s and %s at timestep: %d, and time: %.1f.\n", vname1, vname2, get_current_time_step(), get_current_time_step() * get_time_step_length());

	stats.back().collisions = stats.back().collisions + 1;
}

void event_vehicle_out_of_bounds(NumericalID veh_id) {
	char* vname1 = get_vehicle_name(veh_id);
	printf("Vehicle %s is out of bounds at time %.2f, at pos:%f,%f.\n", vname1, get_current_time_step() * get_time_step_length(), get_position_x(veh_id), get_position_y(veh_id));
	stats.back().carsOutOfBounds = stats.back().carsOutOfBounds + 1;
}

double differenceFromDesiredSpeed(double desiredSpeed, double actualSpeed) {
	return abs(desiredSpeed - actualSpeed);
}

void simulate(laneFreeGlobalState state, int depth) {

}

void updateGraphStatistics(laneFreeGlobalState state, std::vector<std::pair<int, int>> coordActions) {

}


