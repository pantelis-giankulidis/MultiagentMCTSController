#ifdef DEFINE_VARIABLES
#define EXTERN /* nothing */
#else
#define EXTERN extern
#endif /* DEFINE_VARIABLES */




//NumericalID is used as a data type for ids.
typedef long long int NumericalID;

//Note: memory management is performed automatically. You do not free the pointers you receive through this API (See LaneFree.cpp for usage)

//returns all ids in the network
EXTERN NumericalID* (*get_all_ids)();
//returns the size of all ids in the network
EXTERN NumericalID(*get_all_ids_size)();


//returns all lane free ids in the network
EXTERN NumericalID* (*get_lane_free_ids)();
//returns the size of all lane free ids in the network
EXTERN NumericalID(*get_lane_free_ids_size)();


//returns the vehicle's name based on the id argument
EXTERN char* (*get_vehicle_name)(NumericalID veh_id);


//returns an array with all the edges
EXTERN NumericalID* (*get_all_edges)();


//returns the size of all the edges
EXTERN NumericalID(*get_all_edges_size)();


//returns the id of the edge for a given vehicle, based on the vehicle's id
EXTERN NumericalID(*get_edge_of_vehicle)(NumericalID veh_id);

//returns the name of the edge
EXTERN char* (*get_edge_name)(NumericalID edge_id);

//returns all ids in a given edge, based on the edge's id, and ordered according to their longitudinal position
EXTERN NumericalID* (*get_all_ids_in_edge)(NumericalID edge_id);
//returns the size of all ids in a given edge, based on the edge's id
EXTERN NumericalID(*get_all_ids_in_edge_size)(NumericalID edge_id);


//apply longitudinal acceleration accel_x m/s^2 and lateral acceleration accel_y m/s^2 to the vehicle with id veh_id
EXTERN void (*apply_acceleration)(NumericalID veh_id, double accel_x, double accel_y);


//returns the longitudinal speed in m/s of vehicle with id veh_id
EXTERN double (*get_speed_x)(NumericalID veh_id);


//returns the longitudinal speed in m/s of vehicle with id veh_id
EXTERN double (*get_speed_y)(NumericalID veh_id);


//returns the desired speed in m/s of vehicle with id veh_id
EXTERN double (*get_desired_speed)(NumericalID veh_id);


//sets the desired speed in m/s of vehicle with id veh_id
EXTERN void (*set_desired_speed)(NumericalID veh_id, double new_des_speed);


//returns the longitudinal position of a vehicle based on the local coordinates (according to the road the vehicle is now)
EXTERN double (*get_position_x)(NumericalID veh_id);


//returns the lateral position of a vehicle based on the local coordinates (according to the road the vehicle is now)
EXTERN double (*get_position_y)(NumericalID veh_id);


//returns the relative longitudinal distance of a vehicle with respect to an ego vehicle
EXTERN double (*get_relative_distance_x)(NumericalID ego_id, NumericalID other_id);


//returns the relative longitudinal position of a vehicle with respect to an ego vehicle
EXTERN double (*get_relative_position_x)(NumericalID ego_id, NumericalID other_id);


//returns the relative lateral distance of a vehicle with respect to an ego vehicle
EXTERN double (*get_relative_distance_y)(NumericalID ego_id, NumericalID other_id);


//determines if a given vehicle will be transported at the beginning of the road edge it currently is into, based on the value of circular (true:enables the behavior, false:disables the behavior)
EXTERN void (*set_circular_movement)(NumericalID veh_id, bool circular);


//returns the time-step length
EXTERN double (*get_time_step_length)();


//returns the current time-step
EXTERN int (*get_current_time_step)();


//returns the type of a given vehicle id
EXTERN NumericalID (*get_veh_type_id)(NumericalID veh_id);


//returns the name of a given type id
EXTERN char* (*get_veh_type_name)(NumericalID veh_id);

//returns the length of a given vehicle id
EXTERN double (*get_veh_length)(NumericalID veh_id);

//returns the width of a given vehicle id
EXTERN double (*get_veh_width)(NumericalID veh_id);

//returns the length of a given edge id
EXTERN double (*get_edge_length)(NumericalID edge_id);

//returns the width of a given edge id
EXTERN double (*get_edge_width)(NumericalID edge_id);


//returns the seed
EXTERN int (*get_seed)();


//returns the ids of the detectors
EXTERN NumericalID* (*get_detectors_ids)();


//returns the size of the ids of the detectors
EXTERN NumericalID (*get_detectors_size)();


//returns the detector's name, based on the detector id
EXTERN char* (*get_detector_name)(NumericalID detector_id);


//returns the values of all detectors (number of vehicles for each detector)
EXTERN int* (*get_detectors_values)();

//returns the value of a single detector, based on the detector's id
EXTERN int (*get_detector_value)(NumericalID detector_id);

//returns the value that corresponds to the specified type of a single detector, based on the detector's id
EXTERN int (*get_detector_value_for_type)(NumericalID detector_id, char* veh_type);

//returns the density of vehicles for a given segment region for a given edge id
EXTERN int (*get_density_on_segment_region_on_edge)(NumericalID edge_id, double segment_start, double segment_end);

//returns the density of vehicles for a given segment region for a given edge id and for a given type of vehicles
EXTERN int (*get_density_on_segment_region_on_edge_for_type)(NumericalID edge_id, double segment_start, double segment_end, char* veh_type);

//returns the average speed of vehicles for a given segment region for a given edge id
EXTERN double (*get_average_speed_on_segment_region_on_edge)(NumericalID edge_id, double segment_start, double segment_end);

//returns the average speed of vehicles for a given segment region for a given edge id and for a given type of vehicles
EXTERN double (*get_average_speed_on_segment_region_on_edge_for_type)(NumericalID edge_id, double segment_start, double segment_end, char* veh_type);

//returns the density of vehicles per segment for a given edge id, and a segment length
EXTERN int* (*get_density_per_segment_per_edge)(NumericalID edge_id, double segment_length);

//returns the size of the array provided above
EXTERN int (*get_density_per_segment_per_edge_size)(NumericalID edge_id, double segment_length);

//returns the ids of front vehicles that may be located beyond the veh_id's road edge, according to the front distance provided, and based on its routing. 
//you need "cross_edge=1" to look for vehicles beyond the current edge, and pass the address of a variable to "neighbors_size" in order to acquire the size of the resulting array.
EXTERN NumericalID* (*get_all_neighbor_ids_front)(NumericalID veh_id, double front_distance, int cross_edge, size_t* neighbors_size);

//returns the ids of vehicles one the back that may be located before the veh_id's road edge, according to the back distance provided, and based on its routing. 
//you need "cross_edge=1" to look for vehicles beyond the current edge, and pass the address of a variable to "neighbors_size" in order to acquire the size of the resulting array.
EXTERN NumericalID* (*get_all_neighbor_ids_back)(NumericalID veh_id, double back_distance, int cross_edge, size_t* neighbors_size);


//insert a new vehicle (route_id and type_id need to be defined in the scenario tested)
EXTERN NumericalID (*insert_new_vehicle)(char* veh_name, char* route_id, char* type_id, double pos_x, double pos_y, double speed_x, double speed_y, double theta);

//returns 1 if the vehicle is currently on an acceleration lane and needs to merge
EXTERN int (*am_i_on_acceleration_lane)(NumericalID veh_id);

//returns the global x position of the vehicle
EXTERN double (*get_global_position_x)(NumericalID veh_id);

//returns the global y position of the vehicle
EXTERN double (*get_global_position_y)(NumericalID veh_id);

//returns the destination edge id of the vehicle
EXTERN NumericalID(*get_destination_edge_id)(NumericalID veh_id);

//returns the current orientation of the vehicle, in radians, with respect to the residing road
EXTERN double (*get_veh_orientation)(NumericalID veh_id);

//apply control for vehicles adhering to the bicycle model by providing the F, and delta values for vehicle with numerical id veh_id
EXTERN void (*apply_control_bicycle_model)(NumericalID veh_id, double F, double delta);

//return the speed of vehicle with numerical id veh_id which adheres to the bicycle model
EXTERN double (*get_speed_bicycle_model)(NumericalID veh_id);

//is called once before the first time-step
EXTERN void simulation_initialize();

//is called in every time-step
EXTERN void simulation_step();

//is called when the simulation ends
EXTERN void simulation_finalize();


//is called when a new vehicle enters
EXTERN void event_vehicle_enter(NumericalID veh_id);

//is called when a new vehicle exits
EXTERN void event_vehicle_exit(NumericalID veh_id);


//is called when two vehicles collide
EXTERN void event_vehicles_collide(NumericalID veh_id1, NumericalID veh_id2);



//is called when a vehicle exceeds the road boundaries
EXTERN void event_vehicle_out_of_bounds(NumericalID veh_id);
