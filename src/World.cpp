#include "World.h"

#include "Map_Node.h"
#include "Agent.h"
#include "Proabability_Node.h"
#include "Agent_Coordinator.h"
#include "Agent_Planning.h"
#include "Goal.h"

#include <vector>
#include <random>
#include <iostream>
#include <fstream>
 
World::World(){}

bool World::init(const std::string &param_file){
	/*
	ROS_INFO("writing param file");
	std::string filename = "/home/nvidia/catkin_ws/src/costmap_bridge/hardware_params.xml";
    cv::FileStorage fr(filename, cv::FileStorage::WRITE);
    fr << "cells_width" << 1000;
    fr << "cells_height" << 1000;
	fr << "nw_longitude" <<-123.251004;
	fr << "nw_latitude" << 44.539847;
	fr << "se_longitude" << -123.247446;
	fr << "se_latitude" << 44.538552;
    fr << "obstacle_image" << "/home/nvidia/catkin_ws/src/costmap_bridge/hardware_obstacles.png";
    fr << "pay_obstacle_costs" << 1;
    fr.release();
    ROS_INFO("wrote param file");
	*/
    cv::FileStorage fs(param_file, cv::FileStorage::READ);
    if (!fs.isOpened()){
        ROS_ERROR("World::World::Failed to open %s", param_file.c_str());
        return false;
    }
	ROS_INFO("World::World::Opened: %s", param_file.c_str());
    
	fs["nw_longitude"] >> this->NW_Corner.x;
	fs["nw_latitude"] >> this->NW_Corner.y;
	fs["se_longitude"] >> this->SE_Corner.x;
	fs["se_latitude"] >> this->SE_Corner.y;
	this->agent_work = std::vector<double>(0.0, 1.0);

	cv::Scalar red(0,0,255);
	cv::Scalar blue(255,0,0);


    fs["n_nodes"] >> this->n_nodes;
	char* node_num;
	for(int i=0; i<this->n_nodes; i++){
		sprintf(node_num, "node_%i", i);
		std::vector<double> n_stuff;
		fs[node_num] >> n_stuff;
		// n_stuff = [x,y,type];
		cv::Scalar color;
		if(int(n_stuff[2]) == 1){
			color = red;
		}
		else{
			color = blue;
		}
		Map_Node* m = new Map_Node(n_stuff[0], n_stuff[1], i, int(n_stuff[2]), this->agent_work, color, this);		
		this->nodes.push_back(m);
	}


	int n_edges = 0;
	fs["n_edges"] >> n_edges;
	char* edge_num;
	for(int i=0; i<n_edges; i++){
		sprintf(edge_num, "edge_%i", i);
		std::vector<double> n_stuff;
		fs[edge_num] >> n_stuff;
		// n_stuff = [node i, node j, free dist, obstacle dist]
		this->nodes[int(n_stuff[0])]->add_nbr(int(n_stuff[1]), n_stuff[2], n_stuff[3]);
		this->nodes[int(n_stuff[1])]->add_nbr(int(n_stuff[0]), n_stuff[2], n_stuff[3]);
	}
    std::string img_name;
	fs["map_img"] >> img_name;
	fs.release();
	ROS_INFO("Dist_Planner::World::initialize_costmap::origin: %0.12f / %0.12f", this->NW_Corner.x, this->NW_Corner.y);

	// initialize cost tracking
	this->cumulative_open_reward = 0.0;
	
	// display world and pause
	//this->last_plot_time = clock() / CLOCKS_PER_SEC;
	//this->display_world(1);
	return true;
}

bool World::on_map(const cv::Point2d &loc){
	ROS_WARN("World::on_map::TODO check if map bounding is right");
	if(loc.x > this->NW_Corner.x && loc.x < this->SE_Corner.x){
		if(loc.y > this->NW_Corner.y && loc.y > this->SE_Corner.y){
			return true;
		}
	}
	return false;
}

bool World::get_prm_location(const cv::Point2d &loc, cv::Point &edge, double &edge_progress){
	// am I on the map?
	if(!this->on_map(loc)){
		return false;
	}

	double min1 = double(INFINITY);
	double min2 = double(INFINITY);
	int ind1 = -1;
	int ind2 = -1;
	for(int i=0; i<this->n_nodes; i++){
		double d = this->get_local_euclidian_distance(loc, this->nodes[i]->get_loc());
		if(d < min1){
			min2 = min1;
			ind2 = ind1;
			min1 = d;
			ind1 = i;
		}
		else if(d < min1){
			min2 = d;
			ind2 = i;
		}
	}
	
	if(ind1 > -1 && ind2 > -1){
		edge.x = ind1;
		edge.y = ind2;
		double dist = this->get_local_euclidian_distance(this->nodes[ind1]->get_loc(), this->nodes[ind2]->get_loc());
		edge_progress = min1 / dist;
		return true;
	}
	else{
		return false;
	}
}

double World::get_local_heading(const cv::Point2d &l1, const cv::Point2d &l2){
	double heading = atan2(l2.x-l1.x,l2.y-l1.y);
	return heading;
}

double World::get_local_euclidian_distance(const cv::Point2d &a, const cv::Point2d &b){
	return sqrt( pow(a.x - b.x,2) + pow(a.y - b.y,2) );
}

double World::get_global_distance(const cv::Point2d &g1, const cv::Point2d &g2){
	double R = 6378136.6; // radius of the earth in meters

	double lat1 = this->to_radians(g1.y);
	double lon1 = this->to_radians(g1.x);
	double lat2 = this->to_radians(g2.y);
	double lon2 = this->to_radians(g2.x);

	double dlon = lon2 - lon1;
	double dlat = lat2 - lat1;

	double a = pow(sin(dlat / 2), 2) + cos(lat1) * cos(lat2) * pow(sin(dlon / 2), 2);
	double c = 2 * atan2(sqrt(a), sqrt(1 - a));

	double distance = R * c; // in meters
	return distance;
}

double World::get_global_heading(const cv::Point2d &g1, const cv::Point2d &g2){
	double lat1 = this->to_radians(g1.y);
	double lat2 = this->to_radians(g2.y);

	double dLong = this->to_radians(g2.x - g1.x);

	double x = sin(dLong) * cos(lat2);
	double y = cos(lat1) * sin(lat2) - (sin(lat1)*cos(lat2)*cos(dLong));

	double heading = atan2(x, y);

	return heading;
}

double World::to_radians(const double &deg){
	return deg*3.141592653589 / 180.0;
}

World::~World(){
	for (size_t i = 0; i < this->nodes.size(); i++) {
		delete this->nodes[i];
	}
	this->nodes.clear();
}


double World::get_team_probability_at_time_except(const double &time, const int &task, const int &except_agent) {
	double p_task_I_time = 0.0;
	for (int a = 0; a < this->n_agents; a++) {
		if (a != except_agent) { // ignore the specified agent
			Agent_Coordinator* coord = this->agents[a]->get_coordinator(); // get coordinator for readability
			double p_t = coord->get_prob_actions()[task]->get_probability_at_time(time); // get probable actions of agent a
			p_task_I_time = coord->get_prob_actions()[task]->probability_update_inclusive(p_task_I_time, p_t); // add to cumulative actions of team
		}
	}
	return p_task_I_time;
}


bool World::a_star(const int &start, const int &goal, std::vector<int> &path, double &length) {

	if (start < 0 || start >= this->n_nodes) {
		std::cerr << "World::a_star:: start off graph" << std::endl;
		return false;
	}
	if (goal < 0 || goal >= this->n_nodes) {
		std::cerr << "World::a_star:: goal off graph" << std::endl;
		return false;
	}

	// garbage variables
	int trash_i = -1;
	double trash_d = -1.0;

	// The set of nodes already evaluated
	std::vector<int> closed_set;

	// The set of currently discovered nodes that are not evaluated yet.
	// Initially, only the start node is known.
	std::vector<int> open_set;
	open_set.push_back(start);

	// For each node, which node it can most efficiently be reached from.
	// If a node can be reached from many nodes, cameFrom will eventually contain the
	// most efficient previous step.
	std::vector<int> came_from(this->n_nodes, -1);

	// For each node, the cost of getting from the start node to that node.
	std::vector<double> gScore(this->n_nodes, double(INFINITY));

	// The cost of going from start to start is zero.
	gScore[start] = 0.0;

	// For each node, the total cost of getting from the start node to the goal
	// by passing by that node. That value is partly known, partly heuristic.
	std::vector<double> fScore(this->n_nodes, double(INFINITY));

	// For the first node, that value is completely heuristic.
	this->dist_between_nodes(start, goal, fScore[start]);

	while (open_set.size() > 0) {
		// the node in openSet having the lowest fScore[] value
		int current = -1;
		if (!get_mindex(fScore, current, trash_d)) {
			return false;
		}
		if (current == goal) {
			path.clear();
			length = 0.0;
			path.push_back(current);
			while (current != start) {
				double l;
				this->dist_between_nodes(current, came_from[current], l);
				length += l;
				current = came_from[current];
				path.push_back(current);
			}
			return true;
		}

		int index = 0;
		if (this->get_index(open_set, current, index)) {
			fScore[current] = double(INFINITY);
			open_set.erase(open_set.begin() + index);
		}
		closed_set.push_back(current);

		int n_nbrs = this->nodes[current]->get_n_nbrs();
		for (int ni = 0; ni < n_nbrs; ni++) {
			int neighbor = -1;
			if (this->nodes[current]->get_nbr_i(ni, neighbor)) {
				if (this->get_index(closed_set, neighbor, trash_i)) { // returns false if not in set
					continue;	// Ignore the neighbor which is already evaluated.
				}

				if (!this->get_index(open_set, neighbor, trash_i)) {// Discover a new node
					open_set.push_back(neighbor);
				}

				// The distance from start to a neighbor
				if (this->dist_between_nodes(current, neighbor, trash_d)) {
					double tentative_gScore = gScore[current] + trash_d;
					if (tentative_gScore >= gScore[neighbor]) {
						continue;		// This is not a better path.
					}

					// This path is the best until now. Record it!
					came_from[neighbor] = current;
					gScore[neighbor] = tentative_gScore;
					if (this->dist_between_nodes(neighbor, goal, trash_d)) {
						fScore[neighbor] = gScore[neighbor] + trash_d;
					}
				}
			}
		}
	}
	return false;
}

bool World::dist_between_nodes(const int &n1, const int &n2, double &d) {
	// am I a node?
	if (n1 >= 0 && n1 < this->n_nodes && n2 >= 0 && n2 < this->n_nodes) {
		// get distance between nodes
		d = sqrt(pow(this->nodes[n1]->get_x() - this->nodes[n2]->get_x(), 2) + pow(this->nodes[n1]->get_y() - this->nodes[n2]->get_y(), 2));
		return true;
	}
	else {
		return false;
	}
}


bool World::get_mindex(const std::vector<double> &vals, int &mindex, double &minval) {
	minval = double(INFINITY);
	mindex = -1;

	for (size_t i = 0; i < vals.size(); i++) {
		if (vals[i] < minval) {
			minval = vals[i];
			mindex = int(i);
		}
	}
	if (mindex == -1) {
		return false;
	}
	else {
		return true;
	}
}

bool World::get_index(const std::vector<int> &vals, const int &key, int &index) {
	for (index = 0; index < int(vals.size()); index++) {
		if (vals[index] == key) {
			return true;
		}
	}
	return false;
}

/*
void World::display_world(const int &ms) {
	if (!this->show_display) {
		return;
	}

	cv::Scalar red(0.0, 0.0, 255.0);
	cv::Scalar blue(255.0, 0.0, 0.0);
	cv::Scalar green(0.0, 255.0, 0.0);
	cv::Scalar white(255.0, 255.0, 255.0);
	cv::Scalar orange(69.0, 100.0, 255.0);
	cv::Scalar black(0.0, 0.0, 0.0);
	cv::Scalar gray(127.0, 127.0, 127.0);

	cv::Mat temp = this->obstacle_mat.clone();
	cv::Mat map = cv::Mat::zeros(cv::Size(int(this->map_width), int(this->map_height) + 100), CV_8UC3);
	temp.copyTo(map(cv::Rect(0, 0, temp.cols, temp.rows)));

	// draw active tasks
	for (int i = 0; i < this->n_nodes; i++) {
		if (this->nodes[i]->is_active()) {
			double d = 15.0;
			cv::Point2d l = this->nodes[i]->get_loc();
			cv::Point2d tl = cv::Point2d(l.x - d, l.y + d);
			cv::Point2d br = cv::Point2d(l.x + d, l.y - d);
			cv::rectangle(map, cv::Rect(tl, br), this->nodes[i]->get_color(), -1);
		}
	}

	// draw agent goals
	for (int i = 0; i < this->n_agents; i++) {
		cv::Point2d l = this->nodes[this->agents[i]->get_goal()->get_index()]->get_loc();
		cv::circle(map, l, 8, this->agents[i]->get_color(), -1);
		cv::circle(map, l, 4, black, -1);
	}

	// draw agents
	for (int i = 0; i < this->n_agents; i++) {
		cv::Point2d l = this->agents[i]->get_loc2d();
		cv::circle(map, l, 8, this->agents[i]->get_color(), -1);
		//cv::Point2d tl = cv::Point2d(l.x - 3.0, l.y + 3.0);
		//cv::Point2d br = cv::Point2d(l.x + 3.0, l.y - 3.0);
		//cv::rectangle(map, cv::Rect(tl, br), orange, -1);
	}

	cv::putText(map, this->task_selection_method[this->test_iter], cv::Point2d(40.0, this->map_height + 30.0), CV_FONT_HERSHEY_COMPLEX, 1.0, white, 3);
	char time[200];
	sprintf_s(time, "Time: %.2f of %.2f", this->c_time, this->end_time);
	cv::putText(map, time, cv::Point2d(30.0, this->map_height + 80.0), CV_FONT_HERSHEY_COMPLEX, 1.0, white, 3);


	double current_plot_time = 1000.0 * clock() / double(CLOCKS_PER_SEC);
	cv::namedWindow("Map", cv::WINDOW_NORMAL);
	cv::imshow("Map", map);
	if (ms == 0) {
		cv::waitKey(0);
	}
	else if (double(ms) - (this->last_plot_time - current_plot_time) < 0.0) {
		cv::waitKey(ms - int(floor(this->last_plot_time - current_plot_time)));
	}
	else {
		cv::waitKey(1);
	}
	this->last_plot_time = 1000.0 * clock() / double(CLOCKS_PER_SEC);
}

Point2d Planner_Utils::local_to_global(const Point2d &local){
	Point2d global;

	double C_EARTH = 6378136.6;
	double dlati = local.x / C_EARTH;
	double lati = dlati + this->origin_lati;
	dlongti = y / ( C_EARTH * math.cos(lati / 2.0 + origin_lati / 2.0) )
	longti = dlongti + origin_longti

	return [lati, longti]


	return global;
}

<<<<<<< HEAD
Point2d Planner_Utils::global_to_local(const Point2d &global){
	Point2d local;
	double d = distance_from_a_to_b( home_lati, origin_longti, lati, longti )
	double b = heading_from_a_to_b( home_lati, origin_longti, lati, longti )

	local.x = -d*math.sin(b)
	local.y = d*math.cos(b)

=======
bool Planner_Utils::initialize_costmap(){

		

	// initialize cells
	cv::Mat a = cv::Mat::ones( this->map_size_cells.y, this->map_size_cells.x, CV_16S)*this->infFree;
	this->cells = a.clone();
	ROS_INFO("map size: %i, %i (cells)", this->cells.cols, this->cells.rows);
	
	// seed euclid distance, makes everything faster
	a = cv::Mat::ones( this->cells.size(), CV_32FC1)*-1;
	this->euclidDist = a.clone();


	// seed into cells satelite information
	this->seed_img();

	ROS_INFO("nw / se: (%0.6f, %0.6f) / (%0.6f, %0.6f)", this->NW_Corner.x, this->NW_Corner.y, this->SE_Corner.x, this->SE_Corner.y);
	// set map width / height in meters
	double d = this->get_global_distance(this->NW_Corner, this->SE_Corner);
	double b = this->get_global_heading(this->NW_Corner, this->SE_Corner);
	this->map_size_meters.x = abs(d*sin(b));
	this->map_size_meters.y = abs(d*cos(b));
	
	ROS_INFO("map size: %0.2f, %0.2f (meters)", this->map_size_meters.x, this->map_size_meters.y);
	
	// set cells per meter
	this->meters_per_cell.x = this->map_size_meters.x / double(this->map_size_cells.x);
	this->meters_per_cell.y = this->map_size_meters.y / double(this->map_size_cells.y);

	// set meters per cell
	this->cells_per_meter.x = double(this->map_size_cells.x) / this->map_size_meters.x;
	this->cells_per_meter.y = double(this->map_size_cells.y) / this->map_size_meters.y;

	if(pay_obstacle_costs){
		this->obsFree_cost = 0.0;
		this->infFree_cost = 0.05;
		this->infOcc_cost = 1.0;
		this->obsOcc_cost = 10.0;
	}
	else{
		this->obsFree_cost = 0.0;
		this->infFree_cost = 0.0;
		this->infOcc_cost = 0.0;
		this->obsOcc_cost = 0.0;
	}

	// announce I am initialized!
	this->need_initialization = false;

	ROS_INFO("Planner_Utils::initialize_costmap::complete");
	return true;
}

bool World::get_prm_location(const cv::Point2d&loc, cv::Point &edge, double &edge_progress){
	// am I on the map?
	if(!this->on_map(loc){
		return false;
	}

	double min1 = double(INFINITY);
	double min2 = double(INFINITY);
	int ind1 = -1;
	int ind2 = -1;
	for(int i=0; i<this->n_nodes; i++){
		double d = this->get_euclid_dist(loc, this->map_nodes[i]->get_loc());
		if(d < min1){
			min2 = min1;
			ind2 = ind1;
			min1 = d;
			ind1 = i;
		}
		else if(d < min1){
			min2 = d;
			ind2 = i;
		}
	}
	
	if(ind1 > -1 && ind2 > -1){
		edge.x = ind1;
		edge.y = ind2;
		double dist = this->get_euclid_dist(this->map_nodes[ind1]->get_loc(), this->map_nodes[ind2]->get_loc());
		edge_progress = min1/dist;
		return true;
	}
	else{
		return false;
	}
}

double World::get_local_heading(const cv::Point2d &l1, const cv::Point2d &l2){
	double heading = atan2(l2.x-l1.x,l2.y-l1.y);
	return heading;
}

double World::get_local_euclidian_distance(const cv::Point2d &a, const cv::Point2d &b){
	return sqrt( pow(a.x - b.x,2) + pow(a.y - b.y,2) );
}

double World::get_global_distance(const cv::Point2d &g1, const cv::Point2d &g2){
	double R = 6378136.6; // radius of the earth in meters

	double lat1 = this->to_radians(g1.y);
	double lon1 = this->to_radians(g1.x);
	double lat2 = this->to_radians(g2.y);
	double lon2 = this->to_radians(g2.x);

	double dlon = lon2 - lon1;
	double dlat = lat2 - lat1;

	double a = pow(sin(dlat / 2), 2) + cos(lat1) * cos(lat2) * pow(sin(dlon / 2), 2);
	double c = 2 * atan2(sqrt(a), sqrt(1 - a));

	double distance = R * c; // in meters
	return distance;
}

double World::get_global_heading(const cv::Point2d &g1, const cv::Point2d &g2){
	double lat1 = this->to_radians(g1.y);
	double lat2 = this->to_radians(g2.y);

	double dLong = this->to_radians(g2.x - g1.x);

	double x = sin(dLong) * cos(lat2);
	double y = cos(lat1) * sin(lat2) - (sin(lat1)*cos(lat2)*cos(dLong));

	double heading = atan2(x, y);

	return heading;
}

double World::to_radians(const double &deg){
	return deg*3.141592653589 / 180.0;
}

/*
Point2d Planner_Utils::local_to_global(const Point2d &local){
	Point2d global;

	double C_EARTH = 6378136.6;
	double dlati = local.x / C_EARTH;
	double lati = dlati + this->origin_lati;
	dlongti = y / ( C_EARTH * math.cos(lati / 2.0 + origin_lati / 2.0) )
	longti = dlongti + origin_longti

	return [lati, longti]


	return global;
}

Point2d Planner_Utils::global_to_local(const Point2d &global){
	Point2d local;
	double d = distance_from_a_to_b( home_lati, origin_longti, lati, longti )
	double b = heading_from_a_to_b( home_lati, origin_longti, lati, longti )

	local.x = -d*math.sin(b)
	local.y = d*math.cos(b)

>>>>>>> 6c97866f7b60da99457c5b66d843a3df63bf554c
	return local;
}

void World::score_and_record_all() {

	char result_file_name[200];
	int n = sprintf_s(result_file_name, "C:/Users/sgtas/OneDrive/Documents/GitHub/Dist_MCTS_Testing/Dist_MCTS_Testing/results/results_for_params_%i.txt", this->rand_seed);
	std::ofstream record_out(result_file_name, std::ios::app);

	if (record_out.is_open()) {
		record_out << this->c_time << ",";
		record_out << this->n_agents << ",";
		record_out << this->n_task_types << ",";
		for (int a = 0; a < this->n_agents; a++) {
			record_out << this->agents[a]->get_work_done() << ",";
		}
		for (int a = 0; a < this->n_agents; a++) {
			record_out << this->agents[a]->get_travel_done() << ",";
		}
		for (int a = 0; a < this->n_agents; a++) {
			record_out << this->agents[a]->get_type() << ",";
		}
		//record_out << this->agents[0]->get_task_selection_method() << ",";
		//record_out << this->agents[0]->get_task_claim_method() << ",";
		//record_out << this->agents[0]->get_task_claim_time() << ",";

		record_out << this->get_time_step_open_reward() << "\n";
	}
	record_out.close();
}

void World::load_params() {

	char temp_char[200];
	sprintf_s(temp_char, 200, "C:/Users/sgtas/OneDrive/Documents/GitHub/Dist_MCTS_Testing/Dist_MCTS_Testing/param_files/param_file_%i.xml", param_file_index);
	this->param_file = temp_char;
	cv::FileStorage fs;
	fs.open(this->param_file, cv::FileStorage::READ);
	
	// rand seed stuff
	fs["rand_seed"] >> this->rand_seed;

	// time stuff
	fs ["c_time"] >> this->c_time;
	fs ["dt"] >> this->dt;
	fs ["end_time"] >> this->end_time;

	// map and PRM stuff
	fs ["map_height"] >> this->map_height;
	fs ["map_widht"] >> this->map_width;
	fs ["n_nodes"] >> this->n_nodes;
	fs ["k_map_connections"] >> this->k_map_connections;
	fs ["k_connection_radius"] >> this->k_connection_radius;
	fs["p_connect"] >> this->p_connect;

	// task stuff
	fs ["n_task_types"] >> this->n_task_types; // how many types of tasks are there
	fs ["p_task_initially_active"] >> this->p_task_initially_active; // how likely is it that a task is initially active
	fs ["p_impossible_task"] >> this->p_impossible_task; // how likely is it that an agent is created that cannot complete a task
	fs ["p_active_task"] >> this->p_activate_task; // how likely is it that I will activate a task each second? *dt accounts per iters per second
	fs ["min_task_time"] >>this->min_task_time; // shortest time to complete a task
	fs ["max_task_time"] >> this->max_task_time; // longest time to complete a task

	// agent stuff
	fs ["n_agents"] >> this->n_agents; // how many agents
	fs ["n_agent_types"] >> this->n_agent_types; // how many types of agents
	fs ["min_travel_vel"] >> this->min_travel_vel; // slowest travel speed
	fs ["max_travel_vel"] >> this->max_travel_vel; // fastest travel speed

	fs.release();
}

bool World::get_task_completion_time(const int &ai, const int &ti, double &time) {
	if (this->valid_agent(ai) && this->valid_node(ti)) {
		time = this->nodes[ti]->get_time_to_complete(this->agents[ai], this);
		return true;
	}
	else {
		return false;
	}
}

bool World::valid_agent(const int a) {
	if (a < this->n_agents && a > -1) {
		return true;
	}
	else {
		return false;
	}
}

bool World::get_travel_time(const int &s, const int &g, const double &step_dist, double &time) {
	if (this->valid_node(s) && this->valid_node(g)) {
		double dist;
		std::vector<int> path;
		if (this->a_star(s, g, path, dist)) {
			time = dist / step_dist;
			return true;
		}
		else {
			return false;
		}
	
	}
	else {
		return false;
	}
}

bool World::valid_node(const int &n) {
	if (n < this->n_nodes && n > -1) {
		return true;
	}
	else {
		return false;
	}
}

double World::get_time_step_open_reward() {

	double time_step_reward = 0.0;
	double c_time = std::clock() / double(CLOCKS_PER_SEC);
	for (int i = 0; i < this->n_nodes; i++) {
		if (this->nodes[i]->is_active()) {
			double task_reward = this->nodes[i]->get_reward_at_time(c_time);
			time_step_reward += task_reward;
		}
	}
	//std::cout << "Time step reward: " << time_step_reward << std::endl;
	return time_step_reward;
}

void World::initialize_PRM() {
	// connect all nodes within radius
	for (int i = 0; i < this->n_nodes; i++) {
		for (int j = i+1; j < this->n_nodes; j++) {
			double d;
			if (this->dist_between_nodes(i, j, d)) {
				if (d < this->k_connection_radius && rand() < this->p_connect) {
					this->nodes[i]->add_nbr(j, d);
					this->nodes[j]->add_nbr(i, d);
				}
			}
		}

		// if not enough connections were done, add some more connections until min is reached
		while(this->nodes[i]->get_n_nbrs() < this->k_map_connections){
			double min_dist = double(INFINITY);
			int mindex = -1;
			
			for (int j = 0; j < this->n_nodes; j++) {
				if (i != j) {
					bool in_set = false;
					for (int k = 0; k < this->nodes[i]->get_n_nbrs(); k++) {
						int n_i = 0;
						if (this->nodes[i]->get_nbr_i(k,n_i)){
							if (n_i == j) {
								in_set = true;
								break;
							}
						}
					}
					if (!in_set) {
						double d;
						if (this->dist_between_nodes(i, j, d)) {
							if (d < min_dist) {
								min_dist = d;
								mindex = j;
							}
						}
					}
				}
			}
			this->nodes[i]->add_nbr(mindex, min_dist);
			this->nodes[mindex]->add_nbr(i, min_dist);
		}
	}
}

double World::rand_double_in_range(const double &min, const double &max) {
	// get random double between min and max
	return (max - min) * double(rand()) / double(RAND_MAX) + min;
}

double World::dist2d(const double &x1, const double &x2, const double &y1, const double &y2) {
	// get distance between 1 and 2
	return sqrt(pow(x1 - x2, 2) + pow(y1 - y2, 2));
}

void World::initialize_nodes_and_tasks() {
	std::vector<std::vector<double>> task_times;
	std::vector<cv::Scalar> task_colors;
	for (int t = 0; t < n_task_types; t++) {

		double r = rand_double_in_range(0.0, 255.0);
		double b = rand_double_in_range(0.0, 255.0);
		double g = rand_double_in_range(0.0, 255.0);

		cv::Scalar color(b, g, r);
		task_colors.push_back(color);

		std::vector<double> at;
		for (int a = 0; a < n_agent_types; a++) {
			if (this->rand_double_in_range(0.0, 1.0) < p_impossible_task) {
				at.push_back(double(INFINITY));
			}
			else {
				double tt = this->rand_double_in_range(min_task_time, max_task_time);
				at.push_back(tt);
			}
		}
		task_times.push_back(at);
	}

	for (int i = 0; i < this->n_nodes; i++) {
		double x = this->rand_double_in_range(0, this->map_width);
		double y = this->rand_double_in_range(0, this->map_height);
		int task_type = rand() % n_task_types;
		Map_Node* n = new Map_Node(x, y, i, this->p_task_initially_active, task_type, task_times[task_type], task_colors[task_type], this);
		this->nodes.push_back(n);
	}
}
*/

