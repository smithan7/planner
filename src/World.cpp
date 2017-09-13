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
 
World::World(){
	// initialize cost tracking
	this->cumulative_open_reward = 0.0;
	this->dt = 0.2;
	this->map_size_meters.x = 0.0;
	this->map_size_meters.y = 0.0;

}

bool World::init(const int &test_environment_number, const int &test_scenario_number){

	this->show_display = true;

	// initialize PRM and tasks	
	if(!this->load_PRM_vertices(test_environment_number)){
		ROS_ERROR("Dist Planner::World::init::could not load vertices");
		return false;
	}
	if(!this->load_PRM_edges(test_environment_number)){
		ROS_ERROR("Dist Planner::World::init::could not load edges");
		return false;
	}
	// load general information
	if(!this->load_test_scenario(test_scenario_number)){	
		ROS_ERROR("Dist Planner::World::init::could not load test conditions");
		return false;	
	}
	if(!this->load_human_tasks(test_environment_number)){
		ROS_ERROR("Dist Planner::World::init::could not load human tasks");
		return false;	
	}
	if(!this->load_robot_tasks(test_environment_number)){
		ROS_ERROR("Dist Planner::World::init::could not load robot tasks");
		return false;	
	}

	ROS_INFO("Dist planner::World::init::created PRM with %i nodes and %i edges", this->n_nodes, this->n_edges);
	ROS_INFO("Dist_Planner::World::init::PRM origin: %0.12f / %0.12f", this->NW_Corner.x, this->NW_Corner.y);
	ROS_INFO("Dist_Planner::World::init::Map Size: %0.2f / %0.2f", this->map_size_meters.x, this->map_size_meters.y);
	ROS_INFO("Dist_Planner::World::init::%i human and %i robot tasks", this->n_human_tasks, this->n_robot_tasks);


	this->show_prm_plot();

	return true;
}

bool World::load_test_scenario(const int &test_scenario_number){
	char sc_file[200];
	//sprintf(sc_file, "/home/nvidia/catkin_ws/src/distributed_planner/params/test_scenario%i.xml", test_scenario_number);
    sprintf(sc_file, "/home/andy/catkin_ws/src/distributed_planner/params/test_scenario%i.xml", test_scenario_number);
    cv::FileStorage f_scenario(sc_file, cv::FileStorage::READ);
    if (!f_scenario.isOpened()){
        ROS_ERROR("Dist planner::World::init::Failed to open %s", sc_file);
        return false;
    }
	ROS_INFO("Dist planner::World::init::Opened: %s", sc_file);
    
	f_scenario["num_agents"] >> this->n_agents;
	ROS_INFO("Dist planner::World::init::n_agents: %i", this->n_agents);
	for(int i=0; i<this->n_agents; i++){
		Agent_Coordinator* a = new Agent_Coordinator(this->n_nodes);
		this->agent_coords.push_back(a);	
	}
	ROS_INFO("Dist planner::World::init::created %i agent coordinators", this->n_agents);

	f_scenario["n_agent_types"] >> this->n_agent_types;
	f_scenario["agent_types"] >> this->agent_types;
	f_scenario["n_task_types"] >> this->n_task_types;

    char type_num[200];
	this->agent_work.clear();
	for(int i=0; i<this->n_task_types; i++){
		sprintf(type_num, "type%i", i);
		std::vector<double> n_stuff;
		f_scenario[type_num] >> n_stuff;
		this->agent_work.push_back(n_stuff);
	}
	f_scenario.release();

	ROS_INFO("Dist planner::World::init::set work for %i task types", this->n_task_types);
	ROS_INFO("   %0.1f, %0.1f", this->agent_work[0][0], this->agent_work[0][1]);
	ROS_INFO("   %0.1f, %0.1f", this->agent_work[1][0], this->agent_work[1][1]); 
    return true;
}

bool World::load_PRM_vertices(const int &test_environment_number){

	char vert_file[200];
	sprintf(vert_file, "/home/andy/catkin_ws/src/distributed_planner/params/hardware%i_vertices.xml", test_environment_number);
	//sprintf(vert_file, "/home/nvidia/catkin_ws/src/distributed_planner/params/hardware%i_vertices.xml", test_environment_number);
    cv::FileStorage f_verts(vert_file, cv::FileStorage::READ);
    if (!f_verts.isOpened()){
        ROS_ERROR("Dist planner::World::init::Failed to open %s", vert_file);
        return false;
    }
	ROS_INFO("Dist planner::World::init::Opened: %s", vert_file);
    
	std::vector<double> corners;
	f_verts["corners"] >> corners;
	this->NW_Corner.x = corners[0];
	this->NW_Corner.y = corners[1];
	this->SE_Corner.x = corners[2];
	this->SE_Corner.y = corners[3];
	this->SW_Corner.x = corners[0];
	this->SW_Corner.y = corners[3];
	this->NE_Corner.x = corners[2];
	this->NE_Corner.y = corners[1];

	this->map_size_meters.x = 0.0;
	this->map_size_meters.y = 0.0;
	this->map_size_meters = this->global_to_local(this->NE_Corner);
	this->map_size_meters.x = abs(this->map_size_meters.x);
	this->map_size_meters.y = abs(this->map_size_meters.y);
	
	ROS_INFO("    map size: %0.2f, %0.2f (meters)", this->map_size_meters.x, this->map_size_meters.y);

	// set map nodes
	cv::Scalar red(0,0,255);
	std::vector<double> default_work = std::vector<double>(1.0, 0.0);

    f_verts["n_vertices"] >> this->n_nodes;
	char node_num[200];
	for(int i=0; i<this->n_nodes; i++){
		sprintf(node_num, "vertex%i_gps", i);
		std::vector<double> n_stuff;
		f_verts[node_num] >> n_stuff;
		// n_stuff = [x,y];
		//ROS_INFO("vertice[%i]: %0.6f, %0.6f", i, n_stuff[0], n_stuff[1]);
		Map_Node* m = new Map_Node(n_stuff[0], n_stuff[1], i, 0, default_work, red, this);	
		this->nodes.push_back(m);
		//ROS_INFO("v[%i] local loc: %0.2f, %0.2f", i, m->get_local_loc().x, m->get_local_loc().y);
	}
	f_verts.release();
	return true;
}

bool World::load_PRM_edges(const int &test_environment_number){
	char edge_file[200];
	//sprintf(edge_file, "/home/nvidia/catkin_ws/src/distributed_planner/params/hardware%i_edges.xml", test_environment_number);
	sprintf(edge_file, "/home/andy/catkin_ws/src/distributed_planner/params/hardware%i_edges.xml", test_environment_number);
    cv::FileStorage f_edges(edge_file, cv::FileStorage::READ);
    if (!f_edges.isOpened()){
        ROS_ERROR("Dist planner::World::init::Failed to open %s", edge_file);
        return false;
    }
	ROS_INFO("Dist planner::World::init::Opened: %s", edge_file);
    
    int n_edges = 0;
	f_edges["n_edges"] >> this->n_edges;
	//ROS_INFO("n_edges: %i", this->n_edges);
	char edge_num[200];
	for(int i=0; i<this->n_edges; i++){
		sprintf(edge_num, "edge%i", i);
		std::vector<double> n_stuff;
		f_edges[edge_num] >> n_stuff;
		// n_stuff = [node i, node j, obstacle dist, obstacle var, free distance]
		//ROS_INFO("node[%i], node[%i], obs dist[%.2f], free dist[%.2f]", int(n_stuff[0]), int(n_stuff[1]), int(n_stuff[2]), int(n_stuff[4]));
		this->nodes[int(n_stuff[0])]->add_nbr(int(n_stuff[1]), n_stuff[4], n_stuff[2]);
		this->nodes[int(n_stuff[1])]->add_nbr(int(n_stuff[0]), n_stuff[4], n_stuff[2]);
		//ROS_INFO("nodes[%i].n_nbrs(): %i", int(n_stuff[0]), this->nodes[int(n_stuff[0])]->get_n_nbrs());
	}
    f_edges.release();
    return true;
} 

bool World::load_human_tasks(const int &test_environment_number){
	char human_file[200];
	sprintf(human_file, "/home/andy/catkin_ws/src/distributed_planner/params/hardware%i_human_tasks.xml", test_environment_number);
	//sprintf(human_file, "/home/nvidia/catkin_ws/src/distributed_planner/params/hardware%i_human_tasks.xml", test_environment_number);
    cv::FileStorage f_human(human_file, cv::FileStorage::READ);
    if (!f_human.isOpened()){
        ROS_ERROR("Dist planner::World::init::Failed to open %s", human_file);
        return false;
    }
	ROS_INFO("Dist planner::World::init::Opened: %s", human_file);
    
	f_human["n_tasks"] >> this->n_human_tasks;
	char task_num[200];
	for(int i=0; i<n_human_tasks; i++){
		sprintf(task_num, "task%i", i);
		int n_stuff;
		f_human[task_num] >> n_stuff;
		// n_stuff = [x], where node "x" is a human task]
		this->nodes[n_stuff]->set_work(this->agent_work[0]);
		this->nodes[n_stuff]->set_task_type(0);
		this->nodes[n_stuff]->activate(this);
	}
    f_human.release();
    return true;
}

bool World::load_robot_tasks(const int &test_environment_number){

	char robot_file[200];
	sprintf(robot_file, "/home/andy/catkin_ws/src/distributed_planner/params/hardware%i_uav_tasks.xml", test_environment_number);
	//sprintf(robot_file, "/home/nvidia/catkin_ws/src/distributed_planner/params/hardware%i_uav_tasks.xml", test_environment_number);
    cv::FileStorage f_robot(robot_file, cv::FileStorage::READ);
    if (!f_robot.isOpened()){
        ROS_ERROR("Dist planner::World::init::Failed to open %s", robot_file);
        return false;
    }
	ROS_INFO("Dist planner::World::init::Opened: %s", robot_file);
    
    f_robot["n_tasks"] >> this->n_robot_tasks;
	for(int i=0; i<n_robot_tasks; i++){
		char task_num[200];
		sprintf(task_num, "task%i", i);
		int n_stuff;
		f_robot[task_num] >> n_stuff;
		// n_stuff = [x], where node "x" is a robot task]
		this->nodes[n_stuff]->set_work(this->agent_work[1]);
		this->nodes[n_stuff]->set_task_type(1);
		this->nodes[n_stuff]->activate(this);
	}
    f_robot.release();
    return true;
}

void World::add_stop_to_agents_path(const int &agent_index, const std::vector<int> &task_index, const std::vector<double> &probability, const std::vector<double> &time){
	ROS_WARN("Dist Planner::World::add stop to agent path: TODO: only using a single bid");
	this->agent_coords[agent_index]->reset_prob_actions();

	for(size_t i=0; i<task_index.size(); i++){
		this->agent_coords[agent_index]->add_stop_to_path(task_index[i], time[i], probability[i]);
	}
}


bool World::on_map(const cv::Point2d &loc){
	if(loc.x > 0.0  && loc.x < this->map_size_meters.x){
		if(loc.y > 0.0 && loc.y < this->map_size_meters.y){
			return true;
		}
	}
	
	/*	
	if(loc.x > this->NW_Corner.x && loc.x < this->SE_Corner.x){
		if(loc.y > this->NW_Corner.y && loc.y > this->SE_Corner.y){
			return true;
		}
	}
	*/
	return false;
}

cv::Point2d World::global_to_local(const cv::Point2d &loc){
	double b = this->get_global_heading(this->SW_Corner, loc);
	double d = this->get_global_distance(this->SW_Corner, loc);
	//ROS_INFO("Distance/Bearing: %0.2f, %0.2f", d,b);
	//ROS_INFO("SW_Corner: %0.6f, %0.6f", this->SW_Corner.x, this->SW_Corner.y);
	cv::Point2d l;
	l.x = d*sin(b);
	l.y =this->map_size_meters.y - d*cos(b);
	//ROS_INFO("Point: %0.2f, %0.2f", d*sin(b), d*cos(b));
	//ROS_INFO("Map size: %0.2f, %0.2f", this->map_size_meters.x, this->map_size_meters.y);
	

	return l;
}

bool World::get_prm_location(const cv::Point2d &loc, cv::Point &edge, double &edge_progress, const double  &radius, std::vector<int> &visiting_nodes){
	// am I on the map?
	visiting_nodes.clear();	
	if(!this->on_map(loc)){
		return false;
	}

	double min1 = double(INFINITY);
	double min2 = double(INFINITY);
	int ind1 = -1;
	int ind2 = -1;
	for(int i=0; i<this->n_nodes; i++){
		double d = this->get_local_euclidian_distance(loc, this->nodes[i]->get_local_loc());
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
		
		if(d < radius){
			visiting_nodes.push_back(i);
		}
	}
	
	if(ind1 > -1 && ind2 > -1){
		edge.x = ind1;
		edge.y = ind2;
		double dist = this->get_local_euclidian_distance(this->nodes[ind1]->get_local_loc(), this->nodes[ind2]->get_loc());
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
			double p_t = this->agent_coords[a]->get_prob_actions()[task]->get_probability_at_time(time); // get probable actions of agent a
			p_task_I_time = this->agent_coords[a]->get_prob_actions()[task]->probability_update_inclusive(p_task_I_time, p_t); // add to cumulative actions of team
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
		d = sqrt(pow(this->nodes[n1]->get_local_loc().x - this->nodes[n2]->get_local_loc().x, 2) + pow(this->nodes[n1]->get_local_loc().y - this->nodes[n2]->get_local_loc().y, 2));
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

void World::show_prm_plot() {
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

	cv::Mat map = cv::Mat::zeros(cv::Size(int(this->map_size_meters.x*10), int(this->map_size_meters.y*10) + 100), CV_8UC3);

	// draw active tasks
for (int i = 0; i < this->n_nodes; i++) {
		if (this->nodes[i]->is_active()) {
			cv::Point2d l = this->nodes[i]->get_local_loc();
			l.x *= 10;
			l.y *= 10;
			if(this->nodes[i]->get_task_type() == 0){
				cv::circle(map, l, 10, red, -1);
			}
			else{
				cv::circle(map, l, 10, green, -1);
			}
		}
		else{
			double d = 15.0;
			cv::Point2d l = this->nodes[i]->get_local_loc();
			l.x *= 10;
			l.y *= 10;
			cv::circle(map, l, 10, white, -1);
		
		}
	}

	cv::putText(map, this->task_selection_method, cv::Point2d(40.0, 10*this->map_size_meters.y + 30.0), CV_FONT_HERSHEY_COMPLEX, 1.0, white, 3);
	char time[200];
	sprintf(time, "Time: %.2f of %.2f", this->c_time, this->end_time);
	cv::putText(map, time, cv::Point2d(30.0, 10*this->map_size_meters.y + 80.0), CV_FONT_HERSHEY_COMPLEX, 1.0, white, 3);

	//cv::namedWindow("map", CV_WINDOW_NORMAL);
	//cv::imshow("map", map);
	//cv::waitKey(10);

}

void World::display_world(Agent* agent) {
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
	cv::Mat map_d = cv::Mat::zeros(cv::Size(int(this->map_size_meters.x*10), int(this->map_size_meters.y*10)), CV_8UC3);

	char img_file[200];
	sprintf(img_file, "/home/andy/catkin_ws/src/distributed_planner/params/hardware%i_img.png", 4);
	//sprintf(img_file, "/home/nvidia/catkin_ws/src/distributed_planner/params/hardware%i_img.png", 4);
	cv::Mat map = cv::imread(img_file, CV_LOAD_IMAGE_COLOR);
	cv::resize(map, map, map_d.size());

	for (int i = 0; i < this->n_nodes; i++) {
		cv::Point2d l = this->nodes[i]->get_local_loc();
		l.x *= 10.0;
		l.y *= 10.0;
		for(int j=0; j<this->nodes[i]->get_n_nbrs(); j++){
			int ni;
			this->nodes[i]->get_nbr_i(j, ni);
			cv::Point2d n = this->nodes[ni]->get_local_loc();
			n.x *= 10.0;
			n.y *= 10.0;
			cv::line(map, l, n, black, 1);
		}
	}
	

	// draw tasks
	for (int i = 0; i < this->n_nodes; i++) {
		cv::Point2d l = this->nodes[i]->get_local_loc();
		l.x *= 10.0;
		l.y *= 10.0;
		if (this->nodes[i]->is_active()) {
			if(this->nodes[i]->get_task_type() == 0){
				cv::circle(map, l, 20, red, -1);
			}
			else{
				cv::circle(map, l, 20, green, -1);
			}
		}
		else{ // draw inactive tasks
			cv::circle(map, l, 10, white, -1);
		}
		//char time[20];
		//sprintf(time, "%i", i);
		//cv::putText(map, time, l, CV_FONT_HERSHEY_COMPLEX, 1.0, white, 3);

	}
	

	// draw agent goals
	cv::Point2d l = this->nodes[agent->get_goal()->get_index()]->get_local_loc();
	l.x *= 10;
	l.y *= 10;
	if(this->nodes[agent->get_goal()->get_index()]->get_task_type() == 0){
		cv::circle(map, l, 20, blue, -1);
		cv::circle(map, l, 10, red, -1);
	}
	else{
		cv::circle(map, l, 20, blue, -1);
		cv::circle(map, l, 10, green, -1);
	}

	l = this->nodes[agent->get_edge().x]->get_local_loc();
	l.x *= 10;
	l.y *= 10;
	cv::circle(map, l, 5, orange, -1);

	l = this->nodes[agent->get_edge().y]->get_local_loc();
	l.x *= 10;
	l.y *= 10;
	cv::circle(map, l, 5, orange, -1);

	// draw agents
	l = agent->get_loc2d();
	l.x *= 10;
	l.y *= 10;
	cv::circle(map, l, 20, blue, -1);
	
	cv::putText(map, this->task_selection_method, cv::Point2d(40.0, 10*this->map_size_meters.x + 30.0), CV_FONT_HERSHEY_COMPLEX, 1.0, white, 3);
	char time[200];
	sprintf(time, "Time: %.2f of %.2f", this->c_time, this->end_time);
	cv::putText(map, time, cv::Point2d(30.0, 10*this->map_size_meters.y + 80.0), CV_FONT_HERSHEY_COMPLEX, 1.0, white, 3);

	cv::namedWindow("map", CV_WINDOW_NORMAL);
	cv::imshow("map", map);
	cv::waitKey(10);
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

