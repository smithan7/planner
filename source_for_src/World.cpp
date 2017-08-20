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
#include <windows.h>

#include <opencv2\core.hpp>
#include <opencv2\highgui.hpp>
#include <opencv2\imgproc.hpp>

World::World(const int &param_file, const bool &display_plot, const std::vector<cv::String> &task_selection_method, const cv::String &img_name ) {
	this->show_display = display_plot;
	this->param_file_index = param_file;
	this->n_iterations = int(task_selection_method.size());
	this->task_selection_method = task_selection_method;
	this->test_iter = 0;

	if (this->param_file_index > 0) {
		// load  everything from xml
		this->load_params();
	}
	else { // generate_params and then write them

		// seed the randomness in the simulator
		srand(int(time(NULL)));
		this->rand_seed = rand() % 1000000;
		srand(this->rand_seed);

		// time stuff
		this->c_time = 0.0;
		this->dt = 0.1;
		this->end_time = 60.0;

		// should I plot?
		this->show_display = true;

		// map and PRM stuff
		this->map_height = 1000.0;
		this->map_width = 1000.0;
		this->n_nodes = 200;
		this->k_map_connections = 5;
		this->k_connection_radius = 500.0;
		this->p_connect = 1.0;
		this->n_obstacles = 10;
		this->min_obstacles_radius = 20;
		this->max_obstacle_radius = 100;
		
		// task stuff
		this->n_task_types = 4; // how many types of tasks are there
		this->p_task_initially_active = 0.25; // how likely is it that a task is initially active
		this->p_impossible_task = 0.0; // how likely is it that an agent is created that cannot complete a task
		this->p_activate_task = 1.0*this->dt; // how likely is it that I will activate a task each second? *dt accounts per iters per second
		this->min_task_time = 1.0; // shortest time to complete a task
		this->max_task_time = 5.0; // longest time to complete a task

		// agent stuff
		this->n_agents = 5; // how many agents
		this->n_agent_types = 4; // how many types of agents
		this->min_travel_vel = 100.0; // slowest travel speed
		this->max_travel_vel = 500.0; // fastest travel speed

		// write params
		this->write_params();
	}

	for (this->test_iter = 0; this->test_iter < this->n_iterations; this->test_iter++) {
		// reset randomization
		srand(this->rand_seed);

		// initialize cost tracking
		this->cumulative_open_reward = 0.0;

		// initialize clock
		this->c_time = 0.0;

		// initialize map, tasks, and agents
		this->initialize_nodes_and_tasks();
		this->initialize_PRM();
		
		// create obstacles on map
		this->create_random_obstacles();
		cv::namedWindow("this->obstacle_mat", cv::WINDOW_NORMAL);
		cv::imshow("this->obstacle_mat", this->obstacle_mat);
		cv::waitKey(10);
		this->find_obstacle_costs();
		
		// initialize agents
		this->initialize_agents();

		// display world and pause
		this->last_plot_time = clock() / CLOCKS_PER_SEC;
		this->display_world(1);


		// run the complete simulation
		this->run_simulation();

		// clear everything out
		this->clean_up_from_sim();
	}
}

void World::create_random_obstacles() {
	this->obstacle_mat = cv::Mat::zeros(int(this->map_width), int(this->map_height), CV_8UC3);

	for (int i = 0; i < this->n_obstacles; i++) {
		int r = (rand() %  (this->max_obstacle_radius - this->min_obstacles_radius)) + this->min_obstacles_radius;
		cv::Point c;
		c.x = rand() % int(this->map_width);
		c.y = rand() % int(this->map_height);
		cv::circle(this->obstacle_mat, c, r, cv::Vec3b(0,0,100), -1);
	}
}

void World::find_obstacle_costs() {
	// go through each node and for each of their nbrs, get the obstacle cost for going through each node
	for (int i = 0; i < this->n_nodes; i++) {
		// get node[i]'s location on the img
		cv::Point me = this->nodes[i]->get_loc();
		// go through all node[i]'s nbrs
		int iter = 0;
		int ni = 0;
		// get their node[?] index
		while(this->nodes[i]->get_nbr_i(iter, ni)) {
			// get their location
			cv::Point np = this->nodes[ni]->get_loc();
			// get a line iterator from me to them
			cv::LineIterator lit(this->obstacle_mat, me, np);
			double val_sum = 0.0;
			// count every obstacles cell between me and them!
			for (int i = 0; i < lit.count; i++, ++lit) {
				// count along line
				if (this->obstacle_mat.at<cv::Vec3b>(lit.pos()) == cv::Vec3b(0,0,100)) {
					// hit an obstacle
					val_sum++;
				}
			}
			double mean_val = val_sum / double(lit.count);
			this->nodes[i]->set_nbr_obstacle_cost(iter, mean_val);
			iter++;
		}
	}

	// draw PRM connections
	for (int i = 0; i < this->n_nodes; i++) {
		int index = -1;
		int iter = 0;
		while (this->nodes[i]->get_nbr_i(iter, index)) {
			iter++; // tracks which nbr I am on
			double cost = 0.0;
			if (this->nodes[i]->get_nbr_obstacle_cost(iter, cost)) {
				cv::line(this->obstacle_mat, this->nodes[i]->get_loc(), this->nodes[index]->get_loc(), cv::Scalar(0,0,0), 5);
				cv::Vec3b pink(255.0*(1 - cost), 255.0*(1 - cost), 255);
				cv::line(this->obstacle_mat, this->nodes[i]->get_loc(), this->nodes[index]->get_loc(), pink, 2);
			}
		}
	}

	// draw nodes
	for (int i = 0; i < this->n_nodes; i++) {
		cv::circle(this->obstacle_mat, this->nodes[i]->get_loc(), 5, cv::Scalar(255,255,255), -1);
	}

	// label tasks
	for (int i = 0; i < this->n_nodes; i++) {
		double d = -5.0;
		cv::Point2d l = this->nodes[i]->get_loc();
		cv::Point2d tl = cv::Point2d(l.x - d, l.y + d);
		char text[10];
		sprintf_s(text, "%i", i);
		cv::putText(this->obstacle_mat, text, tl, CV_FONT_HERSHEY_SIMPLEX, 1.0, cv::Scalar(255,255,255), 3);
	}

}

void World::clean_up_from_sim() {
	for (int i = 0; i < this->n_nodes; i++) {
		delete this->nodes[i];
	}
	this->nodes.clear();

	for (int i = 0; i < this->n_agents; i++) {
		delete this->agents[i];
	}
	this->agents.clear();
}

void World::run_simulation() {
	while (this->c_time < this->end_time) {
		this->iterate_all();
		this->score_and_record_all();
	}
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

void World::write_params() {

	char temp_char[200];
	sprintf_s(temp_char, 200, "C:/Users/sgtas/OneDrive/Documents/GitHub/Dist_MCTS_Testing/Dist_MCTS_Testing/param_files/param_file_%i.xml", rand_seed);
	this->param_file = temp_char;
	cv::FileStorage fs;
	fs.open(this->param_file, cv::FileStorage::WRITE);

	// randomizing stuff in a controlled way
	fs << "rand_seed" << this->rand_seed;

	// time stuff
	fs << "c_time" << this->c_time;
	fs << "dt" << this->dt;
	fs << "end_time" << this->end_time;

	// map and PRM stuff
	fs << "map_height" << this->map_height;
	fs << "map_widht" << this->map_width;
	fs << "n_nodes" << this->n_nodes;
	fs << "k_map_connections" << this->k_map_connections;
	fs << "k_connection_radius" << this->k_connection_radius;
	fs << "p_connect" << this->p_connect; // probability of connecting

	// task stuff
	fs << "n_task_types" << this->n_task_types; // how many types of tasks are there
	fs << "p_task_initially_active" << this->p_task_initially_active; // how likely is it that a task is initially active
	fs << "p_impossible_task" << this->p_impossible_task; // how likely is it that an agent is created that cannot complete a task
	fs << "p_active_task" << this->p_activate_task; // how likely is it that I will activate a task each second? *dt accounts per iters per second
	fs << "min_task_time" << this->min_task_time; // shortest time to complete a task
	fs << "max_task_time" << this->max_task_time; // longest time to complete a task

   // agent stuff
	fs << "n_agents" << this->n_agents; // how many agents
	fs << "n_agent_types" << this->n_agent_types; // how many types of agents
	fs << "min_travel_vel" << this->min_travel_vel; // slowest travel speed
	fs << "max_travel_vel" << this->max_travel_vel; // fastest travel speed

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

void World::iterate_agents() {
	for (int j = 0; j < this->n_agents; j++) {
		this->agents[j]->act();
	}
}

void World::generate_tasks() {
	// generate with certain probability each time step
	if (true) {
		double r = this->rand_double_in_range(0.0, 1.0);
		if (r < this->p_activate_task) {
			bool flag = true;
			int iter = 0;
			while (flag && iter < 100) {
				iter++;
				int c_task = rand() % this->n_nodes;
				if (!this->nodes[c_task]->is_active()) {
					this->nodes[c_task]->activate(this);
					flag = false;
				}
			}
		}

	}
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

void World::iterate_all() {
	// create new tasks as needed
	this->generate_tasks();

	// go through each agents planning / acting
	this->iterate_agents();

	// iterate time
	this->c_time += this->dt;

	// display the map of the world
	this->display_world(100);
}


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


void World::initialize_agents() {
	std::vector<double> agent_travel_vels;
	std::vector<cv::Scalar> agent_colors;
	for (int i = 0; i < this->n_agent_types; i++) {
		double tv = rand_double_in_range(this->min_travel_vel, this->max_travel_vel);
		agent_travel_vels.push_back(tv);
		double r = rand_double_in_range(0.0, 255.0);
		double b = rand_double_in_range(0.0, 255.0);
		double g = rand_double_in_range(0.0, 255.0);

		cv::Scalar color(b, g, r);
		agent_colors.push_back(color);
	}

	for (int i = 0; i < this->n_agents; i++) {
		int loc = rand() % n_nodes;
		int type = rand() % n_agent_types;
		Agent* a = new Agent(loc, i, type, agent_travel_vels[type], agent_colors[type], this);
		this->agents.push_back(a);
	}
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

World::~World(){
	for (size_t i = 0; i < this->nodes.size(); i++) {
		delete this->nodes[i];
	}
	this->nodes.clear();

	for (size_t i = 0; i < this->agents.size(); i++) {
		delete this->agents[i];
	}
	this->agents.clear();
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