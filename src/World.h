#ifndef WORLD_H_
#define WORLD_H_

#include <ros/ros.h>
#include <iostream>
#include <stdlib.h>
#include <stdio.h>
#include <math.h>

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <vector>

class Map_Node;
class Agent;
class Task;

class World
{
public:
	World(const int &param_file, const bool &display_plot, const std::vector<cv::String> &task_selection_method, const cv::String &img_name);
	// doing everything
	double get_team_probability_at_time_except(const double & time, const int & task, const int & except_agent);
	~World();

	// accessing private vars
	std::vector<Agent*> get_agents() { return this->agents; };
	std::vector<Map_Node*> get_nodes() { return this->nodes; };
	int get_n_nodes() { return this->n_nodes; };
	int get_n_agents() { return this->n_agents; };
	double get_c_time() { return this->c_time; };
	double get_dt() { return this->dt; };
	double get_end_time() { return this->end_time; };
	cv::String get_task_selection_method() { return this->task_selection_method[this->test_iter]; };


	// utility functions
	bool a_star(const int & start, const int & goal, std::vector<int>& path, double & length);
	double dist2d(const double & x1, const double & x2, const double & y1, const double & y2);
	bool dist_between_nodes(const int & n1, const int & n2, double & d);
	void display_world(const int & ms);
	bool get_index(const std::vector<int>& vals, const int & key, int & index);
	bool get_mindex(const std::vector<double> &vals, int &mindex, double &minval);
	bool get_travel_time(const int & s, const int & g, const double & step_dist, double & time);
	bool get_task_completion_time(const int &agent_index, const int &task_index, double &time);
	double rand_double_in_range(const double & min, const double & max);
	bool valid_node(const int & n);
	bool valid_agent(const int a);
	
private:

	int test_iter, n_iterations;
	double c_time, dt, end_time;

	bool show_display;
	double last_plot_time;
	int n_nodes, n_agents;
	int n_agent_types, n_task_types;

	std::vector<std::vector<double> > node_transitions;

	std::vector<Map_Node*> nodes;
	std::vector<Agent*> agents;
	cv::String task_selection_method;
	
	// initialize everything
	int param_file_index;
	char* param_file;
	void load_params();
	void initialize_nodes_and_tasks();
	void initialize_PRM();

	// get cost of each time step
	void score_and_record_all();
	double get_time_step_open_reward();
	double cumulative_open_reward;
};
#endif
