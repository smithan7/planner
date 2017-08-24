#ifndef WORLD_H_
#define WORLD_H_

#include <ros/ros.h>
#include <iostream>
#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include <vector>
#include <string>

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>


class Map_Node;
class Agent;
class Task;
class Agent_Coordinator;

class World{
public:
	World();	
	bool init(const int &test_environment_number, const int &test_scenario_number);
	double get_team_probability_at_time_except(const double & time, const int & task, const int & except_agent);
	void add_stop_to_agents_path(const int &agent_index, const int &task_index, const double &probability, const double &time);
	~World();

	// accessing private vars
	//std::vector<Agent*> get_agents() { return this->agents; };
	std::vector<Agent_Coordinator*> get_agent_coords() { return this->agent_coords; };
	std::vector<Map_Node*> get_nodes() { return this->nodes; };
	int get_n_nodes() { return this->n_nodes; };
	int get_n_agents() { return this->n_agents; };
	double get_c_time() { return this->c_time; };
	double get_dt() { return this->dt; };
	double get_end_time() { return this->end_time; };
	cv::Point2d get_NW_Corner() {return this->NW_Corner; };
	std::string get_task_selection_method() { return this->task_selection_method; };


	// utility functions
	bool a_star(const int & start, const int & goal, std::vector<int>& path, double & length);
	double dist2d(const double & x1, const double & x2, const double & y1, const double & y2);
	bool dist_between_nodes(const int & n1, const int & n2, double & d);
	void display_world(Agent* agent);
	void show_prm_plot();
	bool get_index(const std::vector<int>& vals, const int & key, int & index);
	bool get_mindex(const std::vector<double> &vals, int &mindex, double &minval);
	bool get_travel_time(const int & s, const int & g, const double & step_dist, double & time);
	bool get_task_completion_time(const int &agent_index, const int &task_index, double &time);
	double rand_double_in_range(const double & min, const double & max);
	bool valid_node(const int & n);
	bool valid_agent(const int a);

	// brought over from ROS
	bool get_prm_location(const cv::Point2d&loc, cv::Point &edge, double &edge_progress);
	cv::Point2d global_to_local(const cv::Point2d &loc);
	double to_radians(const double &deg);
	double get_global_heading(const cv::Point2d &g1, const cv::Point2d &g2);
	double get_global_distance(const cv::Point2d &g1, const cv::Point2d &g2);
	double get_local_euclidian_distance(const cv::Point2d &a, const cv::Point2d &b);
	double get_local_heading(const cv::Point2d &l1, const cv::Point2d &l2);

private:

	cv::Point2d NW_Corner, SE_Corner;
	cv::Point2d map_size_meters;
	std::vector<std::vector<double> > agent_work;

	double c_time, dt, end_time;

	bool show_display;
	double last_plot_time;
	int n_nodes, n_edges, n_agents, n_human_tasks, n_robot_tasks;
	int n_agent_types, n_task_types;
	std::vector<int> agent_types;

	std::vector<Map_Node*> nodes;
	//std::vector<Agent*> agents;
	std::vector<Agent_Coordinator*> agent_coords;
	std::string task_selection_method;
	


	// initialize everything
	//void initialize_PRM();
	bool on_map(const cv::Point2d &loc);

	// get cost of each time step
	void score_and_record_all();
	double get_time_step_open_reward();
	double cumulative_open_reward;
	bool load_robot_tasks(const int &test_environment_number);
	bool load_human_tasks(const int &test_environment_number);
	bool load_PRM_edges(const int &test_environment_number);
	bool load_PRM_vertices(const int &test_environment_number);
	bool load_test_scenario(const int &test_scenario_number);
};
#endif
