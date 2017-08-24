#ifndef MAP_NODE_H_
#define MAP_NODE_H_

#include <ros/ros.h>
#include <iostream>
#include <stdlib.h>
#include <stdio.h>
#include <math.h>

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <vector>

class Agent;
class World;

class Map_Node{
public:
	Map_Node(const double &x, const double &y, const int &index, const int &task_type, const std::vector<double> &work, const cv::Scalar &color, World* world);
	bool is_active() { return this->active; }; // is the task active
	double get_reward_at_time(const double &time); // get the reward at time
	double get_acted_upon(Agent* agent); // act for one time step, are there agents working on me, should I deactivate 
	cv::Scalar get_color() { return this->color; };
	void activate(World* world); // activate the function
	void deactivate(); // deactivate the function
	~Map_Node();

	double get_x() { return this->x; };
	double get_y() { return this->y; };
	cv::Point2d get_loc() { return this->loc; }; // get  node location
	cv::Point2d get_local_loc() {return this->local_loc; };
	bool get_nbr_i(const int &index, int &nbr_index);
	bool get_nbr_free_distance(const int &index, double &nbr_dist);
	bool get_nbr_obstacle_distance(const int& index, double &nbr_cost);
	int get_n_nbrs() { return this->n_nbrs; };
	int get_task_type() { return this->task_type; };
	void set_task_type(const int &t) {this->task_type = t; };

	double get_time_to_complete(Agent* agent, World* world);
	int get_index() { return this->index; };
	void add_nbr(const int &nbr, const double &free_dist, const double &obs_dist);
	void set_work(const std::vector<double> &work);

private:
	double x;
	double y;
	cv::Point2d loc, goal;
	cv::Point2d local_loc;
	cv::Point2d map_size_meters;

	int index;
	cv::Scalar color;
	int n_nbrs;
	std::vector<int> nbrs;
	std::vector<double> nbr_free_distances;
	std::vector<double> nbr_obstacle_distances;

	// number of types of tasks
	int n_task_types;
	// number of types of reward windows
	int n_reward_window_types;
	// what type of task am I?
	int task_type;
	// time task started, and when will it end
	double start_time, end_time;
	// am I active?
	bool active;
	// how long does it take for each agent to complete me
	std::vector<double> agent_work;

	// what type of reward window type do I have?
	int reward_window_type;
	// how much is the reward at t_0
	double initial_reward;
	// linear decay window reward function info
	double reward_slope, reward_offset;
	// exponential decay window reward function info
	double reward_decay, max_reward_decay, min_reward_decay;
	// range of reward
	double max_reward, min_reward;
	// range of time for tasks to be available and set rewards
	double min_time, max_time;
	// how much work does it take to complete this type of task
	double remaining_work;


	////////////////////////////////////////////////////
	/////////////////////// functions //////////////////
	////////////////////////////////////////////////////

	// update task every time step
	void update_task(World* world);
};
#endif
