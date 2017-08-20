#pragma once

#include <vector>
#include <opencv2\core.hpp>

class World;
class Agent_Planning;
class Agent_Coordinator;
class Goal;

class Agent
{
public:
	// functions
	Agent(int loc, int index, int type, double travel_vel, cv::Scalar color, World* world_in);
	bool at_node(int node);
	void act();
	~Agent();

	// access private variables
	int get_index() { return this->index; };
	cv::Point2d get_loc2d();
	int get_loc() { return this->edge.x; };

	Goal* get_goal() { return this->goal_node; };
	Agent_Coordinator* get_coordinator() { return this->coordinator; };
	Agent_Planning* get_planner() { return this->planner; };
	double get_arrival_time() { return this->arrival_time; };
	int get_type() { return this->type; };
	double get_travel_vel() { return this->travel_vel; };
	double get_travel_step() { return this->travel_step; };

	cv::Scalar get_color() { return this->color; };
	cv::Point2i get_edge() { return this->edge; };
	std::string get_task_selection_method() { return this->task_selection_method; };
	std::string get_task_claim_method() { return this->task_claim_method; };
	std::string get_task_claim_time() { return this->task_claim_time; };

	double get_work_done() { return this->work_done; };
	double get_travel_done() { return this->travel_done; };

private:

	// planning and coordinator
	Goal* goal_node;
	Agent_Planning* planner;
	Agent_Coordinator* coordinator;
	World* world;

	std::string task_selection_method; // how do I select tasks
	std::string task_claim_method; // how / when do I claim tasks
	std::string task_claim_time; // when do I claim my task


	double arrival_time; // for travel and arrival plan
	double expected_value;

	double work_done; // accumulated reward
	double travel_done; // distance I have travelled

	cv::Point2i edge; // x:=where am I? y:=where am I going?
	double edge_progress; // how far along the edge am I?
	
	int index; // who am I in the world?
	int type; // what type of agent am I?
	double travel_vel; // how fast can I move?
	double travel_step; // how far do I move in one time step
	cv::Scalar color; // what color am I plotted?
	int n_tasks; // how many tasks are there

	// functions
	bool at_node(); // am I at a node, by edge_progress
	bool at_goal(); // am I at my goal node?
	
	// working and planning
	void work_on_task(); // work on the task I am at
	
	// moving and path planning 
	void move_along_edge(); // move along the current edge
	void select_next_edge(); // have a goal, select next edge to get to goal
};

