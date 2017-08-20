#include "Agent.h"
#include "World.h"
#include "Map_Node.h"
#include "Agent_Coordinator.h"
#include "Agent_Planning.h"
#include "Goal.h"

#include <iostream>
#include <ctime>

Agent::Agent(int loc, int index, int type, double travel_vel, cv::Scalar color, World* world_in){
	this->world = world_in;
	////////////////////// How do I select my goal ////////////////////////////////////////////
	{
		//////////////////////////////// greedy methods
		{
			//this->task_selection_method = "greedy_arrival_time"; // choose the closest active task
			//this->task_selection_method = "greedy_completion_time"; // choose the task I can finish first
			
			//this->task_selection_method = "greedy_current_reward"; // choose the task with the largest reward currently
			//this->task_selection_method = "greedy_arrival_reward"; // choose the task with the largest reward at the time I will arrive
			//this->task_selection_method = "greedy_completion_reward"; // choose the task with the largest reward at the time I will complete
		}
		//////////////////////////////// value methods
		{
			//this->task_selection_method = "value_current"; // choose task by value now, value = reward(t_current) - (travel_time + work_time)
			//this->task_selection_method = "value_arrival"; // choose task by value at time of arrival, value = reward(t_arrival) - (travel_time + work_time)
			//this->task_selection_method = "value_completion"; // choose task by value at time of completion, value = reward(t_complete) - (travel_time + work_time)
		}
		//////////////////////////////// impact methods
		{
			//this->task_selection_method = "impact_completion_reward"; // choose task by impact reward at time of completion, impact_reward = reward(t_complete) - reward(t^{next closest agent}_complete)
			//this->task_selection_method = "impact_completion_value"; // choose task by impact at value time of completion, impact_value = reward(t_complete) - reward(t^{next closest agent}_complete) - (travel_time + work_time)
		}


		//////////////////////////////// random methods
		{
			//this->task_selection_method = "random_nbr"; // choose a random nbr
			//this->task_selection_method = "random_node"; // choose a random node on the map
			//this->task_selection_method = "random_task"; // choose a random active task
		}
		///////////////////////////////// MCTS methods
		{
			//this->task_selection_method = "MCTS_value"; // use MCTS to plan a sequence of values 
		}
		this->task_selection_method =  this->world->get_task_selection_method();
	}
	//////////////////////// When do I claim my tasks /////////////////////////////////////////
	{
		this->task_claim_time = "completion_time"; // when I will complete the task
		//this->task_claim_time = "arrival_time"; // when I will arrive at the task
		//this->task_claim_time = "immediate"; // I claim it from right now
		//this->task_claim_time = "none"; // I do NOT claim the task at all
	}
	/////////////////////// How do I claim my tasks ///////////////////////////////////////////
	{
		this->task_claim_method = "greedy"; // whatever task is best gets P(t) = 1.0, else P(t) = 0.0;
		//this->task_claim_method = "sample"; // all tasks get P(t) = (V(t)-V_min)/(V_max-V_min);
	}


	this->edge.x = loc;
	this->edge.y = loc;
	this->edge_progress = 1.0;
	this->index = index;

	this->type = type;
	this->travel_vel = travel_vel;
	this->travel_step = travel_vel *  this->world->get_dt();
	this->color = color;
	this->n_tasks =  this->world->get_n_nodes();

	this->goal_node = new Goal();
	this->planner = new Agent_Planning(this, world);
	this->coordinator = new Agent_Coordinator(this, n_tasks);
}

bool Agent::at_node(int node) {
	if (this->edge_progress >= 1.0 && this->edge.y == node) {
		return true;
	}
	else if (this->edge_progress == 0.0 && this->edge.x == node) {
		return true;
	}
	else {
		return false;
	}
}

cv::Point2d Agent::get_loc2d() {
	cv::Point2d p(0.0, 0.0);
	p.x = ( this->world->get_nodes()[this->edge.y]->get_x() -  this->world->get_nodes()[this->edge.x]->get_x())*this->edge_progress +  this->world->get_nodes()[this->edge.x]->get_x();
	p.y = ( this->world->get_nodes()[this->edge.y]->get_y() -  this->world->get_nodes()[this->edge.x]->get_y())*this->edge_progress +  this->world->get_nodes()[this->edge.x]->get_y();
	return p;
}

void Agent::move_along_edge() {
	// how long is the current edge?
	double dist;
	if ( this->world->dist_between_nodes(this->edge.x, this->edge.y, dist)) {

		if (dist == 0) {
			this->edge_progress = 1.0;
			this->edge.x = this->edge.y;
		}
		else {
			// move along edge 1 time step dist = this->travel vel * time, time = 1, step dist = travel vel
			this->edge_progress = this->edge_progress + this->travel_step / dist;
			if (this->edge_progress >= 1.0) {
				this->travel_done += (this->edge_progress - 1.0)*dist;
				this->edge_progress = 1.0;
				this->edge.x = this->edge.y;
			}
			else {
				this->travel_done += this->travel_step;
			}
		}
	}
	else {
		std::cerr << "Agent::move_along_edge::bad request" << std::endl;
	}
}

void Agent::work_on_task() {
	this->work_done +=  this->world->get_nodes()[this->goal_node->get_index()]->get_acted_upon(this);
}

void Agent::select_next_edge() {

	std::vector<int> path;
	double length = 0.0;

	this->edge_progress = 0.0;
	if ( this->world->a_star(this->edge.x, this->goal_node->get_index(), path, length)) {
		if (path.size() >= 2) {
			this->edge.y = path.end()[-2];
		}
	}
	else {
		std::cerr << "Agent::select_next_edge::a_star failed to find path" << std::endl;
	}
}

bool Agent::at_node() { // am I at a node, by edge progress?
	if (this->edge_progress == 0.0 || this->edge_progress >= 1.0) {
		return true;
	}
	else {
		return false;
	}
}

bool Agent::at_goal() { // am I at my goal node?
	if (this->edge_progress >= 1.0 && this->edge.y == this->goal_node->get_index()) {
		return true;
	}
	else if (this->edge_progress == 0.0 && this->edge.x == this->goal_node->get_index()) {
		return true;
	}
	else{
		return false;
	}
}

void Agent::act() {
	// am  I at a node?
	if (this->at_node()) {
		// I am at a node, am I at my goal and is it active still?
		if (this->at_goal() &&  this->world->get_nodes()[this->goal_node->get_index()]->is_active()) {
			this->work_done +=  this->world->get_nodes()[this->goal_node->get_index()]->get_acted_upon(this); // work on my goal
		}
		else {
			this->planner->plan(); // I am not at my goal, select new goal
			this->coordinator->advertise_task_claim(this->world); // select the next edge on the path to goal 
			this->select_next_edge(); // plan the next edge
			this->move_along_edge(); // on the right edge, move along edge
		}
	}
	else { // not at a node
		this->move_along_edge(); // move along edge
	}
}

Agent::~Agent(){
	delete this->planner;
	delete this->coordinator;
	delete this->goal_node;
}