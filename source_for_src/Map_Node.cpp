#include "Map_Node.h"
#include "Agent.h"
#include "World.h"

#include <random>

Map_Node::Map_Node(double x, double y, int index, double p_active, int task_type, std::vector<double> work, cv::Scalar color, World* world){
	// where am I?
	this->x = x;
	this->y = y;
	this->loc = cv::Point2d(x, y);
	this->index = index;
	this->color = color;

	// set up the tasks
	this->n_reward_window_types = 4;
	this->task_type = task_type;
	this->agent_work = work;

	// range of rewards for tasks
	this->max_reward = 10.0;
	this->min_reward = 1.0;

	// range of time available for rewards and used to set rewards
	this->min_time = 2.0;
	this->max_time = 10.0;

	// how much work does it take to complete this task
	this->min_work = 1.0;
	this->max_work = 100.0;

	// start setting it up
	this->active = false;
	if (p_active > world->rand_double_in_range(0.0, 1.0)) {
		// should I be active upon spawning?
		this->activate(world);
	}
}

void Map_Node::deactivate() {
	this->active = false;
	this->initial_reward = 0.0;
}


void Map_Node::update_task(World* world) {
	if (this->active) {
		if (world->get_c_time() > this->end_time || world->get_c_time() < this->start_time) {
			this->deactivate();
		}
	}
}

double Map_Node::get_acted_upon(Agent* agent) {
	// this agent says it is working on me, check

	// are they actually at my location
	if (agent->at_node(this->index)) {
		// they are actually at my location, what type are they?
		int agent_type = agent->get_type();
		double agent_work = this->agent_work[agent_type];
		this->remaining_work -= agent_work;
		if (this->remaining_work <= 0.0) {
			this->deactivate();
			return agent_work + this->remaining_work;
		}
		else {
			return agent_work;
		}
	}
	else {
		return 0.0;
	}
}

double Map_Node::get_time_to_complete(Agent* agent, World* world) {
	int agent_type = agent->get_type();
	double agent_work = this->agent_work[agent_type];
	double time_to_complete = world->get_dt() * (this->remaining_work / agent_work);
	return time_to_complete;
}

void Map_Node::activate(World* world) {
	this->active = true;
	this->start_time = world->get_c_time();

	// what type of reward window will I have?
	this->reward_window_type = rand() % this->n_reward_window_types;
	// what is my initial reward?
	this->initial_reward = (this->max_reward - this->min_reward) * double(rand()) / double(RAND_MAX) + this->min_reward;
	// how much work to complete me?
	this->remaining_work = (this->max_work - this->min_work) * double(rand()) / double(RAND_MAX) + this->min_work;

	if (this->reward_window_type == 0) {
		// constant value, do nothing
		this->end_time = double(INFINITY);
	}
	else if (this->reward_window_type == 1) {
		// linear decline
		double active_interval = (this->max_time - this->min_time) * double(rand()) / double(RAND_MAX) + this->min_time;
		this->end_time = this->start_time + active_interval;
		this->reward_slope = (this->min_reward - this->initial_reward) / active_interval;
		this->reward_offset = this->initial_reward - this->reward_slope*this->start_time;
	}
	else if (this->reward_window_type == 2) {
		// constant window
		double active_interval = (this->max_time - this->min_time) * double(rand()) / double(RAND_MAX) + this->min_time;
		this->end_time = this->start_time + active_interval;
	}
	else if (this->reward_window_type == 3) {
		// exponential decline
		double active_interval = (this->max_time - this->min_time) * double(rand()) / double(RAND_MAX) + this->min_time;
		this->end_time = this->start_time + active_interval;
		this->reward_decay = log(this->min_reward / this->initial_reward) / active_interval;
	}
}

double Map_Node::get_reward_at_time(double time) {
	if (!this->active || time > this->end_time || time < this->start_time) {
		return 0.0;
	}

	if (this->reward_window_type == 0) {
		// constant value
		return this->initial_reward;
	}
	else if (this->reward_window_type == 1) {
		// linear decline
		double r_out = this->reward_slope*time + reward_offset;
		return r_out;
	}
	else if (this->reward_window_type == 2) {
		// constant window, time constraints ensure that I am good
		return this->initial_reward;
	}
	else if (this->reward_window_type == 3) {
		// exponential decline
		double dt = time - this->start_time;
		double r_out = this->initial_reward * exp(this->reward_decay * dt);
		return r_out;
	}
	else {
		return 0.0;
	}
}

Map_Node::~Map_Node(){}

void Map_Node::set_nbr_obstacle_cost(const int &ni, const double &obs_cost) {
	if (ni < this->n_nbrs) {
		this->nbr_obstacle_costs[ni] = obs_cost;
	}
}

void Map_Node::add_nbr(int nbr, double dist) {
	this->nbrs.push_back(nbr);
	this->nbr_distances.push_back(dist);
	this->n_nbrs++;
	this->nbr_obstacle_costs.push_back(0.0);
}

bool Map_Node::get_nbr_obstacle_cost(const int &index, double &nbr_cost) {
	if (index < this->n_nbrs) {
		nbr_cost = this->nbr_obstacle_costs[index];
		return true;
	}
	return false;
}

bool Map_Node::get_nbr_distance(const int &index, double &nbr_dist) {
	if (index < this->n_nbrs) {
		nbr_dist = this->nbr_distances[index];
		return true;
	}
	return false;
}

bool Map_Node::get_nbr_i(const int &index, int &nbr_index) {
	if (index < this->n_nbrs) {
		nbr_index = this->nbrs[index];
		return true;
	}
	return false;
}