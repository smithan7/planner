#include "Agent_Planning.h"
#include "Agent_Coordinator.h"
#include "Map_Node.h"
#include "Agent.h"
#include "World.h"
#include "Goal.h"

#include <iostream>


Agent_Planning::Agent_Planning(Agent* agent, World* world_in){
	this->owner = agent;
	this->world = world_in;
	this->task_selection_method = agent->get_task_selection_method();

	this->set_goal(this->owner->get_edge().x);
}


Agent_Planning::~Agent_Planning(){}

void Agent_Planning::plan() {
	// randomly select nbr node
	if (this->task_selection_method.compare("random_nbr") == 0) {
		this->select_random_nbr();
	}

	// randomly select a node on the graph
	else if (this->task_selection_method.compare("random_node") == 0) {
		this->select_random_node();
	}

	// randomly select a node on the graph
	else if (this->task_selection_method.compare("random_task") == 0) {
		this->select_random_task();
	}

	// greedily select a task by current reward
	else if (this->task_selection_method.compare("greedy_current_reward") == 0) {
		this->select_greedy_task_by_current_reward();
	}

	// greedily select a task by current reward
	else if (this->task_selection_method.compare("greedy_arrival_reward") == 0) {
		this->select_greedy_task_by_arrival_reward();
	}

	// greedily select a task by current reward
	else if (this->task_selection_method.compare("greedy_completion_reward") == 0) {
		this->select_greedy_task_by_completion_reward();
	}

	// greedily select task by arrival time
	else if (this->task_selection_method.compare("greedy_arrival_time") == 0) {
		this->select_greedy_task_by_arrival_time();
	}

	// greedily select task by completion time
	else if (this->task_selection_method.compare("greedy_completion_time") == 0) {
		this->select_greedy_task_by_completion_time();
	}

	// select task by current value
	else if (this->task_selection_method.compare("value_current") == 0) {
		this->select_task_by_current_value();
	}

	// select task by value at arrival time
	else if (this->task_selection_method.compare("value_arrival") == 0) {
		this->select_task_by_arrival_value();
	}

	// select task by value at completion time
	else if (this->task_selection_method.compare("value_completion") == 0) {
		this->select_task_by_completion_value();
	}

	// // select task by impact reward at time of completion, impact_reward = reward(t_complete) - reward(t^{next closest agent}_complete
	else if (this->task_selection_method.compare("impact_completion_reward") == 0) {
		this->select_task_by_impact_completion_reward();
	}

	// select task by impact reward at time of completion, impact_reward = reward(t_complete) - reward(t^{next closest agent}_complete - (travel_time + work_time)
	else if (this->task_selection_method.compare("impact_completion_value") == 0) {
		this->select_task_by_impact_completion_value();
	}
	// select task by MCTS using reward at time of completion
	else if (this->task_selection_method.compare("mcts_task_by_completion_reward") == 0) {
		this->MCTS_task_by_completion_reward();
	}
	// select task by MCTS using value at time of completion
	else if (this->task_selection_method.compare("mcts_task_by_completion_value") == 0) {
		this->MCTS_task_by_completion_value();
	}
	// select task by MCTS using reward impact at time of completion
	else if (this->task_selection_method.compare("mcts_task_by_completion_reward_impact") == 0) {
		this->MCTS_task_by_completion_reward_impact();
	}
	// select task by MCTS using value impact at time of completion
	else if (this->task_selection_method.compare("mcts_task_by_completion_value_impact") == 0) {
		this->MCTS_task_by_completion_value_impact();
	}
	// method was not found, let user know
	else {
		std::cerr << "Agent_Planning::plan:: goal finding method unspecified" << std::endl;
	}
}

void Agent_Planning::select_task_by_impact_completion_reward() {
	// select task by impact reward at time of completion, impact_reward = reward(t_complete) - reward(t^{next closest agent}_complete
	Agent_Coordinator* my_coord = this->owner->get_coordinator();
	double max_distance = 0.0;
	int max_index = -1;
	double max_arrival_time = 0.0;
	double max_completion_time = 0.0;
	double max_arr_reward = -double(INFINITY);
	double max_comp_reward = -double(INFINITY);
	double max_comp_impact = -double(INFINITY);

	for (int i = 0; i < this->world->get_n_nodes(); i++) {
		if (world->get_nodes()[i]->is_active()) {
			double e_dist = double(INFINITY);
			// get euclidean dist first
			if (world->dist_between_nodes(this->owner->get_edge().x, i, e_dist)) {
				double e_time = e_dist / this->owner->get_travel_step();
				double w_time = this->world->get_nodes()[i]->get_time_to_complete(this->owner, this->world);
				double e_reward = this->world->get_nodes()[i]->get_reward_at_time(world->get_c_time() + e_time + w_time);
				// is my euclidean travel time reward better?
				if (e_reward > max_comp_impact) {
					// I am euclidean reward better, check a star
					std::vector<int> path;
					double a_dist = double(INFINITY);
					if (world->a_star(this->owner->get_edge().x, i, path, a_dist)) {
						// am I a star better?
						double arr_time = this->world->get_c_time() + a_dist / this->owner->get_travel_step();
						double arr_reward = this->world->get_nodes()[i]->get_reward_at_time(arr_time);
						double comp_time = arr_time + w_time;
						double comp_reward = this->world->get_nodes()[i]->get_reward_at_time(comp_time);
						// is it still the best with A*?
						if (comp_reward > max_comp_impact) {
							// is it taken by someone else?
							double prob_taken = 0.0;
							if (my_coord->get_advertised_task_claim_probability(i, comp_time, prob_taken, this->world)) {
								if (prob_taken == 0.0) {
									double impact = my_coord->get_reward_impact(i, this->owner->get_index(), comp_time, this->world);
									if (impact > max_comp_impact) {}
									// if not taken, then accept as possible goal
									max_comp_impact = impact;
									max_comp_reward = comp_reward;
									max_arr_reward = arr_reward;
									max_distance = a_dist;
									max_index = i;
									max_arrival_time = arr_time;
									max_completion_time = comp_time;
								}
								else {
									int a = i + 1;
								}
							}
						}
					}
				}
			}
		}
	}
	if (max_index > -1) {
		std::vector<std::string> args;
		std::vector<double> vals;
		args.push_back("distance");
		vals.push_back(max_distance);

		args.push_back("current_time");
		vals.push_back(world->get_c_time());

		args.push_back("arrival_time");
		vals.push_back(max_arrival_time);

		args.push_back("completion_time");
		vals.push_back(max_completion_time);

		args.push_back("arrival_reward");
		vals.push_back(max_arr_reward);

		args.push_back("completion_reward");
		vals.push_back(max_comp_reward);

		this->set_goal(max_index, args, vals);
	}

}

void Agent_Planning::select_task_by_impact_completion_value() {
	// select task by impact reward at time of completion, impact_reward = reward(t_complete) - reward(t^{next closest agent}_complete - (travel_time + work_time)
	// select task by impact reward at time of completion, impact_reward = reward(t_complete) - reward(t^{next closest agent}_complete
	Agent_Coordinator* my_coord = this->owner->get_coordinator();
	double max_distance = 0.0;
	int max_index = -1;
	double max_arrival_time = 0.0;
	double max_completion_time = 0.0;
	double max_arr_reward = -double(INFINITY);
	double max_comp_reward = -double(INFINITY);
	double max_comp_impact = -double(INFINITY);

	for (int i = 0; i < this->world->get_n_nodes(); i++) {
		if (world->get_nodes()[i]->is_active()) {
			double e_dist = double(INFINITY);
			// get euclidean dist first
			if (world->dist_between_nodes(this->owner->get_edge().x, i, e_dist)) {
				double e_time = e_dist / this->owner->get_travel_step();
				double w_time = this->world->get_nodes()[i]->get_time_to_complete(this->owner, this->world);
				double e_reward = this->world->get_nodes()[i]->get_reward_at_time(world->get_c_time() + e_time + w_time);
				double e_value = e_reward - (e_time + w_time);
				// is my euclidean travel time reward better?
				if (e_value > max_comp_impact) {
					// I am euclidean reward better, check a star
					std::vector<int> path;
					double a_dist = double(INFINITY);
					if (world->a_star(this->owner->get_edge().x, i, path, a_dist)) {
						// am I a star better?
						double arr_time = this->world->get_c_time() + a_dist / this->owner->get_travel_step();
						double arr_reward = this->world->get_nodes()[i]->get_reward_at_time(arr_time);
						double comp_time = arr_time + w_time;
						double comp_reward = this->world->get_nodes()[i]->get_reward_at_time(comp_time);
						double comp_value = comp_reward - (a_dist / this->owner->get_travel_step() + w_time);
						// is it still the best with A*?
						if (comp_value > max_comp_impact) {
							// is it taken by someone else?
							double prob_taken = 0.0;
							if (my_coord->get_advertised_task_claim_probability(i, comp_time, prob_taken, this->world)) {
								if (prob_taken == 0.0) {
									double impact = my_coord->get_reward_impact(i, this->owner->get_index(), comp_time, this->world);
									double impact_value = impact - (a_dist / this->owner->get_travel_step() + w_time);
									if (impact_value > max_comp_impact) {}
									// if not taken, then accept as possible goal

									max_comp_impact = impact_value;
									max_comp_reward = comp_reward;
									max_arr_reward = arr_reward;
									max_distance = a_dist;
									max_index = i;
									max_arrival_time = arr_time;
									max_completion_time = comp_time;
								}
								else {
									int a = i + 1;
								}
							}
						}
					}
				}
			}
		}
	}
	if (max_index > -1) {
		std::vector<std::string> args;
		std::vector<double> vals;
		args.push_back("distance");
		vals.push_back(max_distance);

		args.push_back("current_time");
		vals.push_back(world->get_c_time());

		args.push_back("arrival_time");
		vals.push_back(max_arrival_time);

		args.push_back("completion_time");
		vals.push_back(max_completion_time);

		args.push_back("arrival_reward");
		vals.push_back(max_arr_reward);

		args.push_back("completion_reward");
		vals.push_back(max_comp_reward);

		this->set_goal(max_index, args, vals);
	}


}

void Agent_Planning::MCTS_task_by_completion_reward()
{
}

void Agent_Planning::MCTS_task_by_completion_value()
{
}

void Agent_Planning::MCTS_task_by_completion_reward_impact()
{
}

void Agent_Planning::MCTS_task_by_completion_value_impact()
{
}

void Agent_Planning::set_goal(const int &goal_index) {
	// create a new goal from scratch, reset everything!
	this->owner->get_goal()->set_index(goal_index);
	std::vector<int> path;
	double length = 0.0;
	if (world->a_star(this->owner->get_edge().x, this->owner->get_goal()->get_index(), path, length)) {
		this->owner->get_goal()->set_distance(length);
	}
	else {
		this->owner->get_goal()->set_distance(double(INFINITY));
	
		std::cerr << "Agent_Planning:set_goal: A* could not find path to node" << std::endl;
	}
	this->owner->get_goal()->set_current_time(world->get_c_time());
	double travel_time = this->owner->get_goal()->get_distance() / (this->owner->get_travel_vel()*world->get_dt());
	double arrival_time = this->owner->get_goal()->get_current_time() + travel_time;
	this->owner->get_goal()->set_arrival_time(arrival_time);
	double work_time = this->world->get_nodes()[goal_index]->get_time_to_complete(this->owner, this->world);
	this->owner->get_goal()->set_completion_time(world->get_c_time() + this->owner->get_goal()->get_arrival_time() + work_time);
	
	this->owner->get_goal()->set_current_reward(world->get_nodes()[goal_index]->get_reward_at_time(this->owner->get_goal()->get_current_time()));
	this->owner->get_goal()->set_arrival_reward(world->get_nodes()[goal_index]->get_reward_at_time(this->owner->get_goal()->get_arrival_time()));
	this->owner->get_goal()->set_completion_reward(world->get_nodes()[goal_index]->get_reward_at_time(this->owner->get_goal()->get_completion_time()));
}

void Agent_Planning::set_goal(const int &goal_index, const std::vector<std::string> &args, const std::vector<double> &vals) {
	
	bool need_distance = true;
	bool need_current_reward = true;
	bool need_arrival_reward = true;
	bool need_completion_reward = true;
	
	bool need_current_time = true;
	bool need_arrival_time = true;
	bool need_completion_time = true;

	bool need_completion_value = true;
	

	for (size_t a = 0; a < args.size(); a++) { // check through all args
		if (args[a].compare("distance") == 0) {
			this->owner->get_goal()->set_distance(vals[a]);
			need_distance = false;
		}
		else if (args[a].compare("current_reward") == 0) {
			this->owner->get_goal()->set_current_reward(vals[a]);
			need_current_reward = false;
		}
		else if (args[a].compare("arrival_reward") == 0) {
			this->owner->get_goal()->set_arrival_reward(vals[a]);
			need_arrival_reward = false;
		}
		else if (args[a].compare("completion_reward") == 0) {
			this->owner->get_goal()->set_completion_reward(vals[a]);
			need_completion_reward = false;
		}
		else if (args[a].compare("current_time") == 0) {
			this->owner->get_goal()->set_current_time(vals[a]);
			need_current_time = false;
		}
		else if (args[a].compare("arrival_time") == 0) {
			this->owner->get_goal()->set_arrival_time(vals[a]);
			need_arrival_time = false;
		}
		else if (args[a].compare("completion_time") == 0) {
			this->owner->get_goal()->set_completion_time(vals[a]);
			need_completion_time = false;
		}
		else if (args[a].compare("completion_value") == 0) {
			this->owner->get_goal()->set_completion_value(vals[a]);
			need_completion_value = false;
		}
		else {
			std::cerr << "Agent_Planning::set_goal: bad arg: " << args[a] << std::endl;
		}
	}
	
	this->owner->get_goal()->set_index(goal_index);
	
	if (need_distance) {
		std::vector<int> path;
		double length = 0.0;
		if (world->a_star(this->owner->get_edge().x, this->owner->get_goal()->get_index(), path, length)) {
			this->owner->get_goal()->set_distance(length);
		}
		else {
			this->owner->get_goal()->set_distance(double(INFINITY));
			std::cerr << "Agent_Planning:set_goal: A* could not find path to node" << std::endl;
		}
	}

	if (need_current_time) {
		this->owner->get_goal()->set_current_time(world->get_c_time());
	}
	if (need_current_time) {
		this->owner->get_goal()->set_current_time(world->get_c_time());
	}
	if (need_arrival_time) {
		double travel_time = this->owner->get_goal()->get_distance() / (this->owner->get_travel_vel()*world->get_dt());
		this->owner->get_goal()->set_arrival_time(this->owner->get_goal()->get_current_time() + travel_time);
	}
	if (need_completion_time) {
		double work_time = this->world->get_nodes()[goal_index]->get_time_to_complete(this->owner, this->world);
		this->owner->get_goal()->set_completion_time(world->get_c_time() + this->owner->get_goal()->get_arrival_time()+ work_time);
	}
	if (need_current_reward) {
		this->owner->get_goal()->set_current_reward(world->get_nodes()[goal_index]->get_reward_at_time(this->owner->get_goal()->get_current_time()));
	}
	if (need_arrival_reward) {
		this->owner->get_goal()->set_arrival_reward(world->get_nodes()[goal_index]->get_reward_at_time(this->owner->get_goal()->get_arrival_time()));
	}
	if (need_completion_reward) {
		this->owner->get_goal()->set_completion_reward(world->get_nodes()[goal_index]->get_reward_at_time(this->owner->get_goal()->get_completion_time()));
	}
	if (need_completion_value) {
		this->owner->get_goal()->set_completion_value(world->get_nodes()[goal_index]->get_reward_at_time(this->owner->get_goal()->get_completion_time()) - (this->owner->get_goal()->get_completion_time() - this->world->get_c_time()));
	}
}

void Agent_Planning::select_random_nbr() {
	int goal_index;
	int n_nbrs = this->world->get_nodes()[this->owner->get_edge().x]->get_n_nbrs();
	int c_nbr = rand() % n_nbrs;
	world->get_nodes()[this->owner->get_edge().x]->get_nbr_i(c_nbr, goal_index);

	this->set_goal(goal_index);
}

void Agent_Planning::select_random_node() {
	int n_nodes = this->world->get_n_nodes();
	int goal_index = rand() % n_nodes;

	this->set_goal(goal_index);
}

void Agent_Planning::select_random_task() {

	int c_goal = this->owner->get_goal()->get_index();
	if (world->get_nodes()[c_goal]->is_active()) {
		return;
	}

	std::vector<int> active_tasks;
	for (int i = 0; i < this->world->get_n_nodes(); i++) {
		if (world->get_nodes()[i]->is_active()) {
			active_tasks.push_back(i);
		}
	}

	if (active_tasks.size() > 0) {
		int goal_index = active_tasks[rand() % int(active_tasks.size())];
		this->set_goal(goal_index);
	}
}

void Agent_Planning::select_greedy_task_by_current_reward() {
	double c_time = std::clock() / double(CLOCKS_PER_SEC);

	double max_reward = -double(INFINITY);
	int max_index = -1;

	for (int i = 0; i < this->world->get_n_nodes(); i++) {
		if (world->get_nodes()[i]->is_active()) {
			double t_reward = this->world->get_nodes()[i]->get_reward_at_time(c_time);
			double prob_taken = 0.0;
			if (this->owner->get_coordinator()->get_advertised_task_claim_probability(i, this->world->get_c_time(), prob_taken, this->world)) {
				if (t_reward*(1 - prob_taken) > max_reward) {
					max_reward = t_reward*(1 - prob_taken);
					max_index = i;
				}
			}
		}
	}
	if (max_index > -1) {
		std::vector<std::string> args;
		std::vector<double> vals;
		
		args.push_back("current_reward");
		vals.push_back(max_reward);

		args.push_back("current_time");
		vals.push_back(world->get_c_time());

		this->set_goal(max_index, args, vals);
	}
}


void Agent_Planning::select_greedy_task_by_arrival_reward() {
	double max_distance = 0.0;
	int max_index = -1;
	double max_arrival_time = 0.0;
	double max_arr_reward = -double(INFINITY);

	for (int i = 0; i < this->world->get_n_nodes(); i++) {
		if (world->get_nodes()[i]->is_active()) {
			double e_dist = double(INFINITY);
			// get euclidean dist first
			if (world->dist_between_nodes(this->owner->get_edge().x, i, e_dist)) {
				double e_time = e_dist / this->owner->get_travel_step();
				double w_time = this->world->get_nodes()[i]->get_time_to_complete(this->owner, this->world);
				double e_reward = this->world->get_nodes()[i]->get_reward_at_time(world->get_c_time() + e_time + w_time);
				// is my euclidean travel time reward better?
				if (e_reward > max_arr_reward) {
					// I am euclidean reward better, check a star
					std::vector<int> path;
					double a_dist = double(INFINITY);
					if (world->a_star(this->owner->get_edge().x, i, path, a_dist)) {
						// am I a star better?
						double arr_time = this->world->get_c_time() + a_dist / this->owner->get_travel_step();
						double arr_reward = this->world->get_nodes()[i]->get_reward_at_time(arr_time);
						if (arr_reward > max_arr_reward) {
							// is it taken by someone else?
							double prob_taken = 0.0;
							if (this->owner->get_coordinator()->get_advertised_task_claim_probability(i, arr_time, prob_taken, this->world)) {
								// if not taken, then accept as possible goal
								if (prob_taken == 0.0) {
									max_arr_reward = arr_reward;
									max_distance = a_dist;
									max_index = i;
									max_arrival_time = arr_time;
								}
								else {
									int a = i + 1;
								}
							}
						}
					}
				}
			}
		}
	}
	if (max_index > -1) {
		std::vector<std::string> args;
		std::vector<double> vals;
		args.push_back("distance");
		vals.push_back(max_distance);

		args.push_back("current_time");
		vals.push_back(world->get_c_time());

		args.push_back("arrival_time");
		vals.push_back(max_arrival_time);

		args.push_back("arrival_reward");
		vals.push_back(max_arr_reward);

		this->set_goal(max_index, args, vals);
	}
}

void Agent_Planning::select_greedy_task_by_completion_reward() {
	double max_distance = 0.0;
	int max_index = -1;
	double max_arrival_time = 0.0;
	double max_completion_time = 0.0;
	double max_arr_reward = -double(INFINITY);
	double max_comp_reward = -double(INFINITY);

	for (int i = 0; i < this->world->get_n_nodes(); i++) {
		if (world->get_nodes()[i]->is_active()) {
			double e_dist = double(INFINITY);
			// get euclidean dist first
			if (world->dist_between_nodes(this->owner->get_edge().x, i, e_dist)) {
				double e_time = e_dist / this->owner->get_travel_step();
				double w_time = this->world->get_nodes()[i]->get_time_to_complete(this->owner, this->world);
				double e_reward = this->world->get_nodes()[i]->get_reward_at_time(world->get_c_time() + e_time + w_time);
				// is my euclidean travel time reward better?
				if (e_reward > max_comp_reward) {
					// I am euclidean reward better, check a star
					std::vector<int> path;
					double a_dist = double(INFINITY);
					if (world->a_star(this->owner->get_edge().x, i, path, a_dist)) {
						// am I a star better?
						double arr_time = this->world->get_c_time() + a_dist / this->owner->get_travel_step();
						double arr_reward = this->world->get_nodes()[i]->get_reward_at_time(arr_time);
						double comp_time = arr_time + w_time;
						double comp_reward = this->world->get_nodes()[i]->get_reward_at_time(comp_time);
						if (comp_reward > max_comp_reward) {
							// is it taken by someone else?
							double prob_taken = 0.0;
							if (this->owner->get_coordinator()->get_advertised_task_claim_probability(i, comp_time, prob_taken, this->world)) {
								// if not taken, then accept as possible goal
								if (prob_taken == 0.0) {
									max_comp_reward = comp_reward;
									max_arr_reward = arr_reward;
									max_distance = a_dist;
									max_index = i;
									max_arrival_time = arr_time;
									max_completion_time = comp_time;
								}
								else {
									int a = i + 1;
								}
							}
						}
					}
				}
			}
		}
	}
	if (max_index > -1) {
		std::vector<std::string> args;
		std::vector<double> vals;
		args.push_back("distance");
		vals.push_back(max_distance);

		args.push_back("current_time");
		vals.push_back(world->get_c_time());

		args.push_back("arrival_time");
		vals.push_back(max_arrival_time);

		args.push_back("completion_time");
		vals.push_back(max_completion_time);

		args.push_back("arrival_reward");
		vals.push_back(max_arr_reward);

		args.push_back("completion_reward");
		vals.push_back(max_comp_reward);

		this->set_goal(max_index, args, vals);
	}
}

void Agent_Planning::select_task_by_current_value() {
	// probably don't do these
}

void Agent_Planning::select_task_by_arrival_value() {
	// probably don't do these
}

void Agent_Planning::select_task_by_completion_value() {
	// what is the value I will recieve for completing this task -> value = c_0*reward(t_complete) - c_1*(travel_time + work_time)
	double max_distance = 0.0;
	int max_index = -1;
	double max_arrival_time = 0.0;
	double max_completion_time = 0.0;
	double max_arr_reward = 0.0;
	double max_comp_reward = 0.0;
	double max_comp_value = -double(INFINITY);

	for (int i = 0; i < this->world->get_n_nodes(); i++) {
		if (world->get_nodes()[i]->is_active()) {
			double e_dist = double(INFINITY);
			// get euclidean dist first
			if (world->dist_between_nodes(this->owner->get_edge().x, i, e_dist)) {
				double e_time = e_dist / this->owner->get_travel_step();
				double w_time = this->world->get_nodes()[i]->get_time_to_complete(this->owner, this->world);
				double e_reward = this->world->get_nodes()[i]->get_reward_at_time(world->get_c_time() + e_time + w_time);
				double e_value = e_reward - (e_time + w_time);
				// is my euclidean travel time reward better?
				if (e_value > max_comp_value) {
					// I am euclidean reward better, check a star
					std::vector<int> path;
					double a_dist = double(INFINITY);
					if (world->a_star(this->owner->get_edge().x, i, path, a_dist)) {
						// am I a star better?
						double a_time = a_dist / this->owner->get_travel_step();
						double arr_time = this->world->get_c_time() + a_time;
						double arr_reward = this->world->get_nodes()[i]->get_reward_at_time(arr_time);
						double comp_time = arr_time + w_time;
						double comp_reward = this->world->get_nodes()[i]->get_reward_at_time(comp_time);
						double comp_value = comp_reward - (a_time + w_time);
						if (comp_value > max_comp_value) {
							// is it taken by someone else?
							double prob_taken = 0.0;
							if (this->owner->get_coordinator()->get_advertised_task_claim_probability(i, comp_time, prob_taken, this->world)) {
								// if not taken, then accept as possible goal
								if (prob_taken == 0.0) {
									max_comp_reward = comp_reward;
									max_arr_reward = arr_reward;
									max_distance = a_dist;
									max_index = i;
									max_arrival_time = arr_time;
									max_completion_time = comp_time;
									max_comp_value = comp_value;
								}
								else {
									int a = i + 1;
								}
							}
						}
					}
				}
			}
		}
	}
	if (max_index > -1) {
		std::vector<std::string> args;
		std::vector<double> vals;
		args.push_back("distance");
		vals.push_back(max_distance);

		args.push_back("current_time");
		vals.push_back(world->get_c_time());

		args.push_back("arrival_time");
		vals.push_back(max_arrival_time);

		args.push_back("completion_time");
		vals.push_back(max_completion_time);

		args.push_back("arrival_reward");
		vals.push_back(max_arr_reward);

		args.push_back("completion_reward");
		vals.push_back(max_comp_reward);

		args.push_back("completion_value");
		vals.push_back(max_comp_value);

		this->set_goal(max_index, args, vals);
	}
}

void Agent_Planning::select_task_by_reward_impact() {
	// choose the task with the highest reward impact. Where impact is the difference in
	// the team reward if I get it now instead of not getting it and waiting on someone else, the
	// difference between now and the next claim or potentially all following claims
}

void Agent_Planning::select_task_by_value_impact() {
	// choose the task with the highest value impact. Where impact is the difference in
	// the team reward if I get it now instead of not getting it and waiting on someone else, the
	// difference between now and the next claim or potentially all following claims

}
void Agent_Planning::select_greedy_task_by_completion_time() {
	double min_distance = double(INFINITY);
	int min_index = -1;
	double min_arrival_time = double(INFINITY);
	double min_completion_time = double(INFINITY);


	for (int i = 0; i < this->world->get_n_nodes(); i++) {
		if (world->get_nodes()[i]->is_active()) {
			double e_dist = double(INFINITY);
			// get euclidean dist first
			if (world->dist_between_nodes(this->owner->get_edge().x, i, e_dist)) {
				// am I euclidean closer?
				if (e_dist < min_distance) {
					// I am euclidean dist closer, check a star
					std::vector<int> path;
					double a_dist = double(INFINITY);
					if (world->a_star(this->owner->get_edge().x, i, path, a_dist)) {
						// am I a star closer?
						if (a_dist < min_distance) {

							double a_time = this->world->get_c_time() + a_dist / (this->get_owner()->get_travel_vel() * this->world->get_dt());
							double c_time = a_time + this->world->get_nodes()[i]->get_time_to_complete(this->owner, this->world);
							// is it taken by someone else?
							double prob_taken = 0.0;
							if (this->owner->get_coordinator()->get_advertised_task_claim_probability(i, c_time, prob_taken, this->world)) {
								// if not taken, then accept as possible goal
								if (prob_taken == 0.0) {
									min_distance = a_dist;
									min_index = i;
									min_arrival_time = a_time;
									min_completion_time = c_time;
								}
								else {
									int a = i + 1;
								}
							}
						}
					}
				}
			}
		}
	}
	if (min_index > -1) {
		std::vector<std::string> args;
		std::vector<double> vals;
		args.push_back("distance");
		vals.push_back(min_distance);

		args.push_back("current_time");
		vals.push_back(world->get_c_time());

		args.push_back("arrival_time");
		vals.push_back(min_arrival_time);

		args.push_back("completion_time");		
		vals.push_back(min_completion_time);

		this->set_goal(min_index, args, vals);
	}
}

void Agent_Planning::select_greedy_task_by_arrival_time() {
	double min_distance = double(INFINITY);
	int min_index = -1;
	double min_arrival_time = double(INFINITY);

	for (int i = 0; i < this->world->get_n_nodes(); i++) {
		if (world->get_nodes()[i]->is_active()) {
			double e_dist = double(INFINITY);
			// get euclidean dist first
			if (world->dist_between_nodes(this->owner->get_edge().x, i, e_dist)) {
				// am I euclidean closer?
				if (e_dist < min_distance) {
					// I am euclidean dist closer, check a star
					std::vector<int> path;
					double a_dist = double(INFINITY);
					if (world->a_star(this->owner->get_edge().x, i, path, a_dist)) {
						// am I a star closer?
						if (a_dist < min_distance) {

							double a_time = this->world->get_c_time() + a_dist / (this->get_owner()->get_travel_vel() * this->world->get_dt());

							// is it taken by someone else?
							double prob_taken = 0.0;
							if (this->owner->get_coordinator()->get_advertised_task_claim_probability(i, a_time, prob_taken, this->world)) {
								// if not taken, then accept as possible goal
								if (prob_taken == 0.0) {
									min_distance = a_dist;
									min_index = i;
									min_arrival_time = a_time;
								}
								else {
									int a = i + 1;
								}
							}
						}
					}
				}
			}
		}
	}
	if (min_index > -1) {
		std::vector<std::string> args;
		std::vector<double> vals;
		args.push_back("distance");
		args.push_back("current_time");
		args.push_back("arrival_time");
		vals.push_back(min_distance);
		vals.push_back(min_arrival_time);
		vals.push_back(world->get_c_time());
		this->set_goal(min_index, args, vals);
	}
}

