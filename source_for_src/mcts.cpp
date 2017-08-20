#include "mcts.h"
#include "World.h"
#include "Agent.h"
#include "Map_Node.h"
#include "Agent_Coordinator.h"

#include <iostream>


MCTS::MCTS(World* world, Map_Node* task_in, Agent* agent_in, MCTS* parent, const int &my_kid_index){
	if (parent) {
		this->depth = parent->get_depth() + 1;
		this->probability = 0.0;
		this->kid_index = my_kid_index;
	}
	else {
		this->depth = 0;
		this->probability = 1.0;
		this->kid_index = -1;
	}
	this->agent = agent_in;
	this->task = task_in;
	this->task_index = task_in->get_index();

	// euclidean distance checks
	this->euclid_distance = -1.0; // distance by euclidean
	this->euclid_time = -1.0; // arrival time if e_dist
	this->euclid_reward = -1.0; // reward at e_time * (1-p_taken at e_time)
	this->euclid_value = -1.0; // value from e_reward

	// a star distance checks
	this->distance = -1.0; // how far by A*
	this->travel_time = -1.0; // when will I arrive by A*
	this->reward = -1.0; // reward * (1-p_taken)
	this->expected_value = -1.0; // expected reward of this child

	// task stuff
	this->probability_task_available = 1.0;
	this->work_time = -1.0;
	this->completion_time = -1.0;

	// MCTS stuff
	this->branch_value = -1.0; // my expected value + all my best kids expected value
	this->explore_value = -1.0; // haven't searched in a while value
	this->search_value = -1.0; // = explore_value + exploit_value
	this->number_pulls = 0; // how many times have I been pulled

	// sampling stuff
	this->max_kid_index = -1; // index of gc
	this->min_kid_index = -1;
	this->max_kid_branch_value = -1.0; // their value
	this->min_kid_branch_value = -1.0;
	this->sum_kid_branch_value = 0.0;

	// few useful constants
	this->wait_time = 5.0f; // first child is always option to stay put
	this->max_kid_distance = double(INFINITY); // how far can I travel to a child
	this->reward_weighting = 1.0; // how important is the reward in the value function
	this->distance_weighting = 1.0; // how important is the travel cost in the value function
	this->beta = 1.41; // ucb = 1.41, d-ucb = 1.41, sw-ucb = 0.705
	this->epsilon = 0.5; // ucb = 0.5, d-ucb = 0.05, sw-ucb = 0.05
	this->gamma = 1.0; // ucb = n/a~1.0, d-ucb = 0.9, sw-ucb = 0.9 
	this->sampling_probability_threshold = 0.1; // how low of probability will I continue to sample and report
}

void MCTS::make_kids(const std::vector<bool> &task_status) {
	if (this->kids.size() > 0) {
		for (size_t i = 0; i < this->kids.size(); i++) {
			delete this->kids[i];
		}
		this->kids.clear();
	}

	// add a kid who stands still
	//MCTS* null_kid = new MCTS(world, this->task, this->agent, this, 0);
	//this->kids.push_back(null_kid);

	// potentially add a kid for each active task
	for (size_t i = 0; i < task_status.size(); i++) {
		// if task i needs to be completed
		if (task_status[i]) {
			MCTS* kiddo = new MCTS(world, world->get_nodes()[i], this->agent, this, int(this->kids.size()));
			if (kiddo->kid_pruning_heuristic(task_status)) {
				this->kids.push_back(kiddo);
			}
		}
	}

	// find min / max / sum of kids!
	this->find_min_branch_value_kid();
	this->find_max_branch_value_kid();
	this->find_sum_kid_branch_value();
}

void MCTS::find_min_branch_value_kid() {
	// find the kid with minimum expected value
	this->min_kid_branch_value = double(INFINITY);
	
	for (size_t i = 0; i < this->kids.size(); i++) {
		if (this->kids[i]->get_branch_value() < this->min_kid_branch_value) {
			this->min_kid_branch_value = this->kids[i]->get_euclid_value();
			this->min_kid_index = int(i);
		}
	}
}

void MCTS::find_max_branch_value_kid() {
	// find the kid with the maximum expected value
	this->max_kid_branch_value = -0.000000000001;
	
	for (size_t i = 0; i < this->kids.size(); i++) {
		if (this->kids[i]->get_branch_value() > this->max_kid_branch_value) {
			this->max_kid_branch_value = this->kids[i]->get_euclid_value();
			this->max_kid_index = int(i);
		}
	}
}

void MCTS::find_sum_kid_branch_value() {
	// find the kid with the maximum expected value
	this->sum_kid_branch_value = 0.0;

	for (size_t i = 0; i < this->kids.size(); i++) {
		this->sum_kid_branch_value += this->kids[i]->get_branch_value();
	}
}

bool MCTS::kid_pruning_heuristic(const std::vector<bool> &task_status) {
	// this is used to determine if a kid should be made or not
	if (task_status[this->task_index]) {
		double travel_dist = 0.0;
		if (this->world->dist_between_nodes(this->task_index, this->parent->get_task_index(), travel_dist)) {
			if (travel_dist < this->max_kid_distance) {
				// save myself some future calcs
				this->euclid_distance = travel_dist;
				this->get_euclid_value();
				// value threshold next?
				if (this->euclid_value > 0.0) {
					this->get_expected_value();
					this->branch_value = this->expected_value; // only because kids don't exist yet!
					if (this->expected_value > 0.0) {
						return true;
					}
				}
			}
		}
	}
	return false;
}

MCTS::~MCTS(){
	for (size_t i = 0; i < this->kids.size(); i++) {
		delete this->kids[i];
	}
}

void MCTS::sample_tree_and_advertise_task_probabilities(Agent_Coordinator* coord_in) {
	
	if (this->probability < this->sampling_probability_threshold) {
		// this should never happen, due to below, but why not double check?
		return;
	}
	else {
		// add my task to coordinatot
		coord_in->add_stop_to_my_path(this->task_index, this->travel_time, this->probability);
		// sample my children and assign probability
		this->find_kid_probabilities();

		// those kids who are good enough I should continue to sample
		for (size_t i = 0; i < this->kids.size(); i++) {
			if (this->kids[i]->get_probability() > this->sampling_probability_threshold) {
				this->kids[i]->sample_tree_and_advertise_task_probabilities(coord_in);
			}
		}
	}
}

void MCTS::find_kid_probabilities() {

	// none of these should be true, but just check
	if (this->sum_kid_branch_value < 0.0) {
		this->find_sum_kid_branch_value();
	}
	if (this->max_kid_branch_value < 0.0) {
		this->find_max_branch_value_kid();
	}
	if (this->min_kid_branch_value < 0.0) {
		this->find_min_branch_value_kid();
	}

	// for all kids, assign their probability
	for (size_t i = 0; i < this->kids.size(); i++) {
		this->kids[i]->set_probability(this->sum_kid_branch_value);
	}
}

void MCTS::set_probability(const double &sum_value) {
	// this sets my probability of being selected by my parent
	this->probability = this->branch_value / sum_value;
}

void MCTS::rollout(const int &c_index, const int &rollout_depth, const double &time_in, std::vector<bool> &task_status, double &passed_branch_value) {
	if (rollout_depth > this->max_rollout_depth) {
		return;
	}

	int max_index = -1;
	double max_comp_reward = 0.0; // I only want positive rewards!
	double max_comp_time = 0.0;

	for (size_t i = 0; i < task_status.size(); i++) { // this intentionally does not use kids to rollout, rolled out nodes don't have kids and I probably don't want to make them yet.
		if (task_status[i]) {
			double e_dist = double(INFINITY);
			// get euclidean dist first
			if (world->dist_between_nodes(c_index, int(i), e_dist) && e_dist < this->max_kid_distance) {
				// this is for all tasks, not kids so need to do these
				double e_time = e_dist / this->agent->get_travel_step();
				double w_time = world->get_nodes()[i]->get_time_to_complete(this->agent, world);
				double comp_time = e_time + w_time + time_in;
				double e_reward = world->get_nodes()[i]->get_reward_at_time(world->get_c_time() + comp_time);
				// is my euclidean travel time reward better?
				if (e_reward > max_comp_reward) {
					// I am euclidean reward better, check if it is taken by someone else?
					double prob_taken = 0.0;
					if (this->agent->get_coordinator()->get_advertised_task_claim_probability(int(i), comp_time, prob_taken, world)) {
						// if not taken, then accept as possible goal
						if ((1-prob_taken)*e_reward > max_comp_reward) {
							max_comp_reward = (1 - prob_taken)*e_reward;
							max_index = int(i);
							max_comp_time = comp_time;
						}
					}
				}
			}
		}
	}
	if (max_index > -1) {
		// found the next kid to rollout, add kids value found in search and search kid.
		passed_branch_value += max_comp_reward; // add this iteration's reward
		
		// search below
		task_status[max_index] = false; // set task I am about to rollout as complete
		rollout(max_index, rollout_depth + 1, max_comp_time, task_status, passed_branch_value); // rollout selected task
		task_status[max_index] = true; // reset task I just rolledout

		return;
	}
	else {
		return;
	}
}


bool MCTS::find_sw_uct_kid(const std::vector<bool> &task_status, MCTS* &gc) {
	// this almost certainly is the wrong way to do this, instead, in explore exploit value and search I should update depending on which search method I am using
	return false;
}

bool MCTS::find_d_uct_kid(const std::vector<bool> &task_status, MCTS* &gc) {
	// this almost certainly is the wrong way to do this, instead, in explore exploit value and search I should update depending on which search method I am using
	return false;
}

bool MCTS::find_uct_kid(const std::vector<bool> &task_status, MCTS* &gc) {
	// this almost certainly is the wrong way to do this, instead, in explore exploit value and search I should update depending on which search method I am using
	double maxval = 0.0;
	gc = NULL;

	for (size_t i = 0; i < this->kids.size(); i++) { // check all of my kids
		if (task_status[this->kids[i]->get_task_index()]) { // are their tasks active?
			double s_val = this->kids[i]->get_euclid_value() + this->kids[i]->get_explore_value();
			if ( s_val > maxval) { // is their euclidean value  better than maxval?
				this->kids[i]->search_value = this->kids[i]->get_explore_value() + this->kids[i]->get_exploit_value(this->min_kid_branch_value, this->max_kid_branch_value);
				if ( this->kids[i]->search_value > maxval) { // is their a* value better than maxval?				
					maxval = this->kids[i]->search_value;
					gc = kids[i];
				}
			}
		}
	}
	if (maxval > 0.0) {
		return true;
	}
	else {
		return false;
	}
}

double MCTS::get_branch_value() {
	this->branch_value = this->get_expected_value() + this->max_kid_branch_value;
	return this->branch_value;
}

double MCTS::get_explore_value() {
	// this is the standard MCTS algorithm that provides a large reward for arms that have not been pulled much
	if (this->explore_value == -1.0) {
		this->explore_value = this->beta*sqrt(this->epsilon*log(this->parent->get_n_pulls()) / std::max(0.0001, this->number_pulls));
		return this->explore_value;
	}
	else {
		return this->explore_value;
	}
}

double MCTS::get_euclid_value() {
	// already know task is incomplete, this is only the value for completing me!
	// DOES NOT INCLUDE KIDS! NOT BRANCH VALUE!

	if (this->work_time < 0.0) {
		this->work_time = this->task->get_time_to_complete(this->agent, this->world);
	}

	// am I being initialized for the first time?
	if (this->euclid_time < 0.0) { // euclid dist already set!
		this->euclid_time = this->euclid_distance / this->agent->get_travel_step();
		this->euclid_reward = this->task->get_reward_at_time(this->euclid_time + this->work_time);
		double p_taken = 0.0;
		if (this->agent->get_coordinator()->get_advertised_task_claim_probability(this->task_index, this->euclid_time, p_taken, this->world)) {
			this->probability_task_available = (1 - p_taken);
			this->euclid_reward *= this->probability_task_available;
		}
		this->euclid_value = this->reward_weighting*this->euclid_reward - this->distance_weighting*this->euclid_distance;
		return this->euclid_value;
	}

	// next chance is that the probabilities changed because of a report
	if (this->probability_task_available < 0.0) {
		double p_taken = 0.0;
		if (this->agent->get_coordinator()->get_advertised_task_claim_probability(this->task_index, this->euclid_time, p_taken, this->world)) {
			this->probability_task_available = (1 - p_taken);
			this->euclid_reward *= this->probability_task_available;
		}
		this->euclid_value = this->reward_weighting*this->euclid_reward - this->distance_weighting*this->euclid_distance;
		return this->euclid_value;
	}

	// don't need to set or update value, just return
	return this->euclid_value;
}

double MCTS::get_exploit_value(const double &min, const double &max) {
	// this is the expected value normalized in range of kid values
	this->exploit_value = (this->get_expected_value() - min) / (max - min);
	return this->exploit_value;
}

double MCTS::get_expected_value() {
	// already know task is incomplete, this is only the value for completing me!
	// DOES NOT INCLUDE KIDS! NOT BRANCH VALUE!

	if (this->work_time < 0.0) {
		this->work_time = this->task->get_time_to_complete(this->agent, this->world);
	}

	// being set for the first time?
	if (this->distance < 0.0) {
		// need to set everything, then return value
		double dist = 0.0;
		std::vector<int> path;
		this->world->a_star(this->task_index, this->parent->get_task_index(), path, dist);
		this->distance = dist;
		this->travel_time = this->distance / this->agent->get_travel_step();
		this->completion_time = this->travel_time + this->work_time;
		this->reward = this->task->get_reward_at_time(this->completion_time);
		double p_taken = 0.0;
		if (this->agent->get_coordinator()->get_advertised_task_claim_probability(this->task_index, this->travel_time, p_taken, this->world)) {
			this->probability_task_available = (1 - p_taken);
			this->reward *= this->probability_task_available;
		}
		this->expected_value = this->reward_weighting*this->reward - this->distance_weighting*this->distance;

		return this->expected_value;
	}

	// next chance is that the probabilities changed because of a report
	if (this->probability_task_available < 0.0) {
		double p_taken = 0.0;
		if (this->agent->get_coordinator()->get_advertised_task_claim_probability(this->task_index, this->travel_time, p_taken, this->world)) {
			this->probability_task_available = (1 - p_taken);
			this->reward *= this->probability_task_available;
		}
		this->expected_value = this->reward_weighting*this->reward - this->distance_weighting*this->distance;

		return this->expected_value;
	}

	// don't need to set value, just return
	return this->expected_value;
}

void MCTS::update_kid_values_with_new_probabilities() {
	// need to update each kids expected value and then get their updated branch value
	for (size_t i = 0; i < this->kids.size(); i++) {
		this->kids[i]->reset_task_availability_probability();
		this->kids[i]->get_expected_value();
		this->kids[i]->get_branch_value();
	}
}

void MCTS::search(double &passed_branch_value, const double &time_in, std::vector<bool> &task_status, const bool &update_probability_tasks_available) {

	if (this->depth > this->max_search_depth) {
		// if I am past the max search depth i have 0 search reward and should return without adding to passed branch value
		return;
	}
	else {
		if (update_probability_tasks_available) {
			this->update_kid_values_with_new_probabilities();
		}

		// I will be searched, count it!
		this->number_pulls++;
		this->explore_value = -1.0; // reset explore value so it is recomputed next iter, this is implemented in get_explore_value()
	}

	if (this->kids.size() > 0) {
		// if I have kids, then select kid with best search value, and search them
		MCTS* gc = NULL;
		if (this->find_uct_kid(task_status, gc)) {

			// initialize kid's branch reward
			double kids_branch_value = 0;
			double kids_prior_branch_value = gc->get_branch_value();

			// search the kid's branch 
			task_status[gc->get_task_index()] = false; // simulate completing the task
			// not sure if this should be my completion time or theirs
			gc->search(kids_branch_value, gc->get_completion_time(), task_status, update_probability_tasks_available);
			task_status[gc->get_task_index()] = true; // mark the task incomplete, undo simulation

			// do something with the reward
			this->update_branch_values(gc, kids_prior_branch_value);
			passed_branch_value = this->branch_value;
		}
	}
	else {
		// I don't have kids, make kids and rollout best kid
		this->make_kids(task_status);
		MCTS* gc = this->kids[this->max_kid_index];
		double kids_rollout_value = 0.0;
		task_status[gc->get_task_index()] = false;
		gc->rollout(gc->get_task_index(), 0, this->world->get_c_time(), task_status, kids_rollout_value);
		task_status[gc->get_task_index()] = true;

		if (kids_rollout_value >= 0) {
			// check if I need to update max/min kids
			this->update_max_branch_value_kid(gc); // also updates my branch value!
			this->update_min_branch_value_kid(gc);
			passed_branch_value = this->branch_value;
		}
	}
}

void MCTS::update_branch_values(MCTS* gc, const double &kids_prior_branch_value) {
	if (gc->get_branch_value() > this->max_kid_branch_value || gc->get_kid_index() == this->max_kid_branch_value) {
		// check if I need to update max kid
		this->update_max_branch_value_kid(gc); // also updates my branch value!
	}
	if (gc->get_branch_value() < this->min_kid_branch_value || gc->get_kid_index() == this->min_kid_branch_value) {
		// check if I need to update min kid
		this->update_min_branch_value_kid(gc);
	}
	this->sum_kid_branch_value += gc->get_branch_value() - kids_prior_branch_value;
}

void MCTS::update_max_branch_value_kid(MCTS* gc) {
	// check if I need to update max kid
	if (gc->get_kid_index() == this->max_kid_index) {
		// I was the max kid, am I still?
		if (gc->get_branch_value() < this->max_kid_branch_value) {
			// my value decreased, I may not be!
			this->find_max_branch_value_kid(); // find new max
			this->branch_value = this->expected_value + this->max_kid_branch_value; // update my branch value
		}
	}
	else if (gc->get_expected_value() > this->max_kid_branch_value) {
		// I was not the max kid, but I beat them, so I guess I am now!
		this->max_kid_branch_value = gc->get_branch_value();
		this->max_kid_index = gc->get_kid_index();
		this->branch_value = this->expected_value + this->max_kid_branch_value; // update branch value
	}
}

void MCTS::update_min_branch_value_kid(MCTS* gc) {
	// check if I need to update min kid
	if (gc->get_kid_index() == this->min_kid_index) {
		// I was the min kid, am I still?
		if (gc->get_branch_value() > this->min_kid_branch_value) {
			// my value increased, I may not be!
			this->find_min_branch_value_kid(); // find new min
		}
	}
	else if (gc->get_expected_value() < this->max_kid_branch_value) {
		// I was not the min kid, but I beat them, so I guess I am now!
		this->min_kid_branch_value = gc->get_branch_value();
		this->min_kid_index = gc->get_kid_index();
	}
}
