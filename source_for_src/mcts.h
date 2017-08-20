#pragma once

#include <vector>

class Map_Node;
class Agent;
class Agent_Coordinator;
class World;

class MCTS
{
public:
	MCTS(World* world, Map_Node* task_in, Agent* agent_in, MCTS* parent, const int &my_kid_index);
	~MCTS();

	Agent* get_agent() { return this->agent; };
	double get_branch_value(); // might have to calc a few things, not a simple return
	int get_depth() { return this->depth; };
	double get_euclid_value(); // might have to calc some things, not a simple return
	double get_expected_value(); // might have to calc some things, not a simple return
	double get_exploit_value(const double &min, const double &max); // might have to calc values, not a simple return
	double get_explore_value(); // might have to calc a few things, not a simple return
	int get_kid_index() { return this->kid_index; }
	double get_n_pulls() { return this->number_pulls; };
	double get_probability() { return this->probability; };
	double get_reward() { return this->reward; };
	Map_Node* get_task() { return this->task; };
	int get_task_index() { return this->task_index; };
	double get_completion_time() { return this->completion_time; };
	
	// call from parent not self
	void search(double &passed_reward, const double &time_in, std::vector<bool> &task_status, const bool &update_probability);
	bool kid_pruning_heuristic(const std::vector<bool> &task_status);
	void reset_task_availability_probability() { this->probability_task_available = -1.0; };
	void set_probability(const double &sum_value);
	void sample_tree_and_advertise_task_probabilities(Agent_Coordinator* coord_in); // get my probabilities to advertise

private:

	// rollout does not create new nodes
	void rollout(const int &c_index, const int &rollout_depth, const double&time_in, std::vector<bool> &task_status, double &passed_value);
	void make_kids(const std::vector<bool> &task_status);
	void find_max_branch_value_kid(); // find the maximum expected value kid
	void find_min_branch_value_kid(); // find the minimum expected value kid 
	void find_sum_kid_branch_value(); // find the sum of all of my kids branch values
	void update_max_branch_value_kid(MCTS* gc); // check if I need to update max kid and update it and my branch value if I have to
	void update_min_branch_value_kid(MCTS* gc); // check if I need to update max kid and update if I have to
	void update_kid_values_with_new_probabilities();
	void update_branch_values(MCTS* gc, const double &kids_prior_branch_value); // update my branch value, min, max, and sum
	
	bool find_uct_kid(const std::vector<bool> &task_status, MCTS* &gc);
	bool find_d_uct_kid(const std::vector<bool> &task_status, MCTS* &gc);
	bool find_sw_uct_kid(const std::vector<bool> &task_status, MCTS* &gc);

	void find_kid_probabilities(); // find and assign my kid probabilities

	Agent* agent;
	Map_Node* task;
	int task_index;
	World* world;
	MCTS* parent;
	int depth, max_rollout_depth, max_search_depth;
	double max_kid_distance; // how far can a node be and still be a child
	
	double probability_task_available;
	double euclid_distance, distance; // how far from parent am I by euclidean and astar?
	double euclid_time, travel_time; // time by euclidean/astar path?
	double work_time; // once there, how long to complete?
	double completion_time; // time I actually finish and am ready to leave
	double euclid_reward, reward; // my reward at e_time/time?
	double euclid_value, expected_value; // my expected value for completing task at e_time/time with e_dist/dist?
	double reward_weighting, distance_weighting; // for value function
	double branch_value; // my and all my best kids expected value combined
	double exploit_value; // weighted expected value
	double explore_value; // value of searching rare arms
	double search_value; // exploit value + explore value

	double wait_time; // how long do I wait if I select a null action

	std::string search_method;
	std::vector<int> kid_pulled; // record keeping
	std::vector<double> reward_recieved; // record keeping

	int kid_index; // my index in my parents kids
	int max_kid_index, min_kid_index; // current golden child
	double max_kid_branch_value, min_kid_branch_value, sum_kid_branch_value; // their value, for normalizing

	double probability; // probability of me being selected, function of my relative value to adjacent kids and parent's probability
	double sampling_probability_threshold; // when probability drops below this, stop searching

	double number_pulls; // how many times have I been pulled
	double beta, epsilon, gamma; // for ucb, d-ducb, sw-ucb
	std::vector<MCTS*> kids; // my kids
};


