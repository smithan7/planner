#pragma once

#include <vector>

class Probability_Node;
class World;
class Agent;

class Agent_Coordinator
{
public:
	Agent_Coordinator(Agent* agent, int n_tasks);
	~Agent_Coordinator();

	Agent* owner;
	std::vector<Probability_Node*> get_prob_actions() { return this->prob_actions; };
	// coordination stuff
	bool get_advertised_task_claim_probability(int task_num, double query_time, double &prob_taken, World* world);
	bool get_claims_after(int task, double query_time, std::vector<double>& prob, std::vector<double>& times);
	bool advertise_task_claim(World* world);
	void reset_prob_actions(); // reset my probable actions
	void add_stop_to_my_path(int task_index, double time, double prob);
	double get_reward_impact(int task, int agent, double completion_time, World* world);

private:

	std::vector<Probability_Node*> prob_actions;
	int n_tasks;

	std::string task_claim_time, task_claim_method;
};

