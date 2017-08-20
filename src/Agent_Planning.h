#pragma once

#include <vector>
#include<iostream>
#include<string> // for string class

class World;
class Probability_Node;
class Agent;
class MCTS;

class Agent_Planning
{
public:
	Agent_Planning(Agent* agent, World* world);
	~Agent_Planning();
	void plan(); // select a new goal, called at every node
	Agent* get_owner() { return this->owner; };

private:
	
	Agent* owner;
	World* world;
	MCTS* my_tree;
	std::string task_selection_method; // how do I select tasks
	std::string task_claim_method; // how do I claim tasks
	std::string task_claim_time; // when do I claim tasks



	void set_goal(const int &goal_index);
	void set_goal(const int &goal_index, const std::vector<std::string> &args, const std::vector<double> &vals);
	// greedy by time
	void select_greedy_task_by_arrival_time();
	void select_greedy_task_by_completion_time();
	// greedy by reward
	void select_greedy_task_by_current_reward();
	void select_greedy_task_by_arrival_reward();
	void select_greedy_task_by_completion_reward();
	// by value
	void select_task_by_current_value(); // don't implement
	void select_task_by_arrival_value(); // don't implement
	void select_task_by_completion_value();
	// by impact at completion time
	void select_task_by_reward_impact();
	void select_task_by_value_impact();
	
	void select_task_by_impact_completion_reward();
	void select_task_by_impact_completion_value();

	// MCTS
	void MCTS_task_by_completion_reward();
	void MCTS_task_by_completion_value();
	void MCTS_task_by_completion_reward_impact();
	void MCTS_task_by_completion_value_impact();


	// random actions
	void select_random_node();
	void select_random_nbr();
	void select_random_task();
};

