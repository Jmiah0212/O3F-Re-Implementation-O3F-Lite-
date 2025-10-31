#pragma once
#include "env.hpp"
#include "option_executor.hpp"
#include <vector>
#include <random>

using namespace std;

// Simplified option set:
//  - Option 0: PICK  (navigate next to TARGET, then 'P')
//  - Option 1: PLACE (navigate next to GOAL, then 'L')
enum OptionType { OPT_PICK = 0, OPT_PLACE = 1 };

class OptionPlannerQL {
public:
    OptionPlannerQL(int seed = 0);

    // Choose option epsilon-greedily given current env state.
    int choose_option(const Env& env, double epsilon);

    // Learn from a transition: (s, a) -> (r, s')
    void update(int s_id, int a, double R, int s_id_next, double gamma);

    // Compute discrete state id from env (18 bins: holding × dist bins × dist bins)
    int state_id(const Env& env) const;

    // For demo: greedy choice with epsilon=0
    int greedy(const Env& env){ return choose_option(env, 0.0); }

    // Execute an option using the executor; returns cumulative reward.
    double execute_option(int option, Env& env, OptionExecutor& exec);

    // Expose Q for inspection
    const vector<double>& qtable() const { return Q; }

private:
    // Q table: S x A = 18 x 2
    vector<double> Q;
    double alpha = 0.15;
    mt19937 rng;

    int dist_bin(int d) const; // 0:[0-2], 1:[3-5], 2:[6+]
    int argmaxQ(int s_id) const;
};
