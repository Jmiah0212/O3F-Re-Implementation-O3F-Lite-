#pragma once

#include <unordered_map>
#include <vector>
#include <memory>
#include <string>

class Environment2D;
class Option;

struct PlannerConfig {
	float alpha = 0.2f; // learning rate
	float gamma = 0.95f; // discount
	float epsilon = 0.1f; // exploration
};

class OptionPlanner {
public:
	OptionPlanner(PlannerConfig cfg);
	int selectAction(const Environment2D& env, const std::vector<std::unique_ptr<Option>>& options);
	void update(const Environment2D& prevEnv, int actionIdx, float reward, const Environment2D& nextEnv, int numActions);

	// explicit API per Step 6/7 naming
	int selectOption(const Environment2D& env, const std::vector<std::unique_ptr<Option>>& options) { return selectAction(env, options); }
	void updateQ(const Environment2D& prevEnv, int optionIdx, float optionReward, const Environment2D& nextEnv, int numActions) { update(prevEnv, optionIdx, optionReward, nextEnv, numActions); }

private:
	PlannerConfig config;
	std::string discretize(const Environment2D& env) const;
	std::unordered_map<std::string, std::vector<float>> qTable;
};
