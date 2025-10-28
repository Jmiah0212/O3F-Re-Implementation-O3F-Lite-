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

private:
	PlannerConfig config;
	// Simple discrete state via coarse quantization of robot and target positions
	std::string discretize(const Environment2D& env) const;
	std::unordered_map<std::string, std::vector<float>> qTable;
};
