#pragma once

#include <unordered_map>
#include <vector>
#include <memory>
#include <string>

class Environment2D;
class Option;

struct PlannerConfig {
	float alpha = 0.1f;         // Learning rate
	float gamma = 0.95f;        // Discount factor (increased from 0.9)
	float epsilon = 0.3f;       // Initial exploration rate (decreased from 1.0)
	float epsilonDecay = 0.995f; // Decay per episode
	float epsilonMin = 0.05f;   // Minimum exploration
};

class OptionPlanner {
public:
	OptionPlanner(PlannerConfig cfg);
	// Save/load Q-table to CSV. CSV rows: state,q0,q1,...
	bool saveQTable(const std::string& path) const;
	bool loadQTable(const std::string& path);
	// Access to internal config so callers can read/update epsilon, decay, etc.
	PlannerConfig& getConfig() { return config; }
	const PlannerConfig& getConfig() const { return config; }
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
