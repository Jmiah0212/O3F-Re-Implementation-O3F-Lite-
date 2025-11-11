#pragma once

#include <memory>
#include <vector>

class Environment2D;
class Option;
class OptionPlanner;
class OptionExecutor;
class Visualizer;

struct AgentConfig {
	float optionDurationSec = 2.0f;
};

class Agent {
public:
	Agent(AgentConfig cfg);
	~Agent();

	void initialize();
	// Runs one episode for a fixed number of steps; returns cumulative reward
	float runEpisode(Environment2D& env, Visualizer& viz, int maxSteps);

private:
	AgentConfig config;
	std::unique_ptr<OptionPlanner> planner;
	std::unique_ptr<OptionExecutor> executor;
	std::vector<std::unique_ptr<Option>> options;

	int currentOptionIdx;
	float timeSinceSelect;
};
