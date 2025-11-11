#include "Agent.hpp"
#include "Env.hpp"
#include "Option.hpp"
#include "Planner.hpp"
#include "Executor.hpp"
#include "Visualizer.hpp"

#include <algorithm>

Agent::Agent(AgentConfig cfg) : config(cfg), currentOptionIdx(-1), timeSinceSelect(0.f) {}
Agent::~Agent() = default;

void Agent::initialize() {
	options = makeDefaultOptions();
	planner.reset(new OptionPlanner(PlannerConfig{}));
	executor.reset(new OptionExecutor());
}

float Agent::runEpisode(Environment2D& env, Visualizer& viz, int maxSteps) {
	float cumulative = 0.f;
	int steps = 0;
	while (viz.isOpen() && steps < maxSteps) {
		bool shouldClose = false, resetRequested = false;
		viz.pollEvents(shouldClose, resetRequested);
		if (shouldClose) break;
		if (resetRequested) env.reset(5);

		(void)viz.frame();

		Environment2D prevState = env;
		int optionIdx = planner->selectOption(env, options);
		options[optionIdx]->onSelect(env);
		float reward = executor->executeOption(env, *options[optionIdx], 20);
		planner->updateQ(prevState, optionIdx, reward, env, (int)options.size());
		cumulative += reward;
		steps += 20; // approximate steps consumed by option execution
		viz.render(env);
	}
	return cumulative;
}
