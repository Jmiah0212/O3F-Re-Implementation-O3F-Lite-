#include "Agent.hpp"
#include "Env.hpp"
#include "Option.hpp"
#include "Planner.hpp"
#include "Executor.hpp"
#include "Visualizer.hpp"

#include <algorithm>

static float rewardStep(const Environment2D& env) {
	// success: objects inside target region; time penalty per step
	float r = 0.f;
	for (const auto& o : env.getObjects()) {
		auto d = o.position - env.getTargetRegion();
		float dist2 = d.x * d.x + d.y * d.y;
		float rad2 = env.getTargetRadius() * env.getTargetRadius();
		if (dist2 <= rad2) r += 1.0f;
	}
	r -= 0.01f; // time penalty
	return r;
}

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

		float dt = viz.frame();
		timeSinceSelect += dt;

		if (currentOptionIdx < 0 || timeSinceSelect >= config.optionDurationSec || options[currentOptionIdx]->isComplete(env)) {
			Environment2D prev = env;
			int action = planner->selectAction(env, options);
			options[action]->onSelect(env);
			float r = rewardStep(env);
			planner->update(prev, action, r, env, (int)options.size());
			currentOptionIdx = action;
			timeSinceSelect = 0.f;
		}

		executor->tick(env, dt);
		env.step(dt);
		cumulative += rewardStep(env);

		viz.render(env);
		steps++;
	}
	return cumulative;
}
