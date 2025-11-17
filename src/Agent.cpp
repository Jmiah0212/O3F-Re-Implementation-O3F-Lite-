#include "Agent.hpp"
#include "Env.hpp"
#include "Option.hpp"
#include "Planner.hpp"
#include "Executor.hpp"
#include "Visualizer.hpp"

#include <algorithm>
#include <iostream>

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
	
	// State machine: 0=ClearObstacles, 1=MoveToTarget, 2=ReturnToObject, 3=MoveObjectToTarget
	int currentPhase = 0;
	const char* phaseNames[] = {"ClearObstacle", "MoveToTarget", "ReturnToObject", "MoveObjectToTarget"};
	
	while (viz.isOpen() && steps < maxSteps) {
		bool shouldClose = false, resetRequested = false;
		viz.pollEvents(shouldClose, resetRequested);
		if (shouldClose) break;
		if (resetRequested) {
			env.reset(5);
			currentPhase = 0;
		}

		(void)viz.frame();

		// Always use the current phase to determine which option to execute
		int optionIdx = currentPhase;
		
		// Execute the current phase's option
		Environment2D prevState = env;
		options[optionIdx]->onSelect(env);
		float reward = executor->executeOption(env, *options[optionIdx], 20);
		
		// Print debug info
		std::cout << "Episode " << steps / 20 << ", Phase: " << phaseNames[currentPhase] 
				  << ", Reward: " << reward << ", Total: " << (cumulative + reward) 
				  << ", Robot at (" << env.getRobotCell().x << "," << env.getRobotCell().y << ")" << std::endl;
		
		// Determine next phase based on current phase conditions
		if (currentPhase == 0) {
			// ClearObstacles phase: continue until no obstacles nearby
			if (!env.hasObstacleNeighbor()) {
				currentPhase = 1;
			}
		} else if (currentPhase == 1) {
			// MoveToTarget phase: continue until at target
			if (env.getRobotCell() == env.getTargetCell()) {
				currentPhase = 2;
			}
		} else if (currentPhase == 2) {
			// ReturnToObject phase: continue until carrying object
			if (env.isCarrying()) {
				currentPhase = 3;
			}
		} else if (currentPhase == 3) {
			// MoveObjectToTarget phase: check if task complete (at target with object)
			if (env.isTaskComplete()) {
				// Task complete! Reset and start over
				currentPhase = 0;
				env.reset(5);
				cumulative += 50.0f; // Big reward for success
				reward = 50.0f;
			}
		}
		
		planner->updateQ(prevState, optionIdx, reward, env, (int)options.size());
		cumulative += reward;
		steps += 20; // approximate steps consumed by option execution
		viz.render(env);
	}
	return cumulative;
}
