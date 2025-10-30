#include "Executor.hpp"
#include "Env.hpp"
#include "Option.hpp"

void OptionExecutor::tick(Environment2D& env, float dt) {
	(void)env;
	(void)dt;
}

float OptionExecutor::runPrimitiveUntil(Environment2D& env, int maxSteps,
	const std::function<bool(const Environment2D&)>& goal,
	const std::function<Action(const Environment2D&)>& policy) {
	float total = 0.f;
	for (int i = 0; i < maxSteps; ++i) {
		if (goal && goal(env)) break;
		Action a = policy ? policy(env) : Action::None;
		total += env.step(a);
	}
	return total;
}

float OptionExecutor::executeOption(Environment2D& env, const Option& option, int maxSteps) {
	float reward = runPrimitiveUntil(env, maxSteps, option.goal(), option.policy());
	// If option is ClearObstacle and we are adjacent, clear one obstacle
	if (option.name() == std::string("ClearObstacle")) {
		if (env.hasObstacleNeighbor()) {
			if (env.clearAnyAdjacentObstacle()) {
				// small extra shaping could be added here if desired
			}
		}
	}
	return reward;
}
