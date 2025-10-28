#include "Executor.hpp"
#include "Env.hpp"

void OptionExecutor::tick(Environment2D& env, float dt) {
	(void)env;
	(void)dt;
}

float OptionExecutor::runPrimitiveUntil(Environment2D& env, int maxSteps, bool (*goal)(const Environment2D&), Action policy(const Environment2D&)) {
	float total = 0.f;
	for (int i = 0; i < maxSteps; ++i) {
		if (goal && goal(env)) break;
		Action a = policy ? policy(env) : Action::None;
		total += env.step(a);
	}
	return total;
}
