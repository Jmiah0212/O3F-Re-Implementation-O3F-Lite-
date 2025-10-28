#include "Executor.hpp"
#include "Env.hpp"

void OptionExecutor::tick(Environment2D& env, float dt) {
	// The low-level is handled in Environment::step via seeking to robotTarget.
	// This executor could implement finer control; for now it just advances the env.
	(void)dt;
}
