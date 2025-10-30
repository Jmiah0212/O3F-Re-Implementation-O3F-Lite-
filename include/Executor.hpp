#pragma once

#include <SFML/System/Vector2.hpp>
#include <functional>

class Environment2D;
class Option;

enum class Action;

class OptionExecutor {
public:
	void tick(Environment2D& env, float dt);

	float runPrimitiveUntil(Environment2D& env, int maxSteps,
		const std::function<bool(const Environment2D&)>& goal,
		const std::function<Action(const Environment2D&)>& policy);

	float executeOption(Environment2D& env, const Option& option, int maxSteps);
};
