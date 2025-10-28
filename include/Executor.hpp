#pragma once

#include <SFML/System/Vector2.hpp>

class Environment2D;

enum class Action;

class OptionExecutor {
public:
	// Drives low-level motion to follow targets set by options
	void tick(Environment2D& env, float dt);

	// Execute primitive actions until goal or max steps; returns accumulated reward
	float runPrimitiveUntil(Environment2D& env, int maxSteps, bool (*goal)(const Environment2D&), Action policy(const Environment2D&));
};
