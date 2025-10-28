#pragma once

#include <SFML/System/Vector2.hpp>

class Environment2D;

class OptionExecutor {
public:
	// Drives low-level motion to follow targets set by options
	void tick(Environment2D& env, float dt);
};
