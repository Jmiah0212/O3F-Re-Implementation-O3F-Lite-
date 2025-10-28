#include "Option.hpp"
#include "Env.hpp"

#include <limits>
#include <cmath>

static float length2(const sf::Vector2f& v) {
	return v.x * v.x + v.y * v.y;
}

MoveToTargetOption::MoveToTargetOption() : optionName("MoveToTarget" ) {}

void MoveToTargetOption::onSelect(Environment2D& env) {
	// Direct the robot toward the target region center
	env.setRobotTarget(env.getTargetRegion());
}

bool MoveToTargetOption::isComplete(const Environment2D& env) const {
	// Completed when robot is close to target region
	auto diff = env.getRobot().position - env.getTargetRegion();
	return length2(diff) <= (env.getTargetRadius() * env.getTargetRadius());
}

PushNearestObjectOption::PushNearestObjectOption() : optionName("PushNearestObject"), activeObjectIndex(-1) {}

void PushNearestObjectOption::onSelect(Environment2D& env) {
	// Find nearest object to robot, set target to that object position
	const auto& objs = env.getObjects();
	const auto& robot = env.getRobot();
	float bestD2 = std::numeric_limits<float>::max();
	int bestIdx = -1;
	for (int i = 0; i < static_cast<int>(objs.size()); ++i) {
		float d2 = length2(objs[i].position - robot.position);
		if (d2 < bestD2) {
			bestD2 = d2;
			bestIdx = i;
		}
	}
	activeObjectIndex = bestIdx;
	if (activeObjectIndex >= 0) {
		env.setRobotTarget(objs[activeObjectIndex].position);
	}
}

bool PushNearestObjectOption::isComplete(const Environment2D& env) const {
	if (activeObjectIndex < 0 || activeObjectIndex >= static_cast<int>(env.getObjects().size())) return true;
	// Complete if that object is inside target region
	auto diff = env.getObjects()[activeObjectIndex].position - env.getTargetRegion();
	return length2(diff) <= (env.getTargetRadius() * env.getTargetRadius());
}

std::vector<std::unique_ptr<Option>> makeDefaultOptions() {
	std::vector<std::unique_ptr<Option>> opts;
	opts.emplace_back(new MoveToTargetOption());
	opts.emplace_back(new PushNearestObjectOption());
	return opts;
}
