#include "Option.hpp"
#include "Env.hpp"

#include <limits>
#include <cmath>

static float length2(const sf::Vector2f& v) {
	return v.x * v.x + v.y * v.y;
}

static Action greedyToCell(const Environment2D& env, const sf::Vector2i& target) {
	sf::Vector2i r = env.getRobotCell();
	if (r.x < target.x) return Action::Right;
	if (r.x > target.x) return Action::Left;
	if (r.y < target.y) return Action::Down;
	if (r.y > target.y) return Action::Up;
	return Action::None;
}

MoveToTargetOption::MoveToTargetOption() : optionName("MoveToTarget" ) {}

void MoveToTargetOption::onSelect(Environment2D& env) {
	(void)env;
}

bool MoveToTargetOption::isComplete(const Environment2D& env) const {
	return env.getRobotCell() == env.getTargetCell();
}

std::function<bool(const Environment2D&)> MoveToTargetOption::goal() const {
	return [](const Environment2D& e) { return e.getRobotCell() == e.getTargetCell(); };
}

std::function<Action(const Environment2D&)> MoveToTargetOption::policy() const {
	return [](const Environment2D& e) { return greedyToCell(e, e.getTargetCell()); };
}

ClearObstacleOption::ClearObstacleOption() : optionName("ClearObstacle") {}

void ClearObstacleOption::onSelect(Environment2D& env) {
	(void)env;
}

bool ClearObstacleOption::isComplete(const Environment2D& env) const {
	// Done when no obstacle adjacent
	return !env.hasObstacleNeighbor();
}

std::function<bool(const Environment2D&)> ClearObstacleOption::goal() const {
	return [](const Environment2D& e) { return e.hasObstacleNeighbor(); };
}

std::function<Action(const Environment2D&)> ClearObstacleOption::policy() const {
	return [](const Environment2D& e) {
		// Move greedily toward target; if obstacle neighbor already, None to allow clearing
		if (e.hasObstacleNeighbor()) return Action::None;
		return greedyToCell(e, e.getTargetCell());
	};
}

GraspTargetOption::GraspTargetOption() : optionName("GraspTarget") {}

void GraspTargetOption::onSelect(Environment2D& env) {
	(void)env;
}

bool GraspTargetOption::isComplete(const Environment2D& env) const {
	return env.getRobotCell() == env.getTargetCell();
}

std::function<bool(const Environment2D&)> GraspTargetOption::goal() const {
	return [](const Environment2D& e) { return e.getRobotCell() == e.getTargetCell(); };
}

std::function<Action(const Environment2D&)> GraspTargetOption::policy() const {
	return [](const Environment2D& e) { return greedyToCell(e, e.getTargetCell()); };
}

std::vector<std::unique_ptr<Option>> makeDefaultOptions() {
	std::vector<std::unique_ptr<Option>> opts;
	opts.emplace_back(new ClearObstacleOption());
	opts.emplace_back(new MoveToTargetOption());
	opts.emplace_back(new GraspTargetOption());
	return opts;
}
