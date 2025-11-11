#include "Option.hpp"
#include "Env.hpp"

#include <limits>
#include <cmath>
#include <algorithm>
#include <vector>

static Action smartPathfinding(const Environment2D& env, const sf::Vector2i& target) {
	sf::Vector2i r = env.getRobotCell();
	
	// Calculate direction to target
	int dx = target.x - r.x;
	int dy = target.y - r.y;
	
	// Prioritize moving along the axis with greater distance
	bool favorX = std::abs(dx) >= std::abs(dy);
	
	struct MoveOption {
		Action action;
		sf::Vector2i pos;
		float priority;
		bool blocked;
	};
	
	std::vector<MoveOption> options;
	
	// Primary moves (toward target on major axis)
	if (favorX) {
		if (dx > 0) options.push_back({Action::Right, {r.x + 1, r.y}, 1.0f, false});
		else if (dx < 0) options.push_back({Action::Left, {r.x - 1, r.y}, 1.0f, false});
		
		if (dy > 0) options.push_back({Action::Down, {r.x, r.y + 1}, 2.0f, false});
		else if (dy < 0) options.push_back({Action::Up, {r.x, r.y - 1}, 2.0f, false});
	} else {
		if (dy > 0) options.push_back({Action::Down, {r.x, r.y + 1}, 1.0f, false});
		else if (dy < 0) options.push_back({Action::Up, {r.x, r.y - 1}, 1.0f, false});
		
		if (dx > 0) options.push_back({Action::Right, {r.x + 1, r.y}, 2.0f, false});
		else if (dx < 0) options.push_back({Action::Left, {r.x - 1, r.y}, 2.0f, false});
	}
	
	// Add perpendicular moves as backup (lower priority)
	if (favorX) {
		if (dy == 0) {
			options.push_back({Action::Up, {r.x, r.y - 1}, 3.0f, false});
			options.push_back({Action::Down, {r.x, r.y + 1}, 3.0f, false});
		}
	} else {
		if (dx == 0) {
			options.push_back({Action::Left, {r.x - 1, r.y}, 3.0f, false});
			options.push_back({Action::Right, {r.x + 1, r.y}, 3.0f, false});
		}
	}
	
	// Check for obstacles and boundaries
	for (auto& opt : options) {
		bool outOfBounds = (opt.pos.x < 0 || opt.pos.x >= env.getGridWidth() || 
		                   opt.pos.y < 0 || opt.pos.y >= env.getGridHeight());
		opt.blocked = outOfBounds || env.isObstacle(opt.pos);
		
		if (opt.blocked) {
			opt.priority = 999.0f; // Very low priority
		}
	}
	
	// Sort by priority (lower is better)
	std::sort(options.begin(), options.end(), [](const MoveOption& a, const MoveOption& b) {
		return a.priority < b.priority;
	});
	
	// Return best unblocked option
	for (const auto& opt : options) {
		if (!opt.blocked) {
			return opt.action;
		}
	}
	
	// All directions blocked - stay still
	return Action::None;
}

MoveToTargetOption::MoveToTargetOption() : optionName("MoveToTarget") {}

void MoveToTargetOption::onSelect(Environment2D& env) {
	(void)env;
}

bool MoveToTargetOption::isComplete(const Environment2D& env) const {
	return env.getRobotCell() == env.getTargetCell();
}

std::function<bool(const Environment2D&)> MoveToTargetOption::goal() const {
	return [](const Environment2D& e) { 
		return e.getRobotCell() == e.getTargetCell(); 
	};
}

std::function<Action(const Environment2D&)> MoveToTargetOption::policy() const {
	return [](const Environment2D& e) { 
		return smartPathfinding(e, e.getTargetCell()); 
	};
}

ClearObstacleOption::ClearObstacleOption() : optionName("ClearObstacle") {}

void ClearObstacleOption::onSelect(Environment2D& env) {
	(void)env;
}

bool ClearObstacleOption::isComplete(const Environment2D& env) const {
	return !env.hasObstacleNeighbor();
}

std::function<bool(const Environment2D&)> ClearObstacleOption::goal() const {
	return [](const Environment2D& e) { 
		return !e.hasObstacleNeighbor();
	};
}

std::function<Action(const Environment2D&)> ClearObstacleOption::policy() const {
	return [](const Environment2D& e) {
		// This option should only execute when there's an obstacle to clear
		if (!e.hasObstacleNeighbor()) {
			// Move toward target if no obstacle nearby
			return smartPathfinding(e, e.getTargetCell());
		}
		
		// Find and approach the most strategic obstacle
		static const int dx[4] = {1, -1, 0, 0};
		static const int dy[4] = {0, 0, 1, -1};
		
		sf::Vector2i bestObstacle = e.getRobotCell();
		bool foundStrategicObstacle = false;
		
		for (int k = 0; k < 4; ++k) {
			int nx = e.getRobotCell().x + dx[k];
			int ny = e.getRobotCell().y + dy[k];
			if (nx >= 0 && nx < e.getGridWidth() && ny >= 0 && ny < e.getGridHeight()) {
				sf::Vector2i obstaclePos(nx, ny);
				if (e.isObstacle(obstaclePos) && e.shouldClearObstacle(obstaclePos)) {
					bestObstacle = obstaclePos;
					foundStrategicObstacle = true;
					break;
				}
			}
		}
		
		if (foundStrategicObstacle) {
			// We're adjacent to strategic obstacle - stay and clear it
			return Action::None;
		} else {
			// No strategic obstacle nearby - move toward target
			return smartPathfinding(e, e.getTargetCell());
		}
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
	return [](const Environment2D& e) { 
		return e.getRobotCell() == e.getTargetCell(); 
	};
}

std::function<Action(const Environment2D&)> GraspTargetOption::policy() const {
	return [](const Environment2D& e) { 
		return smartPathfinding(e, e.getTargetCell()); 
	};
}

std::vector<std::unique_ptr<Option>> makeDefaultOptions() {
	std::vector<std::unique_ptr<Option>> opts;
	opts.emplace_back(new MoveToTargetOption());
	opts.emplace_back(new ClearObstacleOption());
	opts.emplace_back(new GraspTargetOption());
	return opts;
}