#include "Option.hpp"
#include "Env.hpp"

#include <limits>
#include <cmath>
#include <algorithm>
#include <vector>
#include <queue>
#include <unordered_map>

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

// BFS to find the full path from start to target, avoiding obstacles
static std::vector<sf::Vector2i> bfsFullPath(const Environment2D& env, const sf::Vector2i& target) {
	std::vector<sf::Vector2i> emptyPath;
	int w = env.getGridWidth();
	int h = env.getGridHeight();
	auto start = env.getRobotCell();
	if (start == target) return emptyPath;

	auto idx = [&](int x, int y){ return y * w + x; };

	std::vector<int> parent(w * h, -1);
	std::queue<int> q;
	int s = idx(start.x, start.y);
	int g = idx(target.x, target.y);
	q.push(s);
	parent[s] = s;

	static const int dx[4] = {1, -1, 0, 0};
	static const int dy[4] = {0, 0, 1, -1};

	bool found = false;
	while (!q.empty()) {
		int cur = q.front(); q.pop();
		int cx = cur % w;
		int cy = cur / w;
		for (int k = 0; k < 4; ++k) {
			int nx = cx + dx[k];
			int ny = cy + dy[k];
			if (nx < 0 || nx >= w || ny < 0 || ny >= h) continue;
			int ni = idx(nx, ny);
			if (parent[ni] != -1) continue;
			if (env.isObstacle({nx, ny})) continue;
			parent[ni] = cur;
			if (ni == g) { found = true; break; }
			q.push(ni);
		}
		if (found) break;
	}

	if (!found) return emptyPath;

	// Reconstruct path: from goal back to start
	std::vector<sf::Vector2i> path;
	int cur = g;
	while (cur != s) {
		int x = cur % w;
		int y = cur / w;
		path.push_back({x, y});
		cur = parent[cur];
	}
	std::reverse(path.begin(), path.end());
	return path;
}

// BFS to find the next action along a shortest path to `target` avoiding obstacles.
static Action bfsNextAction(const Environment2D& env, const sf::Vector2i& target) {
	int w = env.getGridWidth();
	int h = env.getGridHeight();
	auto start = env.getRobotCell();
	if (start == target) return Action::None;

	auto idx = [&](int x, int y){ return y * w + x; };

	std::vector<int> parent(w * h, -1);
	std::queue<int> q;
	int s = idx(start.x, start.y);
	int g = idx(target.x, target.y);
	q.push(s);
	parent[s] = s;

	static const int dx[4] = {1, -1, 0, 0};
	static const int dy[4] = {0, 0, 1, -1};

	bool found = false;
	while (!q.empty()) {
		int cur = q.front(); q.pop();
		int cx = cur % w;
		int cy = cur / w;
		for (int k = 0; k < 4; ++k) {
			int nx = cx + dx[k];
			int ny = cy + dy[k];
			if (nx < 0 || nx >= w || ny < 0 || ny >= h) continue;
			int ni = idx(nx, ny);
			if (parent[ni] != -1) continue;
			if (env.isObstacle({nx, ny})) continue;
			parent[ni] = cur;
			if (ni == g) { found = true; break; }
			q.push(ni);
		}
		if (found) break;
	}

	if (!found) return Action::None;

	// Reconstruct path: from goal back to start, take the first step after start
	int cur = g;
	int prev = parent[cur];
	while (prev != s) {
		cur = prev;
		prev = parent[cur];
	}
	int nextX = cur % w;
	int nextY = cur / w;

	int rx = start.x;
	int ry = start.y;
	if (nextX == rx + 1 && nextY == ry) return Action::Right;
	if (nextX == rx - 1 && nextY == ry) return Action::Left;
	if (nextX == rx && nextY == ry + 1) return Action::Down;
	if (nextX == rx && nextY == ry - 1) return Action::Up;
	return Action::None;
}

MoveToTargetOption::MoveToTargetOption() : optionName("MoveToTarget") {}

void MoveToTargetOption::onSelect(Environment2D& env) {
	(void)env;
}

bool MoveToTargetOption::isComplete(const Environment2D& env) const {
	// Used by executor to detect early completion
	return env.getRobotCell() == env.getTargetCell();
}

std::function<bool(const Environment2D&)> MoveToTargetOption::goal() const {
	return [](const Environment2D& e) { 
		return e.getRobotCell() == e.getTargetCell(); 
	};
}

std::function<Action(const Environment2D&)> MoveToTargetOption::policy() const {
	return [](const Environment2D& e) { 
		// Move toward target, ignore obstacles and objects
		return smartPathfinding(e, e.getTargetCell()); 
	};
}

ClearObstacleOption::ClearObstacleOption() : optionName("ClearObstacle") {}

void ClearObstacleOption::onSelect(Environment2D& env) {
	(void)env;
}

bool ClearObstacleOption::isComplete(const Environment2D& env) const {
	// If carrying, we consider this option complete (can't/shouldn't clear while carrying)
	if (env.isCarrying()) return true;
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

		// If carrying, do not attempt to clear; instead move toward target
		if (e.isCarrying()) {
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
					// Consider obstacle strategic if it's useful to clear for the global
					// target OR (when not carrying) for the object (so ReturnToObject can
					// make progress). This prevents ClearObstacle from opportunistically
					// clearing obstacles that don't help reach the object.
					if (e.isObstacle(obstaclePos) && (
						 e.shouldClearObstacle(obstaclePos) ||
						 (!e.isCarrying() && e.shouldClearObstacleToward(obstaclePos, e.getObjectCell())))) {
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

MoveToObjectOption::MoveToObjectOption() : optionName("MoveToObject") {}

void MoveToObjectOption::onSelect(Environment2D& env) {
	// Store the path to the object when this option is selected
	pathToObject = bfsFullPath(env, env.getObjectCell());
}

bool MoveToObjectOption::isComplete(const Environment2D& env) const {
	return env.getRobotCell() == env.getObjectCell();
}

std::function<bool(const Environment2D&)> MoveToObjectOption::goal() const {
	return [](const Environment2D& e) { 
		return e.getRobotCell() == e.getObjectCell(); 
	};
}

std::function<Action(const Environment2D&)> MoveToObjectOption::policy() const {
	return [](const Environment2D& e) { 
		return smartPathfinding(e, e.getObjectCell()); 
	};
}

MoveObjectToTargetOption::MoveObjectToTargetOption() : optionName("MoveObjectToTarget"), objectPickupLocation(-1, -1), returnPathIndex(0) {}

void MoveObjectToTargetOption::onSelect(Environment2D& env) {
	// Remember where we picked up the object
	if (env.isCarrying()) {
		objectPickupLocation = env.getObjectCell();
	}
	// Don't reset returnPathIndex here - it should only be reset in setReturnPath()
	// so that the path index persists across multiple onSelect calls
}

bool MoveObjectToTargetOption::isComplete(const Environment2D& env) const {
	// Complete when we're at target AND carrying the object
	return env.isTaskComplete();
}

std::function<bool(const Environment2D&)> MoveObjectToTargetOption::goal() const {
	return [](const Environment2D& e) {
		return e.isTaskComplete();
	};
}

std::function<Action(const Environment2D&)> MoveObjectToTargetOption::policy() const {
	return [this](const Environment2D& e) {
		// If we have a return path stored, follow it in reverse
		if (!returnPath.empty() && returnPathIndex < returnPath.size()) {
			sf::Vector2i currentPos = e.getRobotCell();
			sf::Vector2i nextPos = returnPath[returnPath.size() - 1 - returnPathIndex];
			
			// Check if we've reached this waypoint
			int dx = nextPos.x - currentPos.x;
			int dy = nextPos.y - currentPos.y;
			
			if (dx == 0 && dy == 0) {
				// At waypoint, move to next
				returnPathIndex++;
				if (returnPathIndex < returnPath.size()) {
					nextPos = returnPath[returnPath.size() - 1 - returnPathIndex];
					dx = nextPos.x - currentPos.x;
					dy = nextPos.y - currentPos.y;
				} else {
					// Reached target
					return Action::None;
				}
			}
			
			// Move toward waypoint
			if (dx > 0) return Action::Right;
			if (dx < 0) return Action::Left;
			if (dy > 0) return Action::Down;
			if (dy < 0) return Action::Up;
			
			return Action::None;
		}
		
		// Fallback: use smart pathfinding toward target
		return smartPathfinding(e, e.getTargetCell());
	};
}

// ReturnToObject option implementations
ReturnToObjectOption::ReturnToObjectOption() : optionName("ReturnToObject") {}

void ReturnToObjectOption::onSelect(Environment2D& env) {
	(void)env;
}

bool ReturnToObjectOption::isComplete(const Environment2D& env) const {
	// Complete when carrying the object
	return env.isCarrying();
}

std::function<bool(const Environment2D&)> ReturnToObjectOption::goal() const {
	return [](const Environment2D& e) {
		return e.isCarrying();
	};
}

std::function<Action(const Environment2D&)> ReturnToObjectOption::policy() const {
	return [](const Environment2D& e) {
		// Try deterministic BFS toward the object first
		Action a = bfsNextAction(e, e.getObjectCell());
		if (a != Action::None) return a;

		// If blocked, attempt to approach a strategic obstacle so ClearObstacle can act
		const int maxSearchRadius = 8;
		sf::Vector2i robot = e.getRobotCell();
		sf::Vector2i object = e.getObjectCell();

		bool found = false;
		sf::Vector2i bestObs(0,0);
		float bestScore = std::numeric_limits<float>::infinity();

		for (int dx = -maxSearchRadius; dx <= maxSearchRadius; ++dx) {
			for (int dy = -maxSearchRadius; dy <= maxSearchRadius; ++dy) {
				sf::Vector2i pos(robot.x + dx, robot.y + dy);
				if (pos.x < 0 || pos.x >= e.getGridWidth() || pos.y < 0 || pos.y >= e.getGridHeight()) continue;
				if (!e.isObstacle(pos)) continue;

				float dr = std::hypot((float)(pos.x - robot.x), (float)(pos.y - robot.y));
				float do_ = std::hypot((float)(pos.x - object.x), (float)(pos.y - object.y));
				float score = dr + 0.5f * do_;

				if (score < bestScore) {
					bestScore = score;
					bestObs = pos;
					found = true;
				}
			}
		}

		if (!found) return Action::None;

		static const int ndx[4] = {1, -1, 0, 0};
		static const int ndy[4] = {0, 0, 1, -1};
		for (int k = 0; k < 4; ++k) {
			sf::Vector2i adj(bestObs.x + ndx[k], bestObs.y + ndy[k]);
			if (adj.x < 0 || adj.x >= e.getGridWidth() || adj.y < 0 || adj.y >= e.getGridHeight()) continue;
			if (e.isObstacle(adj)) continue;
			return bfsNextAction(e, adj);
		}

		return Action::None;
	};
}

std::vector<std::unique_ptr<Option>> makeDefaultOptions() {
	std::vector<std::unique_ptr<Option>> opts;
	// New order: clear obstacles, go to target, return to object, then bring object to target
	opts.emplace_back(new ClearObstacleOption());
	opts.emplace_back(new MoveToTargetOption());
	opts.emplace_back(new ReturnToObjectOption());
	opts.emplace_back(new MoveObjectToTargetOption());
	return opts;
}