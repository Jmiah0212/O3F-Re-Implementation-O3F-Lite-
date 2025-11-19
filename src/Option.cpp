#include "Option.hpp"
#include "Env.hpp"

#include <limits>
#include <cmath>
#include <algorithm>
#include <vector>
#include <queue>
#include <unordered_map>

// Helper function to check if a cell is on the boundary (edge of environment)
static bool isBoundaryCell(const sf::Vector2i& cell, int gridW, int gridH) {
	return (cell.x <= 0 || cell.x >= gridW - 1 || cell.y <= 0 || cell.y >= gridH - 1);
}

// Helper function to track move history and detect stuck loops
static void updateMoveHistory(std::vector<Action>& history, int& consecutiveCount, Action newAction) {
	// Keep only last 5 moves for memory efficiency
	if (history.size() > 5) {
		history.erase(history.begin());
	}
	
	// Check if this action is the same as the last action
	if (!history.empty() && history.back() == newAction) {
		consecutiveCount++;
	} else {
		consecutiveCount = 1;
	}
	
	history.push_back(newAction);
}

// Helper function to check if agent is stuck in a loop (same move 3+ times)
static bool isStuckInLoop(int consecutiveCount) {
	return consecutiveCount >= 3;
}

// Helper function to find if path is blocked and return direction to nearest blocking obstacle
// Returns the next action toward the nearest blocking obstacle if path is blocked
// Returns Action::None if path is clear
static Action findNextActionTowardBlockingObstacle(const Environment2D& env, const sf::Vector2i& target) {
	int w = env.getGridWidth();
	int h = env.getGridHeight();
	auto start = env.getRobotCell();
	
	if (start == target) return Action::None;
	
	auto idx = [&](int x, int y){ return y * w + x; };
	
	// First, check if we can reach target via BFS (ignoring obstacles)
	std::vector<int> parent(w * h, -1);
	std::queue<int> q;
	int s = idx(start.x, start.y);
	int g = idx(target.x, target.y);
	q.push(s);
	parent[s] = s;
	
	static const int dx[4] = {1, -1, 0, 0};
	static const int dy[4] = {0, 0, 1, -1};
	
	bool foundTarget = false;
	std::vector<int> distToTarget(w * h, -1);
	distToTarget[s] = std::abs(start.x - target.x) + std::abs(start.y - target.y);
	
	// BFS ignoring obstacles to find theoretical shortest path
	while (!q.empty()) {
		int cur = q.front(); q.pop();
		int cx = cur % w;
		int cy = cur / w;
		
		if (cur == g) {
			foundTarget = true;
			break;
		}
		
		for (int k = 0; k < 4; ++k) {
			int nx = cx + dx[k];
			int ny = cy + dy[k];
			if (nx < 0 || nx >= w || ny < 0 || ny >= h) continue;
			if (isBoundaryCell({nx, ny}, w, h)) continue;
			
			int ni = idx(nx, ny);
			if (parent[ni] != -1) continue;  // Already visited
			
			parent[ni] = cur;
			distToTarget[ni] = std::abs(nx - target.x) + std::abs(ny - target.y);
			q.push(ni);
		}
	}
	
	if (!foundTarget) return Action::None;  // No theoretical path even ignoring obstacles
	
	// Now find obstacles on the theoretical shortest path
	// Walk from start toward target, find first obstacle blocking direct progress
	std::vector<sf::Vector2i> theoreticalPath;
	int cur = g;
	while (cur != s) {
		int x = cur % w;
		int y = cur / w;
		theoreticalPath.push_back({x, y});
		cur = parent[cur];
	}
	std::reverse(theoreticalPath.begin(), theoreticalPath.end());
	
	// Find the first obstacle on this path or adjacent to it
	for (const auto& pathCell : theoreticalPath) {
		if (env.isObstacle(pathCell)) {
			// Found blocking obstacle on the path
			// Move toward it
			int dx_to_obs = pathCell.x - start.x;
			int dy_to_obs = pathCell.y - start.y;
			
			if (dx_to_obs > 0) return Action::Right;
			if (dx_to_obs < 0) return Action::Left;
			if (dy_to_obs > 0) return Action::Down;
			if (dy_to_obs < 0) return Action::Up;
		}
	}
	
	return Action::None;  // No blocking obstacles found
}

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
		bool onBoundary = isBoundaryCell(opt.pos, env.getGridWidth(), env.getGridHeight());
		opt.blocked = outOfBounds || onBoundary || env.isObstacle(opt.pos);
		
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

// BFS to find the full path from start to target, avoiding obstacles and boundary cells
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
			// Avoid boundary cells
			if (isBoundaryCell({nx, ny}, w, h)) continue;
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

// BFS to find next action toward target, ignoring ALL obstacles
// Used for Phase 2 (MoveToObject) where we want BFS to find path and clear obstacles
static Action bfsNextActionIgnoringObstacles(const Environment2D& env, const sf::Vector2i& target) {
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
			// Avoid boundary cells
			if (isBoundaryCell({nx, ny}, w, h)) continue;
			int ni = idx(nx, ny);
			if (parent[ni] != -1) continue;
			// IGNORE OBSTACLES - BFS will find path through them
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

// BFS to find next action toward target, respecting obstacles
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
			// Avoid boundary cells
			if (isBoundaryCell({nx, ny}, w, h)) continue;
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
	// Reset move history when option is selected
	moveHistory.clear();
	consecutiveRepeatedMoves = 0;
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
	return [this](const Environment2D& e) { 
		Action action;
		
		// If stuck in loop, use BFS instead of smart pathfinding
		if (isStuckInLoop(consecutiveRepeatedMoves)) {
			action = bfsNextAction(e, e.getTargetCell());
		} else {
			// Normal smart pathfinding
			action = smartPathfinding(e, e.getTargetCell());
		}
		
		// Track move history to detect loops
		updateMoveHistory(moveHistory, consecutiveRepeatedMoves, action);
		
		return action;
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
		// CRITICAL: When there are obstacles nearby, return Action::None 
		// This tells the Executor to clear obstacles without moving the robot
		// The Executor.cpp handles the actual obstacle clearing in executeOption()
		if (e.hasObstacleNeighbor()) {
			// Stay in place - let Executor.executeOption() handle clearing
			return Action::None;
		}

		// If carrying, do not attempt to clear; instead move toward target
		if (e.isCarrying()) {
			return smartPathfinding(e, e.getTargetCell());
		}
		
		// No obstacles nearby - move toward target to find more
		return smartPathfinding(e, e.getTargetCell());
	};
}

MoveToObjectOption::MoveToObjectOption() : optionName("MoveToObject") {}

void MoveToObjectOption::onSelect(Environment2D& env) {
	// Store the path to the object when this option is selected
	pathToObject = bfsFullPath(env, env.getObjectCell());
	// Reset move history
	moveHistory.clear();
	consecutiveRepeatedMoves = 0;
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
	return [this](const Environment2D& e) { 
		// Use BFS ignoring obstacles - we'll clear obstacles in Phase 2 via ClearObstacle option
		Action action = bfsNextActionIgnoringObstacles(e, e.getObjectCell());
		
		// Track move history
		updateMoveHistory(moveHistory, consecutiveRepeatedMoves, action);
		
		return action;
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
	// Reset move history for loop detection
	moveHistory.clear();
	consecutiveRepeatedMoves = 0;
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
		Action action = Action::None;
		
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
			if (dx > 0) action = Action::Right;
			else if (dx < 0) action = Action::Left;
			else if (dy > 0) action = Action::Down;
			else if (dy < 0) action = Action::Up;
			else action = Action::None;
			
			// Track move for loop detection
			updateMoveHistory(moveHistory, consecutiveRepeatedMoves, action);
			
			// If stuck in loop while following path, abandon path and use pathfinding
			if (isStuckInLoop(consecutiveRepeatedMoves)) {
				consecutiveRepeatedMoves = 0;
				moveHistory.clear();
				action = bfsNextAction(e, e.getTargetCell());
				updateMoveHistory(moveHistory, consecutiveRepeatedMoves, action);
			}
			
			return action;
		}
		
		// Fallback: use smart pathfinding toward target
		action = smartPathfinding(e, e.getTargetCell());
		
		// Track move for loop detection
		updateMoveHistory(moveHistory, consecutiveRepeatedMoves, action);
		
		// If stuck in loop, try BFS instead
		if (isStuckInLoop(consecutiveRepeatedMoves)) {
			action = bfsNextAction(e, e.getTargetCell());
			consecutiveRepeatedMoves = 0;  // Reset counter when switching strategy
			updateMoveHistory(moveHistory, consecutiveRepeatedMoves, action);
		}
		
		return action;
	};
}

// ReturnToObject option implementations
ReturnToObjectOption::ReturnToObjectOption() : optionName("ReturnToObject") {}

void ReturnToObjectOption::onSelect(Environment2D& env) {
	(void)env;
	// Reset move history when option is selected
	moveHistory.clear();
	consecutiveRepeatedMoves = 0;
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
	return [this](const Environment2D& e) {
		// In Phase 2, ClearObstacle is run before this option, but obstacles might still be present
		// Use obstacle-ignoring BFS to find optimal direction toward object
		// The ClearObstacle option will systematically clear obstacles that block this path
		Action a = bfsNextActionIgnoringObstacles(e, e.getObjectCell());
		
		// Track the move
		updateMoveHistory(moveHistory, consecutiveRepeatedMoves, a);
		
		return a;
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