#include "Env.hpp"
#include "utils.h"

#include <random>
#include <cmath>
#include <iostream>

static float length(const sf::Vector2f& v) {
	return std::sqrt(v.x * v.x + v.y * v.y);
}

static sf::Vector2f normalize(const sf::Vector2f& v) {
	float len = length(v);
	if (len <= 1e-5f) return {0.f, 0.f};
	return {v.x / len, v.y / len};
}

Environment2D::Environment2D(unsigned int width_, unsigned int height_)
	: width(width_), height(height_), robotTarget(width_ * 0.5f, height_ * 0.5f), targetRegion(width_ * 0.8f, height_ * 0.5f), targetRadius(30.f) {
	robot.radius = 12.f;
	robot.position = {width * 0.2f, height * 0.5f};
	robot.velocity = {0.f, 0.f};
	robot.maxSpeed = 150.f;
	gridW = GRID_WIDTH;
	gridH = GRID_HEIGHT;
	grid.assign(gridW * gridH, CellType::Empty);
	robotCell = {1, gridH / 2};
	targetCell = {gridW - 2, gridH / 2};
	objectCell = {gridW / 3, gridH / 2};
	carrying = false;
}

void Environment2D::reset(unsigned int numObjects) {
	(void)numObjects;
	// Reset robot position (always start at left side)
	robotCell = {1, gridH / 2};
	
	// Randomize target position (but keep it on the right side)
	grid.assign(gridW * gridH, CellType::Empty);
	std::mt19937 rng(std::random_device{}());
	std::uniform_int_distribution<int> targetX(gridW - 5, gridW - 2); // Right side
	std::uniform_int_distribution<int> targetY(2, gridH - 3); // Avoid edges
	targetCell = {targetX(rng), targetY(rng)};

	// Place a single object somewhere else (left/middle side)
	std::uniform_int_distribution<int> objX(2, std::max(2, gridW/2 - 2));
	std::uniform_int_distribution<int> objY(2, gridH - 3);
	objectCell = {objX(rng), objY(rng)};
	carrying = false;
	
	// Generate obstacles
	std::uniform_int_distribution<int> ox(1, gridW - 2);
	std::uniform_int_distribution<int> oy(1, gridH - 2);

	for (int i = 0; i < gridW * gridH / 2; ++i) {
		sf::Vector2i c{ox(rng), oy(rng)};
		if (c == robotCell || c == targetCell) continue;
		// avoid placing obstacles on the object cell
		if (c == objectCell) continue;
		grid[idx(c.x, c.y)] = CellType::Obstacle;
	}
	grid[idx(targetCell.x, targetCell.y)] = CellType::Target;
	grid[idx(objectCell.x, objectCell.y)] = CellType::Object;
	grid[idx(robotCell.x, robotCell.y)] = CellType::Robot;

	// Debug: print robot and target positions
	std::cout << "Reset: Robot at (" << robotCell.x << "," << robotCell.y << "), Target at (" << targetCell.x << "," << targetCell.y << ")" << std::endl;

	// sync continuous space visualization targets
	robot.position = {robotCell.x * CELL_SIZE + CELL_SIZE * 0.5f, robotCell.y * CELL_SIZE + CELL_SIZE * 0.5f};
	robot.velocity = {0.f, 0.f};
	robotTarget = robot.position;
	targetRegion = {targetCell.x * CELL_SIZE + CELL_SIZE * 0.5f, targetCell.y * CELL_SIZE + CELL_SIZE * 0.5f};
	objects.clear();
}

bool Environment2D::isObstacle(const sf::Vector2i& cell) const {
	if (cell.x < 0 || cell.x >= gridW || cell.y < 0 || cell.y >= gridH) return true;
	CellType t = grid[idx(cell.x, cell.y)];
	return t == CellType::Obstacle;
}

bool Environment2D::hasObstacleNeighbor() const {
	static const int dx[4] = {1, -1, 0, 0};
	static const int dy[4] = {0, 0, 1, -1};
	for (int k = 0; k < 4; ++k) {
		int nx = robotCell.x + dx[k];
		int ny = robotCell.y + dy[k];
		if (nx >= 0 && nx < gridW && ny >= 0 && ny < gridH) {
			if (grid[idx(nx, ny)] == CellType::Obstacle) return true;
		}
	}
	return false;
}

bool Environment2D::clearAnyAdjacentObstacle() {
	static const int dx[4] = {1, -1, 0, 0};
	static const int dy[4] = {0, 0, 1, -1};
	// cannot clear obstacles while carrying an object
	if (carrying) return false;

	for (int k = 0; k < 4; ++k) {
		int nx = robotCell.x + dx[k];
		int ny = robotCell.y + dy[k];
		if (nx >= 0 && nx < gridW && ny >= 0 && ny < gridH) {
			if (grid[idx(nx, ny)] == CellType::Obstacle) {
				grid[idx(nx, ny)] = CellType::Empty;
				std::cout << "Env: cleared obstacle at (" << nx << "," << ny << ")" << std::endl;
				return true;
			}
		}
	}
	return false;
}

float Environment2D::computeReward(const sf::Vector2i& prevRobotCell) const {
	float r = 0.f;
	
	// 1. Success reward for delivering the carried object to target (large bonus)
	if (carrying && robotCell == targetCell) {
		return 50.f; // Big reward to end episode
	}
	
	// 2. Small time penalty per step (encourage efficiency but not too harsh)
	r -= 0.1f;
	
	// 3. Obstacle collision penalty
	if (isObstacle(robotCell)) {
		return -5.f; // Penalty but not devastating
	}
	
	// 4. Distance-based reward (the main signal)
	float prevDist = std::abs(prevRobotCell.x - targetCell.x) + 
	                 std::abs(prevRobotCell.y - targetCell.y);
	float currDist = std::abs(robotCell.x - targetCell.x) + 
	                 std::abs(robotCell.y - targetCell.y);
	
	float distChange = prevDist - currDist;
	
	// Clear signal for progress
	if (distChange > 0) {
		r += 1.5f; // Good reward for getting closer
	} else if (distChange < 0) {
		r -= 1.0f; // Moderate penalty for moving away
	} else {
		// Stayed same distance - small penalty to discourage
		r -= 0.3f;
	}
	
	// 5. Proximity bonus (encourage getting close to target)
	if (currDist <= 3.0f) {
		r += 0.5f;
	}
	
	return r;
}

// New method: compute A* heuristic cost
float Environment2D::computeHeuristicCost(const sf::Vector2i& from, const sf::Vector2i& to) const {
	// Manhattan distance as heuristic
	return std::abs(from.x - to.x) + std::abs(from.y - to.y);
}

// New method: check if clearing obstacle is beneficial
bool Environment2D::shouldClearObstacle(const sf::Vector2i& obstaclePos) const {
	// Calculate cost of clearing vs going around
	float clearCost = 2.0f; // Base cost for clearing
	
	// Check if obstacle blocks a direct path to target
	sf::Vector2i toTarget = targetCell - robotCell;
	sf::Vector2i toObstacle = obstaclePos - robotCell;
	
	// If obstacle is in the direct path to target, clearing might be worth it
	if ((toTarget.x > 0 && toObstacle.x > 0) || (toTarget.x < 0 && toObstacle.x < 0)) {
		if ((toTarget.y > 0 && toObstacle.y > 0) || (toTarget.y < 0 && toObstacle.y < 0)) {
			// Obstacle is roughly in the direction of target
			float directDistance = computeHeuristicCost(robotCell, targetCell);
			float aroundDistance = computeHeuristicCost(robotCell, obstaclePos) + computeHeuristicCost(obstaclePos, targetCell);
			
			// If going around is much longer, clearing might be worth it
			if (aroundDistance > directDistance + clearCost) {
				return true;
			}
		}
	}
	
	return false;
}

bool Environment2D::shouldClearObstacleToward(const sf::Vector2i& obstaclePos, const sf::Vector2i& dest) const {
	// Similar to shouldClearObstacle but compares direction to an explicit destination
	float clearCost = 2.0f; // Base cost for clearing

	sf::Vector2i toDest = dest - robotCell;
	sf::Vector2i toObstacle = obstaclePos - robotCell;

	if ((toDest.x > 0 && toObstacle.x > 0) || (toDest.x < 0 && toObstacle.x < 0)) {
		if ((toDest.y > 0 && toObstacle.y > 0) || (toDest.y < 0 && toObstacle.y < 0)) {
			float directDistance = computeHeuristicCost(robotCell, dest);
			float aroundDistance = computeHeuristicCost(robotCell, obstaclePos) + computeHeuristicCost(obstaclePos, dest);
			if (aroundDistance > directDistance + clearCost) {
				return true;
			}
		}
	}
	return false;
}

float Environment2D::step(Action action) {
	sf::Vector2i prev = robotCell;
	// clear previous robot cell
	if (robotCell.x >= 0 && robotCell.x < gridW && robotCell.y >= 0 && robotCell.y < gridH) {
		if (grid[idx(robotCell.x, robotCell.y)] == CellType::Robot) grid[idx(robotCell.x, robotCell.y)] = CellType::Empty;
	}
	sf::Vector2i next = robotCell;
	switch (action) {
		case Action::Up: next.y -= 1; break;
		case Action::Down: next.y += 1; break;
		case Action::Left: next.x -= 1; break;
		case Action::Right: next.x += 1; break;
		default: break;
	}
	next.x = std::max(0, std::min(gridW - 1, next.x));
	next.y = std::max(0, std::min(gridH - 1, next.y));
	if (!isObstacle(next)) {
		robotCell = next;
	}
	// If robot moved onto the object cell and not already carrying, pick it up
	if (!carrying && robotCell == objectCell) {
		carrying = true;
		// remove object from grid
		grid[idx(objectCell.x, objectCell.y)] = CellType::Empty;
		std::cout << "Env: robot picked up object at (" << objectCell.x << "," << objectCell.y << ")" << std::endl;
	}
	if (grid[idx(targetCell.x, targetCell.y)] != CellType::Robot) {
		grid[idx(targetCell.x, targetCell.y)] = CellType::Target;
	}
	grid[idx(robotCell.x, robotCell.y)] = CellType::Robot;

	robot.position = {robotCell.x * CELL_SIZE + CELL_SIZE * 0.5f, robotCell.y * CELL_SIZE + CELL_SIZE * 0.5f};
	robotTarget = robot.position;
	return computeReward(prev);
}

void Environment2D::setRobotTarget(const sf::Vector2f& target) {
	robotTarget = target;
}

void Environment2D::resolveBoundaries(sf::Vector2f& pos, float r) {
	if (pos.x < r) pos.x = r;
	if (pos.x > width - r) pos.x = width - r;
	if (pos.y < r) pos.y = r;
	if (pos.y > height - r) pos.y = height - r;
}

void Environment2D::step(float dt) {
	sf::Vector2f toTarget = robotTarget - robot.position;
	sf::Vector2f dir = normalize(toTarget);
	robot.velocity = dir * robot.maxSpeed;
	robot.position += robot.velocity * dt;
	resolveBoundaries(robot.position, robot.radius);
}

void Environment2D::render(sf::RenderWindow& window) {
	sf::RectangleShape cellShape({CELL_SIZE - 1.f, CELL_SIZE - 1.f});
	for (int y = 0; y < gridH; ++y) {
		for (int x = 0; x < gridW; ++x) {
			CellType t = grid[idx(x, y)];
			sf::Color c(40, 40, 45);
			if (t == CellType::Obstacle) c = sf::Color(120, 60, 60);
			if (t == CellType::Target) c = sf::Color(60, 120, 60);
			if (t == CellType::Object) c = sf::Color(200, 200, 80);
			if (t == CellType::Robot) c = sf::Color(80, 160, 220);
			cellShape.setFillColor(c);
			cellShape.setPosition(x * CELL_SIZE, y * CELL_SIZE);
			window.draw(cellShape);
		}
	}
}
