#include "Env.hpp"
#include "utils.h"

#include <random>
#include <cmath>

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
}

void Environment2D::reset(unsigned int numObjects) {
	(void)numObjects;
	// Reset grid with random obstacles and one target
	grid.assign(gridW * gridH, CellType::Empty);
	std::mt19937 rng(std::random_device{}());
	std::uniform_int_distribution<int> ox(1, gridW - 2);
	std::uniform_int_distribution<int> oy(1, gridH - 2);

	for (int i = 0; i < gridW * gridH / 8; ++i) {
		sf::Vector2i c{ox(rng), oy(rng)};
		if (c == robotCell || c == targetCell) continue;
		grid[idx(c.x, c.y)] = CellType::Obstacle;
	}
	grid[idx(targetCell.x, targetCell.y)] = CellType::Target;
	grid[idx(robotCell.x, robotCell.y)] = CellType::Robot;

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

float Environment2D::computeReward(const sf::Vector2i& prevRobotCell) const {
	float r = 0.f;
	// +10 for reaching target
	if (robotCell == targetCell) r += 10.f;
	// -0.1 per step
	r -= 0.1f;
	// -5 for collision with obstacle (if attempted)
	if (isObstacle(robotCell)) r -= 5.f;
	// +1 when distance to target decreases
	auto dprev = std::abs(prevRobotCell.x - targetCell.x) + std::abs(prevRobotCell.y - targetCell.y);
	auto dnow = std::abs(robotCell.x - targetCell.x) + std::abs(robotCell.y - targetCell.y);
	if (dnow < dprev) r += 1.f;
	return r;
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
	// clamp into bounds
	next.x = std::max(0, std::min(gridW - 1, next.x));
	next.y = std::max(0, std::min(gridH - 1, next.y));
	// move if not obstacle
	if (!isObstacle(next)) {
		robotCell = next;
	}
	// write back robot cell
	if (grid[idx(targetCell.x, targetCell.y)] != CellType::Robot) {
		grid[idx(targetCell.x, targetCell.y)] = CellType::Target;
	}
	grid[idx(robotCell.x, robotCell.y)] = CellType::Robot;

	// sync continuous visualization positions
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
	// Legacy continuous: keep for smooth visuals while grid drives logic
	sf::Vector2f toTarget = robotTarget - robot.position;
	sf::Vector2f dir = normalize(toTarget);
	robot.velocity = dir * robot.maxSpeed;
	robot.position += robot.velocity * dt;
	resolveBoundaries(robot.position, robot.radius);
}

void Environment2D::render(sf::RenderWindow& window) {
	// draw grid
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

	// continuous overlays (optional) already visible via cell colors
}
