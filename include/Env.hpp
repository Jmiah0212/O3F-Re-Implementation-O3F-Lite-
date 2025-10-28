#pragma once

#include <vector>
#include <functional>
#include <SFML/Graphics.hpp>

enum class CellType { Empty, Obstacle, Object, Target, Robot };

enum class Action { Up, Down, Left, Right, None };

struct Object2D {
	float radius;
	sf::Vector2f position;
	sf::Color color;
};

struct Robot2D {
	float radius;
	sf::Vector2f position;
	sf::Vector2f velocity;
	float maxSpeed;
};

class Environment2D {
public:
	Environment2D(unsigned int width, unsigned int height);

	void reset(unsigned int numObjects);
	// grid step using primitive action, returns reward
	float step(Action action);
	// continuous physics step for legacy behavior
	void step(float dt);

	// Simple physics-lite interactions
	void setRobotTarget(const sf::Vector2f& target);

	// Accessors
	const Robot2D& getRobot() const { return robot; }
	const std::vector<Object2D>& getObjects() const { return objects; }
	unsigned int getWidth() const { return width; }
	unsigned int getHeight() const { return height; }
	const sf::Vector2f& getTargetRegion() const { return targetRegion; }
	float getTargetRadius() const { return targetRadius; }

	// Grid accessors
	int getGridWidth() const { return gridW; }
	int getGridHeight() const { return gridH; }
	sf::Vector2i getRobotCell() const { return robotCell; }
	sf::Vector2i getTargetCell() const { return targetCell; }
	const std::vector<CellType>& getGrid() const { return grid; }

	// Obstacle helpers
	bool hasObstacleNeighbor() const;
	bool clearAnyAdjacentObstacle();
	bool isObstacle(const sf::Vector2i& cell) const;

	// Rendering
	void render(sf::RenderWindow& window);

private:
	unsigned int width;
	unsigned int height;
	Robot2D robot;
	std::vector<Object2D> objects;
	sf::Vector2f robotTarget;
	sf::Vector2f targetRegion;
	float targetRadius;

	// Grid representation
	int gridW;
	int gridH;
	std::vector<CellType> grid;
	sf::Vector2i robotCell;
	sf::Vector2i targetCell;

	void resolveBoundaries(sf::Vector2f& pos, float radius);
	float computeReward(const sf::Vector2i& prevRobotCell) const;
	int idx(int x, int y) const { return y * gridW + x; }
};
