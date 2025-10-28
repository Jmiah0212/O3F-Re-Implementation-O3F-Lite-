#pragma once

#include <vector>
#include <SFML/Graphics.hpp>

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

	void resolveBoundaries(sf::Vector2f& pos, float radius);
};
