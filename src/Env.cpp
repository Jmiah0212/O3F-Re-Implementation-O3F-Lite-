#include "Env.hpp"

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
}

void Environment2D::reset(unsigned int numObjects) {
	objects.clear();
	std::mt19937 rng(std::random_device{}());
	std::uniform_real_distribution<float> xdist(width * 0.3f, width * 0.7f);
	std::uniform_real_distribution<float> ydist(height * 0.2f, height * 0.8f);

	for (unsigned int i = 0; i < numObjects; ++i) {
		Object2D o;
		o.radius = 12.f;
		o.position = {xdist(rng), ydist(rng)};
		o.color = sf::Color(200, 200, 80);
		objects.push_back(o);
	}

	robot.position = {width * 0.2f, height * 0.5f};
	robot.velocity = {0.f, 0.f};
	robotTarget = robot.position;
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
	// Simple seek behavior toward robotTarget
	sf::Vector2f toTarget = robotTarget - robot.position;
	sf::Vector2f dir = normalize(toTarget);
	robot.velocity = dir * robot.maxSpeed;
	robot.position += robot.velocity * dt;
	resolveBoundaries(robot.position, robot.radius);

	// Push interaction: if robot overlaps an object, nudge it along robot velocity
	for (auto& obj : objects) {
		sf::Vector2f diff = obj.position - robot.position;
		float dist = length(diff);
		float minDist = obj.radius + robot.radius;
		if (dist < minDist) {
			obj.position += normalize(diff) * (minDist - dist + 0.5f);
			obj.position += normalize(robot.velocity) * 0.1f;
			resolveBoundaries(obj.position, obj.radius);
		}
	}
}

void Environment2D::render(sf::RenderWindow& window) {
	// Target region
	sf::CircleShape targetCircle(targetRadius);
	targetCircle.setFillColor(sf::Color(50, 200, 50, 40));
	targetCircle.setOutlineColor(sf::Color(50, 200, 50));
	targetCircle.setOutlineThickness(2.f);
	targetCircle.setOrigin(targetRadius, targetRadius);
	targetCircle.setPosition(targetRegion);
	window.draw(targetCircle);

	// Objects
	for (const auto& obj : objects) {
		sf::CircleShape shape(obj.radius);
		shape.setOrigin(obj.radius, obj.radius);
		shape.setFillColor(obj.color);
		shape.setPosition(obj.position);
		window.draw(shape);
	}

	// Robot
	sf::CircleShape rshape(robot.radius);
	rshape.setOrigin(robot.radius, robot.radius);
	rshape.setFillColor(sf::Color(80, 160, 220));
	rshape.setPosition(robot.position);
	window.draw(rshape);

	// Robot target
	sf::CircleShape t(3.f);
	t.setOrigin(3.f, 3.f);
	t.setFillColor(sf::Color::Red);
	t.setPosition(robotTarget);
	window.draw(t);
}
