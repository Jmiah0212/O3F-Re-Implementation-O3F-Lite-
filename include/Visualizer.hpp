#pragma once

#include <SFML/Graphics.hpp>

class Environment2D;

class Visualizer {
public:
	Visualizer(unsigned int width, unsigned int height);
	bool isOpen() const { return window.isOpen(); }
	void pollEvents(bool& shouldClose, bool& resetRequested);
	void render(Environment2D& env);
	float frame();

private:
	sf::RenderWindow window;
	sf::Clock clock;
};
