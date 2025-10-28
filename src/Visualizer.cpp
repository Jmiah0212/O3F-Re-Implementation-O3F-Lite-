#include "Visualizer.hpp"
#include "Env.hpp"

Visualizer::Visualizer(unsigned int width, unsigned int height)
	: window(sf::RenderWindow(sf::VideoMode(width, height), "O3F-Lite Visualizer")) {
	window.setFramerateLimit(60);
}

void Visualizer::pollEvents(bool& shouldClose, bool& resetRequested) {
	shouldClose = false;
	resetRequested = false;
	sf::Event event{};
	while (window.pollEvent(event)) {
		if (event.type == sf::Event::Closed) shouldClose = true;
		if (event.type == sf::Event::KeyPressed) {
			if (event.key.code == sf::Keyboard::R) resetRequested = true;
		}
	}
}

float Visualizer::frame() {
	return clock.restart().asSeconds();
}

void Visualizer::render(Environment2D& env) {
	window.clear(sf::Color(25, 25, 30));
	env.render(window);
	window.display();
}
