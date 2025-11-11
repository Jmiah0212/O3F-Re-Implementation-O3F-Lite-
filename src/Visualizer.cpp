#include "Visualizer.hpp"
#include "Env.hpp"

Visualizer::Visualizer(unsigned int width, unsigned int height)
	: window(sf::RenderWindow(sf::VideoMode(width, height), "O3F-Lite Visualizer")), fontLoaded(false) {
	window.setFramerateLimit(60);
	// Try to load a default font (optional)
	fontLoaded = font.loadFromFile("C:/Windows/Fonts/arial.ttf") || 
	             font.loadFromFile("/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf") ||
	             font.loadFromFile("/System/Library/Fonts/Arial.ttf");
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

void Visualizer::delay(int milliseconds) {
	sf::sleep(sf::milliseconds(milliseconds));
}

void Visualizer::drawText(const std::string& text, float x, float y, sf::Color color) {
	if (!fontLoaded) return;
	sf::Text textObj;
	textObj.setFont(font);
	textObj.setString(text);
	textObj.setCharacterSize(16);
	textObj.setFillColor(color);
	textObj.setPosition(x, y);
	window.draw(textObj);
}

void Visualizer::render(Environment2D& env) {
	window.clear(sf::Color(25, 25, 30));
	env.render(window);
	window.display();
}

void Visualizer::renderWithOverlay(Environment2D& env, int episode, float totalReward, float successRate) {
	window.clear(sf::Color(25, 25, 30));
	env.render(window);
	
	// Draw text overlays
	drawText("Episode: " + std::to_string(episode), 10, 10, sf::Color::White);
	drawText("Reward: " + std::to_string((int)totalReward), 10, 30, sf::Color::Yellow);
	drawText("Success Rate: " + std::to_string((int)(successRate * 100)) + "%", 10, 50, sf::Color::Green);
	
	window.display();
}
