#pragma once

#include <SFML/Graphics.hpp>
#include <string>

class Environment2D;

class Visualizer {
public:
	Visualizer(unsigned int width, unsigned int height);
	bool isOpen() const { return window.isOpen(); }
	void pollEvents(bool& shouldClose, bool& resetRequested);
	void render(Environment2D& env);
	void renderWithOverlay(Environment2D& env, int episode, float totalReward, float successRate);
	float frame();
	void delay(int milliseconds);

private:
	sf::RenderWindow window;
	sf::Clock clock;
	sf::Font font;
	bool fontLoaded;
	
	void drawText(const std::string& text, float x, float y, sf::Color color = sf::Color::White);
};
