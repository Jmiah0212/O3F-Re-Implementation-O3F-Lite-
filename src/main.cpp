#include <SFML/Graphics.hpp>

#include "Env.hpp"
#include "Option.hpp"
#include "Planner.hpp"
#include "Executor.hpp"

#include <memory>
#include <vector>

static float rewardProximity(const Environment2D& env) {
	// small shaping: encourage objects inside target region
	float r = 0.f;
	for (const auto& o : env.getObjects()) {
		sf::Vector2f d = o.position - env.getTargetRegion();
		float dist2 = d.x * d.x + d.y * d.y;
		float rad2 = env.getTargetRadius() * env.getTargetRadius();
		if (dist2 <= rad2) r += 1.0f;
	}
	return r;
}

int main() {
	const unsigned int W = 960, H = 600;
	sf::RenderWindow window(sf::VideoMode(W, H), "O3F-Lite (CSCE4430) - SFML");
	window.setFramerateLimit(60);

	Environment2D env(W, H);
	env.reset(5);
	auto options = makeDefaultOptions();
	OptionExecutor executor;
	OptionPlanner planner(PlannerConfig{});

	int currentOption = -1;
	float timeSinceSelect = 0.f;
	const float optionDuration = 2.0f; // seconds per option for demo

	sf::Font font;
	// Optional: if a default font is not available, text will be skipped

	sf::Clock clock;
	while (window.isOpen()) {
		sf::Event event{};
		while (window.pollEvent(event)) {
			if (event.type == sf::Event::Closed) window.close();
			if (event.type == sf::Event::KeyPressed) {
				if (event.key.code == sf::Keyboard::R) env.reset(5);
			}
		}

		float dt = clock.restart().asSeconds();

		// simple episodic training sketch: select option every few seconds
		timeSinceSelect += dt;
		if (currentOption < 0 || timeSinceSelect >= optionDuration || options[currentOption]->isComplete(env)) {
			Environment2D prev = env; // shallow copy of simple PODs is fine here
			int action = planner.selectAction(env, options);
			options[action]->onSelect(env);
			currentOption = action;
			// reward on switch (previous outcome)
			float reward = rewardProximity(env) - 0.01f; // time penalty
			planner.update(prev, action, reward, env, (int)options.size());
			timeSinceSelect = 0.f;
		}

		// low-level control advances in env.step()
		executor.tick(env, dt);
		env.step(dt);

		window.clear(sf::Color(25, 25, 30));
		env.render(window);

		// UI text - current option
		// (Skipping font load for portability)
		window.display();
	}

	return 0;
}
