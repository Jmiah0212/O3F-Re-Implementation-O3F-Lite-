#include <SFML/Graphics.hpp>

#include "Env.hpp"
#include "Agent.hpp"
#include "Visualizer.hpp"
#include "Planner.hpp"
#include "Executor.hpp"

int main() {
	const unsigned int W = 960, H = 600;
	Environment2D env(W, H);
	env.reset(5);

	Visualizer viz(W, H);
	OptionPlanner planner(PlannerConfig{});
	OptionExecutor executor;
	auto options = makeDefaultOptions();

	int successfulEpisodes = 0;
	const int MAX_EPISODES = 200;

	for (int episode = 0; episode < MAX_EPISODES && viz.isOpen(); ++episode) {
		env.reset(5);
		bool done = false;
		float totalReward = 0.f;
		
		while (!done && viz.isOpen()) {
			bool shouldClose = false, resetRequested = false;
			viz.pollEvents(shouldClose, resetRequested);
			if (shouldClose) break;
			if (resetRequested) env.reset(5);

			viz.renderWithOverlay(env, episode, totalReward, (float)successfulEpisodes / (episode + 1));
			
			int option = planner.selectOption(env, options);
			options[option]->onSelect(env);
			float reward = executor.executeOption(env, *options[option], 20);
			planner.updateQ(env, option, reward, env, (int)options.size());
			totalReward += reward;
			
			if (env.isTaskComplete()) {
				done = true;
				successfulEpisodes++;
			}
			
			viz.delay(75); // 50-100ms delay as requested
		}
	}
	return 0;
}
