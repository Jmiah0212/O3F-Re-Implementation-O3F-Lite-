#include <SFML/Graphics.hpp>

#include "Env.hpp"
#include "Agent.hpp"
#include "Visualizer.hpp"

int main() {
	const unsigned int W = 960, H = 600;
	Environment2D env(W, H);
	env.reset(5);

	Visualizer viz(W, H);
	Agent agent({});
	agent.initialize();

	while (viz.isOpen()) {
		(void)agent.runEpisode(env, viz, 6000); // run long episode chunk
	}
	return 0;
}
