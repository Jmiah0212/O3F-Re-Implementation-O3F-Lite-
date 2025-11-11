#include <SFML/Graphics.hpp>
#include <iostream>
#include <fstream>
#include <iomanip>
#include <ctime>
#include <algorithm>

#include "Env.hpp"
#include "Agent.hpp"
#include "Visualizer.hpp"
#include "Planner.hpp"
#include "Executor.hpp"
#include "Option.hpp"

int main() {
	const unsigned int W = 960, H = 600;
	Environment2D env(W, H);
	env.reset(5);

	Visualizer viz(W, H);
	// Configure planner with explicit hyperparameters so we can decay epsilon
	PlannerConfig plannerCfg;
	plannerCfg.alpha = 0.1f;
	plannerCfg.gamma = 0.95f;
	plannerCfg.epsilon = 1.0f;      // start fully exploratory
	plannerCfg.epsilonDecay = 0.995f;
	plannerCfg.epsilonMin = 0.05f;
	OptionPlanner planner(plannerCfg);

	// Create CSV log for training results
	std::time_t now = std::time(nullptr);
	std::tm* localTime = std::localtime(&now);
	char filename[128];
	std::strftime(filename, sizeof(filename), "training_log_%Y%m%d_%H%M.csv", localTime);
	std::ofstream csv(filename);
	if (csv.is_open()) {
		csv << "episode,total_reward,success,steps,options_used,epsilon\n";
	} else {
		std::cout << "Warning: could not open training log file '" << filename << "' for writing." << std::endl;
	}
	OptionExecutor executor;
	auto options = makeDefaultOptions();

	int successfulEpisodes = 0;
	const int MAX_EPISODES = 200;
	float cumulativeReward = 0.f;

	for (int episode = 0; episode < MAX_EPISODES && viz.isOpen(); ++episode) {
		env.reset(5);
		bool done = false;
		float episodeReward = 0.f;
		int optionCount = 0;
		const int MAX_OPTIONS_PER_EPISODE = 50; // Force episode to end after 50 options
		
		int stepsWithoutProgress = 0;
		int lastDistance = std::abs(env.getRobotCell().x - env.getTargetCell().x) + 
		                   std::abs(env.getRobotCell().y - env.getTargetCell().y);
		
		while (!done && viz.isOpen() && optionCount < MAX_OPTIONS_PER_EPISODE) {
			bool shouldClose = false, resetRequested = false;
			viz.pollEvents(shouldClose, resetRequested);
			if (shouldClose) break;
			if (resetRequested) env.reset(5);

			int option = planner.selectOption(env, options);
			options[option]->onSelect(env);
			float reward = executor.executeOption(env, *options[option], 3); // Reduced from 5 to 3
			planner.updateQ(env, option, reward, env, (int)options.size());
			episodeReward += reward;
			cumulativeReward += reward;
			
			// Debug: print reward info
			if (episode < 3) { // Only print first 3 episodes
				std::cout << "Episode " << episode << ", Option: " << options[option]->name() 
				          << ", Reward: " << reward << ", Total: " << episodeReward 
				          << ", Robot at (" << env.getRobotCell().x << "," << env.getRobotCell().y << ")" << std::endl;
			}
			
			int currentDistance = std::abs(env.getRobotCell().x - env.getTargetCell().x) + 
			                     std::abs(env.getRobotCell().y - env.getTargetCell().y);
			
			if (currentDistance >= lastDistance) {
				stepsWithoutProgress++;
			} else {
				stepsWithoutProgress = 0;
			}
			lastDistance = currentDistance;
			
			// Terminate if stuck for too long
			if (stepsWithoutProgress > 15) {
				episodeReward -= 20.0f; // Penalty for getting stuck
				std::cout << "Episode " << episode << " terminated early - stuck without progress" << std::endl;
				break;
			}
			
			viz.renderWithOverlay(env, episode, episodeReward, (float)successfulEpisodes / (episode + 1));
			
			if (env.isTaskComplete()) {
				done = true;
				successfulEpisodes++;
				std::cout << "Episode " << episode << " SUCCESS! Reward: " << episodeReward << std::endl;
			}
			
			optionCount++;
			viz.delay(50); // Reduced delay for faster decisions
		}
		
		// Print episode summary
		if (episode % 10 == 0) {
			std::cout << "Episode " << episode << " complete. Reward: " << episodeReward 
					  << ", Success rate: " << (float)successfulEpisodes / (episode + 1) * 100 << "%" << std::endl;
		}

		// Log episode to CSV (approximate steps as options_used * maxStepsPerOption(=3) here)
		bool success = env.isTaskComplete();
		int optionsUsed = optionCount;
		int stepsTaken = optionsUsed * 3;
		if (csv.is_open()) {
			csv << episode << "," 
				<< std::fixed << std::setprecision(4) << episodeReward << "," 
				<< (success ? 1 : 0) << "," 
				<< stepsTaken << "," 
				<< optionsUsed << "," 
				<< planner.getConfig().epsilon << "\n";
		}

		// Epsilon decay after each episode
		planner.getConfig().epsilon = std::max(
			planner.getConfig().epsilon * planner.getConfig().epsilonDecay,
			planner.getConfig().epsilonMin);
	}
	
	std::cout << "\nTraining complete!" << std::endl;
	std::cout << "Total successful episodes: " << successfulEpisodes << " / " << MAX_EPISODES << std::endl;
	std::cout << "Success rate: " << (float)successfulEpisodes / MAX_EPISODES * 100 << "%" << std::endl;
	
	return 0;
}
