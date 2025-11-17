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

int main(int argc, char** argv) {
	const unsigned int W = 960, H = 600;
	// Parse simple CLI args
	std::string loadQPath;
	int saveQInterval = 0; // episodes; 0 = disabled
	for (int i = 1; i < argc; ++i) {
		std::string a = argv[i];
		if (a == "--load-q" && i + 1 < argc) {
			loadQPath = argv[++i];
		} else if (a == "--save-q-interval" && i + 1 < argc) {
			saveQInterval = std::stoi(argv[++i]);
		}
	}

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

	// Optionally load a Q-table before training
	if (!loadQPath.empty()) {
		if (planner.loadQTable(loadQPath)) {
			std::cout << "Loaded Q-table from " << loadQPath << std::endl;
		} else {
			std::cout << "Failed to load Q-table from " << loadQPath << std::endl;
		}
	}

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
		
		// State machine: 0=ClearObstacles, 1=MoveToTarget, 2=ReturnToObject, 3=MoveObjectToTarget
		int currentPhase = 0;
		const char* phaseNames[] = {"ClearObstacle", "MoveToTarget", "ReturnToObject", "MoveObjectToTarget"};
		
		int stepsWithoutProgress = 0;
		int lastDistance = std::abs(env.getRobotCell().x - env.getTargetCell().x) + 
		                   std::abs(env.getRobotCell().y - env.getTargetCell().y);
		
		while (!done && viz.isOpen() && optionCount < MAX_OPTIONS_PER_EPISODE) {
			bool shouldClose = false, resetRequested = false;
			viz.pollEvents(shouldClose, resetRequested);
			if (shouldClose) break;
			if (resetRequested) {
				env.reset(5);
				currentPhase = 0;
			}

			// Check phase transition conditions FIRST, before executing any option
			// This ensures we immediately transition when conditions are met
			if (currentPhase == 0) {
				// ClearObstacles phase: transition when no obstacles nearby
				if (!env.hasObstacleNeighbor()) {
					currentPhase = 1;
				}
			} else if (currentPhase == 1) {
				// MoveToTarget phase: transition when at target
				if (env.getRobotCell() == env.getTargetCell()) {
					currentPhase = 2;
					std::cout << "Episode " << episode << " - Reached target! Transitioning to ReturnToObject phase." << std::endl;
				}
			} else if (currentPhase == 2) {
				// ReturnToObject phase: transition when carrying object
				if (env.isCarrying()) {
					currentPhase = 3;
					std::cout << "Episode " << episode << " - Picked up object! Transitioning to MoveObjectToTarget phase." << std::endl;
				}
			}

			// Determine which option to execute based on current phase
			int option = currentPhase;
			
			// Special handling for Phase 2 (ReturnToObject):
			// If we're at the object cell, we should have picked it up (next loop transition check)
			// If we're adjacent to object, move to it instead of clearing obstacles
			if (currentPhase == 2) {
				// Use the option policies to decide: prefer ReturnToObject if it can make progress.
				// Only run ClearObstacle when ReturnToObject cannot move AND ClearObstacle is
				// ready to clear (i.e. its policy returns Action::None indicating an adjacent
				// strategic obstacle).
				auto returnPolicy = options[2]->policy();
				auto clearPolicy = options[0]->policy();
				Action returnAction = returnPolicy(env);
				Action clearAction = clearPolicy(env);

				if (returnAction != Action::None) {
					option = 2; // can make progress toward object
				} else if (!env.isCarrying() && clearAction == Action::None) {
					option = 0; // Clear adjacent strategic obstacle
				} else {
					// fallback: try ReturnToObject (may cause the option's internal logic to
					// approach an obstacle) so we avoid flip-flopping with ClearObstacle.
					option = 2;
				}
			}
			// For phases 0 and 1, clear obstacles opportunistically
			else if (!env.isCarrying() && env.hasObstacleNeighbor() && currentPhase != 3) {
				option = 0; // ClearObstacle option
			}
			
			// Store previous state for Q-learning
			Environment2D prevState = env;
			
			options[option]->onSelect(env);
			float reward = executor.executeOption(env, *options[option], 3);
			planner.updateQ(prevState, option, reward, env, (int)options.size());
			episodeReward += reward;
			cumulativeReward += reward;
			
			// Debug: print reward info
			if (episode < 3) { // Only print first 3 episodes
				std::cout << "Episode " << episode << ", Phase: " << phaseNames[currentPhase] 
				          << " (Option: " << phaseNames[option] << ")"
				          << ", Reward: " << reward << ", Total: " << episodeReward 
				          << ", Robot at (" << env.getRobotCell().x << "," << env.getRobotCell().y << ")";
				if (currentPhase == 3) {
					std::cout << " [Following stored path]";
				}
				std::cout << std::endl;
			}
			
			// Check again after execution if phase should transition
			if (currentPhase == 0) {
				if (!env.hasObstacleNeighbor()) {
					currentPhase = 1;
				}
			} else if (currentPhase == 1) {
				if (env.getRobotCell() == env.getTargetCell()) {
					currentPhase = 2;
					std::cout << "Episode " << episode << " - Reached target! Transitioning to ReturnToObject phase." << std::endl;
				}
			} else if (currentPhase == 2) {
				if (env.isCarrying()) {
					currentPhase = 3;
					std::cout << "Episode " << episode << " - Picked up object! Transitioning to MoveObjectToTarget phase." << std::endl;
					
					// Pass the path taken to reach the object to MoveObjectToTargetOption
					MoveToObjectOption* moveToObjOpt = dynamic_cast<MoveToObjectOption*>(options[2].get());
					MoveObjectToTargetOption* moveObjToTargetOpt = dynamic_cast<MoveObjectToTargetOption*>(options[3].get());
					if (moveToObjOpt && moveObjToTargetOpt) {
						auto path = moveToObjOpt->getPathToObject();
						std::cout << "  Path size: " << path.size() << " waypoints" << std::endl;
						moveObjToTargetOpt->setReturnPath(path);
						std::cout << "  Path set for return journey" << std::endl;
					}
				}
			} else if (currentPhase == 3) {
				// MoveObjectToTarget phase: check if task complete (at target with object)
				if (env.isTaskComplete()) {
					// Task complete!
					done = true;
					successfulEpisodes++;
					reward += 50.0f; // Big reward for success
					episodeReward += 50.0f;
					std::cout << "Episode " << episode << " SUCCESS! Reward: " << episodeReward << std::endl;
				}
			}
			
			int currentDistance = std::abs(env.getRobotCell().x - env.getTargetCell().x) + 
			                     std::abs(env.getRobotCell().y - env.getTargetCell().y);
			
			// In phase 3 (MoveObjectToTarget with stored path), allow backward steps
			// Only track progress in other phases
			if (currentPhase != 3) {
				if (currentDistance >= lastDistance) {
					stepsWithoutProgress++;
				} else {
					stepsWithoutProgress = 0;
				}
				lastDistance = currentDistance;
				
				// Terminate if stuck for too long (only in phases 0-2)
				if (stepsWithoutProgress > 15) {
					episodeReward -= 20.0f; // Penalty for getting stuck
					std::cout << "Episode " << episode << " terminated early - stuck without progress" << std::endl;
					break;
				}
			} else {
				// In phase 3, just track current distance without penalizing backward steps
				lastDistance = currentDistance;
			}
			
			viz.renderWithOverlay(env, episode, episodeReward, (float)successfulEpisodes / (episode + 1));
			
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

		// Periodically save Q-table if requested
		if (saveQInterval > 0 && episode % saveQInterval == 0) {
			std::time_t now2 = std::time(nullptr);
			std::tm* lt = std::localtime(&now2);
			char ts[64];
			std::strftime(ts, sizeof(ts), "%Y%m%d_%H%M", lt);
			std::string qfilename = std::string("qtable_") + ts + "_ep" + std::to_string(episode) + ".csv";
			if (planner.saveQTable(qfilename)) {
				std::cout << "Saved Q-table to " << qfilename << std::endl;
			} else {
				std::cout << "Failed to save Q-table to " << qfilename << std::endl;
			}
		}
	}
	
	// Save final Q-table
	{
		std::time_t now3 = std::time(nullptr);
		std::tm* lt3 = std::localtime(&now3);
		char ts3[64];
		std::strftime(ts3, sizeof(ts3), "%Y%m%d_%H%M", lt3);
		std::string finalQ = std::string("qtable_final_") + ts3 + ".csv";
		if (planner.saveQTable(finalQ)) std::cout << "Saved final Q-table to " << finalQ << std::endl;
		else std::cout << "Failed to save final Q-table to " << finalQ << std::endl;
	}

	std::cout << "\nTraining complete!" << std::endl;
	std::cout << "Total successful episodes: " << successfulEpisodes << " / " << MAX_EPISODES << std::endl;
	std::cout << "Success rate: " << (float)successfulEpisodes / MAX_EPISODES * 100 << "%" << std::endl;
	
	return 0;
}