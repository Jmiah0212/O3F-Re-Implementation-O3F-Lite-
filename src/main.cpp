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
	std::string loadQPath;
	int saveQInterval = 0;
	for (int i = 1; i < argc; ++i) {
		std::string a = argv[i];
		if (a == "--load-q" && i + 1 < argc) {
			loadQPath = argv[++i];
		} else if (a == "--save-q-interval" && i + 1 < argc) {
			saveQInterval = std::stoi(argv[++i]);
		}
	}

	Environment2D env(W, H);
	env.setEpisodeNumber(0);
	env.reset(5);

	Visualizer viz(W, H);
	// configure planner with explicit hyperparameters so we can decay epsilon
	PlannerConfig plannerCfg;
	plannerCfg.alpha = 0.1f;
	plannerCfg.gamma = 0.95f;
	plannerCfg.epsilon = 1.0f;
	plannerCfg.epsilonDecay = 0.995f;
	plannerCfg.epsilonMin = 0.05f;
	OptionPlanner planner(plannerCfg);

	if (!loadQPath.empty()) {
		if (planner.loadQTable(loadQPath)) {
			std::cout << "Loaded Q-table from " << loadQPath << std::endl;
		} else {
			std::cout << "Failed to load Q-table from " << loadQPath << std::endl;
		}
	}

	// create csv log for training results
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
	env.setEpisodeNumber(episode);
	env.reset(5);
		bool done = false;
		float episodeReward = 0.f;
		int optionCount = 0;
		const int MAX_OPTIONS_PER_EPISODE = 150; // allow up to 150 options - more time for complex navigation
		
		// state machine: 0=ClearObstacles, 1=MoveToTarget, 2=ReturnToObject, 3=MoveObjectToTarget
		int currentPhase = 0;
		const char* phaseNames[] = {"ClearObstacle", "MoveToTarget", "ReturnToObject", "MoveObjectToTarget"};
		
		// track whether the robot has reached the target at least once this episode
		bool reachedTargetOnce = false;

		int stepsWithoutProgress = 0;
		int lastDistance = std::abs(env.getRobotCell().x - env.getTargetCell().x) + 
		                   std::abs(env.getRobotCell().y - env.getTargetCell().y);
		
		bool phaseJustChanged = false;  // track if phase just changed this iteration
		
		while (!done && viz.isOpen() && optionCount < MAX_OPTIONS_PER_EPISODE) {
			bool shouldClose = false, resetRequested = false;
			viz.pollEvents(shouldClose, resetRequested);
			if (shouldClose) break;
			if (resetRequested) {
				env.setEpisodeNumber(episode);
				env.reset(5);
				currentPhase = 0;
				phaseJustChanged = false;
			}

			// check phase transition conditions FIRST, before executing any option
			phaseJustChanged = false;
			if (currentPhase == 0) {
				// ClearObstacles phase: transition when no obstacles nearby
				if (!env.hasObstacleNeighbor()) {
					currentPhase = 1;
					phaseJustChanged = true;
				}
			} else if (currentPhase == 1) {
				// MoveToTarget phase: transition when at target
				if (env.getRobotCell() == env.getTargetCell()) {
					currentPhase = 2;
					phaseJustChanged = true;
					std::cout << "Episode " << episode << " - Reached target! Transitioning to ReturnToObject phase." << std::endl;
				}
			} else if (currentPhase == 2) {
				// ReturnToObject phase: transition when carrying object
				if (env.isCarrying()) {
					currentPhase = 3;
					phaseJustChanged = true;
					std::cout << "Episode " << episode << " - Picked up object! Transitioning to MoveObjectToTarget phase." << std::endl;
				}
			}

			// Determine which option to execute based on current phase
			int option = currentPhase;
			
			// Phase 3 (MoveObjectToTarget): DO NOT clear obstacles - robot must navigate around them
			// with stored path or A* pathfinding
			if (currentPhase == 3) {
				// Never clear obstacles in phase 3 - just execute MoveObjectToTarget
				option = 3;
			}
			// Special handling for Phase 2 (ReturnToObject → MoveToObject):
			// Phase 2 strategy: Clear obstacles first, then move toward object
			else if (currentPhase == 2) {
				if (!env.isCarrying() && env.hasObstacleNeighbor()) {
					// Always clear adjacent obstacles in Phase 2
					option = 0;  // ClearObstacle
				} else {
					// No adjacent obstacles - move toward object using ReturnToObject
					option = 2;
				}
			}
			// Phase 1 (MoveToTarget): Don't clear obstacles once at target
			else if (currentPhase == 1) {
				// If already at target, don't trigger obstacle clearing
				if (env.getRobotCell() == env.getTargetCell()) {
					option = 1;  // Stay with MoveToTarget to finish the step cleanly
				} else if (!env.isCarrying() && env.hasObstacleNeighbor()) {
					// If not at target and obstacles nearby, clear them
					option = 0;  // ClearObstacle option
				} else {
					// No obstacles - proceed with MoveToTarget
					option = 1;
				}
			}
			// For phase 0, clear obstacles opportunistically when not carrying
			else if (currentPhase == 0 && !env.isCarrying() && env.hasObstacleNeighbor()) {
				option = 0; // ClearObstacle option
			}
			
			// Store previous state for Q-learning
			Environment2D prevState = env;
			
			options[option]->onSelect(env);
			float reward = executor.executeOption(env, *options[option], 5, currentPhase);
			planner.updateQ(prevState, option, reward, env, (int)options.size());
			episodeReward += reward;
			cumulativeReward += reward;

			// If we just picked up the object prematurely (before phase 3) AND we have NOT
			// reached the target earlier in this episode, drop it one cell to the left so the
			// robot can continue searching/clearing. If we've already reached the target once,
			// allow the pickup to stand.
			if (!prevState.isCarrying() && env.isCarrying() && currentPhase != 3 && !reachedTargetOnce) {
				if (env.dropObjectLeft()) {
					std::cout << "Episode " << episode << ": picked up object prematurely - dropped to left to allow searching for target." << std::endl;
				} else {
					std::cout << "Episode " << episode << ": attempted to drop object but no valid drop cell found; still carrying." << std::endl;
				}
			}
			
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
					reachedTargetOnce = true; // mark that we've reached the target at least once this episode
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
			// Increased from 15 to 40 to allow extended obstacle clearing and navigation
			if (stepsWithoutProgress > 40) {
				episodeReward -= 20.0f; // Penalty for getting stuck
				std::cout << "Episode " << episode << " terminated early - stuck without progress" << std::endl;
				break;
			}
		} else {
			// In phase 3, just track current distance without penalizing backward steps
			lastDistance = currentDistance;
		}			viz.renderWithOverlay(env, episode, episodeReward, (float)successfulEpisodes / (episode + 1));
			
			optionCount++;
			viz.delay(50); // Reduced delay for faster decisions
		}
		
		// Print episode summary
		if (episode % 10 == 0) {
			std::cout << "Episode " << episode << " complete. Reward: " << episodeReward 
					  << ", Success rate: " << (float)successfulEpisodes / (episode + 1) * 100 << "%" << std::endl;
		}

		// Log episode to CSV (approximate steps as options_used * maxStepsPerOption(=5) here)
		bool success = env.isTaskComplete();
		int optionsUsed = optionCount;
		int stepsTaken = optionsUsed * 5; // maxStepsPerOption is now 5
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