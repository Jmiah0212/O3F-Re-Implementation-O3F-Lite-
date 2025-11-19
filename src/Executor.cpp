#include "Executor.hpp"
#include "Env.hpp"
#include "Option.hpp"
#include <iostream>

void OptionExecutor::tick(Environment2D& env, float dt) {
	(void)env;
	(void)dt;
}

float OptionExecutor::runPrimitiveUntil(Environment2D& env, int maxSteps,
	const std::function<bool(const Environment2D&)>& goal,
	const std::function<Action(const Environment2D&)>& policy) {
	float total = 0.f;
	sf::Vector2i lastPos = env.getRobotCell();
	int stepsInSamePlace = 0;
	
	for (int i = 0; i < maxSteps; ++i) {
		if (goal && goal(env)) break;
		
		Action a = policy ? policy(env) : Action::None;
		float stepReward = env.step(a);
		total += stepReward;
		
		// Check if robot is stuck in same position
		if (env.getRobotCell() == lastPos) {
			stepsInSamePlace++;
			// If stuck for 3+ steps, give up on this option
			// Reduced penalty from -5.0 to -2.0 to be more lenient with clearing costs
			if (stepsInSamePlace >= 3) {
				total -= 2.0f; // Reduced penalty for getting stuck
				break;
			}
		} else {
			stepsInSamePlace = 0;
			lastPos = env.getRobotCell();
		}
		
		// Early termination if reward becomes very negative
		if (total < -15.0f) {
			break;
		}
	}
	return total;
}

float OptionExecutor::executeOption(Environment2D& env, const Option& option, int maxSteps) {
	// Default phase is -1 (no special handling)
	return executeOption(env, option, maxSteps, -1);
}

float OptionExecutor::executeOption(Environment2D& env, const Option& option, int maxSteps, int currentPhase) {
	sf::Vector2i startPos = env.getRobotCell();
	float reward = runPrimitiveUntil(env, maxSteps, option.goal(), option.policy());
	sf::Vector2i endPos = env.getRobotCell();
	
	// Additional penalty if option didn't accomplish anything meaningful
	// Skip this penalty for ClearObstacle in Phase 2 - not moving is expected when clearing
	if (startPos == endPos && option.name() != "ClearObstacle") {
		reward -= 3.0f; // Penalty for wasting time
	}
	
	// Handle ClearObstacle option specifically
	if (option.name() == std::string("ClearObstacle")) {
		if (env.hasObstacleNeighbor()) {
			// Check if clearing is beneficial
			static const int dx[4] = {1, -1, 0, 0};
			static const int dy[4] = {0, 0, 1, -1};
			bool clearedSomething = false;
			
			// In Phase 2 (MoveToObject), clear ANY adjacent obstacle freely
			if (currentPhase == 2) {
				if (env.clearAnyAdjacentObstacle()) {
					clearedSomething = true;
					// No reward in phase 2 - clearing is necessary cost
				}
			} else {
				// In other phases, prefer to clear strategic obstacles first
				for (int k = 0; k < 4; ++k) {
					int nx = env.getRobotCell().x + dx[k];
					int ny = env.getRobotCell().y + dy[k];
					if (nx >= 0 && nx < env.getGridWidth() && ny >= 0 && ny < env.getGridHeight()) {
						sf::Vector2i obstaclePos(nx, ny);
						if (env.isObstacle(obstaclePos) && env.shouldClearObstacle(obstaclePos)) {
							if (env.clearAnyAdjacentObstacle()) {
								reward += 2.0f; // Reward for strategic clearing
								clearedSomething = true;
								break;
							}
						}
					}
				}
				// If we didn't clear anything strategic, try to clear any adjacent obstacle
				if (!clearedSomething) {
					if (env.clearAnyAdjacentObstacle()) {
						reward += 1.0f; // smaller reward for non-strategic clear
						clearedSomething = true;
						std::cout << "Cleared adjacent obstacle (non-strategic)\n";
					}
				}
			}
			
			// If still nothing cleared, apply a small penalty instead of a large one
			if (!clearedSomething && env.hasObstacleNeighbor()) {
				reward -= 1.0f; // smaller penalty for failed clear
			}
		} else {
			// No obstacle nearby - this option shouldn't have been selected
			reward -= 3.0f; // reduced penalty
		}
	}
	
	return reward;
}