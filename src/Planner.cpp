#include "Planner.hpp"
#include "Env.hpp"
#include "Option.hpp"

#include <SFML/System/Vector2.hpp>
#include <cmath>
#include <random>

OptionPlanner::OptionPlanner(PlannerConfig cfg) : config(cfg) {}

static int bucketize(float value, float maxValue, int buckets) {
	if (value < 0) value = 0;
	if (value > maxValue) value = maxValue;
	float r = value / maxValue;
	int b = static_cast<int>(r * buckets);
	if (b >= buckets) b = buckets - 1;
	return b;
}

std::string OptionPlanner::discretize(const Environment2D& env) const {
	// Use grid cells directly instead of bucketing continuous space
	sf::Vector2i robotCell = env.getRobotCell();
	sf::Vector2i targetCell = env.getTargetCell();
	
	// Include relative position to target (directional info)
	int dx = targetCell.x - robotCell.x;
	int dy = targetCell.y - robotCell.y;
	
	// Bucket relative distance into ranges
	int distBucket = 0;
	int dist = std::abs(dx) + std::abs(dy);
	if (dist < 5) distBucket = 0;
	else if (dist < 10) distBucket = 1;
	else if (dist < 20) distBucket = 2;
	else distBucket = 3;
	
	// Include direction (8 cardinal + intercardinal directions + same location)
	int direction = 0;
	if (dx == 0 && dy == 0) direction = 0;
	else if (dx > 0 && dy == 0) direction = 1;      // Right
	else if (dx > 0 && dy > 0) direction = 2;      // Down-Right
	else if (dx == 0 && dy > 0) direction = 3;      // Down
	else if (dx < 0 && dy > 0) direction = 4;      // Down-Left
	else if (dx < 0 && dy == 0) direction = 5;      // Left
	else if (dx < 0 && dy < 0) direction = 6;      // Up-Left
	else if (dx == 0 && dy < 0) direction = 7;      // Up
	else if (dx > 0 && dy < 0) direction = 8;      // Up-Right
	
	// Include whether obstacles are nearby (boolean features)
	bool hasObstacle = env.hasObstacleNeighbor();
	
	return std::to_string(distBucket) + ":" + 
	       std::to_string(direction) + ":" + 
	       std::to_string(hasObstacle ? 1 : 0);
}

int OptionPlanner::selectAction(const Environment2D& env, const std::vector<std::unique_ptr<Option>>& options) {
	std::string s = discretize(env);
	auto& q = qTable[s];
	if (q.empty()) q.assign(options.size(), 0.0f);
	// epsilon-greedy
	static thread_local std::mt19937 rng(std::random_device{}());
	std::uniform_real_distribution<float> ud(0.f, 1.f);
	if (ud(rng) < config.epsilon) {
		std::uniform_int_distribution<int> ai(0, (int)options.size() - 1);
		return ai(rng);
	}
	int best = 0;
	for (int i = 1; i < (int)q.size(); ++i) if (q[i] > q[best]) best = i;
	return best;
}

void OptionPlanner::update(const Environment2D& prevEnv, int actionIdx, float reward, const Environment2D& nextEnv, int numActions) {
	std::string s = discretize(prevEnv);
	std::string sp = discretize(nextEnv);
	auto& q = qTable[s];
	if (q.size() < (size_t)numActions) q.resize(numActions, 0.0f);
	auto& qp = qTable[sp];
	if (qp.size() < (size_t)numActions) qp.resize(numActions, 0.0f);
	float maxNext = qp.empty() ? 0.0f : *std::max_element(qp.begin(), qp.end());
	float td = reward + config.gamma * maxNext - q[actionIdx];
	q[actionIdx] += config.alpha * td;
}
