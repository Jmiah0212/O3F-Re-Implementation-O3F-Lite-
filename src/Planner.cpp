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
	int bx = bucketize(env.getRobot().position.x, (float)env.getWidth(), 10);
	int by = bucketize(env.getRobot().position.y, (float)env.getHeight(), 10);
	int tx = bucketize(env.getTargetRegion().x, (float)env.getWidth(), 10);
	int ty = bucketize(env.getTargetRegion().y, (float)env.getHeight(), 10);
	// very coarse encoding
	return std::to_string(bx) + ":" + std::to_string(by) + ":" + std::to_string(tx) + ":" + std::to_string(ty);
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
