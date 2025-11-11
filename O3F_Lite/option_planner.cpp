#include "option_planner.hpp"
#include <algorithm>
#include <limits>

using namespace std;

static constexpr int NB_HOLD = 2;
static constexpr int NB_DBIN = 3; // distance bins
static constexpr int NA = 2;      // options
static constexpr int NS = NB_HOLD * NB_DBIN * NB_DBIN; // 2*3*3 = 18

OptionPlannerQL::OptionPlannerQL(int seed):Q(NS*NA, 0.0),rng(seed){}

int OptionPlannerQL::dist_bin(int d) const {
    if (d <= 2) return 0;
    if (d <= 5) return 1;
    return 2;
}

int OptionPlannerQL::state_id(const Env& env) const {
    auto t = env.find_cell(TARGET);
    auto g = env.find_cell(GOAL);
    // If TARGET got picked, t may be missing; treat distance as large if not holding
    int dt = 8, dg = 8;
    if (t.has_value()) dt = manhattan(env.s.agent, *t);
    if (g.has_value()) dg = manhattan(env.s.agent, *g);

    int h = env.s.holding ? 1 : 0;
    int bt = dist_bin(dt);
    int bg = dist_bin(dg);
    // index = h * (3*3) + bt * 3 + bg
    return h * (NB_DBIN*NB_DBIN) + bt * NB_DBIN + bg;
}

int OptionPlannerQL::argmaxQ(int s_id) const {
    double best = -1e18;
    int arg = 0;
    for (int a=0;a<NA;a++){
        double q = Q[s_id*NA + a];
        if (q > best){ best = q; arg = a; }
    }
    return arg;
}

int OptionPlannerQL::choose_option(const Env& env, double epsilon){
    uniform_real_distribution<double> uni(0.0,1.0);
    int s_id = state_id(env);
    if (uni(rng) < epsilon) {
        uniform_int_distribution<int> ai(0, NA-1);
        return ai(rng);
    }
    return argmaxQ(s_id);
}

void OptionPlannerQL::update(int s_id, int a, double R, int s_id_next, double gamma){
    double& qsa = Q[s_id*NA + a];
    double max_next = Q[s_id_next*NA + argmaxQ(s_id_next)];
    qsa = qsa + alpha * (R + gamma * max_next - qsa);
}

double OptionPlannerQL::execute_option(int option, Env& env, OptionExecutor& exec){
    double R = 0.0;

    if (option == OPT_PICK) {
        // plan to the TARGET (adjacent cell): pick when adjacent
        auto t = env.find_cell(TARGET);
        if (!t.has_value()) {
            // Already picked or missing; small penalty to discourage wasted picks
            return -0.05;
        }
        // Choose a neighbor of TARGET that's not an obstacle/outside
        static const Pos dirs[4]={{1,0},{-1,0},{0,1},{0,-1}};
        Pos bestAdj = *t; bool found=false;
        for (auto d: dirs){
            Pos n{t->x + d.x, t->y + d.y};
            if (env.inb(n) && env.s.grid[env.idx(n)] != OBST) { bestAdj = n; found = true; break; }
        }
        if (!found) return -0.2; // boxed in
        auto path = exec.plan_path(env, env.s.agent, bestAdj);
        R += exec.run_path(env, path);
        R += env.step('P'); // attempt pick
        return R;
    }

    if (option == OPT_PLACE) {
        if (!env.s.holding) {
            return -0.05; // can't place if not holding anything
        }
        auto g = env.find_cell(GOAL);
        if (!g.has_value()) return -0.05;
        static const Pos dirs[4]={{1,0},{-1,0},{0,1},{0,-1}};
        Pos bestAdj = *g; bool found=false;
        for (auto d: dirs){
            Pos n{g->x + d.x, g->y + d.y};
            if (env.inb(n) && env.s.grid[env.idx(n)] != OBST) { bestAdj = n; found = true; break; }
        }
        if (!found) return -0.2;
        auto path = exec.plan_path(env, env.s.agent, bestAdj);
        R += exec.run_path(env, path);
        R += env.step('L'); // attempt place (sets success=true on success)
        return R;
    }

    return -0.01;
}
