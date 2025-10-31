#include "env.hpp"
#include "option_executor.hpp"
#include <iostream>

int main() {
    Env env; env.reset_random();
    OptionExecutor exec;
    env.render();

    // locate target & goal
    Pos target, goal;
    for(int y=0; y<env.s.H; y++)
        for(int x=0; x<env.s.W; x++) {
            if(env.s.grid[y*env.s.W + x] == TARGET) target = {x,y};
            if(env.s.grid[y*env.s.W + x] == GOAL) goal = {x,y};
        }

    //execute: move-pick-move-place
    auto to_target = exec.plan_path(env, env.s.agent, target);
    for(char a : to_target) env.step(a);
    env.step('P'); env.render();

    auto to_goal = exec.plan_path(env, env.s.agent, goal);
    for(char a : to_goal) env.step(a);
    env.step('L'); env.render();

    std::cout<<"Simulation complete in "<<env.s.steps<<" steps.\n";
    return 0;
}