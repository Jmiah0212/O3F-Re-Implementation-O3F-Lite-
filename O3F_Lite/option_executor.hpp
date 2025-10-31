#pragma once 
#include "env.hpp"
#include <vector>

using namespace std;
class OptionExecutor {
    public:
        vector<char> plan_path(const Env&env, Pos start, Pos goal);

        double run_path(Env& env,const vector<char>& acts) {
            double R=0.0;
            for(char a: acts) R += env.step(a);
            return R;
        }
};