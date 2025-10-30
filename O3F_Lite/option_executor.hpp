#pragma once 
#include "env.hpp"
#include <vector>

using namespace std;
class OptionExecutor {
    public:
        vector<char> plan_path(const Env&env, Pos start, Pos goal);
};