#pragma once
#include "utils.hpp"
#include <iostream>
#include <random>

using namespace std;
class Env {
    public:
        State s;
        mt19937 rng{ random_device{}() };

        void reset_random(int W=6, int H=6, int num_obst=4);
        bool inb(Pos p) const;
        int idx(Pos p) const;
        void render() const;
        double step(char a);

    private:
        double try_pick();
        double try_place();
};