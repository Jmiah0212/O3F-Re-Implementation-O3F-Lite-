#pragma once
#include <vector>
#include <random> 

enum Cell { EMPTY, OBST, TARGET, GOAL, AGENT };

struct Pos {
    int x, y;
    bool operator==(const Pos& o) const { return x==o.x && y==o.y; }
    bool operator!=(const Pos& o) const { return !(*this ==o); }
};

struct State {
    int W, H;
    std::vector<Cell> grid;
    Pos agent;
    bool holding=false;
    Pos held{};
    int steps=0;
    int max_steps=200;
};