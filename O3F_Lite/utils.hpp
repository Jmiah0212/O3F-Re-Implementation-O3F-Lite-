#pragma once
#include <vector>
#include <random>
#include <fstream>
#include <string>
#include <cmath>

enum Cell { EMPTY, OBST, TARGET, GOAL, AGENT };

struct Pos {
    int x, y;
    bool operator==(const Pos& o) const { return x==o.x && y==o.y; }
    bool operator!=(const Pos& o) const { return !(*this==o); }
};

struct State {
    int W, H;
    std::vector<Cell> grid;
    Pos agent;
    bool holding=false;
    int steps=0;
    int max_steps=400;
    bool success=false; // becomes true after a successful place near GOAL
};

// simple CSV appender
inline void csv_write_header(const std::string& path, const std::string& header) {
    std::ofstream f(path, std::ios::out);
    f << header << "\n";
}
inline void csv_append(const std::string& path, const std::string& row) {
    std::ofstream f(path, std::ios::app);
    f << row << "\n";
}

// Manhattan distance
inline int manhattan(Pos a, Pos b) {
    return std::abs(a.x - b.x) + std::abs(a.y - b.y);
}
