#pragma once
#include "utils.hpp"
#include <iostream>
#include <random>
#include <optional>

class Env {
public:
    State s;
    std::mt19937 rng{ std::random_device{}() };

    void reset_random(int W=8, int H=8, int num_obst=8); // 1 TARGET, 1 GOAL, random OBST
    bool inb(Pos p) const;
    int idx(Pos p) const;
    void render() const;
    double step(char a); // 'N','S','E','W','P','L'
    std::optional<Pos> find_cell(Cell c) const; // first occurrence
    bool is_terminal() const { return s.success || s.steps >= s.max_steps; }

private:
    double try_pick();
    double try_place();
};
