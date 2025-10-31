#include "env.hpp"
using namespace std;

void Env::reset_random(int W,int H,int num_obst){
    s = {};
    s.W=W; s.H=H; s.grid.assign(W*H,EMPTY);
    uniform_int_distribution<int> dx(0, W-1), dy(0, H-1);

    // Agent
    s.agent = { dx(rng), dy(rng) };
    s.grid[idx(s.agent)] = AGENT;

    // Target
    Pos tgt;
    do { tgt = { dx(rng), dy(rng) }; } while (tgt == s.agent);
    s.grid[idx(tgt)] = TARGET;

    // Goal
    Pos goal;
    do { goal = { dx(rng), dy(rng) }; } while (goal == s.agent || goal == tgt);
    s.grid[idx(goal)] = GOAL;

    // Obstacles
    for (int i=0;i<num_obst;i++){
        Pos p{ dx(rng), dy(rng) };
        if (s.grid[idx(p)] == EMPTY) s.grid[idx(p)] = OBST;
    }

    s.steps = 0;
    s.holding = false;
    s.success = false;
}

bool Env::inb(Pos p) const { return p.x>=0 && p.y>=0 && p.x<s.W && p.y<s.H; }
int  Env::idx(Pos p) const { return p.y*s.W + p.x; }

optional<Pos> Env::find_cell(Cell c) const {
    for (int y=0;y<s.H;y++){
        for (int x=0;x<s.W;x++){
            if (s.grid[y*s.W+x] == c) return Pos{x,y};
        }
    }
    return nullopt;
}

void Env::render() const {
    for (int y=0;y<s.H;y++){
        for (int x=0;x<s.W;x++){
            char ch='.';
            switch (s.grid[y*s.W+x]) {
                case EMPTY:  ch='.'; break;
                case OBST:   ch='#'; break;
                case TARGET: ch='T'; break;
                case GOAL:   ch='G'; break;
                case AGENT:  ch='A'; break;
            }
            cout << ch << ' ';
        }
        cout << "\n";
    }
    cout << "Steps: " << s.steps
              << (s.holding ? " (holding)" : "")
              << (s.success ? " (SUCCESS)" : "")
              << "\n\n";
}

double Env::step(char a){
    s.steps++;
    Pos np = s.agent;
    switch (a) {
        case 'N': np.y--; break;
        case 'S': np.y++; break;
        case 'W': np.x--; break;
        case 'E': np.x++; break;
        case 'P': return try_pick();
        case 'L': return try_place();
        default: break;
    }
    if (inb(np) && s.grid[idx(np)] != OBST) {
        s.grid[idx(s.agent)] = EMPTY;
        s.agent = np;
        s.grid[idx(s.agent)] = AGENT;
    }
    return -0.01; // step penalty
}

double Env::try_pick(){
    if (s.holding) return -0.05;
    static const Pos dirs[4] = {{1,0},{-1,0},{0,1},{0,-1}};
    for (auto d : dirs){
        Pos p{s.agent.x+d.x, s.agent.y+d.y};
        if (inb(p) && s.grid[idx(p)] == TARGET) {
            s.grid[idx(p)] = EMPTY; // lift target off the grid
            s.holding = true;
            return +1.0;
        }
    }
    return -0.1; // failed pick
}

double Env::try_place(){
    if (!s.holding) return -0.05;
    static const Pos dirs[4] = {{1,0},{-1,0},{0,1},{0,-1}};
    for (auto d : dirs){
        Pos p{s.agent.x+d.x, s.agent.y+d.y};
        if (!inb(p)) continue;
        if (s.grid[idx(p)] == GOAL) {
            s.holding = false;
            s.success = true;   // episode success
            return +2.0;
        }
    }
    return -0.1; // failed place (not next to GOAL)
}
