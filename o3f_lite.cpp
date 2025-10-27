#include <iostream>
#include <vector>
#include <random>
#include <iomanip>

using namespace std;

// ====== ENUMS AND STRUCTS =====
enum Cell { EMPTY, OBST, TARGET, GOAL, AGENT, HELD };

struct Pos {
    int x, y;
    bool operator==(const Pos& other) const { return x == other.x && y == other.y; }
};

struct State {
    int W, H;
    vector<Cell> grid;
    Pos agent;
    bool holding = false;
    Pos held{};
    int steps = 0;
    int max_steps = 200;
};

// ===== ENVIRONMENT CLASS =====
class Env {
    public:
    State s;
    mt19937 rng {random_device{}() };

    // Initialize random grid with one target + obstacles
    void reset_random(int W= 6, int H = 6, int num_obst = 4) {
        s = {};
        s.W = W; s.H = H; s.grid.assign(W * H, EMPTY);
        uniform_int_distribution<int> dist(0, W - 1);

        // place agent
        s.agent = { dist(rng), dist(rng) };
        s.grid[s.agent.y * W + s.agent.x] = AGENT;

        // place target
        Pos target;
        do {
            target = { dist(rng), dist(rng) };
        } while (target == s.agent);
        s.grid[target.y * W + target.x] = TARGET;

        // place obsticals
        for (int i = 0; i < num_obst; ++i) {
            Pos p{ dist(rng), dist(rng) };
            if (s.grid[p.y * W + p.x] == EMPTY)
                s.grid[p.y * W + p.x] = OBST;
        }

        s.steps = 0;
        s.holding = false;
    }

    bool in_bounds(Pos p) const {
        return p.x >= 0 && p.y >= 0 && p.x < s.W && p.y < s.H;
    }

    void render() const {
        for (int y=0; y < s.H; ++y) {
            for (int x = 0; x < s.W; ++x) {
                char c;
                switch (s.grid[y * s.W + x]) {
                    case EMPTY:  c = '.'; break;
                    case OBST:   c = '#'; break;
                    case TARGET: c = 'T'; break;
                    case GOAL:   c = 'G'; break;
                    case AGENT:  c = 'A'; break;
                    case HELD:   c = 'H'; break;
                    default:     c = '?'; break;
                }
                cout << c << ' ';
            }
            cout << "\n";
        }
        cout << "Steps: " << s.steps << (s.holding ? " (holding)\n\n" : "\n\n");
    }

    double step(char action) {
        // Actions: N,S,E,W,P (pick), L (place)
        s.steps++;
        Pos new_pos = s.agent;
        switch (action) {
            case 'N': new_pos.y -= 1; break;
            case 'S': new_pos.y += 1; break;
            case 'W': new_pos.x -= 1; break;
            case 'E': new_pos.x += 1; break;
            case 'P': return try_pick();
            case 'L': return try_place();
        }

        if (in_bounds(new_pos) && s.grid[new_pos.y * s.W + new_pos.x] != OBST) {
            s.grid[s.agent.y * s.W + s.agent.x] = EMPTY;
            s.agent = new_pos;
            s.grid[s.agent.y * s.W + s.agent.x] = AGENT;
        }

        return -0.01; // step penalty
    }

    private:
        double try_pick() {
            // pick target if adjacent
            static const Pos dirs[4] = { {1,0},{-1,0},{0,1},{0,-1} };
            for (auto d : dirs) {
                Pos p{ s.agent.x + d.x, s.agent.y + d.y };
                if (in_bounds(p) && s.grid[p.y * s.W + p.x] == TARGET) {
                    s.holding = true;
                    s.grid[p.y * s.W + p.x] = EMPTY;
                    return +1.0; // reward for picking
                }
            }
            return -0.1; // failed pick
        }


        double try_place() {
            if (!s.holding) return -0.05;
            s.holding = false;
            s.grid[s.agent.y * s.W + s.agent.x] = GOAL;
            return +2.0; // reward for successful place
        }
};

// ===== MAIN LOOP =====
int main() {
    Env env;
    env.reset_random();
    env.render();

    uniform_int_distribution<int> dist (0, 5);
    string actions = "NSEWPL";
    double total_reward = 0.0;

    for (int t = 0; t < 30; ++t) {
        char a = actions[dist(env.rng)];
        double r = env.step(a);
        total_reward += r;
        env.render();
    }

    cout << "Total reward: " << fixed << setprecision(2) << total_reward << "\n";
    return 0;
}