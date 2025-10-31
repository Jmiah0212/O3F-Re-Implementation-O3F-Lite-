#include "option_executor.hpp"
#include <queue>
#include <unordered_map>
#include <algorithm>
#include <cstdlib>

using namespace std;

struct Node{Pos p; double g,h;};
struct NodeCmp{bool operator()(const Node&a,const Node&b)const{return a.g+a.h>b.g+b.h;}};
struct PosHash{std::size_t operator()(const Pos&p)const noexcept{return (p.x*73856093) ^ (p.y*19349663);}};

vector<char> OptionExecutor::plan_path(const Env& env, Pos start, Pos goal){
    auto heuristic = [&](Pos a, Pos b)->double { return static_cast<double>(abs(a.x-b.x) + abs(a.y-b.y)); };
    priority_queue<Node, std::vector<Node>, NodeCmp> pq;
    unordered_map<Pos,Pos,PosHash> came;
    unordered_map<Pos,double,PosHash> gscore;

    pq.push(Node{start,0.0,heuristic(start,goal)});
    gscore[start]=0;
    static const Pos dirs[4]={{1,0},{-1,0},{0,1},{0,-1}};

    while(!pq.empty()){
        Node cur = pq.top(); pq.pop();
        if (cur.p == goal) break;
        for (auto d : dirs){
            Pos nb{cur.p.x+d.x, cur.p.y+d.y};
            if (!env.inb(nb)) continue;
            if (env.s.grid[env.idx(nb)] == OBST) continue;
            double ng = cur.g + 1;
            if (!gscore.count(nb) || ng < gscore[nb]) {
                gscore[nb] = ng;
                pq.push(Node{nb, ng, heuristic(nb,goal)});
                came[nb] = cur.p;
            }
        }
    }

    if (!came.count(goal)) return {}; // no path
    vector<Pos> rev; Pos p = goal; rev.push_back(p);
    while (p != start) { p = came[p]; rev.push_back(p); }
    std::reverse(rev.begin(), rev.end());

    vector<char> acts;
    for (size_t i=1;i<rev.size();i++){
        int dx = rev[i].x - rev[i-1].x;
        int dy = rev[i].y - rev[i-1].y;
        if (dx==1) acts.push_back('E');
        else if (dx==-1) acts.push_back('W');
        else if (dy==1) acts.push_back('S');
        else if (dy==-1) acts.push_back('N');
    }
    return acts;
}
