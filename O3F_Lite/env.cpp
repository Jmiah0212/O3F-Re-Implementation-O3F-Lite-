#include "env.hpp"

using namespace std;

void Env::reset_random(int W, int H, int num_obst) {
    s={}; s.W=W; s.H=H; s.grid.assign(W*H, EMPTY);
    uniform_int_distribution<int> d(0, W-1);
    s.agent={d(rng), d(rng)};
    s.grid[idx(s.agent)]=AGENT;
    Pos target;
    do target={d(rng), d(rng)}; while(target==s.agent);
    s.grid[idx(target)]=TARGET;
    Pos goal;
    do goal={d(rng),d(rng)}; while(goal==s.agent||goal==target);
    s.grid[idx(goal)]=GOAL;
    for (int i = 0; i < num_obst; i++) {
        Pos p{d(rng), d(rng)};
        if(s.grid[idx(p)]==EMPTY) s.grid[idx(p)]=OBST;
    }
    s.steps=0; s.holding=false;
}

bool Env::inb(Pos p)const{return p.x>=0&&p.y>=0&&p.x<s.W&&p.y<s.H;}
int Env::idx(Pos p)const{return p.y*s.W+p.x;}

void Env::render()const {
    for (int y=0; y<s.H;y++){
        for(int x=0;x<s.W;x++){
            char c='.';
            switch(s.grid[y*s.W+x]){
                case EMPTY:c='.';break;
                case OBST:c='#';break;
                case TARGET:c='T';break;
                case GOAL:c='G';break;
                case AGENT:c='A';break;
            }
            cout << c <<' ';
        }
        cout <<"\n";
        
    }
    cout<<"Steps:"<<s.steps<<(s.holding?" (holding)\n\n":"\n\n");

    
}

double Env::step(char a) {
        s.steps++;
        Pos np=s.agent;
        switch(a){
            case 'N':np.y--;break;
            case 'S':np.y++;break;
            case 'W':np.x--;break;
            case 'E':np.x++;break;
            case 'P':return try_pick();
            case 'L':return try_place();
        }
        if(inb(np)&&s.grid[idx(np)]!=OBST){
            s.grid[idx(s.agent)]=EMPTY;
            s.agent=np;
            s.grid[idx(s.agent)]=AGENT;
        }
        return -0.01;
    }

    double Env::try_pick(){
        if(s.holding) return -0.05;
        static const Pos dirs[4]={{1,0}, {-1,0},{0,1},{0,-1}};
        for(auto d:dirs){
            Pos p{s.agent.x+d.x,s.agent.y+d.y};
            if(inb(p)&&s.grid[idx(p)]==TARGET) {
                s.holding=true;
                s.grid[idx(p)]==EMPTY;
                return +1.0;
            }
        }
        return -0.1;
    }

    double Env::try_place(){
        if (!s.holding) return -0.05;
        static const Pos dirs[4]={{1,0},{-1,0},{0,1},{0,-1}};
        for(auto d:dirs){
            Pos p{s.agent.x+d.x,s.agent.y+d.y};
            if(inb(p)&&s.grid[idx(p)]==GOAL){
                s.holding=false;
                return +2.0;
            }
        }
        return -0.1;
    }