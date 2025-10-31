#include "env.hpp"
#include "option_executor.hpp"
#include "option_planner.hpp"
#include "utils.hpp"
#include <iostream>
#include <iomanip>

using namespace std;
struct TrainConfig {
    int episodes = 300;      // feel free to increase to 2000+ to learn more robustly
    double gamma = 0.97;
    double eps_start = 0.30;
    double eps_end   = 0.02;
};

int main(){
    // -------- TRAINING --------
    TrainConfig cfg;
    Env env;
    OptionExecutor exec;
    OptionPlannerQL planner(123);

    const std::string logpath = "o3f_train.csv";
    csv_write_header(logpath, "episode,return,steps,success,epsilon");

    for (int ep=1; ep<=cfg.episodes; ++ep){
        // simple linear epsilon decay
        double t = (double)(ep-1) / max(1, cfg.episodes-1);
        double eps = cfg.eps_start + (cfg.eps_end - cfg.eps_start) * t;

        env.reset_random();
        double G = 0.0;
        int start_sid = planner.state_id(env);

        while (!env.is_terminal()){
            int s_id = planner.state_id(env);
            int a    = planner.choose_option(env, eps);
            double r = planner.execute_option(a, env, exec);
            G += r;
            int s_id2 = planner.state_id(env);
            planner.update(s_id, a, r, s_id2, cfg.gamma);

            // small step if option did nothing to avoid loops
            if (r > -1e-12 && r < 1e-12) env.step('N');
        }

        csv_append(logpath,
            to_string(ep) + "," +
            to_string(G)  + "," +
            to_string(env.s.steps) + "," +
            to_string(env.s.success ? 1 : 0) + "," +
            to_string(eps));

        if (ep % 25 == 0) {
            cout << "Episode " << setw(4) << ep
                      << " | Return " << setw(8) << fixed << setprecision(2) << G
                      << " | Steps " << setw(4) << env.s.steps
                      << " | Success " << (env.s.success ? "Y" : "N")
                      << " | eps " << setprecision(3) << eps
                      << "\n";
        }
    }

    cout << "\nTraining complete. Logged to " << logpath << "\n\n";

    // -------- DEMO (GREEDY) --------
    env.reset_random();
    cout << "Initial state:\n";
    env.render();

    while (!env.is_terminal()){
        int a = planner.greedy(env);
        (void)planner.execute_option(a, env, exec);
        // render a few frames only
        env.render();
    }

    cout << "Demo finished in " << env.s.steps
              << (env.s.success ? " with SUCCESS.\n" : " without success.\n");
    cout << "Open o3f_train.csv to plot learning curves (episode return, success).\n";
    return 0;
}
