#include "MultipleStartLS_IterativeLS.h"
#include <iostream>
#include "../utils/Timer.h"
#include "../core/Objective.h"
#include "Greedy.h" // For random_solution
#include "ListMoves.h" // For local_search_steepest_with_LM
#include "LSHelpers.h" // For apply_intra_exchange_edges_2opt in perturbation

using namespace std;

namespace Metaheuristics {

    // Helper: Basic LS wrapper using LM
    static vector<int> basic_LS_for_meta(const Instance& I, mt19937& rng, double& elapsed_ms) {
        Stopwatch sw;
        auto start_tour = GreedyHeuristics::random_solution(I, rng, -1);
        auto improved   = LocalSearch::local_search_steepest_with_LM(start_tour, I, rng);
        elapsed_ms = sw.elapsed_ms();
        return improved;
    }

    // Helper: LS from specific start
    static vector<int> LS_from_start(const Instance& I, const vector<int>& start, mt19937& rng) {
        return LocalSearch::local_search_steepest_with_LM(start, I, rng);
    }

    // Helper: Perturbation (Random 2-opt moves)
    static void perturb_tour_2opt(vector<int>& tour, mt19937& rng, int num_moves, int N) {
        int K = (int)tour.size();
        if (K < 4) return;
        
        // Use the shared LSHelpers to apply moves easily
        LSHelpers h(N, K, tour);

        uniform_int_distribution<int> dist(0, K - 1);
        int tries = 0;
        for (int m = 0; m < num_moves && tries < num_moves * 10; ) {
            int i = dist(rng);
            int j = dist(rng);
            ++tries;
            if (i == j) continue;
            if (i > j) swap(i, j);

            // avoid trivial 2-opt (adjacent edges or whole cycle)
            if (j - i <= 1 || (i == 0 && j == K - 1)) continue;

            // Use the shared helper function
            apply_intra_exchange_edges_2opt(h, i, j);
            ++m;
        }
    }

    double MSLS(const Instance& I, mt19937& rng, MethodResult& MR, int outer_runs, int starts_per_run) {
        MR.all_objs.clear();
        MR.all_times_ms.clear();
        MR.best_obj = LLONG_MAX;
        MR.best_tour.clear();

        Stopwatch run_sw;

        for (int r = 0; r < outer_runs; ++r) {
            run_sw.reset();
            long long best_obj_run = LLONG_MAX;
            vector<int> best_tour_run;

            for (int k = 0; k < starts_per_run; ++k) {
                double ls_time_ms = 0.0;
                auto tour = basic_LS_for_meta(I, rng, ls_time_ms);
                long long obj = Objective::calculate(tour, I);
                if (obj < best_obj_run) {
                    best_obj_run = obj;
                    best_tour_run = tour;
                }
            }

            double run_time = run_sw.elapsed_ms();
            MR.all_objs.push_back(best_obj_run);
            MR.all_times_ms.push_back(run_time);

            if (best_obj_run < MR.best_obj) {
                MR.best_obj = best_obj_run;
                MR.best_tour = best_tour_run;
            }
        }

        double sum_time = 0.0;
        for (double t : MR.all_times_ms) sum_time += t;
        return (MR.all_times_ms.empty()) ? 0.0 : sum_time / MR.all_times_ms.size();
    }

    void ILS(const Instance& I,
             mt19937& rng,
             MethodResult& MR,
             double time_budget_ms,
             vector<int>& ls_runs_per_run,
             int ils_runs,
             int perturb_moves)
    {
        MR.all_objs.clear();
        MR.all_times_ms.clear();
        MR.best_obj = LLONG_MAX;
        MR.best_tour.clear();
        ls_runs_per_run.clear();
        ls_runs_per_run.reserve(ils_runs);

        for (int r = 0; r < ils_runs; ++r) {
            Stopwatch timer;
            int ls_calls = 0;

            // 1) random start + first LS
            auto current = GreedyHeuristics::random_solution(I, rng, -1);
            current = LS_from_start(I, current, rng);
            ++ls_calls;

            long long best_obj_run = Objective::calculate(current, I);
            vector<int> best_tour_run = current;

            // 2) ILS main loop within time budget
            while (timer.elapsed_ms() < time_budget_ms) {
                auto candidate = best_tour_run;               
                perturb_tour_2opt(candidate, rng, perturb_moves, I.N);

                candidate = LS_from_start(I, candidate, rng);
                ++ls_calls;

                long long cand_obj = Objective::calculate(candidate, I);
                if (cand_obj < best_obj_run) {
                    best_obj_run = cand_obj;
                    best_tour_run = candidate;
                }
            }

            double run_time = timer.elapsed_ms();
            MR.all_objs.push_back(best_obj_run);
            MR.all_times_ms.push_back(run_time);
            ls_runs_per_run.push_back(ls_calls);

            if (best_obj_run < MR.best_obj) {
                MR.best_obj = best_obj_run;
                MR.best_tour = best_tour_run;
            }
        }
    }
}