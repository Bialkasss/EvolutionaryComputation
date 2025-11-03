// Replaces <bits/stdc++.h>
#include <iostream>
#include <vector>
#include <string>
#include <random>       // For mt19937
#include <fstream>      // For ofstream
#include <iomanip>      // For setprecision
#include <chrono>       // For Stopwatch
#include <filesystem>   // For filesystem::path
#include <climits>      // For LLONG_MAX
#include <numeric>      // For iota
#include <algorithm>    // For min/max_element
#include <cmath>        // For sqrt/fabs
#include <tuple>        // For tie
#include <unordered_set> // For save_svg

// Core problem and objective definitions
#include "core/Instance.h"
#include "core/Common.h"
#include "core/Objective.h"

// All heuristic algorithms
#include "heuristics/Greedy.h"
#include "heuristics/Regret.h"
#include "heuristics/LocalSearch.h"
#include "heuristics/LocalSearchCandidate.h"

// Utility helpers
#include "utils/Timer.h"
#include "utils/ProgressBar.h"
#include "utils/SvgWriter.h"
// Note: utils/Helpers.h is probably only needed by the .cpp files

using namespace std;

/**
 * @brief Holds the results for a single named method.
 */
struct MethodResult {
    string method;
    long long best_obj = LLONG_MAX;
    vector<int> best_tour; 
    vector<long long> all_objs;
    vector<double> all_times_ms;
};

/**
 * @brief Main experiment runner.
 * * Parses arguments, loads instances, runs all specified heuristics,
 * and writes summary CSVs, best tour files, and SVGs.
 */
int main(int argc, char** argv) {
    ios::sync_with_stdio(false); 
    cin.tie(nullptr); 

    if (argc < 2) {
        cerr << "Usage: " << argv[0] << " <instance1.txt> [instance2.txt ...] "
             << "[--runs-per-start 1] [--random-runs 200] [--seed 106] [--out outdir]\n";
        return 1;
    }

    // --- Argument Parsing ---
    int runs_per_start = 1; 
    int runs_for_random = 200; 
    unsigned seed = 106; 
    string outdir = "out";
    vector<string> files;

    for (int i=1; i<argc; ++i) {
        string a = argv[i];
        if (a == "--runs-per-start" && i+1 < argc) { runs_per_start = stoi(argv[++i]); }
        else if (a == "--random-runs" && i+1 < argc) { runs_for_random = stoi(argv[++i]); }
        else if (a == "--seed" && i+1 < argc) { seed = (unsigned)stoul(argv[++i]); }
        else if (a == "--out" && i+1 < argc) { outdir = argv[++i]; }
        else if (a.rfind("--",0)==0) { cerr << "Unknown flag: " << a << "\n"; return 1; }
        else files.push_back(a);
    }

    filesystem::create_directories(outdir);

    mt19937 rng(seed); // Initialize random number generator

    // --- Prepare Summary CSVs ---
    string short_summary_obj_csv = (filesystem::path(outdir) / "all_results_summary_objectives.csv").string();
    string short_summary_time_csv = (filesystem::path(outdir) / "all_results_summary_times.csv").string();

    static bool headers_written = false;
    if (!headers_written) {
        ofstream SSO(short_summary_obj_csv);
        SSO << "instance,method,summary\n";
        ofstream SST(short_summary_time_csv);
        SST << "instance,method,summary (ms)\n";
        headers_written = true;
    }

    // --- Main Loop: Iterate Over Instance Files ---
    for (const auto& path : files) {
        Instance I;
        // Use the new Instance class method to load
        if (!read_instance(path, I)) {
            cerr << "Failed to load instance: " << path << "\n";
            return 2;
        }
        cout << "Instance: " << I.name << "   N=" << I.N << "  K=" << I.K << "\n";

        Stopwatch sw; // Use timer from utils/Timer.h

        // --- Precompute Candidate List ---
        cout << "Precomputing candidate lists...\n";
        sw.reset();
        auto cl_k5 = LocalSearchCandidate::precompute_candidate_list(I, 5);
        auto cl_k10 = LocalSearchCandidate::precompute_candidate_list(I, 10);
        auto cl_k20 = LocalSearchCandidate::precompute_candidate_list(I, 20);
        cout << "Done precomputing CLs (k=5, 10, 20) in " << sw.elapsed_ms() << " ms.\n";


        // --- Method Definitions ---
        vector<MethodResult> methods;
        methods.push_back({"CH_RANDOM", LLONG_MAX, {}, {}, {}}); //0
        methods.push_back({"CH_NN_END_ONLY", LLONG_MAX, {}, {}, {}}); //1
        methods.push_back({"CH_NN_INSERT_ANYWHERE_PATH", LLONG_MAX, {}, {}, {}}); //2
        methods.push_back({"CH_GREEDY_CYCLE_CHEAPEST_INSERTION", LLONG_MAX, {}, {}, {}}); //3
        // Regret heuristics
        methods.push_back({"CH_NN_PATH_INSERT_ANYWHERE_REGRET2", LLONG_MAX, {}, {}, {}});  //4
        methods.push_back({"CH_NN_PATH_INSERT_ANYWHERE_REGRET2_WEIGHTED_reg1_objchange_1", LLONG_MAX, {}, {}, {}}); //5
        methods.push_back({"CH_NN_PATH_INSERT_ANYWHERE_REGRET2_WEIGHTED_reg2_objchange_1", LLONG_MAX, {}, {}, {}}); //6
        methods.push_back({"CH_NN_PATH_INSERT_ANYWHERE_REGRET2_WEIGHTED_reg1_objchange_2", LLONG_MAX, {}, {}, {}}); //7
        methods.push_back({"CH_GREEDY_CYCLE_REGRET2", LLONG_MAX, {}, {}, {}}); //8
        methods.push_back({"CH_GREEDY_CYCLE_REGRET2_WEIGHTED_reg1_objchange_1", LLONG_MAX, {}, {}, {}}); //9
        methods.push_back({"CH_GREEDY_CYCLE_REGRET2_WEIGHTED_reg2_objchange_1", LLONG_MAX, {}, {}, {}}); //10
        methods.push_back({"CH_GREEDY_CYCLE_REGRET2_WEIGHTED_reg1_objchange_2", LLONG_MAX, {}, {}, {}}); //11
        // Local Search Methods
        // methods.push_back({"LS_Rand_Steep_Node", LLONG_MAX, {}, {}, {}}); // 12
        // methods.push_back({"LS_Rand_Greedy_Node", LLONG_MAX, {}, {}, {}}); // 13
        // methods.push_back({"LS_Rand_Steep_Edge", LLONG_MAX, {}, {}, {}}); // 14
        // methods.push_back({"LS_Rand_Greedy_Edge", LLONG_MAX, {}, {}, {}}); // 15
        // methods.push_back({"LS_Greedy_Steep_Node", LLONG_MAX, {}, {}, {}}); // 16
        // methods.push_back({"LS_Greedy_Greedy_Node", LLONG_MAX, {}, {}, {}}); // 17
        // methods.push_back({"LS_Greedy_Steep_Edge", LLONG_MAX, {}, {}, {}}); // 18
        // methods.push_back({"LS_Greedy_Greedy_Edge", LLONG_MAX, {}, {}, {}}); // 19
        // --- Local Search Methods (NEW ORDER and NAMES) ---
        // 1. Greedy / Node / Rand
        methods.push_back({"LS_GREEDY_NODES_RANDOM_init", LLONG_MAX, {}, {}, {}}); // 12
        // 2. Greedy / Node / Greedy
        methods.push_back({"LS_GREEDY_NODES_GREEDY_init", LLONG_MAX, {}, {}, {}}); // 13
        // 3. Greedy / Edge / Rand
        methods.push_back({"LS_GREEDY_EDGES_RANDOM_init", LLONG_MAX, {}, {}, {}}); // 14
        // 4. Greedy / Edge / Greedy
        methods.push_back({"LS_GREEDY_EDGES_GREEDY_init", LLONG_MAX, {}, {}, {}}); // 15
        // 5. Steep / Node / Rand
        methods.push_back({"LS_STEEP_NODES_RANDOM_init", LLONG_MAX, {}, {}, {}}); // 16
        // 6. Steep / Node / Greedy
        methods.push_back({"LS_STEEP_NODES_GREEDY_init", LLONG_MAX, {}, {}, {}}); // 17
        // 7. Steep / Edge / Rand
        methods.push_back({"LS_STEEP_EDGES_RANDOM_init", LLONG_MAX, {}, {}, {}}); // 18
        // 8. Steep / Edge / Greedy
        methods.push_back({"LS_STEEP_EDGES_GREEDY_init", LLONG_MAX, {}, {}, {}}); // 19

        // Candidate Moves Local Search
        methods.push_back({"LS_Rand_Steep_Edge_Cand5", LLONG_MAX, {}, {}, {}}); // 20
        methods.push_back({"LS_Rand_Steep_Edge_Cand10", LLONG_MAX, {}, {}, {}}); // 21
        methods.push_back({"LS_Rand_Steep_Edge_Cand20", LLONG_MAX, {}, {}, {}}); // 22


        // Storage for starting tours for Local Search
        vector<vector<int>> random_tours;
        vector<vector<int>> greedy_tours;
        random_tours.reserve(runs_for_random);
        greedy_tours.reserve(I.N);

        // --- Run: CH_RANDOM ---
        {
            auto &MR = methods[0];
            for (int r = 0; r < runs_for_random; ++r) {
                if ((r + 1) % 5 == 0 || r == runs_for_random - 1) {
                    print_progress((double)(r + 1) / runs_for_random, "Running CH_RANDOM...");
                }
                sw.reset();
                
                // Call namespaced function from heuristics/Greedy.h
                auto tour = GreedyHeuristics::random_solution(I, rng, -1);
                // Call namespaced function from core/Objective.h
                long long obj = Objective::calculate(tour, I);

                MR.all_objs.push_back(obj);
                if (obj < MR.best_obj) { MR.best_obj = obj; MR.best_tour = tour; }

                random_tours.push_back(tour);  // Save random tour for LS
            }
        }
        cerr << "\nDone." << endl;

        // --- Run: Greedy Construction Heuristics ---
        cout << "Running " << I.N << " greedy starts (for CH and LS)...\n";
        for (int start = 0; start < I.N; ++start) {
            
            double progress = (double)(start + 1) / I.N;
            print_progress(progress, "Greedy Starts (All Methods) Node " + to_string(start + 1) + "/" + to_string(I.N));

            for (int r = 0; r < runs_per_start; ++r) {
                
                #define RUN_CH(idx, func_call) \
                    sw.reset(); \
                    auto tour##idx = func_call; \
                    long long obj##idx = Objective::calculate(tour##idx, I); \
                    methods[idx].all_objs.push_back(obj##idx); \
                    methods[idx].all_times_ms.push_back(sw.elapsed_ms()); \
                    if (obj##idx < methods[idx].best_obj) { methods[idx].best_obj = obj##idx; methods[idx].best_tour = tour##idx; }

                // --- GreedyHeuristics ---
                RUN_CH(1, GreedyHeuristics::nn_end_only(I, start, rng));
                RUN_CH(2, GreedyHeuristics::nn_path_insert_anywhere(I, start, rng));
                RUN_CH(3, GreedyHeuristics::greedy_cycle_cheapest_insertion(I, start, rng));
                
                // --- RegretHeuristics ---
                RUN_CH(4, RegretHeuristics::nn_path_insert_anywhere_regret2(I, start, rng));
                RUN_CH(5, RegretHeuristics::nn_path_insert_anywhere_regret2_weighted(I, start, rng, 1.0, 1.0));
                RUN_CH(6, RegretHeuristics::nn_path_insert_anywhere_regret2_weighted(I, start, rng, 2.0, 1.0));
                RUN_CH(7, RegretHeuristics::nn_path_insert_anywhere_regret2_weighted(I, start, rng, 1.0, 2.0));
                RUN_CH(8, RegretHeuristics::greed_cycle_regret2(I, start, rng));
                RUN_CH(9, RegretHeuristics::greed_cycle_regret2_weighted(I, start, rng, 1.0, 1.0));
                RUN_CH(10, RegretHeuristics::greed_cycle_regret2_weighted(I, start, rng, 2.0, 1.0));
                RUN_CH(11, RegretHeuristics::greed_cycle_regret2_weighted(I, start, rng, 1.0, 2.0));

                // Save one greedy tour for LS
                greedy_tours.push_back(methods[9].best_tour.empty() ? tour9 : methods[9].best_tour);
            }
        }
        cerr << "\nDone." << endl;



        


// --- Run: LOCAL SEARCH from Random Initialization ---
        std::cout << endl << "Running local search...\n";
        cout << "Running Local Search from " << runs_for_random << " random starts...\n";
        int total_ls_rand_runs = random_tours.size() * 4; // 4 LS variants
        int current_ls_rand_run = 0;
        
        // This macro definition for less code (this will run wieh every RUN_LS)
        #define RUN_LS(idx, start_tour, func_call) \
            current_ls_rand_run++; \
            print_progress((double)current_ls_rand_run / total_ls_rand_runs, \
                           "LS from Random (" + methods[idx].method + ") " + to_string(i + 1) + "/" + to_string(random_tours.size())); \
            sw.reset(); \
            auto tour##idx = func_call; \
            long long obj##idx = Objective::calculate(tour##idx, I); \
            methods[idx].all_objs.push_back(obj##idx); \
            methods[idx].all_times_ms.push_back(sw.elapsed_ms()); \
            if (obj##idx < methods[idx].best_obj) { methods[idx].best_obj = obj##idx; methods[idx].best_tour = tour##idx; }

        for (size_t i = 0; i < random_tours.size(); ++i) {
            const auto& start_tour = random_tours[i];
            
            // 1. Greedy / Node / Rand (Index 12)
            RUN_LS(12, start_tour, LocalSearch::local_search_greedy(start_tour, I, LocalSearch::IntraMoveType::NODE_EXCHANGE, rng));
            
            // 3. Greedy / Edge / Rand (Index 14)
            RUN_LS(14, start_tour, LocalSearch::local_search_greedy(start_tour, I, LocalSearch::IntraMoveType::EDGE_EXCHANGE_2OPT, rng));

            // 5. Steep / Node / Rand (Index 16)
            RUN_LS(16, start_tour, LocalSearch::local_search_steepest(start_tour, I, LocalSearch::IntraMoveType::NODE_EXCHANGE, rng));

            // 7. Steep / Edge / Rand (Index 18)
            RUN_LS(18, start_tour, LocalSearch::local_search_steepest(start_tour, I, LocalSearch::IntraMoveType::EDGE_EXCHANGE_2OPT, rng));
        }
        cerr << "\nDone." << endl;


        // --- Run: LOCAL SEARCH from Greedy Initialization ---
        cout << "Running Local Search from " << greedy_tours.size() << " greedy starts...\n";
        int total_ls_greedy_runs = greedy_tours.size() * 4; // 4 LS variants
        int current_ls_greedy_run = 0;
        
        // This undef/redefine is from your main.cpp
        #undef RUN_LS
        #define RUN_LS(idx, start_tour, func_call) \
            current_ls_greedy_run++; \
            print_progress((double)current_ls_greedy_run / total_ls_greedy_runs, \
                           "LS from Greedy (" + methods[idx].method + ") " + to_string(i + 1) + "/" + to_string(greedy_tours.size())); \
            sw.reset(); \
            auto tour##idx = func_call; \
            long long obj##idx = Objective::calculate(tour##idx, I); \
            methods[idx].all_objs.push_back(obj##idx); \
            methods[idx].all_times_ms.push_back(sw.elapsed_ms()); \
            if (obj##idx < methods[idx].best_obj) { methods[idx].best_obj = obj##idx; methods[idx].best_tour = tour##idx; }

        for(size_t i = 0; i < greedy_tours.size(); ++i) {
            const auto& start_tour = greedy_tours[i];
            
            // 2. Greedy / Node / Greedy (Index 13)
            RUN_LS(13, start_tour, LocalSearch::local_search_greedy(start_tour, I, LocalSearch::IntraMoveType::NODE_EXCHANGE, rng));

            // 4. Greedy / Edge / Greedy (Index 15)
            RUN_LS(15, start_tour, LocalSearch::local_search_greedy(start_tour, I, LocalSearch::IntraMoveType::EDGE_EXCHANGE_2OPT, rng));

            // 6. Steep / Node / Greedy (Index 17)
            RUN_LS(17, start_tour, LocalSearch::local_search_steepest(start_tour, I, LocalSearch::IntraMoveType::NODE_EXCHANGE, rng));

            // 8. Steep / Edge / Greedy (Index 19)
            RUN_LS(19, start_tour, LocalSearch::local_search_steepest(start_tour, I, LocalSearch::IntraMoveType::EDGE_EXCHANGE_2OPT, rng));
        }
        cerr << "\nDone." << endl;

/*
        std::cout << endl << "Running local search...\n";
        
        // Helper macro for a single LS run
        #define RUN_LS_SINGLE(idx, start_tour, func_call) \
            sw.reset(); \
            auto tour##idx = func_call; \
            long long obj##idx = Objective::calculate(tour##idx, I); \
            methods[idx].all_objs.push_back(obj##idx); \
            methods[idx].all_times_ms.push_back(sw.elapsed_ms()); \
            if (obj##idx < methods[idx].best_obj) { methods[idx].best_obj = obj##idx; methods[idx].best_tour = tour##idx; }

        // --- 1. Local Search Greedy with Nodes and Random initialization ---
        cout << "Running " << methods[12].method << " from " << random_tours.size() << " random starts...\n";
        for (size_t i = 0; i < random_tours.size(); ++i) {
            print_progress((double)(i + 1) / random_tours.size(), methods[12].method + " " + to_string(i+1) + "/" + to_string(random_tours.size()));
            const auto& start_tour = random_tours[i];
            RUN_LS_SINGLE(12, start_tour, LocalSearch::local_search_greedy(start_tour, I, LocalSearch::IntraMoveType::NODE_EXCHANGE, rng));
        }
        cerr << "\nDone." << endl;

        // --- 2. Local Search Greedy with Nodes and Greedy initialization ---
        cout << "Running " << methods[13].method << " from " << greedy_tours.size() << " greedy starts...\n";
        for (size_t i = 0; i < greedy_tours.size(); ++i) {
            print_progress((double)(i + 1) / greedy_tours.size(), methods[13].method + " " + to_string(i+1) + "/" + to_string(greedy_tours.size()));
            const auto& start_tour = greedy_tours[i];
            RUN_LS_SINGLE(13, start_tour, LocalSearch::local_search_greedy(start_tour, I, LocalSearch::IntraMoveType::NODE_EXCHANGE, rng));
        }
        cerr << "\nDone." << endl;

        // --- 3. Local Search Greedy with Edges and Random initialization ---
        cout << "Running " << methods[14].method << " from " << random_tours.size() << " random starts...\n";
        for (size_t i = 0; i < random_tours.size(); ++i) {
            print_progress((double)(i + 1) / random_tours.size(), methods[14].method + " " + to_string(i+1) + "/" + to_string(random_tours.size()));
            const auto& start_tour = random_tours[i];
            RUN_LS_SINGLE(14, start_tour, LocalSearch::local_search_greedy(start_tour, I, LocalSearch::IntraMoveType::EDGE_EXCHANGE_2OPT, rng));
        }
        cerr << "\nDone." << endl;

        // --- 4. Local Search Greedy with Edges and Greedy initialization ---
        cout << "Running " << methods[15].method << " from " << greedy_tours.size() << " greedy starts...\n";
        for (size_t i = 0; i < greedy_tours.size(); ++i) {
            print_progress((double)(i + 1) / greedy_tours.size(), methods[15].method + " " + to_string(i+1) + "/" + to_string(greedy_tours.size()));
            const auto& start_tour = greedy_tours[i];
            RUN_LS_SINGLE(15, start_tour, LocalSearch::local_search_greedy(start_tour, I, LocalSearch::IntraMoveType::EDGE_EXCHANGE_2OPT, rng));
        }
        cerr << "\nDone." << endl;

        // --- 5. Local Search Steep with Nodes and Random initialization ---
        cout << "Running " << methods[16].method << " from " << random_tours.size() << " random starts...\n";
        for (size_t i = 0; i < random_tours.size(); ++i) {
            print_progress((double)(i + 1) / random_tours.size(), methods[16].method + " " + to_string(i+1) + "/" + to_string(random_tours.size()));
            const auto& start_tour = random_tours[i];
            RUN_LS_SINGLE(16, start_tour, LocalSearch::local_search_steepest(start_tour, I, LocalSearch::IntraMoveType::NODE_EXCHANGE, rng));
        }
        cerr << "\nDone." << endl;

        // --- 6. Local Search Steep with Nodes and Greedy initialization ---
        cout << "Running " << methods[17].method << " from " << greedy_tours.size() << " greedy starts...\n";
        for (size_t i = 0; i < greedy_tours.size(); ++i) {
            print_progress((double)(i + 1) / greedy_tours.size(), methods[17].method + " " + to_string(i+1) + "/" + to_string(greedy_tours.size()));
            const auto& start_tour = greedy_tours[i];
            RUN_LS_SINGLE(17, start_tour, LocalSearch::local_search_steepest(start_tour, I, LocalSearch::IntraMoveType::NODE_EXCHANGE, rng));
        }
        cerr << "\nDone." << endl;

        // --- 7. Local Search Steep with Edges and Random initialization ---
        cout << "Running " << methods[18].method << " from " << random_tours.size() << " random starts...\n";
        for (size_t i = 0; i < random_tours.size(); ++i) {
            print_progress((double)(i + 1) / random_tours.size(), methods[18].method + " " + to_string(i+1) + "/" + to_string(random_tours.size()));
            const auto& start_tour = random_tours[i];
            RUN_LS_SINGLE(18, start_tour, LocalSearch::local_search_steepest(start_tour, I, LocalSearch::IntraMoveType::EDGE_EXCHANGE_2OPT, rng));
        }
        cerr << "\nDone." << endl;

        // --- 8. Local Search Steep with Edges and Greedy initialization ---
        cout << "Running " << methods[19].method << " from " << greedy_tours.size() << " greedy starts...\n";
        for (size_t i = 0; i < greedy_tours.size(); ++i) {
            print_progress((double)(i + 1) / greedy_tours.size(), methods[19].method + " " + to_string(i+1) + "/" + to_string(greedy_tours.size()));
            const auto& start_tour = greedy_tours[i];
            RUN_LS_SINGLE(19, start_tour, LocalSearch::local_search_steepest(start_tour, I, LocalSearch::IntraMoveType::EDGE_EXCHANGE_2OPT, rng));
        }
        cerr << "\nDone." << endl;

        #undef RUN_LS_SINGLE // Clean up the macro

        */

      /*  // --- Run: LOCAL SEARCH from Random Initialization ---
        std::cout << endl << "Running local search...\n";
        cout << "Running Local Search from " << runs_for_random << " random starts...\n";
        int total_ls_rand_runs = random_tours.size() * 4; // 4 LS variants
        int current_ls_rand_run = 0;
        
        #define RUN_LS(idx, start_tour, func_call) \
            current_ls_rand_run++; \
            print_progress((double)current_ls_rand_run / total_ls_rand_runs, \
                           "LS from Random (" + methods[idx].method + ") " + to_string(i + 1) + "/" + to_string(random_tours.size())); \
            sw.reset(); \
            auto tour##idx = func_call; \
            long long obj##idx = Objective::calculate(tour##idx, I); \
            methods[idx].all_objs.push_back(obj##idx); \
            methods[idx].all_times_ms.push_back(sw.elapsed_ms()); \
            if (obj##idx < methods[idx].best_obj) { methods[idx].best_obj = obj##idx; methods[idx].best_tour = tour##idx; }

        for (size_t i = 0; i < random_tours.size(); ++i) {
            const auto& start_tour = random_tours[i];
            
            RUN_LS(12, start_tour, LocalSearch::local_search_steepest(start_tour, I, LocalSearch::IntraMoveType::NODE_EXCHANGE, rng));
            RUN_LS(13, start_tour, LocalSearch::local_search_greedy(start_tour, I, LocalSearch::IntraMoveType::NODE_EXCHANGE, rng));
            RUN_LS(14, start_tour, LocalSearch::local_search_steepest(start_tour, I, LocalSearch::IntraMoveType::EDGE_EXCHANGE_2OPT, rng)); // <-- BASELINE
            RUN_LS(15, start_tour, LocalSearch::local_search_greedy(start_tour, I, LocalSearch::IntraMoveType::EDGE_EXCHANGE_2OPT, rng));
        }
        cerr << "\nDone." << endl;   */

        

// --- Run: LOCAL SEARCH (Candidate List) from Random Initialization ---
        cout << "Running Candidate List Local Search from " << runs_for_random << " random starts...\n";
        
        #define RUN_LS_CAND(idx, start_tour, cl_var, k_val) \
            current_ls_cand_run++; \
            print_progress((double)current_ls_cand_run / total_ls_cand_runs, \
                           "LS-Cand (k=" + to_string(k_val) + ") " + to_string(i + 1) + "/" + to_string(random_tours.size())); \
            sw.reset(); \
            auto tour##idx = LocalSearchCandidate::local_search_steepest_candidate(start_tour, I, LocalSearch::IntraMoveType::EDGE_EXCHANGE_2OPT, cl_var, rng); \
            long long obj##idx = Objective::calculate(tour##idx, I); \
            methods[idx].all_objs.push_back(obj##idx); \
            methods[idx].all_times_ms.push_back(sw.elapsed_ms()); \
            if (obj##idx < methods[idx].best_obj) { methods[idx].best_obj = obj##idx; methods[idx].best_tour = tour##idx; }

        int total_ls_cand_runs = random_tours.size() * 3; // 3 CL variants
        int current_ls_cand_run = 0;

        for (size_t i = 0; i < random_tours.size(); ++i) {
            const auto& start_tour = random_tours[i];
            
            RUN_LS_CAND(20, start_tour, cl_k5, 5);
            RUN_LS_CAND(21, start_tour, cl_k10, 10);
            RUN_LS_CAND(22, start_tour, cl_k20, 20);
        }
        cerr << "\nDone." << endl;
        
        #undef RUN_LS_CAND // Undefine the new macro


 /*       // --- Run: LOCAL SEARCH from Greedy Initialization ---
        cout << "Running Local Search from " << greedy_tours.size() << " greedy starts...\n";
        int total_ls_greedy_runs = greedy_tours.size() * 4; // 4 LS variants
        int current_ls_greedy_run = 0;
        
        #undef RUN_LS
        #define RUN_LS(idx, start_tour, func_call) \
            current_ls_greedy_run++; \
            print_progress((double)current_ls_greedy_run / total_ls_greedy_runs, \
                           "LS from Greedy (" + methods[idx].method + ") " + to_string(i + 1) + "/" + to_string(greedy_tours.size())); \
            sw.reset(); \
            auto tour##idx = func_call; \
            long long obj##idx = Objective::calculate(tour##idx, I); \
            methods[idx].all_objs.push_back(obj##idx); \
            methods[idx].all_times_ms.push_back(sw.elapsed_ms()); \
            if (obj##idx < methods[idx].best_obj) { methods[idx].best_obj = obj##idx; methods[idx].best_tour = tour##idx; }

        for(size_t i = 0; i < greedy_tours.size(); ++i) {
            const auto& start_tour = greedy_tours[i];
            
            RUN_LS(16, start_tour, LocalSearch::local_search_steepest(start_tour, I, LocalSearch::IntraMoveType::NODE_EXCHANGE, rng));
            RUN_LS(17, start_tour, LocalSearch::local_search_greedy(start_tour, I, LocalSearch::IntraMoveType::NODE_EXCHANGE, rng));
            RUN_LS(18, start_tour, LocalSearch::local_search_steepest(start_tour, I, LocalSearch::IntraMoveType::EDGE_EXCHANGE_2OPT, rng));
            RUN_LS(19, start_tour, LocalSearch::local_search_greedy(start_tour, I, LocalSearch::IntraMoveType::EDGE_EXCHANGE_2OPT, rng));
        }
        cerr << "\nDone." << endl; 
    */


        // --- Summaries + Best Tours + SVGs ---
        string summary_csv = (filesystem::path(outdir) / (I.name + "_results_summary.csv")).string();
        ofstream S(summary_csv);
        S << "instance,method,runs,min_obj,max_obj,avg_obj,best_obj,min_time_ms,max_time_ms,avg_time_ms\n";
        
        ofstream SSO(short_summary_obj_csv, ios::app); // Append mode
        ofstream SST(short_summary_time_csv, ios::app); // Append mode


        for (auto &MR : methods) {
            if (MR.all_objs.empty()) continue; // Skip methods that didn't run

            if (!MR.best_tour.empty()) {
                string why;
                // Use namespaced check function
                bool ok = Objective::check(MR.best_tour, I, &why);
                if (!ok) cerr << "WARNING: best tour for " << MR.method << " failed check: " << why << "\n";
            }
            
            // Objective stats
            long long mn_obj = LLONG_MAX, mx_obj = LLONG_MIN; 
            long double sum_obj = 0;
            for (auto v : MR.all_objs) { mn_obj = min(mn_obj, v); mx_obj = max(mx_obj, v); sum_obj += v; }
            long double avg_obj = sum_obj / (long double)MR.all_objs.size();

            // Time stats
            double mn_time = 1e300, mx_time = -1e300, sum_time = 0, avg_time = 0;
            if (MR.all_times_ms.empty()) { // Handle random, which wasn't timed per run
                mn_time = 0; mx_time = 0; avg_time = 0;
            } else {
                for (auto t : MR.all_times_ms) { mn_time = min(mn_time, t); mx_time = max(mx_time, t); sum_time += t; }
                avg_time = sum_time / (double)MR.all_times_ms.size();
            }

            // Instance-specific summary
            S << I.name << "," << MR.method << "," << MR.all_objs.size() << ","
              << mn_obj << "," << mx_obj << "," << (long long)(avg_obj + 0.5) << "," << MR.best_obj << ","
              << mn_time << "," << mx_time << "," << avg_time << "\n";

            // Global summary (Objectives)
            SSO << I.name << "," << MR.method << ","
               << (long long)(avg_obj + 0.5)
               << " (" << mn_obj << " ; " << mx_obj << ")"
               << "\n";

            // Global summary (Times)
            SST << I.name << "," << MR.method << ","
               << fixed << setprecision(3) << avg_time
               << " (" << mn_time << " ; " << mx_time << ")"
               << "\n";

            // Write best tour .txt
            if (!MR.best_tour.empty()) {
                string best_txt = (filesystem::path(outdir) / (I.name + "_best_" + MR.method + ".txt")).string();
                ofstream B(best_txt);
                for (int i = 0; i < (int)MR.best_tour.size(); ++i) {
                    if (i) B << " ";
                    B << MR.best_tour[i];
                }
                B << "\n";

                // Write best tour .svg
                string svg = (filesystem::path(outdir) / (I.name + "_best_" + MR.method + ".svg")).string();
                // Call namespaced function from utils/SvgWriter.h
                save_svg(I, MR.best_tour, svg);
            }
        }
        cout << "Wrote: " << summary_csv << " and best tours/SVGs in " << outdir << "\n";
        cout << "Appended to: " << short_summary_obj_csv << " and " << short_summary_time_csv << "\n";
    } // End of instance loop
    
    return 0;
}