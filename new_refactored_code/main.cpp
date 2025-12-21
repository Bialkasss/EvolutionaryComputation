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
#include <map>
#include <sstream>
#include <cctype>

// Core problem and objective definitions
#include "core/Instance.h"
#include "core/Common.h"
#include "core/Objective.h"

// All heuristic algorithms
#include "heuristics/Greedy.h"
#include "heuristics/Regret.h"
#include "heuristics/LocalSearch.h"
#include "heuristics/LocalSearchCandidate.h"
#include "heuristics/ListMoves.h"
#include "heuristics/MultipleStartLS_IterativeLS.h"
#include "heuristics/LargeNeighbourhoodSearch.h"
#include "heuristics/HybridEvolutionary.h"
#include "heuristics/OwnMethod.h"

// Utility helpers
#include "utils/Timer.h"
#include "utils/ProgressBar.h"
#include "utils/SvgWriter.h"

using namespace std;

// /**
//  * @brief Holds the results for a single named method.
//  */
// struct MethodResult {
//     string method;
//     long long best_obj = LLONG_MAX;
//     vector<int> best_tour; 
//     vector<long long> all_objs;
//     vector<double> all_times_ms;
// };

/**
 * @brief Main experiment runner.
 */
int main(int argc, char** argv) {
    ios::sync_with_stdio(false); 
    cin.tie(nullptr); 

    const bool RUN_BASICS = false;   // New toggle to run all or run needed (ils, mls, lns, evo)

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

    // Final table method names (user-specified order) and their corresponding internal method ids
    vector<string> final_method_names = {
        "Nearest Neighbour (append only)",
        "Nearest Neighbour (insert anywhere)",
        "Greedy Cycle",
        "NN 2-regret",
        "NN weighted 2-regret (same weights)",
        "NN weighted 2-regret (regret weight = 2, objective change weight = 1)",
        "NN weighted 2-regret (regret weight = 1, objective change weight = 2)",
        "Greedy Cycle 2-regret",
        "Greedy Cycle weighted 2-regret (same weights)",
        "Greedy Cycle weighted 2-regret (regret weight = 2, objective change weight = 1)",
        "Greedy Cycle weighted 2-regret (regret weight = 1, objective change weight = 2)",
        "Local Search Greedy with Nodes and Random initialization",
        "Local Search Greedy with Nodes and Greedy initialization",
        "Local Search Greedy with Edges and Random initialization",
        "Local Search Greedy with Edges and Greedy initialization",
        "Local Search Steep with Nodes and Random initialization",
        "Local Search Steep with Nodes and Greedy initialization",
        "Local Search Steep with Edges and Random initialization",
        "Local Search Steep with Edges and Greedy initialization",
        "Local Search with Candidate List = 5",
        "Local Search with Candidate List = 10",
        "Local Search with Candidate List = 15",
        "Steep Local Search with Edges and Random initialization and LM",
        "MSLS",
        "ILS",
        "LNS with LS",
        "LNS without LS",
        "Evolutionary: operator 1",
        "Evolutionary: operator 2 with LS",
        "Evolutionary: operator 2 without LS",
        "Own evolutionary 50p",
        "Own evolutionary 57.5p"
    };

    // internal method names as used in `methods[].method` in this main file, in corresponding order
    vector<string> internal_method_names = {
        "CH_NN_END_ONLY",
        "CH_NN_INSERT_ANYWHERE_PATH",
        "CH_GREEDY_CYCLE_CHEAPEST_INSERTION",
        "CH_NN_PATH_INSERT_ANYWHERE_REGRET2",
        "CH_NN_PATH_INSERT_ANYWHERE_REGRET2_WEIGHTED_reg1_objchange_1",
        "CH_NN_PATH_INSERT_ANYWHERE_REGRET2_WEIGHTED_reg2_objchange_1",
        "CH_NN_PATH_INSERT_ANYWHERE_REGRET2_WEIGHTED_reg1_objchange_2",
        "CH_GREEDY_CYCLE_REGRET2",
        "CH_GREEDY_CYCLE_REGRET2_WEIGHTED_reg1_objchange_1",
        "CH_GREEDY_CYCLE_REGRET2_WEIGHTED_reg2_objchange_1",
        "CH_GREEDY_CYCLE_REGRET2_WEIGHTED_reg1_objchange_2",
        "LS_GREEDY_NODES_RANDOM_init",
        "LS_GREEDY_NODES_GREEDY_init",
        "LS_GREEDY_EDGES_RANDOM_init",
        "LS_GREEDY_EDGES_GREEDY_init",
        "LS_STEEP_NODES_RANDOM_init",
        "LS_STEEP_NODES_GREEDY_init",
        "LS_STEEP_EDGES_RANDOM_init",
        "LS_STEEP_EDGES_GREEDY_init",
        "LS_Rand_Steep_Edge_Cand5",
        "LS_Rand_Steep_Edge_Cand10",
        "LS_Rand_Steep_Edge_Cand15",
        "LS_Rand_Steep_Edge_LM",
        "MSLS_LS_Steep_Edge_LM",
        "ILS_LS_Steep_Edge_LM",
        "LNS_With_LS",
        "LNS_No_LS",
        "Evo_op1",
        "Evo_op2_LS",
        "Evo_op2_NO_LS",
        "OwnHybrid_50_50",
        "Own_Hybrid_57.5"
    };

    // Accumulators per dataset ('TSPA' and 'TSPB') -> method name -> vector of objs/times
    map<string, map<string, vector<long long>>> set_method_objs; // set -> method -> objs
    map<string, map<string, vector<double>>> set_method_times;   // set -> method -> times
    set_method_objs["TSPA"]; set_method_objs["TSPB"];
    set_method_times["TSPA"]; set_method_times["TSPB"];

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
        if (!read_instance(path, I)) {
            cerr << "Failed to load instance: " << path << "\n";
            return 2;
        }
        cout << "Instance: " << I.name << "   N=" << I.N << "  K=" << I.K << "\n";

        Stopwatch sw; 

        // --- Precompute Candidate List ---
        cout << "Precomputing candidate lists...\n";
        sw.reset();
        auto cl_k5 = LocalSearchCandidate::precompute_candidate_list(I, 5);
        auto cl_k10 = LocalSearchCandidate::precompute_candidate_list(I, 10);
        auto cl_k15 = LocalSearchCandidate::precompute_candidate_list(I, 15);
        cout << "Done precomputing CLs (k=5, 10, 15) in " << sw.elapsed_ms() << " ms.\n";


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
        // Local Search Methods (Standard)
        methods.push_back({"LS_GREEDY_NODES_RANDOM_init", LLONG_MAX, {}, {}, {}}); // 12
        methods.push_back({"LS_GREEDY_NODES_GREEDY_init", LLONG_MAX, {}, {}, {}}); // 13
        methods.push_back({"LS_GREEDY_EDGES_RANDOM_init", LLONG_MAX, {}, {}, {}}); // 14
        methods.push_back({"LS_GREEDY_EDGES_GREEDY_init", LLONG_MAX, {}, {}, {}}); // 15
        methods.push_back({"LS_STEEP_NODES_RANDOM_init", LLONG_MAX, {}, {}, {}}); // 16
        methods.push_back({"LS_STEEP_NODES_GREEDY_init", LLONG_MAX, {}, {}, {}}); // 17
        methods.push_back({"LS_STEEP_EDGES_RANDOM_init", LLONG_MAX, {}, {}, {}}); // 18
        methods.push_back({"LS_STEEP_EDGES_GREEDY_init", LLONG_MAX, {}, {}, {}}); // 19

        // Candidate Moves Local Search
        methods.push_back({"LS_Rand_Steep_Edge_Cand5", LLONG_MAX, {}, {}, {}}); // 20
        methods.push_back({"LS_Rand_Steep_Edge_Cand10", LLONG_MAX, {}, {}, {}}); // 21
        methods.push_back({"LS_Rand_Steep_Edge_Cand15", LLONG_MAX, {}, {}, {}}); // 22

        // List of Moves (LM)
        methods.push_back({"LS_Rand_Steep_Edge_LM", LLONG_MAX, {}, {}, {}}); // 23
        methods.push_back({"LS_Greedy_Steep_Edge_LM", LLONG_MAX, {}, {}, {}}); // 24

        // MSLS and ILS
        methods.push_back({"MSLS_LS_Steep_Edge_LM", LLONG_MAX, {}, {}, {}}); // 25
        methods.push_back({"ILS_LS_Steep_Edge_LM",  LLONG_MAX, {}, {}, {}}); // 26

        // LNS
        methods.push_back({"LNS_With_LS",  LLONG_MAX, {}, {}, {}}); // 27
        methods.push_back({"LNS_No_LS",  LLONG_MAX, {}, {}, {}}); // 28

        // Evolutionary
        methods.push_back({"Evo_op1", LLONG_MAX, {}, {}, {}}); //29
        methods.push_back({"Evo_op2_LS", LLONG_MAX, {}, {}, {}}); //30
        methods.push_back({"Evo_op2_NO_LS", LLONG_MAX, {}, {}, {}}); //31
        methods.push_back({"OwnHybrid_50_50", LLONG_MAX, {}, {}, {}}); //32

        methods.push_back({"Own_Hybrid_57.5", LLONG_MAX, {}, {}, {}}); //33

        // iterations storage per method (to record iterations returned by algorithms)
        vector<vector<int>> method_iters(methods.size());


        // Storage for starting tours for Local Search
        vector<vector<int>> random_tours;
        vector<vector<int>> greedy_tours;
        random_tours.reserve(runs_for_random);
        greedy_tours.reserve(I.N);

        // =====================================================================
        //                          BASIC HEURISTICS
        // =====================================================================
        if (RUN_BASICS){
            {
                // --- Run: CH_RANDOM ---
                auto &MR = methods[0];
                for (int r = 0; r < runs_for_random; ++r) {
                    if ((r + 1) % 5 == 0 || r == runs_for_random - 1) {
                        print_progress((double)(r + 1) / runs_for_random, "Running CH_RANDOM...");
                    }
                    sw.reset();
                    auto tour = GreedyHeuristics::random_solution(I, rng, -1);
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

                    // Save one greedy tour for LS (Method 9: Greedy Cycle Regret Weighted 1,1)
                    greedy_tours.push_back(methods[9].best_tour.empty() ? tour9 : methods[9].best_tour);
                }
            }
            cerr << "\nDone running 1-11." << endl;

            // --- Run: LOCAL SEARCH from Random Initialization ---
            std::cout << endl << "Running local search (Random Starts)...\n";
            int total_ls_rand_runs = random_tours.size() * 5; // 4 Standard + 1 LM
            int current_ls_rand_run = 0;
            
            #define RUN_LS(idx, start_tour, func_call) \
                current_ls_rand_run++; \
                print_progress((double)current_ls_rand_run / total_ls_rand_runs, \
                            "LS-Rand (" + methods[idx].method + ") " + to_string(i + 1) + "/" + to_string(random_tours.size())); \
                sw.reset(); \
                auto tour##idx = func_call; \
                long long obj##idx = Objective::calculate(tour##idx, I); \
                methods[idx].all_objs.push_back(obj##idx); \
                methods[idx].all_times_ms.push_back(sw.elapsed_ms()); \
                if (obj##idx < methods[idx].best_obj) { methods[idx].best_obj = obj##idx; methods[idx].best_tour = tour##idx; }

            for (size_t i = 0; i < random_tours.size(); ++i) {
                const auto& start_tour = random_tours[i];
                
                RUN_LS(12, start_tour, LocalSearch::local_search_greedy(start_tour, I, LocalSearch::IntraMoveType::NODE_EXCHANGE, rng));
                RUN_LS(14, start_tour, LocalSearch::local_search_greedy(start_tour, I, LocalSearch::IntraMoveType::EDGE_EXCHANGE_2OPT, rng));
                RUN_LS(16, start_tour, LocalSearch::local_search_steepest(start_tour, I, LocalSearch::IntraMoveType::NODE_EXCHANGE, rng));
                RUN_LS(18, start_tour, LocalSearch::local_search_steepest(start_tour, I, LocalSearch::IntraMoveType::EDGE_EXCHANGE_2OPT, rng));
                
                // LM (List Moves)
                RUN_LS(23, start_tour, LocalSearch::local_search_steepest_with_LM(start_tour, I, rng));
            }
            cerr << "\nDone running LS 12, 14, 16, 18." << endl;


            // --- Run: LOCAL SEARCH from Greedy Initialization ---
            cout << "Running local search (Greedy Starts)...\n";
            int total_ls_greedy_runs = greedy_tours.size() * 5; // 4 Standard + 1 LM
            int current_ls_greedy_run = 0;
            
            #undef RUN_LS
            #define RUN_LS(idx, start_tour, func_call) \
                current_ls_greedy_run++; \
                print_progress((double)current_ls_greedy_run / total_ls_greedy_runs, \
                            "LS-Greedy (" + methods[idx].method + ") " + to_string(i + 1) + "/" + to_string(greedy_tours.size())); \
                sw.reset(); \
                auto tour##idx = func_call; \
                long long obj##idx = Objective::calculate(tour##idx, I); \
                methods[idx].all_objs.push_back(obj##idx); \
                methods[idx].all_times_ms.push_back(sw.elapsed_ms()); \
                if (obj##idx < methods[idx].best_obj) { methods[idx].best_obj = obj##idx; methods[idx].best_tour = tour##idx; }

            for(size_t i = 0; i < greedy_tours.size(); ++i) {
                const auto& start_tour = greedy_tours[i];
                
                RUN_LS(13, start_tour, LocalSearch::local_search_greedy(start_tour, I, LocalSearch::IntraMoveType::NODE_EXCHANGE, rng));
                RUN_LS(15, start_tour, LocalSearch::local_search_greedy(start_tour, I, LocalSearch::IntraMoveType::EDGE_EXCHANGE_2OPT, rng));
                RUN_LS(17, start_tour, LocalSearch::local_search_steepest(start_tour, I, LocalSearch::IntraMoveType::NODE_EXCHANGE, rng));
                RUN_LS(19, start_tour, LocalSearch::local_search_steepest(start_tour, I, LocalSearch::IntraMoveType::EDGE_EXCHANGE_2OPT, rng));
                
                // LM (List Moves)
                RUN_LS(24, start_tour, LocalSearch::local_search_steepest_with_LM(start_tour, I, rng));
            }
            cerr << "\nDone running 13, 15, 17, 19, 24." << endl;


            // --- Run: LOCAL SEARCH (Candidate List) ---
            cout << "Running Candidate List LS...\n";
            
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
                RUN_LS_CAND(22, start_tour, cl_k15, 15);
            }
            cerr << "\nDone running 20, 21, 22." << endl;
            #undef RUN_LS_CAND 
        }
        else {
            cout << "Skipping Basic Heuristics (Random, Greedy, LS) - RUN_BASICS is false.\n";
        }

        // =====================================================================
        //          METAHEURISTICS: MSLS, ILS, LNS
        // =====================================================================
        
        // 1. MSLS (Multiple Start Local Search)
        cout << "\nRunning MSLS (20 runs, 200 LS calls each)...\n";
        double avg_msls_time_ms = Metaheuristics::MSLS(I, rng, methods[25], 20, 200); 
        std::cout << "25 run" << endl;
        cout << "Average MSLS time = " << avg_msls_time_ms << " ms. Using as budget for ILS/LNS.\n";

        // 2. ILS (Iterated Local Search)
        cout << "Running ILS (20 runs)...\n";
        vector<int> ils_ls_runs_count;
        Metaheuristics::ILS(I, rng, methods[26], avg_msls_time_ms, ils_ls_runs_count, 20, 3);
        std::cout << "26 run" << endl;
        
        // Save ILS stats
        string ils_csv = (filesystem::path(outdir) / (I.name + "_ILS_iterations.csv")).string();
        ofstream ILS_F(ils_csv);
        ILS_F << "run,ls_calls\n";
        for(size_t r=0; r<ils_ls_runs_count.size(); ++r) ILS_F << r+1 << "," << ils_ls_runs_count[r] << "\n";
        
        // 3. LNS (Large Neighborhood Search)
        // With Local Search
        cout << "Running LNS with LS (20 runs)...\n";
        string lns_ls_csv = (filesystem::path(outdir) / (I.name + "_LNS_With_LS_iterations.csv")).string();
        ofstream LNS_LS_Stream(lns_ls_csv);
        LNS_LS_Stream << "run,iterations\n";

        for(int r=0; r<20; ++r) {
            print_progress((r+1)/20.0, "LNS+LS");
            sw.reset();
            // LNS::run now returns {tour, iterations}
            auto result = LNS::run(I, avg_msls_time_ms, true, rng);
            double t = sw.elapsed_ms();
            long long obj = Objective::calculate(result.first, I);
            
            LNS_LS_Stream << r+1 << "," << result.second << "\n";

            methods[27].all_objs.push_back(obj);
            methods[27].all_times_ms.push_back(t);
            if(obj < methods[27].best_obj) { methods[27].best_obj = obj; methods[27].best_tour = result.first; }
        }
        cerr << endl;
        std::cout << "27 run" << endl;


        // Without Local Search
        cout << "Running LNS without LS (20 runs)...\n";
        string lns_no_ls_csv = (filesystem::path(outdir) / (I.name + "_LNS_No_LS_iterations.csv")).string();
        ofstream LNS_NoLS_Stream(lns_no_ls_csv);
        LNS_NoLS_Stream << "run,iterations\n";

        for(int r=0; r<20; ++r) {
            print_progress((r+1)/20.0, "LNS-NoLS");
            sw.reset();
            // LNS::run now returns {tour, iterations}
            auto result = LNS::run(I, avg_msls_time_ms, false, rng);
            double t = sw.elapsed_ms();
            long long obj = Objective::calculate(result.first, I);
            
            LNS_NoLS_Stream << r+1 << "," << result.second << "\n";

            methods[28].all_objs.push_back(obj);
            methods[28].all_times_ms.push_back(t);
            if(obj < methods[28].best_obj) { methods[28].best_obj = obj; methods[28].best_tour = result.first; }
        }
        cerr << endl;
        std::cout << "28 run" << endl;

        // =====================================================================
        //               EVOLUTIONARY ALGORITHMS (NEW)
        // =====================================================================
        cout << "\nRunning Evolutionary Algorithms (3 runs each, same time budget)...\n";
        #define RUN_EVO(idx, method_name, func_call) \
            for (int r = 0; r < 20; r++){ \
                print_progress((double)(r+1) /3, string(method_name)); \
                sw.reset(); \
                auto result = func_call; \
                double t = sw.elapsed_ms(); \
                long long obj = Objective::calculate(result.first, I); \
                methods[idx].all_objs.push_back(obj); \
                methods[idx].all_times_ms.push_back(t); \
                method_iters[idx].push_back(result.second); \
                if (obj < methods[idx].best_obj){ \
                    methods[idx].best_obj = obj; \
                    methods[idx].best_tour = result.first; \
                }\
            } \
            cerr << endl;

        // 29: Evo Op1 (Common Subpaths)
        RUN_EVO(29, "Evo_Op1", Evolutionary::Evo_op1(I, rng, avg_msls_time_ms));
        
        // 30: Evo Op2 (Regret Repair + LS)
        RUN_EVO(30, "Evo_Op2_LS", Evolutionary::Evo_op2(I, rng, avg_msls_time_ms, true));

        // 31: Evo Op2 (Regret Repair NO LS)
        RUN_EVO(31, "Evo_Op2_NoLS", Evolutionary::Evo_op2(I, rng, avg_msls_time_ms, false));

        // 32,33: Own Hybrid 
        RUN_EVO(32, "Own_Hybrid_50_50", Own::hybrid_evolutionary_algorithm(I, rng, avg_msls_time_ms, true, 0.5));
        RUN_EVO(33, "Own_Hybrid_57.5", Own::hybrid_evolutionary_algorithm(I, rng, avg_msls_time_ms, true, 0.575));



        // Write per-instance EVO iterations CSV and a summary (min,max,avg)
        string evo_iters_csv = (filesystem::path(outdir) / (I.name + "_evo_iterations.csv")).string();
        ofstream EI(evo_iters_csv);
        EI << "method,run,iterations\n";
        for (int idx : {29,30,31,32,33}) {
            for (size_t r = 0; r < method_iters[idx].size(); ++r) {
                EI << methods[idx].method << "," << (r+1) << "," << method_iters[idx][r] << "\n";
            }
        }
        EI.close();

        string evo_iters_summary = (filesystem::path(outdir) / (I.name + "_evo_iterations_summary.csv")).string();
        ofstream EIS(evo_iters_summary);
        EIS << "method,min,max,avg\n";
        for (int idx : {29,30,31,32,33}) {
            auto &vec = method_iters[idx];
            if (vec.empty()) continue;
            int mn = vec[0], mx = vec[0];
            long long sum = 0;
            for (int v : vec) { mn = min(mn, v); mx = max(mx, v); sum += v; }
            double avg = (double)sum / (double)vec.size();
            EIS << methods[idx].method << "," << mn << "," << mx << "," << fixed << setprecision(3) << avg << "\n";
        }
        EIS.close();


        // --- Summaries + Best Tours + SVGs ---
        string summary_csv = (filesystem::path(outdir) / (I.name + "_results_summary.csv")).string();
        ofstream S(summary_csv);
        S << "instance,method,runs,min_obj,max_obj,avg_obj,best_obj,min_time_ms,max_time_ms,avg_time_ms\n";
        
        ofstream SSO(short_summary_obj_csv, ios::app);
        ofstream SST(short_summary_time_csv, ios::app);

        for (auto &MR : methods) {
            if (MR.all_objs.empty()) continue; 

            if (!MR.best_tour.empty()) {
                string why;
                if (!Objective::check(MR.best_tour, I, &why)) {
                    cerr << "WARNING: best tour for " << MR.method << " failed check: " << why << "\n";
                }
            }
            
            long long mn_obj = LLONG_MAX, mx_obj = LLONG_MIN; 
            long double sum_obj = 0;
            for (auto v : MR.all_objs) { mn_obj = min(mn_obj, v); mx_obj = max(mx_obj, v); sum_obj += v; }
            long double avg_obj = sum_obj / (long double)MR.all_objs.size();

            double mn_time = 1e300, mx_time = -1e300, sum_time = 0, avg_time = 0;
            if (MR.all_times_ms.empty()) { 
                mn_time = 0; mx_time = 0; avg_time = 0;
            } else {
                for (auto t : MR.all_times_ms) { mn_time = min(mn_time, t); mx_time = max(mx_time, t); sum_time += t; }
                avg_time = sum_time / (double)MR.all_times_ms.size();
            }

            S << I.name << "," << MR.method << "," << MR.all_objs.size() << ","
              << mn_obj << "," << mx_obj << "," << (long long)(avg_obj + 0.5) << "," << MR.best_obj << ","
              << mn_time << "," << mx_time << "," << avg_time << "\n";

            SSO << I.name << "," << MR.method << ","
               << (long long)(avg_obj + 0.5)
               << " (" << mn_obj << " ; " << mx_obj << ")" << "\n";

            SST << I.name << "," << MR.method << ","
               << fixed << setprecision(3) << avg_time
               << " (" << mn_time << " ; " << mx_time << ")" << "\n";

            if (!MR.best_tour.empty()) {
                string best_txt = (filesystem::path(outdir) / (I.name + "_best_" + MR.method + ".txt")).string();
                ofstream B(best_txt);
                for (int i = 0; i < (int)MR.best_tour.size(); ++i) {
                    if (i) B << " ";
                    B << MR.best_tour[i];
                }
                B << "\n";

                string svg = (filesystem::path(outdir) / (I.name + "_best_" + MR.method + ".svg")).string();
                save_svg(I, MR.best_tour, svg);
            }
        }

        // --- Accumulate results for final tables (TSPA / TSPB) ---
        string set_name = "";
        // detect dataset by file path or instance name
        string path_upper = path;
        for (auto &c : path_upper) c = toupper((unsigned char)c);
        string iname_upper = I.name;
        for (auto &c : iname_upper) c = toupper((unsigned char)c);
        if (path_upper.find("TSPA") != string::npos || iname_upper.find("TSPA") != string::npos) set_name = "TSPA";
        else if (path_upper.find("TSPB") != string::npos || iname_upper.find("TSPB") != string::npos) set_name = "TSPB";

        if (!set_name.empty()) {
            for (size_t mi = 0; mi < final_method_names.size(); ++mi) {
                string final_name = final_method_names[mi];
                string internal_name = internal_method_names.size() > mi ? internal_method_names[mi] : string();
                // find method in current methods vector
                bool found = false;
                for (auto &MR : methods) {
                    if (MR.method == internal_name) {
                        // append all objs and times
                        for (auto v : MR.all_objs) set_method_objs[set_name][final_name].push_back(v);
                        for (auto t : MR.all_times_ms) set_method_times[set_name][final_name].push_back(t);
                        found = true;
                        break;
                    }
                }
                // if not found, leave vector empty; later will write 0
            }
        }

        cout << "Wrote: " << summary_csv << " and best tours/SVGs in " << outdir << "\n";
    }
    
    // --- Write final tables (aggregated per dataset TSPA / TSPB) ---
    string final_results_csv = (filesystem::path(outdir) / "final_table_results.csv").string();
    ofstream FR(final_results_csv);
    FR << "method,TSPA,TSPB\n";
    for (auto &final_name : final_method_names) {
        // TSPA
        auto &vA = set_method_objs["TSPA"][final_name];
        string cellA;
        if (vA.empty()) cellA = "0";
        else {
            long long mn = vA[0], mx = vA[0]; long double sum = 0;
            for (auto x : vA) { mn = min(mn, x); mx = max(mx, x); sum += x; }
            long long avg = (long long)(sum / vA.size() + 0.5);
            cellA = to_string(avg) + " (" + to_string(mn) + " ; " + to_string(mx) + ")";
        }

        // TSPB
        auto &vB = set_method_objs["TSPB"][final_name];
        string cellB;
        if (vB.empty()) cellB = "0";
        else {
            long long mn = vB[0], mx = vB[0]; long double sum = 0;
            for (auto x : vB) { mn = min(mn, x); mx = max(mx, x); sum += x; }
            long long avg = (long long)(sum / vB.size() + 0.5);
            cellB = to_string(avg) + " (" + to_string(mn) + " ; " + to_string(mx) + ")";
        }

        FR << '"' << final_name << '"' << "," << cellA << "," << cellB << "\n";
    }
    FR.close();

    string final_time_csv = (filesystem::path(outdir) / "final_table_time.csv").string();
    ofstream FT(final_time_csv);
    FT << "method,TSPA,TSPB\n";
    for (auto &final_name : final_method_names) {
        auto &tA = set_method_times["TSPA"][final_name];
        string cellA;
        if (tA.empty()) cellA = "0";
        else {
            double mn = tA[0], mx = tA[0], sum = 0;
            for (auto x : tA) { mn = min(mn, x); mx = max(mx, x); sum += x; }
            double avg = sum / tA.size();
            std::ostringstream oss; oss << fixed << setprecision(3) << avg << " (" << mn << " ; " << mx << ")";
            cellA = oss.str();
        }

        auto &tB = set_method_times["TSPB"][final_name];
        string cellB;
        if (tB.empty()) cellB = "0";
        else {
            double mn = tB[0], mx = tB[0], sum = 0;
            for (auto x : tB) { mn = min(mn, x); mx = max(mx, x); sum += x; }
            double avg = sum / tB.size();
            std::ostringstream oss; oss << fixed << setprecision(3) << avg << " (" << mn << " ; " << mx << ")";
            cellB = oss.str();
        }

        FT << '"' << final_name << '"' << "," << cellA << "," << cellB << "\n";
    }
    FT.close();

    cout << "Wrote final tables: " << final_results_csv << ", " << final_time_csv << "\n";

    return 0;
}