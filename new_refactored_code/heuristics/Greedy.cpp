#include "Greedy.h"

#include <numeric>     // For iota
#include <algorithm>   // For shuffle, swap, sort
#include <climits>     // For LLONG_MAX
#include <vector>
#include <random>

#include "../utils/Helpers.h" // For insert_at, argmin_random_tie

using namespace std;

namespace GreedyHeuristics {

// CH_RANDOM
vector<int> random_solution(const Instance& I, mt19937& rng, int start_anchor) {
    
    vector<int> nodes(I.N);
    iota(nodes.begin(), nodes.end(), 0); // fill with 0,1,2,...,N-1

    if (start_anchor >= 0) {
        swap(nodes[0], nodes[start_anchor]);
        // choose K-1 from remaining and include anchor
        shuffle(nodes.begin()+1, nodes.end(), rng); // shuffle all but the anchor

        vector<int> sel; 
        sel.reserve(I.K);
        sel.push_back(nodes[0]); // include anchor
        for (int i = 1; (int)sel.size() < I.K; ++i) sel.push_back(nodes[i]);
        shuffle(sel.begin(), sel.end(), rng); // shuffle the selected nodes
        return sel;
    } else {
        shuffle(nodes.begin(), nodes.end(), rng);
        vector<int> sel(nodes.begin(), nodes.begin()+I.K); // take first K
        shuffle(sel.begin(), sel.end(), rng); // shuffle the selected nodes
        return sel;
    }
}

// CH_NN_END_ONLY
vector<int> nn_end_only(const Instance& I, int start, mt19937& rng) {
    const auto& D = I.D; const auto& C = I.cost; int K = I.K, N = I.N; 

    vector<char> used(N, 0);
    vector<int> path; path.reserve(K);

    path.push_back(start); used[start] = 1;

    while ((int)path.size() < K) {
        int u = path.back(); // last node in the current path
        long long best = LLONG_MAX;
        vector<int> cand; // candidates for the next node to add

        for (int j = 0; j < N; ++j) if (!used[j]) {
            // Objective: distance from last node + cost of new node
            long long delta = (long long)D[u][j] + C[j];
            if (delta < best) { best = delta; cand.assign(1, j); }
            else if (delta == best) cand.push_back(j);
        }
        
        // random tie break among candidates
        uniform_int_distribution<int> rd(0, (int)cand.size()-1);
        int j = cand[rd(rng)];
        path.push_back(j);
        used[j] = 1;
    }
    return path; // scoring adds closing edge
}

// CH_NN_INSERT_ANYWHERE_PATH
vector<int> nn_path_insert_anywhere(const Instance& I, int start, mt19937& rng) {
    const auto& D = I.D; const auto& C = I.cost; int K = I.K, N = I.N;

    vector<char> used(N, 0);
    vector<int> path; path.reserve(K);

    path.push_back(start); used[start] = 1;

    while ((int)path.size() < K) {
        long long best_val = LLONG_MAX; //same as before we set best score to infinity if we find a better one then we update it 
        //as well as the best node and position (see below)
        int best_j = -1, best_pos = -1;// we track the bestscores to choose at the end of iteration

        for (int j = 0; j < N; ++j) if (!used[j]) { // for each unused node
            // evaluate all positions on the current PATH
            long long local_best = LLONG_MAX; int local_pos = -1;// best position for this j - to witch node it will be inserted - BUT
            //only if it will give us the best score overall - so we compare it to best_val below
            int m = (int)path.size();// current path size

            if (m == 1) { // special case: only one node
                long long delta = D[j][path[0]]; // Path: j -> path[0] or path[0] -> j
                local_best = delta; local_pos = 0; // prepend
            } 
            else {
                // prepend -- insert at pos 0
                {
                    long long delta = D[j][path[0]];
                    if (delta < local_best) { local_best = delta; local_pos = 0; }
                }
                // internal positions -- insert at pos t+1
                for (int t = 0; t+1 < m; ++t) {
                    int u = path[t], v = path[t+1];
                    long long delta = (long long)D[u][j] + D[j][v] - D[u][v];
                    if (delta < local_best) { local_best = delta; local_pos = t+1; }
                }
                // append -- insert at pos m
                {
                    long long delta = D[path[m-1]][j];
                    if (delta < local_best) { local_best = delta; local_pos = m; }
                }
            }

            long long total = local_best + C[j]; // total cost = edge_delta + node_cost
            // compare to overall best
            if (total < best_val) { best_val = total; best_j = j; best_pos = local_pos; }
            else if (total == best_val) {
                if (uniform_int_distribution<int>(0,1)(rng)) { best_j = j; best_pos = local_pos; }
            }
        }
        insert_at(path, best_pos, best_j); // From utils/Helpers.h
        used[best_j] = 1;
    }
    return path;
}


} // namespace GreedyHeuristics
