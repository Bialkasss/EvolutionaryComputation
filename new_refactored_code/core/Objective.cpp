#include "Objective.h"
#include <vector>

using namespace std;

// All implementations are placed inside the Objective namespace
namespace Objective {

    /**
     * @brief (Copied from main.cpp)
     */
    long long tour_edge_sum(const vector<int>& tour, const Instance& inst) {
        long long s = 0;
        int m = (int)tour.size();
        if (m == 0) return 0;
        for (int i = 0; i < m; ++i) {
            int u = tour[i];
            int v = tour[(i+1)%m]; // Wraps around
            s += inst.D[u][v]; // Access D from the Instance object
        }
        return s;
    }

    /**
     * @brief (Copied from main.cpp)
     */
    long long tour_cost_sum(const vector<int>& tour, const Instance& inst) {
        long long s = 0;
        for (int v : tour) s += inst.cost[v]; // Access cost from the Instance object
        return s;
    }

    /**
     * @brief (Copied from main.cpp, renamed to 'calculate')
     */
    long long calculate(const vector<int>& tour, const Instance& inst) {
        // Calls the other helpers in this same namespace
        return tour_edge_sum(tour, inst) + tour_cost_sum(tour, inst);
    }

    /**
     * @brief (Copied from main.cpp, renamed to 'check')
     */
    bool check(const vector<int>& tour, const Instance& inst, string* why) {
        int K = inst.K;
        if ((int)tour.size() != K) { if (why) *why = "wrong size"; return false; }

        vector<int> seen(inst.N, 0);
        
        for (int v : tour) {
            if (v < 0 || v >= inst.N) { if (why) *why = "index out of range"; return false; }
            if (seen[v]) { if (why) *why = "repeated node"; return false; }
            seen[v] = 1;
        }
        return true;
    }

} // namespace Objective
