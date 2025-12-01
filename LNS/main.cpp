// Example usage in your main or experiment loop
#include "LNS.h"

// ... setup Instance I ...

double t_max = 4500.0; // 4.5 seconds (example MSLS average)

// Run LNS with Local Search enabled
vector<int> best_tour_ls = LNS::run(I, t_max, true, rng);

// Run LNS without Local Search
vector<int> best_tour_no_ls = LNS::run(I, t_max, false, rng);