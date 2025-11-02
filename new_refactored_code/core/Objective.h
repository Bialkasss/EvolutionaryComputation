#pragma once
#include <vector>
#include <string>
#include "Instance.h"

using namespace std;

// Group these in a namespace for clarity
namespace Objective {

    // (Moved from main.cpp)
    long long tour_edge_sum(const vector<int>& tour, const Instance& inst);
    long long tour_cost_sum(const vector<int>& tour, const Instance& inst);
    
    // (Renamed from 'objective' to be more specific)
    long long calculate(const vector<int>& tour, const Instance& inst);

    // (Renamed from 'check_solution')
    bool check(const vector<int>& tour, const Instance& inst, string* why = nullptr);

} // namespace Objective