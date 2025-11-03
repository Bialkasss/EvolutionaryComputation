#pragma once

#include <vector>
#include <string>
#include "Common.h" // We'll put Point in here

// Use the standard 'using' directive
using namespace std;

// The struct definition is copied directly from main.cpp
struct Instance {
    string name;
    vector<Point> pts;
    vector<int>    cost;
    vector<vector<int>> D;
    int N = 0;
    int K = 0;
};

// This is the "declaration" or "prototype" of the function.
// It tells other .cpp files that this function exists and what
// its signature is. The *implementation* (the body) will
// be in Instance.cpp.
bool read_instance(const string& path, Instance& inst);

