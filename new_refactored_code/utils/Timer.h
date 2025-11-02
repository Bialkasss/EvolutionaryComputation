#pragma once
#include <chrono>

// --------------------- Timer ---------------------
using hires_clock = std::chrono::high_resolution_clock;
using duration_ms = std::chrono::duration<double, std::milli>;

struct Stopwatch {
    hires_clock::time_point start;
    Stopwatch() : start(hires_clock::now()) {}
    void reset() { start = hires_clock::now(); }
    double elapsed_ms() const {
        return duration_ms(hires_clock::now() - start).count();
    }
};