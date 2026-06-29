#pragma once
#include <vector>
#include <random>
#include "MTSPDSGraph.h"
#include "MTSPDSSolution.h"
#include "DroneSchedulerCombine.h"
#include <chrono>

struct TwoPhaseParams {
	int time_limit_sec = 600; // time limit in seconds
    int rho = 5;          // restarts (paper: 50)
    int delta = 50;        // nonimproving ILS iterations (paper: 50)
    double pi_base = 1.25; // initial sparsification factor (paper: 1.25)
    double phi = 0.2;      // fraction (paper: 0.2)
    double lambda = 2.0;   // factor (paper: 2)
    int max_pool = 2000;   // practical cap
    unsigned seed = 1;
};

class TwoPhaseMatheuristic {
public:
    TwoPhaseMatheuristic(const MTSPDSGraph& g, const TwoPhaseParams& p);

    MTSPDSSolution solve(double opt);

private:
    using Clock = std::chrono::steady_clock;
    Clock::time_point deadline_ = Clock::time_point::max();

    inline bool timeUp() const {
        return Clock::now() >= deadline_;
    };

private:
    const MTSPDSGraph& graph;
    TwoPhaseParams params;
    std::mt19937 rng;

    DroneSchedulerCombine scheduler;

    // solution pool
    std::vector<MTSPDSSolution> pool;

    // helpers
    MTSPDSSolution emptySolution() const;

    MTSPDSSolution constructInitialSolution();
    MTSPDSSolution improveSolution(const MTSPDSSolution& start);

    // evaluation & feasibility
    void evaluate(MTSPDSSolution& S);
    bool feasibleBasic(const MTSPDSSolution& S) const;

    // pool management
    void addToPool(const MTSPDSSolution& S);

    // polish (Phase 2)
    MTSPDSSolution polish(const MTSPDSSolution& bestFromPhase1);

    // --- RVND / neighborhoods ---
    MTSPDSSolution rvndLocalSearch(MTSPDSSolution S, double pi);

    bool tryRelocate(MTSPDSSolution& S, double pi);
    bool trySwap(MTSPDSSolution& S, double pi);
    bool tryMultirangeDroneRelocate(MTSPDSSolution& S);
    bool tryDroneToTruckRelocate(MTSPDSSolution& S, double pi);
    bool tryTruckToDroneRelocate(MTSPDSSolution& S);

    // granular arcs
    std::vector<std::pair<int, int>> buildGranularArcs(double pi) const;
    bool arcInGranular(int u, int v, const std::vector<std::pair<int, int>>& gran) const;

    // shake
    MTSPDSSolution shake(MTSPDSSolution S);
};
