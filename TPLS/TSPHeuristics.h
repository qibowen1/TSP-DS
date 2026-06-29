#pragma once
#include <vector>
#include <random>
#include "MTSPDSGraph.h"

namespace tsp_heur {

    std::vector<std::vector<int>> sweepAssign(
        const MTSPDSGraph& g,
        const std::vector<int>& selectedStations,
        const std::vector<int>& remainingCustomers,
        std::mt19937& rng
    );

    std::vector<int> tspNearestNeighbor(const MTSPDSGraph& g, const std::vector<int>& nodes, int depot);
    void twoOptImprove(const MTSPDSGraph& g, std::vector<int>& tour, int maxIters = 2000);

    double tourTime(const MTSPDSGraph& g, const std::vector<int>& tour);

} // namespace

