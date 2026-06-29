#pragma once
#include <vector>
#include "MTSPDSGraph.h"
#include "MTSPDSSolution.h"

class SetPartitioningPolisher {
public:
    explicit SetPartitioningPolisher(const MTSPDSGraph& g) : graph(g) {}

    // Returns false when no feasible column combination is found.
    bool solve(const std::vector<MTSPDSSolution>& pool,
        const MTSPDSSolution& incumbent,
        MTSPDSSolution& out);

private:
    const MTSPDSGraph& graph;
};
