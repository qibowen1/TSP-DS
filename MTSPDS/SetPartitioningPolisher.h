#pragma once
#include <vector>
#include "MTSPDSGraph.h"
#include "MTSPDSSolution.h"

class SetPartitioningPolisher {
public:
    explicit SetPartitioningPolisher(const MTSPDSGraph& g) : graph(g) {}

    // return false if no CPLEX (or not solved), caller can fallback
    bool solve(const std::vector<MTSPDSSolution>& pool,
        const MTSPDSSolution& incumbent,
        MTSPDSSolution& out);

private:
    const MTSPDSGraph& graph;
};
