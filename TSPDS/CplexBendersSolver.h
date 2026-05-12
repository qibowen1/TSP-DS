#pragma once
#include "TSPDSGraph.h"
#include "TSPDSSolution.h"

#include <ilcplex/ilocplex.h>
ILOSTLBEGIN

#include <vector>
#include <unordered_map>
#include <limits>

class CplexBendersSolver {
public:
    struct Params {
        double time_limit_sec = 600.0;
        int threads = 1;
        double mip_gap = 0.0;
        bool verbose = true;

        double cut_eps = 1e-6;
        double vio_eps = 1e-6;
        int max_cuts_per_call = 50;

        double bsp_time_limit_sec = 600.0;
        int bsp_threads = 3;

        int multifit_iters = 30;
    };

    CplexBendersSolver() : params() {}                 // Ä¬ÈÏ¹¹Ôì
    explicit CplexBendersSolver(const Params& p) : params(p) {}

    bool solve(const TSPDSGraph& g, TSPDSSolution& sol);

    

private:
    Params params;
};


