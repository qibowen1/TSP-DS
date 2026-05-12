#pragma once
// CplexF1Solver.h
#pragma once
#include "TSPDSGraph.h"
#include "TSPDSSolution.h"
#include "VNSLogger.h"
#include <vector>
#include <iostream>
#include <limits>
#include <cmath>

class CplexF1Solver {
public:
    struct Params {
        double time_limit_sec = 600.0;
        int threads = 1;
        double mip_gap = 0.0;   // 0=证明最优；可设 1e-4 等
        bool verbose = true;
    };
    VNSLogger logger;  // 添加日志记录器成员
	TSPDSAlgorithmParams aparams;

    explicit CplexF1Solver(Params p , TSPDSGraph graph) : params(p) , logger(graph, aparams) {}
    bool solve(const TSPDSGraph& g, TSPDSSolution& sol);


    // 日志相关的方法（委托给logger）
    void enableLogging(bool enabled) {
        logger.setLoggingEnabled(enabled);
    }
    void setLogFilename(const std::string& filename) {
        logger.setLogFilename(filename);
    }
private:
    Params params;

    static double maxTruckTime(const TSPDSGraph& g) {
        double mx = 0.0;
        int n = (int)g.truck_time.size();
        for (int i = 0; i < n; ++i)
            for (int j = 0; j < n; ++j)
                mx = std::max(mx, g.truck_time[i][j]);
        return mx;
    }
};
