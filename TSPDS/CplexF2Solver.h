#pragma once
#include "TSPDSGraph.h"
#include "TSPDSSolution.h"

#include <ilcplex/ilocplex.h>
ILOSTLBEGIN

#include <vector>
#include <string>
#include <memory>
#include <algorithm>   // std::max

class CplexF2Solver {
public:
    struct Params {
        double time_limit_sec;
        int threads;
        double mip_gap;     // 0=证明最优；也可设 1e-4
        bool verbose;

        // cut 分离参数
        double cut_eps;     // support graph 阈值
        double vio_eps;     // 违反阈值
        int max_cuts_per_call;

        Params()
            : time_limit_sec(600.0),
            threads(1),
            mip_gap(0.0),
            verbose(true),
            cut_eps(1e-6),
            vio_eps(1e-6),
            max_cuts_per_call(50) {
        }
    };

    CplexF2Solver() : params() {}                      // 默认构造
    explicit CplexF2Solver(const Params& p) : params(p) {}  // 传参构造

    bool solve(const TSPDSGraph& g, TSPDSSolution& sol);

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
