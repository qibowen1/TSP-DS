#include "DroneSchedulerCombine.h"
#include <algorithm>
#include <numeric>
#include <limits>

static inline double roundTripTime(const MTSPDSGraph& g, int s, int j) {
    return 2.0 * g.drone_time[s][j];
}

std::uint64_t DroneSchedulerCombine::fnv1a64(const std::vector<int>& a) {
    std::uint64_t h = 1469598103934665603ULL;
    for (int v : a) {
        std::uint64_t x = (std::uint64_t)(unsigned)v + 0x9e3779b97f4a7c15ULL;
        h ^= x;
        h *= 1099511628211ULL;
    }
    return h;
}

StationSchedule DroneSchedulerCombine::scheduleWithCache(int s, const std::vector<int>& custs) {
    std::vector<int> keyvec = custs;
    std::sort(keyvec.begin(), keyvec.end());
    DroneScheduleKey key{ s, fnv1a64(keyvec) };

    auto it = cache.find(key);
    if (it != cache.end()) return it->second;

    StationSchedule sch = scheduleCOMBINE(s, keyvec);
    cache.emplace(key, sch);
    return sch;
}

StationSchedule DroneSchedulerCombine::scheduleLPT(int s, const std::vector<int>& custs) const {
    StationSchedule res;
    int D = graph.DN;
    res.tasks.assign(D, {});
    res.load.assign(D, 0.0);

    std::vector<int> jobs = custs;
    std::sort(jobs.begin(), jobs.end(), [&](int a, int b) {
        return roundTripTime(graph, s, a) > roundTripTime(graph, s, b);
        });

    for (int j : jobs) {
        int best = 0;
        for (int d = 1; d < D; ++d) if (res.load[d] < res.load[best]) best = d;
        res.tasks[best].push_back(j);
        res.load[best] += roundTripTime(graph, s, j);
    }
    res.makespan = *std::max_element(res.load.begin(), res.load.end());
    return res;
}

// MULTIFIT-like feasibility test via First-Fit Decreasing (FFD)
bool DroneSchedulerCombine::feasibleFFD(int s, const std::vector<int>& custs, double T, StationSchedule* out) const {
    int D = graph.DN;
    std::vector<double> cap(D, T);
    std::vector<std::vector<int>> bins(D);

    std::vector<int> jobs = custs;
    std::sort(jobs.begin(), jobs.end(), [&](int a, int b) {
        return roundTripTime(graph, s, a) > roundTripTime(graph, s, b);
        });

    for (int j : jobs) {
        double w = roundTripTime(graph, s, j);
        bool placed = false;
        for (int d = 0; d < D; ++d) {
            if (cap[d] + 1e-12 >= w) {
                cap[d] -= w;
                bins[d].push_back(j);
                placed = true;
                break;
            }
        }
        if (!placed) return false;
    }

    if (out) {
        out->tasks = std::move(bins);
        out->load.assign(D, 0.0);
        for (int d = 0; d < D; ++d) {
            double used = 0.0;
            for (int j : out->tasks[d]) used += roundTripTime(graph, s, j);
            out->load[d] = used;
        }
        out->makespan = *std::max_element(out->load.begin(), out->load.end());
    }
    return true;
}

// COMBINE: LPT gives tight UB, then MULTIFIT (here: FFD feasibility) refines
StationSchedule DroneSchedulerCombine::scheduleCOMBINE(int s, const std::vector<int>& custs) const {
    StationSchedule empty;
    if (custs.empty()) {
        empty.tasks.assign(graph.DN, {});
        empty.load.assign(graph.DN, 0.0);
        empty.makespan = 0.0;
        return empty;
    }

    StationSchedule lpt = scheduleLPT(s, custs);
    double UB = lpt.makespan;

    double sum = 0.0, mx = 0.0;
    for (int j : custs) {
        double w = roundTripTime(graph, s, j);
        sum += w; mx = std::max(mx, w);
    }
    double LB = std::max(mx, sum / (double)graph.DN);

    // binary search on T in [LB, UB]
    StationSchedule best = lpt;
    double lo = LB, hi = UB;
    for (int it = 0; it < 40; ++it) {
        double mid = 0.5 * (lo + hi);
        StationSchedule tmp;
        if (feasibleFFD(s, custs, mid, &tmp)) {
            hi = mid;
            best = tmp;
        }
        else {
            lo = mid;
        }
    }
    return best;
}
