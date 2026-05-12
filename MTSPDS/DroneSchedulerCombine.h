#pragma once
#include <vector>
#include <unordered_map>
#include <cstdint>
#include "MTSPDSGraph.h"
#include "MTSPDSSolution.h"

struct DroneScheduleKey {
    int station = -1;
    std::uint64_t hash = 0;
    bool operator==(const DroneScheduleKey& o) const { return station == o.station && hash == o.hash; }
};

struct DroneScheduleKeyHasher {
    std::size_t operator()(const DroneScheduleKey& k) const noexcept {
        return (std::size_t)k.hash ^ (std::size_t)(k.station * 1315423911u);
    }
};

class DroneSchedulerCombine {
public:
    explicit DroneSchedulerCombine(const MTSPDSGraph& g) : graph(g) {}

    // schedule for station s with assigned customers "custs"
    StationSchedule scheduleWithCache(int s, const std::vector<int>& custs);

private:
    const MTSPDSGraph& graph;

    std::unordered_map<DroneScheduleKey, StationSchedule, DroneScheduleKeyHasher> cache;

    static std::uint64_t fnv1a64(const std::vector<int>& sortedVals);

    StationSchedule scheduleLPT(int s, const std::vector<int>& custs) const;
    bool feasibleFFD(int s, const std::vector<int>& custs, double T, StationSchedule* out) const;
    StationSchedule scheduleCOMBINE(int s, const std::vector<int>& custs) const;
};
