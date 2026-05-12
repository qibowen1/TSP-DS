#pragma once
#include <vector>
#include <unordered_set>
#include <unordered_map>
#include <sstream>
#include <iomanip>
#include <limits>
#include <cmath>
#include <string>
#include <algorithm>

// 你已有：
#include "TSPDSGraph.h"
#include "TSPDSSolution.h"

struct ValidationIssue {
    enum Level { WARN, ERROR };
    Level level = ERROR;
    std::string msg;
};

struct ValidationReport {
    bool ok = true;
    std::vector<ValidationIssue> issues;

    void addError(const std::string& s) {
        ok = false;
        issues.push_back({ ValidationIssue::ERROR, s });
    }
    void addWarn(const std::string& s) {
        issues.push_back({ ValidationIssue::WARN, s });
    }

    std::string toString() const {
        std::ostringstream oss;
        oss << "[Validation] ok=" << (ok ? "true" : "false") << ", issues=" << issues.size() << "\n";
        for (size_t i = 0; i < issues.size(); ++i) {
            oss << "  - " << (issues[i].level == ValidationIssue::ERROR ? "ERROR" : "WARN")
                << ": " << issues[i].msg << "\n";
        }
        return oss.str();
    }
};

static inline bool nearlyEqual(double a, double b, double eps) {
    return std::fabs(a - b) <= eps * (1.0 + std::max(std::fabs(a), std::fabs(b)));
}

static inline double droneRoundTrip(const TSPDSGraph& g, int station, int i) {
    return g.drone_time[station][i] + g.drone_time[i][station];
}

// 计算 truck 总时间（按 route 边）
static inline double computeTruckTimeFromRoute(const TSPDSGraph& g, const std::vector<int>& route) {
    double t = 0.0;
    for (int k = 0; k + 1 < (int)route.size(); ++k) {
        int a = route[k], b = route[k + 1];
        t += g.truck_time[a][b];
    }
    return t;
}

// 计算 station activation time：depot -> ... -> first station 的前缀时间
static inline double computeActivationTimeFromRoute(const TSPDSGraph& g, const std::vector<int>& route, int depot, int station) {
    double t = 0.0;
    for (int k = 0; k + 1 < (int)route.size(); ++k) {
        int a = route[k], b = route[k + 1];
        t += g.truck_time[a][b];
        if (b == station) return t;
    }
    return std::numeric_limits<double>::infinity(); // station 不在 route
}

// 计算无人机并行完成时间：activation + max_v sum(rt(i) for i in drone v)
static inline double computeDroneCompletionFromAssignments(
    const TSPDSGraph& g,
    int station,
    int V,
    const std::unordered_map<int, std::vector<int>>& drone_assignments,
    double activationTime
) {
    if (V <= 0) return activationTime;

    std::vector<double> load(V, 0.0);
    for (const auto& kv : drone_assignments) {
        int v = kv.first;
        if (v < 0 || v >= V) continue;
        for (int node : kv.second) {
            load[v] += droneRoundTrip(g, station, node);
        }
    }
    double mx = 0.0;
    for (double x : load) mx = std::max(mx, x);
    return activationTime + mx;
}

/**
 * 验证 TSP-DS 解是否合法
 * - eps：数值容忍
 * - checkTimes：是否校验 sol.truck_completion_time / station_activation_time / drone_completion_time / makespan 的一致性
 * - checkInternalFlags：是否检查 served_by_* / node_to_drone / pos_in_truck 等内部字段自洽（有些字段你可能没填全，关掉可避免误报）
 */
static inline bool validateTSPDSolution(
    const TSPDSGraph& g,
    const TSPDSSolution& sol,
    ValidationReport& rep,
    double eps = 1e-6,
    bool checkTimes = true,
    bool checkInternalFlags = true
) {
    rep.ok = true;
    rep.issues.clear();

    const int n = (int)g.truck_time.size();
    if (n <= 0) { rep.addError("Graph has no nodes."); return rep.ok; }

    const int depot = g.depot;
    const int station = g.drone_station;
    const int V = std::max(1, g.drone_count);

    auto inRangeNode = [&](int v) { return 0 <= v && v < n; };

    // ---------- 0) 基本 size 检查 ----------
    if ((int)sol.truck_route.size() < 2) rep.addError("truck_route is too short.");
    if (!inRangeNode(depot) || !inRangeNode(station)) rep.addError("depot/station index out of range.");

    // ---------- 1) truck_route 合法性 ----------
    if (!sol.truck_route.empty()) {
        if (sol.truck_route.front() != depot) rep.addError("truck_route does not start at depot.");
        if (sol.truck_route.back() != depot) rep.addError("truck_route does not end at depot.");
    }

    // 检查节点索引合法 + station 是否出现
    int stationCount = 0;
    for (int v : sol.truck_route) {
        if (!inRangeNode(v)) rep.addError("truck_route contains invalid node index: " + std::to_string(v));
        if (v == station) stationCount++;
    }
    if (station == depot) {
        // 退化情况（一般不会）
        rep.addWarn("station == depot (degenerate case).");
    }
    else {
        if (stationCount != 1) {
            rep.addError("station must appear exactly once in truck_route, but got " + std::to_string(stationCount));
        }
    }

    // 检查卡车访问节点是否重复（允许 depot 重复两次；不允许其它重复）
    {
        std::unordered_set<int> seen;
        for (int k = 0; k < (int)sol.truck_route.size(); ++k) {
            int v = sol.truck_route[k];
            if (v == depot) continue;
            if (seen.count(v)) {
                rep.addError("truck_route visits node more than once: " + std::to_string(v));
            }
            else {
                seen.insert(v);
            }
        }
    }

    // ---------- 2) 服务唯一性：每个客户必须恰好由 truck 或 drone 服务 ----------
    // 定义客户集合：除 depot 外的所有点（station 通常也算必须访问点）
    std::vector<int> servedByTruck(n, 0), servedByDrone(n, 0);

    // truck_route 中出现的（含 station）视为 truck 服务
    for (int v : sol.truck_route) {
        if (inRangeNode(v)) servedByTruck[v] = 1;
    }

    // drone_assignments -> drone 服务
    std::unordered_set<int> droneTaskSet;
    for (const auto& kv : sol.drone_assignments) {
        int droneId = kv.first;
        if (droneId < 0 || droneId >= V) {
            rep.addWarn("drone_assignments has key out of range [0,V): " + std::to_string(droneId));
        }
        for (int node : kv.second) {
            if (!inRangeNode(node)) {
                rep.addError("drone_assignments contains invalid node index: " + std::to_string(node));
                continue;
            }
            servedByDrone[node]++;

            // 同一个节点不能被多个无人机重复分配
            if (droneTaskSet.count(node)) {
                rep.addError("A node is assigned to drones more than once: " + std::to_string(node));
            }
            else {
                droneTaskSet.insert(node);
            }
        }
    }

    // depot/station 不允许被 drone 服务
    if (servedByDrone[depot] > 0)   rep.addError("depot is assigned to drone tasks (invalid).");
    if (servedByDrone[station] > 0) rep.addError("station is assigned to drone tasks (invalid).");

    // 每个非 depot 节点：要么 truck，要么 drone；station 必须 truck
    for (int i = 0; i < n; ++i) {
        if (i == depot) continue;

        int t = servedByTruck[i];
        int d = servedByDrone[i] > 0 ? 1 : 0;

        if (i == station) {
            if (t != 1) rep.addError("station is not served by truck (must be).");
            if (d != 0) rep.addError("station is served by drone (must not).");
            continue;
        }

        // 普通客户：必须 exactly one
        if (t + d != 1) {
            rep.addError("Node " + std::to_string(i) + " is not served exactly once (truck=" +
                std::to_string(t) + ", drone=" + std::to_string(d) + ").");
        }
    }

    // ---------- 3) truck-only / drone-eligible / range 可行性 ----------
    // truck_only 强制不能被无人机服务
    if ((int)g.is_truck_only.size() == n) {
        for (int i = 0; i < n; ++i) {
            if (i == depot || i == station) continue;
            if (g.is_truck_only[i] && servedByDrone[i] > 0) {
                rep.addError("truck-only node is served by drone: " + std::to_string(i));
            }
        }
    }

    // drone 服务的点必须 drone-eligible 且满足 range
    for (int i = 0; i < n; ++i) {
        if (servedByDrone[i] <= 0) continue;

        if ((int)g.is_drone_eligible.size() == n) {
            if (!g.is_drone_eligible[i]) {
                rep.addError("Node served by drone but is_drone_eligible=false: " + std::to_string(i));
            }
        }

        double rt = droneRoundTrip(g, station, i);
        if (rt > g.drone_range + eps) {
            rep.addError("Node served by drone but round-trip exceeds drone_range: node=" +
                std::to_string(i) + ", rt=" + std::to_string(rt) +
                ", range=" + std::to_string(g.drone_range));
        }
    }

    // ---------- 4) node_to_drone / served_by_* / pos_in_truck 的内部一致性（可选） ----------
    if (checkInternalFlags) {
        // served_by_truck / served_by_drone size
        if ((int)sol.served_by_truck.size() != n) rep.addWarn("served_by_truck size != n (may be uninitialized for exact solver).");
        if ((int)sol.served_by_drone.size() != n) rep.addWarn("served_by_drone size != n (may be uninitialized for exact solver).");

        // node_to_drone 与 drone_assignments 一致
        for (const auto& kv : sol.node_to_drone) {
            int node = kv.first;
            int dr = kv.second;
            if (!inRangeNode(node)) {
                rep.addWarn("node_to_drone contains invalid node: " + std::to_string(node));
                continue;
            }
            if (servedByDrone[node] <= 0) {
                rep.addWarn("node_to_drone has entry but node not in drone_assignments: node=" + std::to_string(node));
            }
            if (dr < 0 || dr >= V) rep.addWarn("node_to_drone has drone id out of range: node=" + std::to_string(node) + " drone=" + std::to_string(dr));
        }


        if (station != depot && sol.pos_station_in_truck >= 0) {
            if (sol.pos_station_in_truck >= (int)sol.truck_route.size() ||
                sol.truck_route[sol.pos_station_in_truck] != station) {
                rep.addWarn("pos_station_in_truck mismatch.");
            }
        }
    }

    // ---------- 5) 时间一致性（可选） ----------
    if (checkTimes && !sol.truck_route.empty() && station != depot) {
        double truckT = computeTruckTimeFromRoute(g, sol.truck_route);
        double actT = computeActivationTimeFromRoute(g, sol.truck_route, depot, station);
        double droneT = computeDroneCompletionFromAssignments(g, station, V, sol.drone_assignments, actT);
        double makespanT = std::max(truckT, droneT);

        if (!std::isfinite(actT)) rep.addError("Activation time computed as INF: station not reachable in truck_route.");

        // 允许一些误差（你 truck_time/drone_time 是 int，通常应当完全一致）
        if (!nearlyEqual(sol.truck_completion_time, truckT, eps)) {
            rep.addWarn("truck_completion_time mismatch: sol=" + std::to_string(sol.truck_completion_time) +
                " recompute=" + std::to_string(truckT));
        }
        if (!nearlyEqual(sol.station_activation_time, actT, eps)) {
            rep.addWarn("station_activation_time mismatch: sol=" + std::to_string(sol.station_activation_time) +
                " recompute=" + std::to_string(actT));
        }
        // sol.drone_completion_time 你定义为“已加激活时间的”
        if (!nearlyEqual(sol.drone_completion_time, droneT, eps)) {
            rep.addWarn("drone_completion_time mismatch: sol=" + std::to_string(sol.drone_completion_time) +
                " recompute=" + std::to_string(droneT));
        }
        if (!nearlyEqual(sol.makespan, makespanT, eps)) {
            rep.addWarn("makespan mismatch: sol=" + std::to_string(sol.makespan) +
                " recompute=" + std::to_string(makespanT));
        }
    }

    return rep.ok;
}

