#include "LocalSearchOperators.h"
#include <algorithm>
#include <cmath>
#include <iostream>
#include <vector>
#include <unordered_set>
#include <chrono>
#include <array>
#include <string>
#include <iomanip>
#include <cassert>


struct OpPerfStat {
    long long calls = 0;
    long long accepts = 0;
    double op_ms = 0.0;      // 仅“算子本身”耗时
    double eval_ms = 0.0;    // evaluateSolution 耗时
    double total_ms = 0.0;   // op_ms + eval_ms
    double accept_total_ms = 0.0; // 被接受的 move 的 total_ms

    double avgOpMs() const { return calls ? op_ms / calls : 0.0; }
    double avgEvalMs() const { return calls ? eval_ms / calls : 0.0; }
    double avgTotalMs() const { return calls ? total_ms / calls : 0.0; }
    double accRate() const { return calls ? (double)accepts / (double)calls : 0.0; }
};

static inline double toMs(const std::chrono::steady_clock::duration& d) {
    return std::chrono::duration<double, std::milli>(d).count();
}

static void printOpStats(
    const std::array<OpPerfStat, 32>& st,
    const std::array<std::string, 32>& names,
    const std::string& title = "VND Operator Stats"
) {
    std::cout << "\n========== " << title << " ==========\n";
    std::cout << std::left
        << std::setw(3) << "ID"
        << std::setw(28) << "Operator"
        << std::right
        << std::setw(10) << "Calls"
        << std::setw(10) << "Acc"
        << std::setw(10) << "AccRate"
        << std::setw(12) << "Op(ms)"
        << std::setw(12) << "Eval(ms)"
        << std::setw(12) << "Total(ms)"
        << std::setw(14) << "AvgTot(ms)"
        << "\n";

    for (int i = 0; i < (int)st.size(); ++i) {
        if (st[i].calls == 0) continue;
        std::cout << std::left
            << std::setw(3) << i
            << std::setw(28) << names[i]
            << std::right
                << std::setw(10) << st[i].calls
                << std::setw(10) << st[i].accepts
                << std::setw(10) << std::fixed << std::setprecision(3) << st[i].accRate()
                << std::setw(12) << std::fixed << std::setprecision(2) << st[i].op_ms
                << std::setw(12) << std::fixed << std::setprecision(2) << st[i].eval_ms
                << std::setw(12) << std::fixed << std::setprecision(2) << st[i].total_ms
                << std::setw(14) << std::fixed << std::setprecision(2) << st[i].avgTotalMs()
                << "\n";
    }
    std::cout << "======================================\n";
}


LocalSearchOperators::LocalSearchOperators(const TSPDSGraph& graph, const TSPDSAlgorithmParams& params)
    : graph(graph), params(params), utils(graph, params),
    gen(std::random_device{}()), uni(0.0, 1.0) {
}


bool LocalSearchOperators::acceptMove(const TSPDSSolution& curr,
    const TSPDSSolution& cand)
{
    const double EPS_T = 1e-6;
    const double EPS_S = 1e-9;

    if (cand.makespan < curr.makespan - EPS_T) return true;


    return false;
}



LocalSearchOperators::TargetScore LocalSearchOperators::calculateTargetScore(
    const TSPDSSolution& solution,
    double target) const
{
    TargetScore score;
    score.makespan = solution.makespan;
    if (!std::isfinite(target) || target <= 0.0) {
        score.phi = std::numeric_limits<double>::infinity();
        return score;
    }

    const int m = std::max(1, graph.drone_count);
    const double capacity = target - solution.station_activation_time;
    score.truck_over = std::max(0.0, solution.truck_completion_time - target);

    std::vector<double> loads(m, 0.0);
    double totalLoad = 0.0;
    double maxJob = 0.0;
    for (const auto& kv : solution.drone_assignments) {
        int d = kv.first;
        if (d < 0 || d >= m) continue;
        for (int node : kv.second) {
            if (node < 0 || node >= static_cast<int>(graph.nodes.size())) continue;
            if (node == graph.depot || node == graph.drone_station) continue;
            double p = utils.getDroneRoundTripTime(node);
            loads[d] += p;
            totalLoad += p;
            maxJob = std::max(maxJob, p);
        }
    }

    for (double load : loads) {
        score.drone_over += std::max(0.0, load - capacity);
    }
    score.total_over = std::max(0.0, totalLoad - static_cast<double>(m) * capacity);
    score.maxjob_over = std::max(0.0, maxJob - capacity);
    score.phi = score.truck_over + score.drone_over + score.total_over + score.maxjob_over;
    return score;
}

bool LocalSearchOperators::acceptTargetMove(
    const TSPDSSolution& curr,
    const TSPDSSolution& cand,
    double target) const
{
    const double EPS_PHI = 1e-6;
    const double EPS_T = 1e-6;
    TargetScore a = calculateTargetScore(curr, target);
    TargetScore b = calculateTargetScore(cand, target);
    if (b.phi < a.phi - EPS_PHI) return true;
    if (std::fabs(b.phi - a.phi) <= EPS_PHI && cand.makespan < curr.makespan - EPS_T) return true;
    return false;
}

TSPDSSolution LocalSearchOperators::targetLocalSearch(const TSPDSSolution& start, double target)
{
    TSPDSSolution current = start;
    if (current.makespan <= 0.0) utils.evaluateSolution(current, /*needCalDrone=*/true);
    if (!std::isfinite(target) || target <= 0.0) return current;

    const int n = static_cast<int>(graph.nodes.size());
    const int m = std::max(1, graph.drone_count);
    const int topTruck = std::max(1, params.target_vnd_top_truck);
    const int topDrone = std::max(1, params.target_vnd_top_drone);

    auto isDroneEligible = [&](int node) {
        if (node <= 0 || node >= n) return false;
        if (node == graph.depot || node == graph.drone_station) return false;
        if (!graph.is_drone_eligible.empty() && !graph.is_drone_eligible[node]) return false;
        double p = utils.getDroneRoundTripTime(node);
        if (std::isfinite(graph.drone_range) && p > graph.drone_range + 1e-9) return false;
        return true;
        };

    auto eraseTruckNode = [&](TSPDSSolution& s, int node) -> bool {
        auto it = std::find(s.truck_route.begin(), s.truck_route.end(), node);
        if (it == s.truck_route.end() || it == s.truck_route.begin() || it == s.truck_route.end() - 1) return false;
        s.truck_route.erase(it);
        s.served_by_truck[node] = false;
        s.served_by_drone[node] = true;
        return true;
        };

    auto moveDroneNodeToTruck = [&](TSPDSSolution& s, int node) -> bool {
        if (node <= 0 || node >= n) return false;
        if (node == graph.depot || node == graph.drone_station) return false;
        if (!s.served_by_drone[node] || s.served_by_truck[node]) return false;
        if (!utils.insertNodeToTruckRoute(s, node, InsertPolicy::ANYWHERE)) return false;
        s.served_by_drone[node] = false;
        s.served_by_truck[node] = true;
        return true;
        };

    auto evaluateAndMaybeAccept = [&](TSPDSSolution& best, TSPDSSolution cand) -> bool {
        utils.evaluateSolution(cand, /*needCalDrone=*/true);
        if (acceptTargetMove(best, cand, target)) {
            best = std::move(cand);
            return true;
        }
        return false;
        };

    auto reduceTruckOverTarget = [&]() -> bool {
        TargetScore score = calculateTargetScore(current, target);
        if (score.truck_over <= 1e-9) return false;

        utils.buildPosInTruck(current);
        std::vector<double> removeSaving;
        utils.buildRemoveSaving(current, removeSaving);

        struct Cand { int node; double key; };
        std::vector<Cand> candidates;
        for (int i = 1; i + 1 < static_cast<int>(current.truck_route.size()); ++i) {
            int node = current.truck_route[i];
            if (!isDroneEligible(node)) continue;
            if (!current.served_by_truck[node]) continue;
            double p = std::max(1.0, utils.getDroneRoundTripTime(node));
            double saving = (node < static_cast<int>(removeSaving.size())) ? removeSaving[node] : 0.0;
            candidates.push_back({ node, saving / p });
        }
        std::sort(candidates.begin(), candidates.end(), [](const Cand& a, const Cand& b) {
            return a.key > b.key;
            });
        if (static_cast<int>(candidates.size()) > topTruck) candidates.resize(topTruck);

        TSPDSSolution best = current;
        bool improved = false;
        for (const auto& candNode : candidates) {
            TSPDSSolution test = current;
            if (!eraseTruckNode(test, candNode.node)) continue;
            improved = evaluateAndMaybeAccept(best, std::move(test)) || improved;
        }
        if (improved) current = std::move(best);
        return improved;
        };

    auto reduceDroneOverTarget = [&](bool forceTotal) -> bool {
        TargetScore score = calculateTargetScore(current, target);
        const double capacity = target - current.station_activation_time;
        if (!forceTotal && score.drone_over <= 1e-9) return false;
        if (forceTotal && score.total_over <= 1e-9) return false;

        std::vector<double> loads(m, 0.0);
        for (const auto& kv : current.drone_assignments) {
            int d = kv.first;
            if (d < 0 || d >= m) continue;
            for (int node : kv.second) loads[d] += utils.getDroneRoundTripTime(node);
        }

        struct Cand { int node; double key; };
        std::vector<Cand> candidates;
        for (const auto& kv : current.drone_assignments) {
            int d = kv.first;
            if (d < 0 || d >= m) continue;
            if (!forceTotal && loads[d] <= capacity + 1e-9) continue;
            for (int node : kv.second) {
                if (node <= 0 || node >= n) continue;
                int bestPos = -1;
                double inc = utils.estimateBestInsertionIncrease(current, node, InsertPolicy::ANYWHERE, bestPos);
                if (!std::isfinite(inc) || bestPos < 0) continue;
                double p = std::max(1.0, utils.getDroneRoundTripTime(node));
                candidates.push_back({ node, inc / p });
            }
        }
        std::sort(candidates.begin(), candidates.end(), [](const Cand& a, const Cand& b) {
            return a.key < b.key;
            });
        if (static_cast<int>(candidates.size()) > topDrone) candidates.resize(topDrone);

        TSPDSSolution best = current;
        bool improved = false;
        for (const auto& candNode : candidates) {
            TSPDSSolution test = current;
            if (!moveDroneNodeToTruck(test, candNode.node)) continue;
            improved = evaluateAndMaybeAccept(best, std::move(test)) || improved;
        }
        if (improved) current = std::move(best);
        return improved;
        };

    auto targetCompoundExchange = [&]() -> bool {
        utils.buildPosInTruck(current);
        std::vector<double> removeSaving;
        utils.buildRemoveSaving(current, removeSaving);

        struct TruckCand { int node; double saving; };
        struct DroneCand { int node; double key; };
        std::vector<TruckCand> truck;
        std::vector<DroneCand> drone;

        for (int i = 1; i + 1 < static_cast<int>(current.truck_route.size()); ++i) {
            int node = current.truck_route[i];
            if (!isDroneEligible(node) || !current.served_by_truck[node]) continue;
            double saving = (node < static_cast<int>(removeSaving.size())) ? removeSaving[node] : 0.0;
            if (saving <= 0.0) continue;
            truck.push_back({ node, saving });
        }
        std::sort(truck.begin(), truck.end(), [](const TruckCand& a, const TruckCand& b) {
            return a.saving > b.saving;
            });
        if (static_cast<int>(truck.size()) > std::min(topTruck, 12)) truck.resize(std::min(topTruck, 12));

        for (const auto& kv : current.drone_assignments) {
            for (int node : kv.second) {
                int bestPos = -1;
                double inc = utils.estimateBestInsertionIncrease(current, node, InsertPolicy::ANYWHERE, bestPos);
                if (!std::isfinite(inc) || bestPos < 0) continue;
                double p = std::max(1.0, utils.getDroneRoundTripTime(node));
                drone.push_back({ node, inc / p });
            }
        }
        std::sort(drone.begin(), drone.end(), [](const DroneCand& a, const DroneCand& b) {
            return a.key < b.key;
            });
        if (static_cast<int>(drone.size()) > std::min(topDrone, 12)) drone.resize(std::min(topDrone, 12));

        TSPDSSolution best = current;
        bool improved = false;
        for (const auto& t : truck) {
            for (const auto& d : drone) {
                if (t.node == d.node) continue;
                TSPDSSolution test = current;
                if (!eraseTruckNode(test, t.node)) continue;
                if (!moveDroneNodeToTruck(test, d.node)) continue;
                improved = evaluateAndMaybeAccept(best, std::move(test)) || improved;
            }
        }
        if (improved) current = std::move(best);
        return improved;
        };

    auto targetDroneRepack = [&]() -> bool {
        TSPDSSolution test = current;
        utils.evaluateSolution(test, /*needCalDrone=*/true);
        if (acceptTargetMove(current, test, target)) {
            current = std::move(test);
            return true;
        }
        return false;
        };

    bool moved = true;
    int passes = 0;
    while (moved && passes < params.target_vnd_max_passes) {
        moved = false;
        moved = reduceDroneOverTarget(true) || moved;
        moved = reduceDroneOverTarget(false) || moved;
        moved = reduceTruckOverTarget() || moved;
        moved = targetCompoundExchange() || moved;
        moved = targetDroneRepack() || moved;
        moved = reduceTruckOverTarget() || moved;
        ++passes;
    }

    return current;
}


// ===== Adaptive probabilistic local search =====
TSPDSSolution LocalSearchOperators::localSearch(const TSPDSSolution& start, bool afterShake)
{
    (void)afterShake;
    using Clock = std::chrono::steady_clock;

    TSPDSSolution current = start;
    if (current.makespan <= 0.0) utils.evaluateSolution(current, /*needCalDrone=*/true);

    std::array<OpPerfStat, 32> st;

    std::array<std::string, 32> opName;
    opName.fill("N/A");
    opName[1] = "applyTruckRouteSwap";
    opName[2] = "applyTwoOpt";
    opName[3] = "moveTruckNodeAcrossStation";
    opName[4] = "balanceDroneLoad";
    opName[5] = "optimizeTruckBottleneck";
    opName[6] = "optimizeDroneBottleneck";
    opName[7] = "swapTruckDroneNodes";
    opName[8] = "assignFarthestTruckToDrone";
    opName[9] = "compoundTruckDroneExchange";
    opName[10] = "applyOrOpt1";
    opName[11] = "applyOrOpt2";
    opName[12] = "applyOrOpt3";

    auto runOp = [&](int opId, const auto& fn) -> std::pair<TSPDSSolution, double> {
        auto t0 = Clock::now();
        TSPDSSolution out = fn();
        auto t1 = Clock::now();
        double op_ms = toMs(t1 - t0);
        st[opId].calls++;
        st[opId].op_ms += op_ms;
        return { std::move(out), op_ms };
        };

    auto buildNeighborhoodOrder = [&](const TSPDSSolution& s) -> std::vector<int> {
        const double eps = 1e-9;
        const bool droneBottleneck = s.drone_completion_time > s.truck_completion_time + eps;
        std::vector<int> order = droneBottleneck
            ? std::vector<int>{ 6, 4, 5, 9, 7, 3, 8, 2 }
            : std::vector<int>{ 5, 6, 9, 2, 8, 3, 4, 7 };
        if (params.route_oropt_enabled) {
            if (droneBottleneck) {
                order.push_back(10);
                order.push_back(11);
            }
            else {
                order.insert(order.begin() + 1, 10);
                order.insert(order.begin() + 2, 11);
                order.insert(order.begin() + 3, 12);
            }
        }
        auto enabled = [&](int nt) {
            switch (nt) {
            case 2: return params.ls_two_opt_enabled;
            case 3: return params.ls_cross_station_enabled;
            case 4: return params.ls_drone_balance_enabled;
            case 5: return params.ls_truck_bottleneck_enabled;
            case 6: return params.ls_drone_bottleneck_enabled;
            case 7: return params.ls_truck_drone_swap_enabled;
            case 8: return params.ls_farthest_truck_to_drone_enabled;
            case 9: return params.compound_exchange_enabled;
            default: return true;
            }
        };
        order.erase(std::remove_if(order.begin(), order.end(), [&](int nt) { return !enabled(nt); }), order.end());
        return order;
    };
    const std::vector<int> stochasticNeighborhoods = { 1, 2, 3, 4, 5, 6, 7, 8, 9 };
    std::vector<double> improvementCount(stochasticNeighborhoods.size(), 0.0);
    int noImprove = 0;
    const int maxNoImprove = std::max(1, params.local_search_no_improve_limit);

    auto runDeterministicNeighborhood = [&](int nt) -> bool {
        TSPDSSolution cand = current;
        switch (nt) {
        case 1: cand = applyTruckRouteSwap(cand); break;
        case 2: cand = applyTwoOpt(cand); break;
        case 3: cand = moveTruckNodeAcrossStation(cand, /*firstImprove=*/true); break;
        case 4: cand = balanceDroneLoad(cand); break;
        case 5: cand = optimizeTruckBottleneck(cand); break;
        case 6: cand = optimizeDroneBottleneck(cand); break;
        case 7: cand = swapTruckDroneNodes(cand); break;
        case 8: cand = assignFarthestTruckToDrone(cand, 20, /*firstImprove=*/true); break;
        case 9: cand = compoundTruckDroneExchange(cand); break;
        case 10: cand = applyOrOpt(cand, 1); break;
        case 11: cand = applyOrOpt(cand, 2); break;
        case 12: cand = applyOrOpt(cand, 3); break;
        default: return false;
        }
        utils.evaluateSolution(cand, /*needCalDrone=*/true);
        if (acceptMove(current, cand)) {
            current = std::move(cand);
            return true;
        }
        return false;
    };

    if (!params.stochastic_neighborhoods) {
        bool moved = true;
        while (moved) {
            moved = false;
            for (int nt : buildNeighborhoodOrder(current)) {
                moved = runDeterministicNeighborhood(nt) || moved;
            }
        }
        return current;
    }

    while (noImprove < maxNoImprove) {
        std::vector<double> neighborhoodWeights(improvementCount.size(), 1.0);
        const double maxCount = *std::max_element(improvementCount.begin(), improvementCount.end());
        for (int i = 0; i < static_cast<int>(improvementCount.size()); ++i) {
            neighborhoodWeights[i] = std::exp(improvementCount[i] - maxCount);
        }

        std::discrete_distribution<int> pick(neighborhoodWeights.begin(), neighborhoodWeights.end());
        const int idx = pick(gen);
        const int nt = stochasticNeighborhoods[idx];

        TSPDSSolution cand = current;
        double last_op_ms = 0.0;

        switch (nt) {
        case 1: {
            auto ret = runOp(nt, [&]() { return applyTruckRouteSwap(cand); });
            cand = std::move(ret.first);
            last_op_ms = ret.second;
            break;
        }
        case 2: {
            auto ret = runOp(nt, [&]() { return applyTwoOpt(cand); });
            cand = std::move(ret.first);
            last_op_ms = ret.second;
            break;
        }
        case 3: {
            auto ret = runOp(nt, [&]() { return moveTruckNodeAcrossStation(cand, /*firstImprove=*/true); });
            cand = std::move(ret.first);
            last_op_ms = ret.second;
            break;
        }
        case 4: {
            auto ret = runOp(nt, [&]() { return balanceDroneLoad(cand); });
            cand = std::move(ret.first);
            last_op_ms = ret.second;
            break;
        }
        case 5: {
            auto ret = runOp(nt, [&]() { return optimizeTruckBottleneck(cand); });
            cand = std::move(ret.first);
            last_op_ms = ret.second;
            break;
        }
        case 6: {
            auto ret = runOp(nt, [&]() { return optimizeDroneBottleneck(cand); });
            cand = std::move(ret.first);
            last_op_ms = ret.second;
            break;
        }
        case 7: {
            auto ret = runOp(nt, [&]() { return swapTruckDroneNodes(cand); });
            cand = std::move(ret.first);
            last_op_ms = ret.second;
            break;
        }
        case 8: {
            auto ret = runOp(nt, [&]() { return assignFarthestTruckToDrone(cand, 20, /*firstImprove=*/true); });
            cand = std::move(ret.first);
            last_op_ms = ret.second;
            break;
        }
        case 9: {
            auto ret = runOp(nt, [&]() { return compoundTruckDroneExchange(cand); });
            cand = std::move(ret.first);
            last_op_ms = ret.second;
            break;
        }
        case 10: {
            auto ret = runOp(nt, [&]() { return applyOrOpt(cand, 1); });
            cand = std::move(ret.first);
            last_op_ms = ret.second;
            break;
        }
        case 11: {
            auto ret = runOp(nt, [&]() { return applyOrOpt(cand, 2); });
            cand = std::move(ret.first);
            last_op_ms = ret.second;
            break;
        }
        case 12: {
            auto ret = runOp(nt, [&]() { return applyOrOpt(cand, 3); });
            cand = std::move(ret.first);
            last_op_ms = ret.second;
            break;
        }
        default:
            noImprove++;
            continue;
        }

        auto te0 = Clock::now();
        utils.evaluateSolution(cand, /*needCalDrone=*/true);
        auto te1 = Clock::now();
        double eval_ms = toMs(te1 - te0);

        st[nt].eval_ms += eval_ms;
        st[nt].total_ms += (last_op_ms + eval_ms);

        if (acceptMove(current, cand)) {
            current = std::move(cand);
            st[nt].accepts++;
            st[nt].accept_total_ms += (last_op_ms + eval_ms);
            improvementCount[idx] += 1.0;
            noImprove = 0;
        }
        else {
            noImprove++;
        }
    }

    auto runCleanupNeighborhood = [&](int nt) -> bool {
        TSPDSSolution cand = current;
        switch (nt) {
        case 1: cand = applyTruckRouteSwap(cand); break;
        case 2: cand = applyTwoOpt(cand); break;
        case 3: cand = moveTruckNodeAcrossStation(cand, /*firstImprove=*/true); break;
        case 4: cand = balanceDroneLoad(cand); break;
        case 5: cand = optimizeTruckBottleneck(cand); break;
        case 6: cand = optimizeDroneBottleneck(cand); break;
        case 7: cand = swapTruckDroneNodes(cand); break;
        case 8: cand = assignFarthestTruckToDrone(cand, 20, /*firstImprove=*/true); break;
        case 9: cand = compoundTruckDroneExchange(cand); break;
        case 10: cand = applyOrOpt(cand, 1); break;
        case 11: cand = applyOrOpt(cand, 2); break;
        case 12: cand = applyOrOpt(cand, 3); break;
        default: return false;
        }
        utils.evaluateSolution(cand, /*needCalDrone=*/true);
        if (acceptMove(current, cand)) {
            current = std::move(cand);
            return true;
        }
        return false;
    };

    for (int pass = 0; pass < params.final_vnd_passes; ++pass) {
        bool passImproved = false;
        for (int nt : buildNeighborhoodOrder(current)) {
            passImproved = runCleanupNeighborhood(nt) || passImproved;
        }
        if (!passImproved) break;
    }

    return current;
}



TSPDSSolution LocalSearchOperators::optimizeTruckBottleneck(TSPDSSolution& solution)
{
    TSPDSSolution current = solution;
    if (current.makespan <= 0.0) utils.evaluateSolution(current, /*needCalDrone=*/true);
    bool improved = true;
    TSPDSSolution best = current;
    while (improved) {
        improved = false;

        current = best;
        // 1) 一次性构建缓存：pos、removeSaving、droneLoadCache
        utils.buildPosInTruck(current);

        std::vector<double> removeSaving;
        utils.buildRemoveSaving(current, removeSaving);

        DroneLoadCache dcache = utils.buildDroneLoadCache(current);

        // 2) 扫一遍 truck_route，把 eligible & truck-served 的点加进候选
        std::vector<std::pair<int, double>> fastCand;
        fastCand.reserve(current.truck_route.size());

        for (int k = 1; k + 1 < (int)current.truck_route.size(); ++k) {
            int node = current.truck_route[k];
            if (node == graph.depot || node == graph.drone_station) continue;
            if (!graph.is_drone_eligible.empty() && !graph.is_drone_eligible[node]) continue;
            if (!current.served_by_truck.empty() && !current.served_by_truck[node]) continue;

            double pot = utils.estimatePotentialTruckToDrone_O1(current, node, removeSaving, dcache);
            if (pot > 0) {
                fastCand.push_back({ node, pot });
            }
        }

        if (fastCand.empty()) return current;

        // 3) 按潜力降序排序（pot 越大越优先试）
        std::sort(fastCand.begin(), fastCand.end(),
            [](const auto& x, const auto& y) { return x.second > y.second; });

        int topK_big = std::min(static_cast<int>(fastCand.size()),
            std::max(1, static_cast<int>(std::ceil(fastCand.size() * params.golbal_attemp_per_truck))));

        // 4) 生成试验顺序
        std::vector<int> order;
        order.reserve(topK_big);
        for (int i = 0; i < topK_big; ++i) order.push_back(i);

        for (int idx : order) {
            int node = fastCand[idx].first;

            TSPDSSolution test = current;

            // 5) 迁移：truck -> drone
            if (!utils.reassignTruckToDrone(test, node)) continue;

            // 6) 先精确评估

            utils.evaluateSolution(test, /*needCalDrone=*/true);

            if (acceptMove(best, test)) {
                //test = applyTwoOpt(test);
                best = test;
                improved = true;
            }
        }
    }

    return best;
}


bool LocalSearchOperators::moveDroneToTruckAtPos(TSPDSSolution& sol, int node, int insPos)
{
    if (node == graph.depot || node == graph.drone_station) return false;
    if (!sol.served_by_drone[node] || sol.served_by_truck[node]) return false;

    // 防重复插入：如果 node 已在 route，拒绝
    if (!sol.pos_in_truck.empty() && node >= 0 && node < (int)sol.pos_in_truck.size() && sol.pos_in_truck[node] != -1)
        return false;

    sol.served_by_drone[node] = false;
    sol.served_by_truck[node] = true;

    auto& r = sol.truck_route;
    if (r.size() < 2) return false;

    if (insPos < 1) insPos = 1;
    if (insPos > (int)r.size() - 1) insPos = (int)r.size() - 1;

    r.insert(r.begin() + insPos, node);
    return true;
}

/**
 * LS2: 无人机瓶颈优化 —— 将 drone 上的节点转移回 truck
 */
TSPDSSolution LocalSearchOperators::optimizeDroneBottleneck(TSPDSSolution& solution)
{
    TSPDSSolution current = solution;
    if (current.makespan <= 0.0) utils.evaluateSolution(current, /*needCalDrone=*/true);

    const double EPS = 1e-9;

    bool improved = true;

    int iter = 0;
    TSPDSSolution best = current;
    while (improved) {
        improved = false;
        current = best;
        // 1) 位置缓存
        utils.buildPosInTruck(current);
        if (current.pos_station_in_truck < 0) break;

        // 2) 无人机负载缓存
        DroneLoadCacheDT dcache = utils.buildDroneLoadCacheDT(current);

        // 3) 候选：所有 无人机(们)的任务
        std::vector<int> candNodes;
        candNodes.reserve(64);

        const int m = std::max(1, graph.drone_count);
        for (int d = 0; d < m; ++d) {

            auto it = current.drone_assignments.find(d);
            if (it == current.drone_assignments.end()) continue;

            for (int node : it->second) {
                candNodes.push_back(node);
            }
        }

        if (candNodes.empty()) break;
        std::sort(candNodes.begin(), candNodes.end());
        candNodes.erase(std::unique(candNodes.begin(), candNodes.end()), candNodes.end());

        // 4) 批量预计算插入增量/位置
        std::vector<double> bestInc;
        std::vector<int> bestPos;
        utils.buildBestInsertionCacheForNodes(current, candNodes, InsertPolicy::ANYWHERE, bestInc, bestPos);

        // 5) O(1) potential 排序
        std::vector<std::pair<int, double>> ranked;
        ranked.reserve(candNodes.size());

        for (int node : candNodes) {
            double inc = bestInc[node];
            int pos = bestPos[node];
            if (!std::isfinite(inc) || pos < 0) continue;

            double pot = utils.estimatePotentialDroneToTruck_O1(current, node, inc, pos, dcache);
            ranked.push_back({ node, pot });
        }

        if (ranked.empty()) break;

        std::sort(ranked.begin(), ranked.end(),
            [](const auto& a, const auto& b) { return a.second > b.second; });

        int pool = std::min(static_cast<int>(ranked.size()),
            std::max(1, static_cast<int>(std::ceil(ranked.size() * params.golbal_attemp_per_drone))));


        // 非随机：直接按 score 从高到低依次尝试
        for (int i = 0; i < pool; ++i) {
            int node = ranked[i].first;
            int pos = bestPos[node];
            if (pos < 0) continue;

            TSPDSSolution test = current;

            if (!moveDroneToTruckAtPos(test, node, pos)) continue;

            // full evaluate
            utils.evaluateSolution(test, /*needCalDrone=*/true);

            if (acceptMove(best, test)) {
                //test = applyTwoOpt(test);
                best = test;
                improved = true;
            }
        }
    }

    return best;
}


/**
 * LS3: Hybrid 2-opt（对 station 前后两段分别 2-opt）
 */
TSPDSSolution LocalSearchOperators::applyTwoOpt(TSPDSSolution& solution)
{
    // 1. 备份原始解 & 工作解
    TSPDSSolution original = solution;
    TSPDSSolution work = solution;

    // 2. 找到无人机站在 truck_route 中的位置
    const std::vector<int>& route = work.truck_route;
    if (route.size() < 4) {
        // 路径太短，不值得 2-opt
        return original;
    }

    int stationIndex = -1;
    for (int i = 0; i < static_cast<int>(route.size()); ++i) {
        if (route[i] == graph.drone_station) {
            stationIndex = i;
            break;
        }
    }

    // 如果 station 不在路径中或在路径首尾，退化为普通全局 2-opt
    if (stationIndex <= 0 || stationIndex >= static_cast<int>(route.size()) - 1) {
        std::vector<int> tmpRoute = work.truck_route;
        runSegmentTwoOpt(tmpRoute, /*protectFirst=*/true, /*protectLast=*/true);
        work.truck_route = tmpRoute;

        utils.evaluateSolution(work, /*needCalDrone=*/false);

        if (work.combined_score < original.combined_score - 1e-6 ||
            work.truck_completion_time < original.truck_completion_time - 1e-6) {
            return work;
        }
        else {
            return original;
        }
    }

    // 3. 构造前后两段
    // pre: depot ... station
    std::vector<int> preSegment(route.begin(), route.begin() + stationIndex + 1);
    // post: station ... depot
    std::vector<int> postSegment(route.begin() + stationIndex, route.end());

    // 4. 对 preSegment 做 2-opt（保持起点=depot，终点=station）
    runSegmentTwoOpt(preSegment, /*protectFirst=*/true, /*protectLast=*/true);

    // 5. 对 postSegment 做 2-opt（保持起点=station，终点=depot）
    runSegmentTwoOpt(postSegment, /*protectFirst=*/true, /*protectLast=*/true);

    // 6. 重新拼接完整 truck_route
    std::vector<int> newRoute;
    newRoute.reserve(route.size());
    newRoute.insert(newRoute.end(), preSegment.begin(), preSegment.end());
    newRoute.insert(newRoute.end(), postSegment.begin() + 1, postSegment.end());

    work.truck_route = std::move(newRoute);

    utils.evaluateSolution(work, /*needCalDrone=*/false);

    // 8. 接受/拒绝：保证不会恶化解
    if (work.combined_score < original.combined_score - 1e-6 ||
        work.truck_completion_time < original.truck_completion_time - 1e-6) {
        return work;
    }
    else {
        return original;
    }
}


/**
 * TSwap: exchange two truck-served nodes on the same side of the drone station.
 * The depot and drone station are separators, so swaps are only tried inside
 * depot..station or station..depot segments.
 */
TSPDSSolution LocalSearchOperators::applyTruckRouteSwap(TSPDSSolution& solution)
{
    TSPDSSolution base = solution;
    if (base.makespan <= 0.0) utils.evaluateSolution(base, /*needCalDrone=*/true);

    const std::vector<int>& route = base.truck_route;
    const int n = static_cast<int>(route.size());
    if (n < 5) return base;
    if (route.front() != graph.depot || route.back() != graph.depot) return base;

    int stationIndex = -1;
    for (int i = 0; i < n; ++i) {
        if (route[i] == graph.drone_station) {
            stationIndex = i;
            break;
        }
    }
    if (stationIndex < 0) return base;

    auto isSwappableTruckNode = [&](int pos) -> bool {
        if (pos <= 0 || pos >= n - 1) return false;
        int node = route[pos];
        if (node == graph.depot || node == graph.drone_station) return false;
        if (!base.served_by_truck.empty() && !base.served_by_truck[node]) return false;
        return true;
        };

    auto betterThan = [&](const TSPDSSolution& a, const TSPDSSolution& b) -> bool {
        if (a.makespan < b.makespan - 1e-9) return true;
        if (std::fabs(a.makespan - b.makespan) <= 1e-9 &&
            a.combined_score < b.combined_score - 1e-9) return true;
        if (std::fabs(a.makespan - b.makespan) <= 1e-9 &&
            std::fabs(a.combined_score - b.combined_score) <= 1e-9 &&
            a.truck_completion_time < b.truck_completion_time - 1e-9) return true;
        return false;
        };

    TSPDSSolution best = base;
    bool foundImprovement = false;

    auto trySegment = [&](int firstPos, int lastPos) {
        if (lastPos - firstPos + 1 < 2) return;

        for (int i = firstPos; i <= lastPos - 1; ++i) {
            if (!isSwappableTruckNode(i)) continue;

            for (int j = i + 1; j <= lastPos; ++j) {
                if (!isSwappableTruckNode(j)) continue;

                TSPDSSolution cand = base;
                std::swap(cand.truck_route[i], cand.truck_route[j]);

                utils.evaluateSolution(cand, /*needCalDrone=*/false);

                if (betterThan(cand, best)) {
                    best = std::move(cand);
                    foundImprovement = true;
                }
            }
        }
        };

    trySegment(1, stationIndex - 1);
    trySegment(stationIndex + 1, n - 2);

    return foundImprovement ? best : base;
}




TSPDSSolution LocalSearchOperators::compoundTruckDroneExchange(TSPDSSolution& solution)
{
    if (!params.compound_exchange_enabled) return solution;

    TSPDSSolution current = solution;
    if (current.makespan <= 0.0) utils.evaluateSolution(current, /*needCalDrone=*/true);

    bool improved = true;
    TSPDSSolution best = current;
    struct NodeScore { int node; double score; };

    auto applyCompound = [&](TSPDSSolution& sol,
        const std::vector<int>& truckToDrone,
        const std::vector<int>& droneToTruck) -> bool {
        TSPDSSolution backup = sol;

        for (int node : truckToDrone) {
            if (node == graph.depot || node == graph.drone_station) { sol = backup; return false; }
            if (!graph.is_drone_eligible.empty() && !graph.is_drone_eligible[node]) { sol = backup; return false; }
            if (!sol.served_by_truck[node]) { sol = backup; return false; }

            auto it = std::find(sol.truck_route.begin(), sol.truck_route.end(), node);
            if (it == sol.truck_route.end() || it == sol.truck_route.begin() || it == sol.truck_route.end() - 1) {
                sol = backup;
                return false;
            }
            sol.truck_route.erase(it);
            sol.served_by_truck[node] = false;
            sol.served_by_drone[node] = true;
        }

        for (int node : droneToTruck) {
            if (node == graph.depot || node == graph.drone_station) { sol = backup; return false; }
            if (!sol.served_by_drone[node] || sol.served_by_truck[node]) { sol = backup; return false; }
            if (!utils.insertNodeToTruckRoute(sol, node, InsertPolicy::ANYWHERE)) {
                sol = backup;
                return false;
            }
            sol.served_by_truck[node] = true;
            sol.served_by_drone[node] = false;
        }

        return true;
    };

    while (improved) {
        improved = false;
        current = best;

        utils.buildPosInTruck(current);

        std::vector<double> removeSaving;
        utils.buildRemoveSaving(current, removeSaving);
        DroneLoadCacheDT dcache = utils.buildDroneLoadCacheDT(current);

        std::vector<NodeScore> truckScores;
        truckScores.reserve(current.truck_route.size());
        for (int pos = 1; pos + 1 < static_cast<int>(current.truck_route.size()); ++pos) {
            int node = current.truck_route[pos];
            if (node == graph.depot || node == graph.drone_station) continue;
            if (!graph.is_drone_eligible.empty() && !graph.is_drone_eligible[node]) continue;
            if (!current.served_by_truck.empty() && !current.served_by_truck[node]) continue;
            double saving = (node >= 0 && node < static_cast<int>(removeSaving.size())) ? removeSaving[node] : 0.0;
            if (saving <= 0.0) continue;
            truckScores.push_back({ node, saving });
        }

        const int busyDrone = dcache.argmax;
        std::vector<NodeScore> droneScores;
        if (busyDrone >= 0 && busyDrone < graph.drone_count) {
            auto it = current.drone_assignments.find(busyDrone);
            if (it != current.drone_assignments.end()) {
                droneScores.reserve(it->second.size());
                for (int node : it->second) {
                    if (node == graph.depot || node == graph.drone_station) continue;
                    if (!graph.is_drone_eligible.empty() && !graph.is_drone_eligible[node]) continue;
                    if (!current.served_by_drone.empty() && !current.served_by_drone[node]) continue;
                    double duration = 2.0 * graph.drone_time[graph.drone_station][node];
                    double score = duration;
                    if (params.compound_lkh_refine) {
                        int bestPos = -1;
                        double inc = utils.estimateBestInsertionIncrease(current, node, InsertPolicy::ANYWHERE, bestPos);
                        if (!std::isfinite(inc) || bestPos < 0) continue;
                        score = -inc;
                    }
                    droneScores.push_back({ node, score });
                }
            }
        }

        if (truckScores.empty() || droneScores.empty()) return best;

        std::sort(truckScores.begin(), truckScores.end(),
            [](const NodeScore& a, const NodeScore& b) { return a.score > b.score; });
        std::sort(droneScores.begin(), droneScores.end(),
            [](const NodeScore& a, const NodeScore& b) { return a.score > b.score; });

        const int K = std::min(static_cast<int>(truckScores.size()), params.compound_top_truck);
        const int L = std::min(static_cast<int>(droneScores.size()), params.compound_top_drone);
        const int pairK = std::min(K, params.compound_pair_top);
        const int pairL = std::min(L, params.compound_pair_top);

        auto tryMove = [&](const std::vector<int>& truckToDrone, const std::vector<int>& droneToTruck) {
            TSPDSSolution test = current;
            if (!applyCompound(test, truckToDrone, droneToTruck)) return;
            utils.evaluateSolution(test, /*needCalDrone=*/true);
            if (params.compound_lkh_refine) {
                utils.optimizeTruckRouteWithLKHIntern(test);
                utils.evaluateSolution(test, /*needCalDrone=*/true);
            }
            if (acceptMove(best, test)) {
                best = std::move(test);
                improved = true;
            }
        };

        for (int i = 0; i < K; ++i) {
            int truckNode = truckScores[i].node;
            for (int j = 0; j < L; ++j) {
                int droneNode = droneScores[j].node;
                if (truckNode == droneNode) continue;
                tryMove({ truckNode }, { droneNode });
            }
        }

        for (int i = 0; i < pairK; ++i) {
            for (int i2 = i + 1; i2 < pairK; ++i2) {
                int t1 = truckScores[i].node;
                int t2 = truckScores[i2].node;
                for (int j = 0; j < pairL; ++j) {
                    int d = droneScores[j].node;
                    if (d == t1 || d == t2) continue;
                    tryMove({ t1, t2 }, { d });
                }
            }
        }

        for (int j = 0; j < pairL; ++j) {
            for (int j2 = j + 1; j2 < pairL; ++j2) {
                int d1 = droneScores[j].node;
                int d2 = droneScores[j2].node;
                for (int i = 0; i < pairK; ++i) {
                    int t = truckScores[i].node;
                    if (t == d1 || t == d2) continue;
                    tryMove({ t }, { d1, d2 });
                }
            }
        }
    }

    return best;
}

TSPDSSolution LocalSearchOperators::swapTruckDroneNodes(TSPDSSolution& solution)
{
    if (solution.makespan <= 0.0) utils.evaluateSolution(solution, true);
    TSPDSSolution base = solution;

    bool improved = true;
    bool started = true;//下面哪个d循环多次执行
    TSPDSSolution best = base;
    while (improved && started) {
        improved = false;
        started = true;
        base = best;
        // ---- 1) 一次性构建缓存 ----
        utils.buildPosInTruck(base);

        std::vector<double> removeSaving;
        utils.buildRemoveSaving(base, removeSaving);

        //无人机最小负载缓存
        DroneLoadCache dcache = utils.buildDroneLoadCache(base);

        // 详细无人机负载 cache（loads/max1/max2/argmax/countMax/node2drone）
        DroneLoadCacheDT dtcache = utils.buildDroneLoadCacheDT(base);

        // ---- 2) 收集候选集合----
        std::vector<int> truckCandNodes;
        for (int k = 1; k + 1 < (int)base.truck_route.size(); ++k) {
            int i = base.truck_route[k];
            if (i == graph.depot || i == graph.drone_station)
                continue;
            if (!graph.is_drone_eligible[i])
                continue;
            if (!base.served_by_truck[i])
                continue;
            truckCandNodes.push_back(i);
        }

        std::vector<int> droneCandNodes;

        const int m = std::max(1, graph.drone_count);
        for (int d = 0; d < m; ++d) {

            auto it = base.drone_assignments.find(d);
            if (it == base.drone_assignments.end()) continue;

            for (int node : it->second) {
                if (node == graph.depot || node == graph.drone_station)
                    continue;
                if (!graph.is_drone_eligible[node])
                    continue;
                if (!base.served_by_drone[node])
                    continue;
                droneCandNodes.push_back(node);
            }
        }
        std::sort(droneCandNodes.begin(), droneCandNodes.end());
        droneCandNodes.erase(std::unique(droneCandNodes.begin(), droneCandNodes.end()), droneCandNodes.end());

        if (truckCandNodes.empty() || droneCandNodes.empty()) return base;

        // ---- 3) 批量预计算：drone->truck 插入代价（policy 和 performSwap 一致！）----
        std::vector<double> bestInc;
        std::vector<int> bestPos;
        InsertPolicy policy = InsertPolicy::ANYWHERE;
        utils.buildBestInsertionCacheForNodes(base, droneCandNodes, policy, bestInc, bestPos);

        // ---- 4) O(1) 打分 ----
        struct NS { int node; double score; };
        std::vector<NS> truckScores, droneScores;
        truckScores.reserve(truckCandNodes.size());
        droneScores.reserve(droneCandNodes.size());

        for (int t : truckCandNodes) {
            double s = utils.estimatePotentialTruckToDrone_O1(base, t, removeSaving, dcache /*min/Cmax 或 DT 版都行*/);
            truckScores.push_back({ t, s });
        }

        for (int d : droneCandNodes) {
            double inc = bestInc[d];
            int pos = bestPos[d];
            if (!std::isfinite(inc) || pos < 0) continue;
            double s = utils.estimatePotentialDroneToTruck_O1(base, d, inc, pos, dtcache);
            droneScores.push_back({ d, s });
        }

        std::sort(truckScores.begin(), truckScores.end(),
            [](const NS& a, const NS& b) { return a.score > b.score; });
        std::sort(droneScores.begin(), droneScores.end(),
            [](const NS& a, const NS& b) { return a.score > b.score; });


        if (truckScores.empty() || droneScores.empty()) return base;
        int K = std::min(static_cast<int>(truckScores.size()),
            std::max(1, static_cast<int>(std::ceil(truckScores.size() * params.golbal_attemp_per_truck))));
        int L = std::min(static_cast<int>(droneScores.size()),
            std::max(1, static_cast<int>(std::ceil(droneScores.size() * params.golbal_attemp_per_drone))));

        std::vector<int> candTruck, candDrone;
        candTruck.reserve(K); candDrone.reserve(L);
        for (int i = 0; i < K; ++i) candTruck.push_back(truckScores[i].node);
        for (int i = 0; i < L; ++i) candDrone.push_back(droneScores[i].node);

        for (int it = 0; it < K ; ++it) {
            int t = candTruck[it];

            for (int id = 0; id < L; ++id) {
                int d = candDrone[id];

                if (estimateQuickSwapBound(base, t, d) < 0.0) continue;

                TSPDSSolution test = base;
                if (!performSwap(test, t, d)) continue;

                utils.evaluateSolution(test, /*needCalDrone=*/true);

                if (acceptMove(best, test)) {
                    //test = applyTwoOpt(test);
                    best = test;
                    improved = true;
                }
            }
        }
    }
    return best;
}

// 将一个卡车节点跨越 station 搬到另一侧，并在另一侧找最小插入增量位置
// firstImprove=true：找到第一个可接受改进就返回

TSPDSSolution LocalSearchOperators::applyOrOpt(TSPDSSolution& solution, int segmentLen)
{
    TSPDSSolution current = solution;
    if (current.makespan <= 0.0) utils.evaluateSolution(current, /*needCalDrone=*/true);

    const int len = std::max(1, segmentLen);
    const int n = static_cast<int>(current.truck_route.size());
    if (n <= len + 3) return current;

    TSPDSSolution best = current;
    const auto& baseRoute = current.truck_route;

    for (int start = 1; start + len <= n - 1; ++start) {
        bool hasProtected = false;
        for (int k = 0; k < len; ++k) {
            int node = baseRoute[start + k];
            if (node == graph.depot || node == graph.drone_station) {
                hasProtected = true;
                break;
            }
        }
        if (hasProtected) continue;

        std::vector<int> segment(baseRoute.begin() + start, baseRoute.begin() + start + len);
        std::vector<int> removed = baseRoute;
        removed.erase(removed.begin() + start, removed.begin() + start + len);

        for (int pos = 1; pos < static_cast<int>(removed.size()); ++pos) {
            if (pos == start) continue;
            std::vector<int> newRoute = removed;
            newRoute.insert(newRoute.begin() + pos, segment.begin(), segment.end());
            if (newRoute.front() != graph.depot || newRoute.back() != graph.depot) continue;
            if (std::count(newRoute.begin(), newRoute.end(), graph.drone_station) != 1) continue;

            TSPDSSolution cand = current;
            cand.truck_route = std::move(newRoute);
            utils.evaluateSolution(cand, /*needCalDrone=*/false);
            if (acceptMove(best, cand)) {
                best = std::move(cand);
            }
        }
    }

    return best;
}


// firstImprove=false：遍历所有点，返回最好的可接受解

TSPDSSolution LocalSearchOperators::moveTruckNodeAcrossStation(TSPDSSolution& solution, bool firstImprove)
{
    TSPDSSolution baseSol = solution;
    if (baseSol.makespan <= 0.0) utils.evaluateSolution(baseSol, /*needCalDrone=*/true);

    const auto& baseRoute = baseSol.truck_route;
    const int baseLen = (int)baseRoute.size();
    if (baseLen <= 4) return baseSol;

    // station 在 baseRoute 中的位置
    int stationIdx = -1;
    for (int i = 0; i < baseLen; ++i) {
        if (baseRoute[i] == graph.drone_station) { stationIdx = i; break; }
    }

    auto betterThan = [&](const TSPDSSolution& a, const TSPDSSolution& b) -> bool {
        if (a.makespan < b.makespan - 1e-9) return true;
        if (std::fabs(a.makespan - b.makespan) <= 1e-9 &&
            a.combined_score < b.combined_score - 1e-9) return true;
        return false;
        };

    bool foundImprovement = false;
    TSPDSSolution bestSol = baseSol;

    // 遍历所有可移动的卡车节点位置（排除首尾 depot，排除 station）
    for (int nodeIdx = 1; nodeIdx <= baseLen - 2; ++nodeIdx) {
        int movedNode = baseRoute[nodeIdx];
        if (movedNode == graph.depot || movedNode == graph.drone_station) continue;
        if (!baseSol.served_by_truck.empty() && !baseSol.served_by_truck[movedNode]) continue;

        const bool nodeIsBeforeStation = (nodeIdx < stationIdx);

        // 1) 删除 movedNode，得到 routeWithoutNode
        std::vector<int> routeWithoutNode = baseRoute;
        routeWithoutNode.erase(routeWithoutNode.begin() + nodeIdx);

        // 2) 删除后 station 的位置（如果删的是 station 前的点，stationIdx 要左移 1）
        int stationIdxAfterErase = stationIdx - (nodeIsBeforeStation ? 1 : 0);
        int lenAfterErase = (int)routeWithoutNode.size();

        // 3) 确定“插入边”的范围：要插到 station 的另一侧
        // 插入是对边 (i -> i+1) 插入，插入位置是 i+1
        int edgeStart = 0;
        int edgeEnd = lenAfterErase - 2;

        if (nodeIsBeforeStation) {
            // station 前 -> station 后：从 stationIdxAfterErase 这条边开始插
            edgeStart = stationIdxAfterErase;
            edgeEnd = lenAfterErase - 2;
        }
        else {
            // station 后 -> station 前：只能插到 station 前，最多到 stationIdxAfterErase-1 的边
            edgeStart = 0;
            edgeEnd = stationIdxAfterErase - 1;
        }
        if (edgeStart > edgeEnd) continue;

        // 4) 在目标侧找“卡车增量最小”的插入位置
        double bestInsertIncrease = std::numeric_limits<double>::infinity();
        int bestInsertPos = -1;

        for (int edgeIdx = edgeStart; edgeIdx <= edgeEnd; ++edgeIdx) {
            int from = routeWithoutNode[edgeIdx];
            int to = routeWithoutNode[edgeIdx + 1];

            double increase =
                graph.truck_time[from][movedNode] +
                graph.truck_time[movedNode][to] -
                graph.truck_time[from][to];

            if (increase < bestInsertIncrease) {
                bestInsertIncrease = increase;
                bestInsertPos = edgeIdx + 1; // 真正插入的位置
            }
        }
        if (bestInsertPos < 0) continue;

        // 5) 构造候选解 candSol
        TSPDSSolution candSol = baseSol;
        candSol.truck_route = std::move(routeWithoutNode);
        candSol.truck_route.insert(candSol.truck_route.begin() + bestInsertPos, movedNode);

        // 防御：station 唯一、端点 depot 不变
        if (candSol.truck_route.front() != graph.depot || candSol.truck_route.back() != graph.depot) continue;
        if (std::count(candSol.truck_route.begin(), candSol.truck_route.end(), graph.drone_station) != 1) continue;

        utils.evaluateSolution(candSol, /*needCalDrone=*/true);

        // acceptMove：你自己的规则（通常是 makespan/score 改进或允许微弱接受）
        if (acceptMove(baseSol, candSol)) {
            //candSol = applyTwoOpt(candSol);
            if (firstImprove) return candSol;

            if (!foundImprovement || betterThan(candSol, bestSol)) {
                bestSol = candSol;
                foundImprovement = true;
            }
        }
    }

    return foundImprovement ? bestSol : baseSol;
}



/**
 * Truck 插入 node 的最小时间增量
 */
double LocalSearchOperators::estimateTruckTimeIncrease(const TSPDSSolution& solution, int node)
{
    int bestPos = -1;
    return utils.estimateBestInsertionIncrease(solution, node, InsertPolicy::ANYWHERE, bestPos);
} 


/**
 * segment 内部的 first-improvement 2-opt
 */
void LocalSearchOperators::runSegmentTwoOpt(std::vector<int>& segment,
    bool protectFirst,
    bool protectLast)
{
    int n = static_cast<int>(segment.size());
    if (n <= 3) return;

    int startI = protectFirst ? 1 : 0;
    int endJLimit = protectLast ? (n - 2) : (n - 1);

    bool improved = true;
    while (improved) {
        improved = false;

        for (int i = startI; i < n - 2; ++i) {
            for (int j = i + 1; j <= endJLimit; ++j) {
                if (j + 1 >= n) continue;

                double delta = utils.calculateTwoOptDelta(segment, i, j);
                if (delta < -1e-9) {
                    std::reverse(segment.begin() + i, segment.begin() + j + 1);
                    improved = true;
                    goto RESTART_SCAN;
                }
            }
        }

    RESTART_SCAN:
        if (!improved) break;
    }
}


/**
 * Swap operator 中的实际交换：truckNode <-> droneNode
 */
bool LocalSearchOperators::performSwap(TSPDSSolution& sol, int truckNode, int droneNode)
{
    auto backup = sol;

    if (!graph.is_drone_eligible.empty() &&
        (truckNode < 0 || truckNode >= static_cast<int>(graph.is_drone_eligible.size()) ||
         !graph.is_drone_eligible[truckNode])) {
        return false;
    }

    auto it = std::find(sol.truck_route.begin(), sol.truck_route.end(), truckNode);
    if (it == sol.truck_route.end() || it == sol.truck_route.begin() || it == sol.truck_route.end() - 1)
        return false;

    sol.truck_route.erase(it);
    sol.served_by_truck[truckNode] = false;
    sol.served_by_drone[truckNode] = true;

    if (!utils.insertNodeToTruckRoute(sol, droneNode, InsertPolicy::ANYWHERE)) {
        sol = backup;            // ★回滚
        return false;
    }
    sol.served_by_truck[droneNode] = true;
    sol.served_by_drone[droneNode] = false;

    return true;
}



/**
 * 估计 truck->drone 的收益（用于剪枝上界）
 */
double LocalSearchOperators::estimateTruckToDroneBenefit(const TSPDSSolution& S, int node)
{
    return utils.estimateTruckRemovalSaving(S, node);
}


/**
 * 估计 drone->truck 的收益（用于剪枝上界）
 */
double LocalSearchOperators::estimateDroneToTruckBenefit(const TSPDSSolution& S, int node)
{
    auto it = S.node_to_drone.find(node);
    if (it == S.node_to_drone.end()) return 0.0;
    int droneId = it->second;
    return utils.calculateActualDroneTimeReduction(S, node, droneId);
}



/**
 * QUICK PRUNING BOUND: 跳过不可能有正收益的 (t,d) 交换对
 */
double LocalSearchOperators::estimateQuickSwapBound(const TSPDSSolution& S, int t, int d)
{
    double truckToDroneSave = estimateTruckToDroneBenefit(S, t);
    double droneToTruckSave = estimateDroneToTruckBenefit(S, d);
    double truckInsertPenalty = estimateTruckTimeIncrease(S, d);

    // 上界：将 t 加入无人机系统的负载代价（station 往返时间）
    double droneAddPenalty = 2.0 * graph.drone_time[graph.drone_station][t];

    double approx =
        truckToDroneSave +
        droneToTruckSave -
        truckInsertPenalty -
        droneAddPenalty;

    return approx;
}

//new 

double LocalSearchOperators::jobDurationForDrone(int node)
{
    // 这里假设每个无人机任务是 station->node->station
    // 如果你有 service_time[node]，可以加进去
    int s = graph.drone_station;
    return 2.0 * graph.drone_time[s][node];
}

/**
 * 根据 node_to_drone / served_by_drone 重新算每个无人机的负载
 */
void LocalSearchOperators::recomputeDroneLoads(TSPDSSolution& S,
    std::vector<double>& loads)
{
    int m = S.drone_assignments.size(); // 如果你的字段不是这个名字，改一下
    loads.assign(m, 0.0);

    for (int node = 0; node < (int)graph.nodes.size(); ++node) {
        if (!S.served_by_drone[node]) continue;
        int k = S.node_to_drone[node];
        if (k < 0 || k >= m) continue;
        loads[k] += jobDurationForDrone(node);
    }
}

int LocalSearchOperators::getBusiestDrone(TSPDSSolution& S,
    const std::vector<double>& loads)
{
    int m = S.drone_assignments.size();
    int idx = 0;
    double best = loads.empty() ? 0.0 : loads[0];
    for (int k = 1; k < m; ++k) {
        if (loads[k] > best + 1e-9) {
            best = loads[k];
            idx = k;
        }
    }
    return idx;
}

int LocalSearchOperators::getIdlestDrone(TSPDSSolution& S,
    const std::vector<double>& loads)
{
    int m = S.drone_assignments.size();
    int idx = 0;
    double best = loads.empty() ? 0.0 : loads[0];
    for (int k = 1; k < m; ++k) {
        if (loads[k] < best - 1e-9) {
            best = loads[k];
            idx = k;
        }
    }
    return idx;
}

/**
 * 取某架无人机上最重的前 K 个任务节点
 */
std::vector<int> LocalSearchOperators::getTopKNodesOfDrone(TSPDSSolution& S,
    int droneId,
    int K)
{
    struct NodeDur {
        int node;
        double dur;
    };
    std::vector<NodeDur> v;

    for (int node = 0; node < (int)graph.nodes.size(); ++node) {
        if (!S.served_by_drone[node]) continue;
        if (S.node_to_drone[node] != droneId) continue;
        if (node == graph.drone_station) continue;

        double d = jobDurationForDrone(node);
        v.push_back({ node, d });
    }

    std::sort(v.begin(), v.end(),
        [](const NodeDur& a, const NodeDur& b) {
            return a.dur > b.dur;
        });

    std::vector<int> result;
    for (int i = 0; i < (int)v.size() && i < K; ++i) {
        result.push_back(v[i].node);
    }
    return result;
}

TSPDSSolution LocalSearchOperators::balanceDroneLoad(TSPDSSolution& solution)
{
    if (solution.makespan <= 0.0)
        utils.evaluateSolution(solution, /*needCalDrone=*/true);

    TSPDSSolution current = solution;

    const int m = std::max(1, graph.drone_count);
    if (m <= 1) return current;

    std::vector<double> loads(m, 0.0);
    bool improved = true;
    int iter = 0;
    const int maxIter = params.drone_balance_maxIter;

    while (improved && iter < maxIter) {
        improved = false;
        ++iter;

        recomputeDroneLoads(current, loads);
        int worstDrone = getBusiestDrone(current, loads);
        int bestDrone = getIdlestDrone(current, loads);

        if (worstDrone == bestDrone) break;

        std::vector<int> heavyNodes =
            getTopKNodesOfDrone(current, worstDrone, params.balance_topK_nodes);

        TSPDSSolution bestLocal = current;
        bool foundMove = false;

        for (int node : heavyNodes) {
            TSPDSSolution test = current;

            if (!moveNodeBetweenDrones(test, node, worstDrone, bestDrone))
                continue;

            // 关键：这里不要重新做全局调度，而是基于当前 assignment 重算
            utils.evaluateSolution(test, /*needCalDrone=*/false);

            if (test.drone_completion_time < bestLocal.drone_completion_time - 1e-6) {
                bestLocal = test;
                foundMove = true;
            }
        }

        if (foundMove) {
            current = bestLocal;
            improved = true;
        }
    }

    return current;
}

TSPDSSolution LocalSearchOperators::assignFarthestTruckToDrone(TSPDSSolution& solution,
    int maxAcceptedMoves,
    bool firstImprove)
{
    TSPDSSolution current = solution;
    if (current.makespan <= 0.0) utils.evaluateSolution(current, /*needCalDrone=*/true);

    if (current.truck_route.size() < 4) return current;
    if (current.truck_route.front() != graph.depot || current.truck_route.back() != graph.depot) return current;
    if (std::count(current.truck_route.begin(), current.truck_route.end(), graph.drone_station) != 1) return current;

    const int station = graph.drone_station;
    if (station < 0) return current;

    struct Candidate {
        int node;
        double roundTrip;
    };

    std::vector<Candidate> candidates;
    candidates.reserve(current.truck_route.size());
    for (int pos = 1; pos + 1 < static_cast<int>(current.truck_route.size()); ++pos) {
        const int node = current.truck_route[pos];
        if (node == graph.depot || node == graph.drone_station) continue;
        if (!current.served_by_truck.empty() && !current.served_by_truck[node]) continue;
        if (!graph.is_drone_eligible.empty() && !graph.is_drone_eligible[node]) continue;
        if (!graph.is_truck_only.empty() && graph.is_truck_only[node]) continue;
        candidates.push_back({ node, graph.drone_time[station][node] + graph.drone_time[node][station] });
    }
    if (candidates.empty()) return current;

    std::sort(candidates.begin(), candidates.end(),
        [](const Candidate& lhs, const Candidate& rhs) {
            return lhs.roundTrip > rhs.roundTrip;
        });

    int accepted = 0;
    for (const Candidate& candidate : candidates) {
        if (maxAcceptedMoves > 0 && accepted >= maxAcceptedMoves) break;

        TSPDSSolution test = current;
        if (!utils.reassignTruckToDrone(test, candidate.node)) continue;
        utils.evaluateSolution(test, /*needCalDrone=*/true);

        if (acceptMove(current, test)) {
            current = std::move(test);
            ++accepted;
            if (firstImprove) return current;
        }
    }

    return current;
}


bool LocalSearchOperators::moveNodeBetweenDrones(TSPDSSolution& sol, int node, int from, int to)
{
    if (!sol.served_by_drone[node]) return false;
    if (sol.node_to_drone[node] != from) return false;

    auto& fromList = sol.drone_assignments[from];
    auto it = std::find(fromList.begin(), fromList.end(), node);
    if (it == fromList.end()) return false;

    fromList.erase(it);
    sol.drone_assignments[to].push_back(node);
    sol.node_to_drone[node] = to;
    return true;
}








