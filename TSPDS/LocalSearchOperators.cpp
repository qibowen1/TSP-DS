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

    //if (std::fabs(cand.makespan - curr.makespan) <= EPS_T &&
    //    cand.combined_score < curr.combined_score - EPS_S) return true;

    return false;
}


// ===== VND 主过程：邻域顺序随机 =====
TSPDSSolution LocalSearchOperators::localSearch(const TSPDSSolution& start, bool afterShake)
{
    using Clock = std::chrono::steady_clock;

    TSPDSSolution current = start;
    // --- 统计：0..10 ---
    std::array<OpPerfStat, 32> st;

    std::array<std::string, 32> opName;
    opName.fill("N/A");
    opName[1] = "optimizeTruckBottleneck"; // 0 1 2 3 4 6 9
    opName[2] = "optimizeDroneBottleneck";
    opName[3] = "balanceDroneLoad";
    opName[4] = "moveTruckNodeAcrossStation";
    opName[5] = "applyTwoOpt";
    opName[6] = "swapTruckDroneNodes";
    opName[7] = "assignFarthestTruckToDrone";

    // --- 包装：计时 + 计数 ---
    auto runOp = [&](int opId, const auto& fn) -> std::pair<TSPDSSolution, double> {
        auto t0 = Clock::now();
        TSPDSSolution out = fn();
        auto t1 = Clock::now();
        double op_ms = toMs(t1 - t0);
        st[opId].calls++;
        st[opId].op_ms += op_ms;
        return { std::move(out), op_ms };
        };

    bool moved = true;



    while (moved) {
        moved = false;
        auto buildNeighborhoodOrder = [&](const TSPDSSolution& s) -> std::vector<int> {
            const double eps = 1e-9;
            bool droneBottleneck = (s.drone_completion_time > s.truck_completion_time + eps);

            // 你当前启用的 neighborhoods 集合：{0,1,2,3,4,6,9}
            // 规则：若两者相等（<=eps）也走 truck-first
            if (!droneBottleneck) {
                // truck-first：
                return { 0,4, 9, 3, 2, 6, 1};//{ 0,4, 9, 3, 2, 6, 1};
            }
            else {
                // drone-first
                return { 1,2, 6, 0, 3, 9, 4 };//{ 1,2, 6, 0, 3, 9, 4 }
            }
            };
        std::vector<int> neighborhoods = buildNeighborhoodOrder(current);
		//随机领域顺序
		//std::shuffle(neighborhoods.begin(), neighborhoods.end(), gen);
        // ---- first-accept ----
        for (int i = 0; i < (int)neighborhoods.size();) {
            TSPDSSolution cand = current;
            int nt = neighborhoods[i];
            double last_op_ms = 0.0;
            switch (nt) {
                
            case 0: {
                auto ret = runOp(nt, [&]() { return optimizeTruckBottleneck(cand); });
                cand = std::move(ret.first);
                last_op_ms = ret.second;
                break;
            }
            case 1: {
                auto ret = runOp(nt, [&]() { return optimizeDroneBottleneck(cand); });
                cand = std::move(ret.first);
                last_op_ms = ret.second;
                break;
            }
            case 2: {
                auto ret = runOp(nt, [&]() { return balanceDroneLoad(cand); });
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
                auto ret = runOp(nt, [&]() { return applyTwoOpt(cand); });
                cand = std::move(ret.first);
                last_op_ms = ret.second;
                break;
            }
            case 6: {
                auto ret = runOp(nt, [&]() { return swapTruckDroneNodes(cand); });
                cand = std::move(ret.first);
                last_op_ms = ret.second;
                break;
            }
            case 9: {
                auto ret = runOp(nt, [&]() { return assignFarthestTruckToDrone(cand, 20, /*firstImprove=*/true); });
                cand = std::move(ret.first);
                last_op_ms = ret.second;
                break;
            }
            default:
                break;
            }

            // --- evaluate 也计时（建议计，因为 evaluate 可能很重） ---
            auto te0 = Clock::now();
			utils.evaluateSolution(cand, /*needCalDrone=*/true);
            auto te1 = Clock::now();
            double eval_ms = toMs(te1 - te0);

            if (last_op_ms > 0.0) {
                st[nt].eval_ms += eval_ms;
                st[nt].total_ms += (last_op_ms + eval_ms);
            }

            if (acceptMove(current, cand)) {
                current = cand;
                moved = true;

                if (last_op_ms > 0.0) {
                    st[nt].accepts++;
                    st[nt].accept_total_ms += (last_op_ms + eval_ms);
                }

                //i = 0; 
            }
            else {
                i++;
            }
        }
    }
    return current;
}



TSPDSSolution LocalSearchOperators::optimizeTruckBottleneck(TSPDSSolution& solution)
{
    TSPDSSolution current = solution;
    if (current.makespan <= 0.0) utils.evaluateSolution(current, /*needCalDrone=*/true);
    bool started = true;//下面哪个循环只一次
    bool improved = true;
    TSPDSSolution best = current;
    while (improved && started) {
        improved = false;
        started = false;

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

        int topK_big = (int)fastCand.size() * params.golbal_attemp_per_truck;


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
    bool started = true;//下面哪个循环是否只一次

    int iter = 0;
    TSPDSSolution best = current;
    while (improved && started) {
        improved = false;
        started = false;
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

		int pool = (int)ranked.size() * params.golbal_attemp_per_drone;


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




// LS5: 交换卡车节点和无人机节点（Truck <-> Drone swap）——采样版

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


        int K = params.golbal_attemp_per_truck * (int)truckScores.size();
        int L = params.golbal_attemp_per_drone * (int)droneScores.size();
        if (K <= 0 || L <= 0) return base;

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

//远的给无人机
TSPDSSolution LocalSearchOperators::assignFarthestTruckToDrone(TSPDSSolution& solution,
    int maxAcceptedMoves,
    bool firstImprove)
{
    TSPDSSolution base = solution;
    if (base.makespan <= 0.0) utils.evaluateSolution(base, /*needCalDrone=*/true);

    // 基本防御
    if (base.truck_route.size() < 4) return base;
    if (base.truck_route.front() != graph.depot || base.truck_route.back() != graph.depot) return base;
    if (std::count(base.truck_route.begin(), base.truck_route.end(), graph.drone_station) != 1) return base;

    const int S = graph.drone_station;
    if (S < 0) return base;

    // 1) 收集候选：truck_route 内、卡车服务、且可无人机服务
    struct Cand { int node; double keyRT; };
    std::vector<Cand> cands;
    cands.reserve(base.truck_route.size());

    for (int k = 1; k + 1 < (int)base.truck_route.size(); ++k) {
        int v = base.truck_route[k];
        if (v == graph.depot || v == graph.drone_station) continue;

        if (!base.served_by_truck.empty() && !base.served_by_truck[v]) continue;
        if (!graph.is_drone_eligible.empty() && !graph.is_drone_eligible[v]) continue;
        if (!graph.is_truck_only.empty() && graph.is_truck_only[v]) continue; // truck-only 禁止转无人机

        double rt = graph.drone_time[S][v] + graph.drone_time[v][S];

        cands.push_back({ v, rt });
    }
    if (cands.empty()) return base;

    // 2) 远->近排序（往返越大越远）
    std::sort(cands.begin(), cands.end(),
        [](const Cand& a, const Cand& b) { return a.keyRT > b.keyRT; });

    // 3) 依次尝试 truck->drone（只接受 makespan 变好）
    TSPDSSolution current = base;
    int accepted = 0;

    for (const auto& cd : cands) {
        if (maxAcceptedMoves > 0 && accepted >= maxAcceptedMoves) break;

        int node = cd.node;

        TSPDSSolution test = current;

        if (!utils.reassignTruckToDrone(test, node)) continue;

        utils.evaluateSolution(test, /*needCalDrone=*/true);

        if (acceptMove(current, test)) {
            current = test;
            ++accepted;
            if (firstImprove) return current;
        }
    }

    return current;
}








