#include "TSPDSSolver.h"
#include <algorithm>
#include <limits>
#include <cmath>
#include <iostream>
#include <random>
#include <chrono>
#include <numeric>
#include <random>   // 放在文件顶部
#include <thread>
#include <functional>
#include <unordered_set>
#include <sstream>



using namespace std;

// 随机数生成器初始化
random_device rd2;
mt19937 gen(rd2());
/*
带无人机站的单旅行商问题（TSP-DS）。具体来说：
有一个仓库（depot）、多个客户点和一个无人机站。
一辆卡车从depot出发，必须访问无人机站（以激活它），无人机站激活后无人机可以去访问顾客，卡车可以继续访问其他客户点，最后卡车返回depot。
无人机站配备多个无人机，每个无人机可以服务位于其操作范围内的客户点，通过往返行程（从站点的出发和返回）。
目标是最小化完工时间（makespan），即卡车完成整个路径的时间与无人机完成所有配送任务的时间中的最大值。
这个问题结合了路径优化（卡车路径）和调度优化（无人机任务分配），需要设计一个高效的启发式算法来处理。
可以优化卡车访问的节点，卡车路径，无人机访问的节点，无人机站激活的时间等等

注意：改变解结构时，因为无人机任务和卡车路径是耦合的，所以可能需要重新评估无人机任务分配以确保解的可行性和效率。
比如，当卡车路径改变时，可能需要重新分配无人机任务以适应新的卡车到达无人机站的时间。
比如，提前访问无人机站可能允许无人机更早开始任务，能够容纳更多无人机任务，也可能会绕远路访问这个无人机站点，从而影响整体完工时间。
无人机数量 1 3 5 7 9，这些不同数量的无人机会影响无人机任务的分配和调度策略。

*/
TSPDSSolver::TSPDSSolver(const TSPDSGraph& graph) : graph(graph), initGenerator(graph, params), perturbOps(graph, params), logger(graph, params), utils(graph, params), localSearchOps(graph, params){
    std::random_device rd;
    random_seed = rd();
    initGenerator.setRandomSeed(random_seed);
}

TSPDSSolver::TSPDSSolver(const TSPDSGraph& graph, int runId)
    : graph(graph),
    initGenerator(graph, params),
    perturbOps(graph, params),
    logger(graph, params),
    utils(graph, params),
    localSearchOps(graph, params),
    run_id(runId)                    // 记录传进来的 run_id
{
    std::random_device rd;
    random_seed = rd();
    initGenerator.setRandomSeed(random_seed);
}

void TSPDSSolver::setRandomSeed(unsigned int seed)
{
    random_seed = seed;
    initGenerator.setRandomSeed(seed);

}

namespace {
    constexpr double SOLVER_EPS = 1e-9;

    int findStationPos(const std::vector<int>& route, int station) {
        for (int i = 0; i < static_cast<int>(route.size()); ++i) {
            if (route[i] == station) return i;
        }
        return -1;
    }

    bool isValidNode(const TSPDSGraph& graph, int node) {
        return node >= 0 && node < static_cast<int>(graph.nodes.size());
    }

    bool isDroneEligibleNode(const TSPDSGraph& graph, int node) {
        if (!isValidNode(graph, node)) return false;
        if (node == graph.depot || node == graph.drone_station) return false;
        if (graph.is_drone_eligible.empty()) return true;
        return graph.is_drone_eligible[node];
    }

    std::vector<int> extractPreStationSegment(const TSPDSSolution& solution, const TSPDSGraph& graph) {
        int pos = findStationPos(solution.truck_route, graph.drone_station);
        if (pos < 0) return { graph.depot, graph.drone_station };
        return std::vector<int>(solution.truck_route.begin(), solution.truck_route.begin() + pos + 1);
    }

    std::vector<int> extractPostStationSegment(const TSPDSSolution& solution, const TSPDSGraph& graph) {
        int pos = findStationPos(solution.truck_route, graph.drone_station);
        if (pos < 0) return { graph.drone_station, graph.depot };
        return std::vector<int>(solution.truck_route.begin() + pos, solution.truck_route.end());
    }

    std::vector<int> extractDroneScheduleNodes(const TSPDSSolution& solution, const TSPDSGraph& graph) {
        std::vector<int> nodes;
        if (!solution.served_by_drone.empty()) {
            for (int node = 0; node < static_cast<int>(solution.served_by_drone.size()); ++node) {
                if (solution.served_by_drone[node] && isDroneEligibleNode(graph, node)) {
                    nodes.push_back(node);
                }
            }
            return nodes;
        }

        std::unordered_set<int> seen;
        for (const auto& assignment : solution.drone_assignments) {
            for (int node : assignment.second) {
                if (isDroneEligibleNode(graph, node) && seen.insert(node).second) {
                    nodes.push_back(node);
                }
            }
        }
        return nodes;
    }

    int partTypeIndex(TSPDSSolver::CrossoverPartType type) {
        switch (type) {
        case TSPDSSolver::CrossoverPartType::PreStation:
            return 0;
        case TSPDSSolver::CrossoverPartType::PostStation:
            return 1;
        case TSPDSSolver::CrossoverPartType::DroneSchedule:
            return 2;
        }
        return -1;
    }

    std::vector<std::unordered_set<int>> buildDroneTaskSets(
        const TSPDSSolution& solution,
        const TSPDSGraph& graph) {
        const int m = std::max(0, graph.drone_count);
        std::vector<std::unordered_set<int>> sets(m);

        for (int d = 0; d < m; ++d) {
            auto it = solution.drone_assignments.find(d);
            if (it == solution.drone_assignments.end()) continue;

            for (int node : it->second) {
                if (!isDroneEligibleNode(graph, node)) continue;
                if (node < static_cast<int>(solution.served_by_drone.size()) &&
                    !solution.served_by_drone[node]) {
                    continue;
                }
                sets[d].insert(node);
            }
        }

        return sets;
    }

    double jaccardDistance(
        const std::unordered_set<int>& lhs,
        const std::unordered_set<int>& rhs) {
        if (lhs.empty() && rhs.empty()) return 0.0;

        int intersectionSize = 0;
        const auto& smaller = (lhs.size() <= rhs.size()) ? lhs : rhs;
        const auto& larger = (lhs.size() <= rhs.size()) ? rhs : lhs;

        for (int node : smaller) {
            if (larger.find(node) != larger.end()) ++intersectionSize;
        }

        const int unionSize = static_cast<int>(lhs.size() + rhs.size()) - intersectionSize;
        if (unionSize == 0) return 0.0;
        return 1.0 - static_cast<double>(intersectionSize) / static_cast<double>(unionSize);
    }

    double symmetricDroneAssignmentDistance(
        const TSPDSSolution& a,
        const TSPDSSolution& b,
        const TSPDSGraph& graph) {
        const int m = std::max(0, graph.drone_count);
        if (m == 0) return 0.0;

        std::vector<std::unordered_set<int>> aSets = buildDroneTaskSets(a, graph);
        std::vector<std::unordered_set<int>> bSets = buildDroneTaskSets(b, graph);

        std::vector<std::vector<double>> cost(m, std::vector<double>(m, 0.0));
        for (int i = 0; i < m; ++i) {
            for (int j = 0; j < m; ++j) {
                cost[i][j] = jaccardDistance(aSets[i], bSets[j]);
            }
        }

        const int maskCount = 1 << m;
        std::vector<double> dp(maskCount, std::numeric_limits<double>::infinity());
        dp[0] = 0.0;

        for (int mask = 0; mask < maskCount; ++mask) {
            if (!std::isfinite(dp[mask])) continue;

            int matched = 0;
            for (int bit = 0; bit < m; ++bit) {
                if (mask & (1 << bit)) ++matched;
            }
            if (matched >= m) continue;

            for (int j = 0; j < m; ++j) {
                if (mask & (1 << j)) continue;
                const int nextMask = mask | (1 << j);
                dp[nextMask] = std::min(dp[nextMask], dp[mask] + cost[matched][j]);
            }
        }

        return dp[maskCount - 1];
    }
}

std::vector<TSPDSSolution> TSPDSSolver::generateInitialPopulation() {
    std::vector<TSPDSSolution> population;
    const int targetSize = std::max(1, params.population_size);
    population.reserve(targetSize);

    std::mt19937 seedRng(
        random_seed ^
        (0x9e3779b9u + static_cast<unsigned>(run_id + 31)) ^
        static_cast<unsigned>(std::chrono::steady_clock::now().time_since_epoch().count())
    );

    for (int i = 0; i < targetSize; ++i) {
        initGenerator.setRandomSeed(seedRng());
        TSPDSSolution solution = initGenerator.generateInitialSolution();
        utils.evaluateSolution(solution, /*needCalDrone=*/true);
        solution = localSearchOps.localSearch(solution, false);
        utils.evaluateSolution(solution, /*needCalDrone=*/true);
        solution.init_Id = i;
        population.push_back(std::move(solution));
    }

    return population;
}

std::pair<int, int> TSPDSSolver::selectParents(const std::vector<TSPDSSolution>& population, std::mt19937& rng) {
    if (population.empty()) return { -1, -1 };
    if (population.size() == 1) return { 0, 0 };

    std::uniform_int_distribution<int> dist(0, static_cast<int>(population.size()) - 1);
    auto tournamentPick = [&]() -> int {
        int a = dist(rng);
        int b = dist(rng);
        return (population[a].makespan <= population[b].makespan) ? a : b;
        };

    int first = tournamentPick();
    int second = tournamentPick();
    int guard = 0;
    while (second == first && guard < 16) {
        second = tournamentPick();
        ++guard;
    }
    if (second == first) {
        second = (first + 1) % static_cast<int>(population.size());
    }
    return { first, second };
}

void TSPDSSolver::insertMissingTruckNode(TSPDSSolution& solution, int node, TruckCompletionSide side) {
    auto& route = solution.truck_route;
    if (route.size() < 2) {
        route = { graph.depot, graph.drone_station, graph.depot };
    }
    if (std::find(route.begin(), route.end(), node) != route.end()) return;

    int stationPos = findStationPos(route, graph.drone_station);
    if (stationPos < 0) {
        route.insert(route.end() - 1, graph.drone_station);
        stationPos = findStationPos(route, graph.drone_station);
    }

    int startEdge = 0;
    int endEdge = static_cast<int>(route.size()) - 2;
    if (side == TruckCompletionSide::BeforeStation) {
        endEdge = std::max(0, stationPos - 1);
    }
    else if (side == TruckCompletionSide::AfterStation) {
        startEdge = std::min(stationPos, static_cast<int>(route.size()) - 2);
    }

    if (startEdge > endEdge) {
        startEdge = 0;
        endEdge = static_cast<int>(route.size()) - 2;
    }

    double bestInc = std::numeric_limits<double>::infinity();
    int bestPos = std::max(1, static_cast<int>(route.size()) - 1);

    for (int edge = startEdge; edge <= endEdge; ++edge) {
        int from = route[edge];
        int to = route[edge + 1];
        double inc = graph.truck_time[from][node] + graph.truck_time[node][to] - graph.truck_time[from][to];
        if (inc < bestInc - SOLVER_EPS) {
            bestInc = inc;
            bestPos = edge + 1;
        }
    }

    route.insert(route.begin() + bestPos, node);
}

TSPDSSolution TSPDSSolver::makeCrossoverChild(const CrossoverPartSpec& first,
    const CrossoverPartSpec& second,
    bool preferFirst) {
    const int n = static_cast<int>(graph.nodes.size());
    std::vector<int> rawPre;
    std::vector<int> rawPost;
    std::vector<int> rawDrone;
    bool hasPre = false;
    bool hasPost = false;
    bool hasDrone = false;

    auto loadPart = [&](const CrossoverPartSpec& part) {
        if (part.solution == nullptr) return;
        switch (part.type) {
        case CrossoverPartType::PreStation:
            rawPre = extractPreStationSegment(*part.solution, graph);
            hasPre = true;
            break;
        case CrossoverPartType::PostStation:
            rawPost = extractPostStationSegment(*part.solution, graph);
            hasPost = true;
            break;
        case CrossoverPartType::DroneSchedule:
            rawDrone = extractDroneScheduleNodes(*part.solution, graph);
            hasDrone = true;
            break;
        }
        };

    loadPart(first);
    loadPart(second);

    std::vector<char> inPre(n, 0), inPost(n, 0), inDrone(n, 0);
    auto markTruckSegment = [&](const std::vector<int>& segment, std::vector<char>& mark) {
        for (int node : segment) {
            if (!isValidNode(graph, node)) continue;
            if (node == graph.depot || node == graph.drone_station) continue;
            mark[node] = 1;
        }
        };

    markTruckSegment(rawPre, inPre);
    markTruckSegment(rawPost, inPost);
    for (int node : rawDrone) {
        if (isDroneEligibleNode(graph, node)) inDrone[node] = 1;
    }

    const int preferredType = partTypeIndex(preferFirst ? first.type : second.type);
    const int firstType = partTypeIndex(first.type);
    const int secondType = partTypeIndex(second.type);
    std::vector<int> keepType(n, -1);

    for (int node = 0; node < n; ++node) {
        if (node == graph.depot || node == graph.drone_station) continue;
        bool present[3] = { inPre[node] != 0, inPost[node] != 0, inDrone[node] != 0 };
        int count = static_cast<int>(present[0]) + static_cast<int>(present[1]) + static_cast<int>(present[2]);
        if (count == 0) continue;

        if (count == 1) {
            for (int t = 0; t < 3; ++t) {
                if (present[t]) keepType[node] = t;
            }
        }
        else if (preferredType >= 0 && present[preferredType]) {
            keepType[node] = preferredType;
        }
        else if (firstType >= 0 && present[firstType]) {
            keepType[node] = firstType;
        }
        else if (secondType >= 0 && present[secondType]) {
            keepType[node] = secondType;
        }
    }

    auto buildPre = [&]() -> std::vector<int> {
        std::vector<int> pre;
        pre.push_back(graph.depot);
        if (hasPre) {
            std::unordered_set<int> added;
            for (int node : rawPre) {
                if (!isValidNode(graph, node)) continue;
                if (node == graph.depot || node == graph.drone_station) continue;
                if (keepType[node] == 0 && added.insert(node).second) pre.push_back(node);
            }
        }
        pre.push_back(graph.drone_station);
        return pre;
        };

    auto buildPost = [&]() -> std::vector<int> {
        std::vector<int> post;
        post.push_back(graph.drone_station);
        if (hasPost) {
            std::unordered_set<int> added;
            for (int node : rawPost) {
                if (!isValidNode(graph, node)) continue;
                if (node == graph.depot || node == graph.drone_station) continue;
                if (keepType[node] == 1 && added.insert(node).second) post.push_back(node);
            }
        }
        post.push_back(graph.depot);
        return post;
        };

    std::unordered_set<int> fixedDroneNodes;
    if (hasDrone) {
        for (int node : rawDrone) {
            if (isDroneEligibleNode(graph, node) && keepType[node] == 2) {
                fixedDroneNodes.insert(node);
            }
        }
    }

    TSPDSSolution child;
    child.initialize(n);
    std::vector<int> pre = buildPre();
    std::vector<int> post = buildPost();
    child.truck_route = pre;
    if (!post.empty()) {
        child.truck_route.insert(child.truck_route.end(), post.begin() + 1, post.end());
    }

    std::vector<int> cleaned;
    cleaned.reserve(child.truck_route.size());
    cleaned.push_back(graph.depot);
    std::vector<char> seenTruck(n, 0);
    bool stationSeen = false;
    for (int i = 1; i + 1 < static_cast<int>(child.truck_route.size()); ++i) {
        int node = child.truck_route[i];
        if (!isValidNode(graph, node)) continue;
        if (node == graph.depot) continue;
        if (node == graph.drone_station) {
            if (!stationSeen) {
                cleaned.push_back(node);
                stationSeen = true;
            }
            continue;
        }
        if (!seenTruck[node]) {
            cleaned.push_back(node);
            seenTruck[node] = 1;
        }
    }
    if (!stationSeen) cleaned.push_back(graph.drone_station);
    cleaned.push_back(graph.depot);
    child.truck_route = std::move(cleaned);

    for (int node : child.truck_route) {
        if (!isValidNode(graph, node)) continue;
        child.served_by_truck[node] = true;
        child.served_by_drone[node] = false;
    }

    for (int node : fixedDroneNodes) {
        if (!isValidNode(graph, node)) continue;
        if (!child.served_by_truck[node]) child.served_by_drone[node] = true;
    }

    TruckCompletionSide completionSide = TruckCompletionSide::Anywhere;
    if (hasPre && !hasPost) completionSide = TruckCompletionSide::AfterStation;
    if (!hasPre && hasPost) completionSide = TruckCompletionSide::BeforeStation;

    for (int node = 0; node < n; ++node) {
        if (node == graph.depot || node == graph.drone_station) continue;
        if (child.served_by_truck[node] || child.served_by_drone[node]) continue;

        if (!hasDrone && hasPre && hasPost && isDroneEligibleNode(graph, node)) {
            child.served_by_drone[node] = true;
        }
        else {
            child.served_by_truck[node] = true;
            insertMissingTruckNode(child, node, completionSide);
        }
    }

    child.served_by_truck[graph.depot] = true;
    child.served_by_truck[graph.drone_station] = true;
    child.served_by_drone[graph.depot] = false;
    child.served_by_drone[graph.drone_station] = false;
    child.drone_assignments.clear();
    child.node_to_drone.clear();
    for (int d = 0; d < graph.drone_count; ++d) {
        child.drone_assignments[d] = std::vector<int>();
    }

    utils.evaluateSolution(child, /*needCalDrone=*/true);
    return child;
}

std::vector<TSPDSSolution> TSPDSSolver::crossover(const TSPDSSolution& a, const TSPDSSolution& b) {
    CrossoverPartSpec a1{ &a, CrossoverPartType::PreStation };
    CrossoverPartSpec a2{ &a, CrossoverPartType::PostStation };
    CrossoverPartSpec a3{ &a, CrossoverPartType::DroneSchedule };
    CrossoverPartSpec b1{ &b, CrossoverPartType::PreStation };
    CrossoverPartSpec b2{ &b, CrossoverPartType::PostStation };
    CrossoverPartSpec b3{ &b, CrossoverPartType::DroneSchedule };

    std::vector<TSPDSSolution> children;
    children.reserve(12);
    auto addChild = [&](const CrossoverPartSpec& first, const CrossoverPartSpec& second, bool preferFirst) {
        children.push_back(makeCrossoverChild(first, second, preferFirst));
        };

    addChild(a1, b2, true);
    addChild(a1, b2, false);
    addChild(a2, b1, true);
    addChild(a2, b1, false);
    addChild(a1, b3, true);
    addChild(a1, b3, false);
    addChild(a3, b1, true);
    addChild(a3, b1, false);
    addChild(a2, b3, true);
    addChild(a2, b3, false);
    addChild(a3, b2, true);
    addChild(a3, b2, false);

    return children;
}

double TSPDSSolver::solutionHammingDistance(const TSPDSSolution& a, const TSPDSSolution& b) const {
    const int n = static_cast<int>(graph.nodes.size());
    std::vector<int> ea(n, -2), eb(n, -2);

    for (int i = 0; i < static_cast<int>(a.truck_route.size()); ++i) {
        int node = a.truck_route[i];
        if (isValidNode(graph, node)) ea[node] = i;
    }
    for (int i = 0; i < static_cast<int>(b.truck_route.size()); ++i) {
        int node = b.truck_route[i];
        if (isValidNode(graph, node)) eb[node] = i;
    }
    for (int node = 0; node < n; ++node) {
        if (node < static_cast<int>(a.served_by_drone.size()) && a.served_by_drone[node]) ea[node] = -1;
        if (node < static_cast<int>(b.served_by_drone.size()) && b.served_by_drone[node]) eb[node] = -1;
    }

    double dist = 0.0;
    for (int node = 0; node < n; ++node) {
        if (node == graph.depot) continue;
        const bool aDrone = (node < static_cast<int>(a.served_by_drone.size()) && a.served_by_drone[node]);
        const bool bDrone = (node < static_cast<int>(b.served_by_drone.size()) && b.served_by_drone[node]);
        if (aDrone && bDrone) continue;
        if (ea[node] != eb[node]) ++dist;
    }

    return dist + symmetricDroneAssignmentDistance(a, b, graph);
}

std::string TSPDSSolver::solutionSignature(const TSPDSSolution& solution) const {
    std::ostringstream oss;
    for (int node : solution.truck_route) {
        oss << node << ',';
    }
    oss << "|D:";
    for (int node = 0; node < static_cast<int>(solution.served_by_drone.size()); ++node) {
        if (solution.served_by_drone[node]) oss << node << ',';
    }
    return oss.str();
}

void TSPDSSolver::updatePopulation(std::vector<TSPDSSolution>& population,
    const std::vector<TSPDSSolution>& candidates,
    const TSPDSSolution& bestSolution) {
    const int targetSize = std::max(1, params.population_size);
    std::vector<TSPDSSolution> pool = population;
    pool.insert(pool.end(), candidates.begin(), candidates.end());
    pool.push_back(bestSolution);

    for (auto& solution : pool) {
        if (solution.makespan <= 0.0) utils.evaluateSolution(solution, /*needCalDrone=*/true);
    }

    if (static_cast<int>(pool.size()) <= targetSize) {
        population = std::move(pool);
        return;
    }

    std::vector<double> objectives(pool.size(), 0.0);
    std::vector<double> diversities(pool.size(), 0.0);
    std::vector<double> imbalances(pool.size(), 0.0);
    double objMin = std::numeric_limits<double>::infinity();
    double objMax = -std::numeric_limits<double>::infinity();
    double imbalanceMax = 0.0;

    for (int i = 0; i < static_cast<int>(pool.size()); ++i) {
        objectives[i] = pool[i].makespan;
        objMin = std::min(objMin, objectives[i]);
        objMax = std::max(objMax, objectives[i]);

        imbalances[i] = std::fabs(pool[i].truck_completion_time - pool[i].drone_completion_time);
        imbalanceMax = std::max(imbalanceMax, imbalances[i]);
    }

    double divMin = std::numeric_limits<double>::infinity();
    double divMax = -std::numeric_limits<double>::infinity();
    for (int i = 0; i < static_cast<int>(pool.size()); ++i) {
        double minDist = std::numeric_limits<double>::infinity();
        for (int j = 0; j < static_cast<int>(pool.size()); ++j) {
            if (i == j) continue;
            minDist = std::min(minDist, solutionHammingDistance(pool[i], pool[j]));
        }
        if (!std::isfinite(minDist)) minDist = 0.0;
        diversities[i] = minDist;
        divMin = std::min(divMin, diversities[i]);
        divMax = std::max(divMax, diversities[i]);
    }

    std::vector<double> fitness(pool.size(), 0.0);
    constexpr double bMin = 0.05;
    constexpr double bMax = 0.15;
    for (int i = 0; i < static_cast<int>(pool.size()); ++i) {
        double objectiveNorm = 0.0;
        if (objMax > objMin + SOLVER_EPS) {
            objectiveNorm = (objectives[i] - objMin) / (objMax - objMin);
        }

        double diversityNorm = 0.0;
        if (divMax > divMin + SOLVER_EPS) {
            diversityNorm = (diversities[i] - divMin) / (divMax - divMin);
        }

        double imbalanceNorm = 0.0;
        if (imbalanceMax > SOLVER_EPS) {
            imbalanceNorm = imbalances[i] / imbalanceMax;
        }

        const double aWeight = objectiveNorm;
        const double bWeight = bMin + (bMax - bMin) * imbalanceNorm;
        fitness[i] =
            aWeight * objectiveNorm +
            (1.0 - aWeight) * diversityNorm +
            bWeight * imbalanceNorm;
    }

    std::vector<int> order(pool.size());
    std::iota(order.begin(), order.end(), 0);
    std::sort(order.begin(), order.end(), [&](int lhs, int rhs) {
        if (std::fabs(fitness[lhs] - fitness[rhs]) > SOLVER_EPS) return fitness[lhs] > fitness[rhs];
        return pool[lhs].makespan < pool[rhs].makespan;
        });

    int bestIdx = 0;
    for (int i = 1; i < static_cast<int>(pool.size()); ++i) {
        if (pool[i].makespan < pool[bestIdx].makespan - 1e-6) bestIdx = i;
    }

    population.clear();
    population.reserve(targetSize);
    population.push_back(pool[bestIdx]);
    for (int idx : order) {
        if (idx == bestIdx) continue;
        population.push_back(pool[idx]);
        if (static_cast<int>(population.size()) >= targetSize) break;
    }
}
TSPDSSolution TSPDSSolver::solve(double Best) {
    std::mt19937 rng(random_seed ^ (0x9e3779b9u + static_cast<unsigned>(run_id + 17)));
    std::uniform_real_distribution<double> uniDist(0.0, 1.0);

    using Clock = std::chrono::steady_clock;
    const auto start_time = Clock::now();

    auto elapsedSecs = [&]() -> double {
        return std::chrono::duration_cast<std::chrono::duration<double>>(Clock::now() - start_time).count();
        };

    auto elapsedMs = [&]() -> long long {
        return std::chrono::duration_cast<std::chrono::milliseconds>(Clock::now() - start_time).count();
        };

    const std::size_t tid = std::hash<std::thread::id>{}(std::this_thread::get_id());

    std::vector<TSPDSSolution> population = generateInitialPopulation();
    if (population.empty()) {
        TSPDSSolution fallback = initGenerator.generateInitialSolution();
        utils.evaluateSolution(fallback, /*needCalDrone=*/true);
        fallback.total_iter = 0;
        return fallback;
    }

    TSPDSSolution bestSolution = *std::min_element(population.begin(), population.end(),
        [](const TSPDSSolution& lhs, const TSPDSSolution& rhs) {
            return lhs.makespan < rhs.makespan;
        });
    bestSolution.max_resAt = 0;
    bestSolution.find_best_time = static_cast<int>(elapsedMs());

    int iteration = 0;
    int noImprovePopulation = 0;

    while (elapsedSecs() < params.max_run_time) {
        if (params.verbose) {
            std::cout << "\n[Run " << run_id << ", Thread " << tid << "] "
                << "-- Population Iteration " << iteration
                << " , best = " << bestSolution.makespan
                << " , elapsed = " << elapsedSecs() << "s / " << params.max_run_time << "s --\n";
        }

        if (noImprovePopulation >= params.population_rebuild_threshold) {
            if (params.verbose) {
                std::cout << "Population rebuilding triggered after "
                    << noImprovePopulation << " non-improving generations.\n";
            }
            population = generateInitialPopulation();
            updatePopulation(population, {}, bestSolution);
            noImprovePopulation = 0;
        }

        auto [pa, pb] = selectParents(population, rng);
        if (pa < 0 || pb < 0) break;

        std::vector<TSPDSSolution> offspring = crossover(population[pa], population[pb]);
        std::vector<TSPDSSolution> improvedOffspring;
        improvedOffspring.reserve(offspring.size());
        bool generationImproved = false;

        for (auto& child : offspring) {
            if (elapsedSecs() >= params.max_run_time) break;

            if (uniDist(rng) < params.mutation_probability) {
                int mutationK = std::max(1, params.mutation_k);
                child = perturbOps.perturbSolution3(child, mutationK);
                utils.evaluateSolution(child, /*needCalDrone=*/true);
            }

            child = localSearchOps.localSearch(child, true);
            utils.evaluateSolution(child, /*needCalDrone=*/true);

            if (child.makespan < bestSolution.makespan - 1e-6) {
                utils.optimizeTruckRouteWithLKHIntern(child);
                utils.evaluateSolution(child, /*needCalDrone=*/true);

                if (child.makespan < bestSolution.makespan - 1e-6) {
                    bestSolution = child;
                    bestSolution.max_resAt = iteration;
                    bestSolution.find_best_time = static_cast<int>(elapsedMs());
                    generationImproved = true;
                    noImprovePopulation = 0;

                    if (params.verbose) {
                        std::cout << "*** New global best: " << bestSolution.makespan << "\n";
                        utils.printSolution(bestSolution);
                    }

                    if (std::fabs(bestSolution.makespan - Best) <= 1e-9) {
                        if (params.verbose) std::cout << "Known best reached, stop early.\n";
                        improvedOffspring.push_back(child);
                        updatePopulation(population, improvedOffspring, bestSolution);
                        bestSolution.total_iter = iteration + 1;
                        return bestSolution;
                    }
                }
            }

            improvedOffspring.push_back(std::move(child));
        }

        updatePopulation(population, improvedOffspring, bestSolution);
        if (!generationImproved) {
            ++noImprovePopulation;
        }

        ++iteration;
    }

    if (params.verbose) {
        std::cout << "*** Final global best: " << bestSolution.makespan << "\n";
        utils.printSolution(bestSolution);
    }

    bestSolution.total_iter = iteration;
    utils.optimizeTruckRouteWithLKHIntern(bestSolution);
    utils.evaluateSolution(bestSolution, /*needCalDrone=*/true);
    return bestSolution;
}
