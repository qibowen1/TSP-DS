#include "TSPDSUtils.h"
#include <algorithm>
#include <random>
#include <cmath>
#include <chrono>
#include <iostream>
#include <unordered_map>
#include <queue>
#include <numeric>
#include <limits>
#include <unordered_set>  // 必须包含该头文件
#include <fstream>
#include <sstream>
#include <string>
#include <filesystem>
#include <iomanip>
#include <ctime>
#include <functional>
#include <vector>
#include <mutex>
#include <thread>

// 放在匿名 namespace 里，整个文件共享这一个互斥量
namespace {
    std::mutex g_lkh_mutex;

static long long makeLkhSeed(const std::string& tag = std::string()) {
    const auto ticks = std::chrono::high_resolution_clock::now().time_since_epoch().count();
    const auto tid = std::hash<std::thread::id>{}(std::this_thread::get_id());
    const auto h = std::hash<std::string>{}(tag);
    long long seed = static_cast<long long>((static_cast<unsigned long long>(ticks) ^ (tid << 1) ^ (h << 7)) & 0x7fffffffULL);
    return std::max(1LL, seed);
}

}


// LKH库函数声明
#ifdef __cplusplus
extern "C" {
#endif
    int LKH_main(int argc, char* argv[]);
#ifdef __cplusplus
}
#endif


// 静态成员初始化
std::random_device TSPDSUtils::rd;
std::mt19937 TSPDSUtils::gen(rd());

TSPDSUtils::TSPDSUtils(const TSPDSGraph& graph, const TSPDSAlgorithmParams& params)
	:graph(graph), params(params) {
}


void TSPDSUtils::evaluateSolution(TSPDSSolution& solution, bool needCalDrone) {
    
    if (needCalDrone)
    {
        droneScheduler.scheduleDrones(solution, graph);
    }
    // 2. 计算 makespan & 各时间指标
    solution.makespan = calculateMakespan(solution);

    double T = solution.makespan;
    double Tt = solution.truck_completion_time;
    double Td = solution.drone_completion_time;//现在是drone_completion_time = station_activation_time + Cmax
    double Ta = solution.station_activation_time;

    // 3. 诊断谁是瓶颈（和你原来一致）
    if (Tt > Td) {
        solution.diagnosis = "TRUCK_BOTTLENECK";
    }
    else {
        solution.diagnosis = "DRONE_BOTTLENECK";
    }

    // ====== 4. 计算各个指标 ======

    // (1) 不平衡度 [0,1]
    double imbalance = std::fabs(Tt - Td) / T;

    // (2) 卡车利用率 [0,1] —— 越接近 1 越好
    double truck_util = Tt / T;
    double truck_util_penalty = 1.0 - truck_util;    // 惩罚项

    // (3) 无人机利用率 [0,1] —— 在可用时间段内越满越好
    double drone_util = 0.0;
    if (T > Ta + 1e-9 && Td > Ta) {
        drone_util = (Td - Ta) / (T - Ta);
        if (drone_util > 1.0) drone_util = 1.0;
        if (drone_util < 0.0) drone_util = 0.0;
    }
    double drone_util_penalty = 1.0 - drone_util;    // 惩罚项

    // (4) 激活时间占比 [0,1] —— 越早越好
    double activation_ratio = Ta / T;

    // ====== 5. 组合评分 ======
    // 权重不要太大，保证 makespan 始终是主导
    const double w_imb = 0.5;   // 不平衡度权重
    const double w_ut = 0.2;   // 卡车利用率
    const double w_ud = 0.2;   // 无人机利用率
    const double w_act = 0.1;   // 激活时间

    double secondary =
        w_imb * imbalance +
        w_ut * truck_util_penalty +
        w_ud * drone_util_penalty +
        w_act * activation_ratio;

    // secondary 大概在 [0,1] 内，加到 T 上只作为“细微 tie-breaker”
    solution.combined_score = solution.makespan + secondary;
}

bool TSPDSUtils::isSolutionValid(const TSPDSSolution& solution) {
    bool valid = true;

    // 1. 检查卡车路径的基本约束
    if (!validateTruckRoute(solution)) {
        std::cerr << "卡车路径验证失败!" << std::endl;
        valid = false;
    }

    // 2. 检查节点服务分配约束
    if (!validateNodeAssignment(solution)) {
        std::cerr << "节点分配验证失败!" << std::endl;
        valid = false;
    }

    // 3. 检查无人机任务约束
    if (!validateDroneAssignments(solution)) {
        std::cerr << "无人机分配验证失败!" << std::endl;
        valid = false;
    }

    // 4. 检查时间约束
    if (!validateTimeConstraints(solution)) {
        std::cerr << "时间约束验证失败!" << std::endl;
        valid = false;
    }

    // 5. 检查无人机站激活约束
    if (!validateDroneStationActivation(solution)) {
        std::cerr << "无人机站激活验证失败!" << std::endl;
        valid = false;
    }

    // 6. 检查解的完整性
    if (!validateSolutionCompleteness(solution)) {
        std::cerr << "解完整性验证失败!" << std::endl;
        valid = false;
    }

    if (valid) {
        std::cout << "✓ 解验证通过!" << std::endl;
    }
    else {
        std::cout << "✗ 解验证失败!" << std::endl;
    }

    return valid;
}

bool TSPDSUtils::validateTruckRoute(const TSPDSSolution& solution) {

    // 1. 检查路径是否以仓库开始和结束
    if (solution.truck_route.empty()) {
        std::cerr << "错误: 卡车路径为空" << std::endl;
        return false;
    }

    if (solution.truck_route.front() != graph.depot) {
        std::cerr << "错误: 卡车路径不以仓库开始" << std::endl;
        return false;
    }

    if (solution.truck_route.back() != graph.depot) {
          std::cerr << "错误: 卡车路径不以仓库结束" << std::endl;
        return false;
    }

    // 2. 检查无人机站是否被访问
    bool droneStationVisited = false;
    for (int node : solution.truck_route) {
        if (node == graph.drone_station) {
            droneStationVisited = true;
            break;
        }
    }

    if (!droneStationVisited) {
        std::cerr << "错误: 无人机站未被卡车访问" << std::endl;
        return false;
    }

    // 3. 检查路径连续性
    for (size_t i = 0; i < solution.truck_route.size() - 1; ++i) {
        int from = solution.truck_route[i];
        int to = solution.truck_route[i + 1];

        if (graph.truck_time[from][to] < 0) {
            std::cerr << "错误: 节点 " << from << " 到 " << to << " 的旅行时间无效" << std::endl;
            return false;
        }
    }

    // 4. 检查子回路消除（简单检查）
    std::vector<bool> visited(graph.nodes.size(), false);
    for (int node : solution.truck_route) {
        if (node != graph.depot && visited[node]) {
            std::cerr << "错误: 节点 " << node << " 被重复访问" << std::endl;
            return false;
        }
        visited[node] = true;
    }
    return true;
}

bool TSPDSUtils::validateNodeAssignment(const TSPDSSolution& solution) {

    // 1. 检查所有客户点都被服务
    for (int i = 0; i < graph.nodes.size(); ++i) {
        if (i == graph.depot) continue; // 跳过仓库

        bool servedByTruck = solution.served_by_truck[i];
        bool servedByDrone = solution.served_by_drone[i];

        if (!servedByTruck && !servedByDrone) {
            std::cerr << "错误: 节点 " << i << " 未被服务" << std::endl;
            return false;
        }

        if (servedByTruck && servedByDrone) {
            std::cerr << "错误: 节点 " << i << " 被重复服务" << std::endl;
            return false;
        }
    }
    for (int i = 0; i < graph.nodes.size(); ++i) {
        if (solution.served_by_drone[i]) {
            if (graph.is_drone_eligible.empty() || i < 0 || i >= static_cast<int>(graph.is_drone_eligible.size()) ||
                !graph.is_drone_eligible[i]) {
                std::cerr << "Error: node " << i << " is served by drone but is not drone eligible" << std::endl;
                return false;
            }
            double droneTime = graph.drone_time[graph.drone_station][i];
            if (droneTime < 0 || droneTime > std::numeric_limits<double>::max() / 2) {
                std::cerr << "Error: node " << i << " has invalid drone travel time" << std::endl;
                return false;
            }
        }
    }

    return true;
}

bool TSPDSUtils::validateDroneAssignments(const TSPDSSolution& solution) {

    // 1. 检查无人机数量约束
    if (solution.drone_assignments.size() > graph.drone_count) {
        std::cerr << "错误: 使用的无人机数量超过限制" << std::endl;
        return false;
    }

    // 2. 检查每个无人机任务的有效性
    std::vector<bool> droneServed(graph.nodes.size(), false);

    for (const auto& assignment : solution.drone_assignments) {
        int droneId = assignment.first;
        const auto& tasks = assignment.second;

        // 检查无人机ID有效性
        if (droneId < 0 || droneId >= graph.drone_count) {
            std::cerr << "错误: 无效的无人机ID " << droneId << std::endl;
            return false;
        }

        // 检查每个任务节点
        for (int node : tasks) {
            // 检查节点有效性
            if (node < 0 || node >= graph.nodes.size()) {
                std::cerr << "错误: 无效的节点ID " << node << std::endl;
                return false;
            }

            // 检查节点是否由无人机服务
            if (!solution.served_by_drone[node]) {
                std::cerr << "错误: 节点 " << node << " 未被标记为无人机服务" << std::endl;
                return false;
            }

            // 检查节点是否可被无人机服务
            if (!graph.is_drone_eligible[node]) {
                std::cerr << "错误: 节点 " << node << " 不可被无人机服务" << std::endl;
                return false;
            }

            // 检查节点是否被重复分配
            if (droneServed[node]) {
                std::cerr << "错误: 节点 " << node << " 被多个无人机服务" << std::endl;
                return false;
            }
            droneServed[node] = true;
        }
    }

    // 3. 验证node_to_drone映射的一致性
    for (const auto& mapping : solution.node_to_drone) {
        int node = mapping.first;
        int droneId = mapping.second;

        // 检查映射关系是否一致
        const auto& tasks = solution.drone_assignments.at(droneId);
        if (std::find(tasks.begin(), tasks.end(), node) == tasks.end()) {
            std::cerr << "错误: node_to_drone映射不一致" << std::endl;
            return false;
        }
    }

    return true;
}

bool TSPDSUtils::validateTimeConstraints(const TSPDSSolution& solution) {

    // 1. 检查时间值的有效性
    if (solution.makespan < 0) {
        std::cerr << "错误: 无效的makespan值" << std::endl;
        return false;
    }

    if (solution.truck_completion_time < 0 || solution.drone_completion_time < 0) {
        std::cerr << "错误: 无效的完成时间" << std::endl;
        return false;
    }

    // 2. 检查makespan计算是否正确
    double expectedMakespan = std::max(solution.truck_completion_time,
        solution.drone_completion_time);

    if (std::abs(solution.makespan - expectedMakespan) > 1e-6) {
        std::cerr << "错误: makespan计算不正确" << std::endl;
        std::cerr << "期望值: " << expectedMakespan
            << ", 实际值: " << solution.makespan << std::endl;
        return false;
    }

    // 3. 检查无人机完成时间计算
    if (solution.drone_completion_time < solution.station_activation_time - 1e-6) {
        std::cerr << "错误: 无人机完成时间早于无人机站激活时间" << std::endl;
        return false;
    }

    return true;
}

bool TSPDSUtils::validateDroneStationActivation(const TSPDSSolution& solution) {

    // 查找无人机站在卡车路径中的位置
    int stationPos = -1;
    double activationTime = 0.0;

    for (size_t i = 0; i < solution.truck_route.size() - 1; ++i) {
        int from = solution.truck_route[i];
        int to = solution.truck_route[i + 1];
        activationTime += graph.truck_time[from][to];

        if (to == graph.drone_station) {
            stationPos = i + 1;
            break;
        }
    }

    if (stationPos == -1) {
        std::cerr << "错误: 无法找到无人机站激活时间" << std::endl;
        return false;
    }

    // 验证激活时间计算
    if (std::abs(activationTime - solution.station_activation_time) > 1e-6) {
        std::cerr << "错误: 无人机站激活时间计算不正确" << std::endl;
        std::cerr << "计算值: " << activationTime
            << ", 存储值: " << solution.station_activation_time << std::endl;
        return false;
    }


    return true;
}

bool TSPDSUtils::validateSolutionCompleteness(const TSPDSSolution& solution) {


    // 1. 检查所有必须访问的节点都被服务
    std::vector<int> mustVisit = { graph.drone_station };
    for (int i = 0; i < graph.nodes.size(); ++i) {
        if (i != graph.depot) {
            mustVisit.push_back(i);
        }
    }

    for (int node : mustVisit) {
        bool visited = false;

        // 检查是否在卡车路径中
        for (int routeNode : solution.truck_route) {
            if (routeNode == node) {
                visited = true;
                break;
            }
        }

        // 检查是否被无人机服务
        if (!visited) {
            for (const auto& assignment : solution.drone_assignments) {
                for (int droneNode : assignment.second) {
                    if (droneNode == node) {
                        visited = true;
                        break;
                    }
                }
                if (visited) break;
            }
        }

        if (!visited) {
            std::cerr << "错误: 必须访问的节点 " << node << " 未被服务" << std::endl;
            return false;
        }
    }

    // 2. 检查解的诊断信息合理性
    if (solution.diagnosis.empty()) {
        std::cerr << "警告: 解的诊断信息为空" << std::endl;
    }

    // 3. 检查综合评分计算
    if (solution.combined_score < 0) {
        std::cerr << "警告: 综合评分为负值" << std::endl;
    }

    return true;
}

double TSPDSUtils::calculateMakespan(TSPDSSolution& solution) {
    double truckTime = 0;
    double droneActiveTime = 0;
    bool isDroneStationReached = false;
    int locate = 0;
    for (int i = 0; i < solution.truck_route.size() - 1; ++i) {
        locate++;
        int from = solution.truck_route[i];
        int to = solution.truck_route[i + 1];
        if (isDroneStationReached == false && to != graph.drone_station) {
            droneActiveTime += graph.truck_time[from][to];
        }
        if (to == graph.drone_station) {
            droneActiveTime += graph.truck_time[from][to];
            solution.dorne_visit_dep = locate;
            isDroneStationReached = true;
        }
        truckTime += graph.truck_time[from][to];
    }
    solution.truck_completion_time = truckTime;
    solution.station_activation_time = droneActiveTime;
    double droneTime = 0;
    for (const auto& assignment : solution.drone_assignments) {
        double droneTotalTime = 0;
        for (int node : assignment.second) {
            droneTotalTime += 2.0 * graph.drone_time[graph.drone_station][node];
        }
        double completionTime = droneTotalTime;
        if (completionTime > droneTime) {
            droneTime = completionTime;
        }
    }
    solution.drone_completion_time = solution.station_activation_time + droneTime;
    return max(truckTime, solution.drone_completion_time);
}

//辅助函数
// 计算如果将节点从卡车转移给无人机可能节省的时间 full estimate
double TSPDSUtils::calculateTimeSaveIfReassign(const TSPDSSolution& solution, int node)
{
    // 0) 必须保证已评估（避免在这里偷偷 evaluate）
    if (solution.makespan <= 0.0) {
        return 0.0;
    }

    // 1) node 必须真的是卡车服务且 drone-eligible
    if (!solution.served_by_truck[node] ||
        node == graph.depot || node == graph.drone_station ||
        !graph.is_drone_eligible[node]) {
        return 0.0;
    }

    // 2) node 在卡车路径中的位置合法（不能是首尾 depot）
    auto it = std::find(solution.truck_route.begin(), solution.truck_route.end(), node);
    if (it == solution.truck_route.end() ||
        it == solution.truck_route.begin() ||
        it == solution.truck_route.end() - 1) {
        return 0.0;
    }

    const double oldMakespan = solution.makespan;

    // 3) 模拟 truck -> drone
    TSPDSSolution testSolution = solution;
    if (!reassignTruckToDrone(testSolution, node)) {
        return 0.0;
    }

    // 4) full evaluate
    evaluateSolution(testSolution, /*needCalDrone=*/true);
    const double newMakespan = testSolution.makespan;

    return oldMakespan - newMakespan;
}

void TSPDSUtils::buildPosInTruck(TSPDSSolution& sol)
{
    int n = (int)graph.truck_time.size();
    if ((int)sol.pos_in_truck.size() != n) {
        sol.pos_in_truck.assign(n, -1);
    }
    else {
        std::fill(sol.pos_in_truck.begin(), sol.pos_in_truck.end(), -1);
    }

    sol.pos_station_in_truck = -1;

    for (int idx = 0; idx < (int)sol.truck_route.size(); ++idx) {
        int v = sol.truck_route[idx];
        if (v < 0 || v >= n) continue;
        sol.pos_in_truck[v] = idx;
        if (v == graph.drone_station) sol.pos_station_in_truck = idx;
    }
}

void TSPDSUtils::buildRemoveSaving(const TSPDSSolution& sol,
    std::vector<double>& removeSaving)
{
    int n = (int)graph.truck_time.size();
    removeSaving.assign(n, 0.0);

    const auto& R = sol.truck_route;
    if ((int)R.size() < 3) return;

    for (int k = 1; k + 1 < (int)R.size(); ++k) {
        int i = R[k];
        if (i == graph.depot || i == graph.drone_station) continue;
        if (!sol.served_by_truck.empty() && !sol.served_by_truck[i]) continue;
        if (!graph.is_drone_eligible.empty() && !graph.is_drone_eligible[i]) continue;

        int prev = R[k - 1];
        int next = R[k + 1];

        double a = graph.truck_time[prev][i];
        double b = graph.truck_time[i][next];
        double c = graph.truck_time[prev][next];

        double dTt = a + b - c;
        if (dTt < 0) dTt = 0; // 数值保护
        removeSaving[i] = dTt;
    }
}

DroneLoadCache TSPDSUtils::buildDroneLoadCache(const TSPDSSolution& sol)
{
    DroneLoadCache cache;
    int m = std::max(1, graph.drone_count);

    std::vector<double> loads(m, 0.0);

    for (const auto& kv : sol.drone_assignments) {
        int d = kv.first;
        if (d < 0 || d >= m) continue; // 防御：unordered_map key 不规范
        for (int node : kv.second) {
            // 往返任务时间
            loads[d] += 2.0 * graph.drone_time[graph.drone_station][node];
        }
    }

    cache.minLoad = *std::min_element(loads.begin(), loads.end());
    cache.Cmax = *std::max_element(loads.begin(), loads.end());

    // 如果 sol.drone_completion_time 本身就是 Cmax（不含激活），取更大更稳
    cache.Cmax = std::max(cache.Cmax, sol.drone_completion_time);
    return cache;
}


double TSPDSUtils::estimatePotentialTruckToDrone_O1(
    const TSPDSSolution& sol,
    int node,
    const std::vector<double>& removeSaving,
    const DroneLoadCache& cache)
{
    // 可行性检查
    if (node == graph.depot || node == graph.drone_station) return -1e100;
    if (!graph.is_drone_eligible.empty() && !graph.is_drone_eligible[node]) return -1e100;
    if (!sol.served_by_truck.empty() && !sol.served_by_truck[node]) return -1e100;

    int posNode = (node >= 0 && node < (int)sol.pos_in_truck.size()) ? sol.pos_in_truck[node] : -1;
    int posS = sol.pos_station_in_truck;
    if (posNode <= 0 || posNode >= (int)sol.truck_route.size() - 1) return -1e100;
    if (posS < 0) return -1e100;

    double dTt = 0.0;
    if (node >= 0 && node < (int)removeSaving.size()) dTt = removeSaving[node];
    if (dTt < 0) dTt = 0;

    double dTa = (posNode < posS) ? dTt : 0.0;

    // 无人机任务时间 & 航程
    double p = 2.0 * graph.drone_time[graph.drone_station][node];
    if (p > graph.drone_range) return -1e100;

    double Cmax2 = std::max(cache.Cmax, cache.minLoad + p);//当前最晚的无人机 : 将noded插入给当前最轻松的无人机 近似

    // 当前 makespan
    double M = std::max(sol.truck_completion_time,sol.drone_completion_time);

    double Tt2 = sol.truck_completion_time - dTt;
    double Ta2 = sol.station_activation_time - dTa;
    double M2 = std::max(Tt2, Ta2 + Cmax2);

    return M - M2; // >0 表示预计改进
}


//无人机->卡车的缓存，潜力评估
DroneLoadCacheDT TSPDSUtils::buildDroneLoadCacheDT(const TSPDSSolution& sol) const
{
    DroneLoadCacheDT c;
    const int n = (int)graph.drone_time.size();
    const int m = std::max(1, graph.drone_count);

    c.loads.assign(m, 0.0);
    c.node2drone.assign(n, -1);

    // key 已保证合法：0..m-1
    for (const auto& kv : sol.drone_assignments) {
        int d = kv.first;
        if (d < 0 || d >= m) continue;
        for (int node : kv.second) {
            if (node < 0 || node >= n) continue;
            c.node2drone[node] = d;
            c.loads[d] += 2.0 * graph.drone_time[graph.drone_station][node];
        }
    }

    const double EPS = 1e-9;

    // max1/argmax
    c.max1 = -1.0;
    c.argmax = 0;
    for (int d = 0; d < m; ++d) {
        if (c.loads[d] > c.max1 + EPS) {
            c.max1 = c.loads[d];
            c.argmax = d;
        }
    }
    if (c.max1 < 0) { c.max1 = 0.0; c.argmax = 0; }

    // countMax / max2(严格第二大)
    c.countMax = 0;
    double secondStrict = -1.0;
    for (int d = 0; d < m; ++d) {
        double ld = c.loads[d];
        if (std::fabs(ld - c.max1) <= EPS) c.countMax++;
        else if (ld < c.max1 - EPS) secondStrict = std::max(secondStrict, ld);
    }
    c.max2 = (secondStrict < 0.0) ? c.max1 : secondStrict;

    return c;
}

void TSPDSUtils::buildBestInsertionCacheForNodes(
    const TSPDSSolution& sol,
    const std::vector<int>& candNodes,
    InsertPolicy policy,
    std::vector<double>& bestInc,
    std::vector<int>& bestPos) const
{
    const int n = (int)graph.truck_time.size();
    bestInc.assign(n, std::numeric_limits<double>::infinity());
    bestPos.assign(n, -1);

    const auto& r = sol.truck_route;
    if ((int)r.size() < 2) return;

    int stationPos = sol.pos_station_in_truck;
    int edgeStart = 0;

    if (policy == InsertPolicy::AFTER_STATION && stationPos >= 0) {
        edgeStart = stationPos; // 从 station 边开始插
    }
    else {
        edgeStart = 0;          // ANYWHERE
    }

    const int edgeEnd = (int)r.size() - 2; // 遍历边 i->i+1
    if (edgeStart > edgeEnd) return;

    // 去重候选（max-load 无人机任务数）
    std::vector<int> nodes = candNodes;
    std::sort(nodes.begin(), nodes.end());
    nodes.erase(std::unique(nodes.begin(), nodes.end()), nodes.end());

    for (int ei = edgeStart; ei <= edgeEnd; ++ei) {
        int from = r[ei];
        int to = r[ei + 1];
        double base = graph.truck_time[from][to];

        for (int node : nodes) {
            if (node == graph.depot || node == graph.drone_station) continue;

            // 防重复：如果 node 已在 truck_route，跳过
            if (node >= 0 && node < (int)sol.pos_in_truck.size() && sol.pos_in_truck[node] != -1) continue;

            double inc = graph.truck_time[from][node] + graph.truck_time[node][to] - base;
            if (inc < bestInc[node]) {
                bestInc[node] = inc;
                bestPos[node] = ei + 1; // 插入位置
            }
        }
    }
}

double TSPDSUtils::estimatePotentialDroneToTruck_O1(
    const TSPDSSolution& sol,
    int node,
    double insInc,
    int insPos,
    const DroneLoadCacheDT& c) const
{
    if (node == graph.depot || node == graph.drone_station) return -1e100;
    if (!sol.served_by_drone.empty() && !sol.served_by_drone[node]) return -1e100;
    if (!sol.served_by_truck.empty() && sol.served_by_truck[node]) return -1e100;
    if (!std::isfinite(insInc) || insPos < 0) return -1e100;

    // 1) truck 完工时间增加
    const double Tt2 = sol.truck_completion_time + std::max(0.0, insInc);

    // 2) activation 时间变化：仅当插入在 station 之前才会增加 Ta
    double Ta2 = sol.station_activation_time;
    if (sol.pos_station_in_truck >= 0) {
        int edgeIdx = insPos - 1; // 插入的是哪条边 ei->ei+1
        if (edgeIdx < sol.pos_station_in_truck) {
            Ta2 += std::max(0.0, insInc);
        }
    }

    // 3) drone Cmax 变化：移除 node 的往返任务
    int d = (node >= 0 && node < (int)c.node2drone.size()) ? c.node2drone[node] : -1;
    if (d < 0 || d >= (int)c.loads.size()) return -1e100;

    const double EPS = 1e-9;
    double p = 2.0 * graph.drone_time[graph.drone_station][node];

    double Cmax2 = c.max1;
    double load_d = c.loads[d];

    if (load_d < c.max1 - EPS) {
        // 非最大负载无人机：移除不影响 Cmax
        Cmax2 = c.max1;
    }
    else {
        // 这是最大负载无人机（可能并列）
        if (c.countMax >= 2) {
            // 并列最大：移除一个节点后至少还有一台保持 max1
            Cmax2 = c.max1;
        }
        else {
            // 唯一最大：可能降到 max(max2, load_d - p)
            double newLoad = std::max(0.0, load_d - p);
            Cmax2 = std::max(c.max2, newLoad);
        }
    }

    // 4) makespan 改变量
    const double M = std::max(sol.truck_completion_time, sol.drone_completion_time);
    const double M2 = std::max(Tt2, Ta2 + Cmax2);

    return M - M2; // >0 预计变好 只是预判断
}


// 便宜潜力：仅估计“从卡车路径移除 node 能节省多少卡车时间” quick estimate
// 用途：排序/筛选，不保证与最终 makespan 改善一致，但非常快
double TSPDSUtils::estimateTruckEdgeSavingIfRemove(const TSPDSSolution& solution, int node)
{
    if (!solution.served_by_truck[node] ||
        node == graph.depot || node == graph.drone_station ||
        !graph.is_drone_eligible[node]) {
        return 0.0;
    }

    int idx = solution.pos_in_truck[node];
    int prev = solution.truck_route[idx - 1];
    int next = solution.truck_route[idx + 1];


    // TODO: 用你自己的“卡车行驶时间/距离”接口替换下面三行
    double a = graph.truck_time[prev][node];
    double b = graph.truck_time[node][next];
    double c = graph.truck_time[prev][next];

    return (a + b - c);
}





double TSPDSUtils::calculateTransferPotential(const TSPDSSolution& solution, int node) {
    // 计算将节点从无人机转移给卡车的潜力
    // 潜力公式：无人机时间节省 - 卡车时间增加

    double droneTimeSaved = 0.0;
    double truckTimeAdded = 0.0;

    // 计算无人机时间节省（从无人机站到该节点的往返时间） 
    if (solution.node_to_drone.find(node) != solution.node_to_drone.end()) {
        int droneId = solution.node_to_drone.at(node);
        droneTimeSaved = 2.0 * graph.drone_time[graph.drone_station][node];

        // 如果这是该无人机的唯一任务，可能释放整个无人机
        if (solution.drone_assignments.at(droneId).size() == 1) {
            droneTimeSaved *= 1.2; // 额外奖励，因为可能完全平衡负载
        }
    }

    // 估算添加到卡车路径的最佳位置时间增加
    truckTimeAdded = estimateTruckTimeIncrease(solution, node);

    // 潜力 = 时间节省 - 时间增加，考虑无人机瓶颈的权重
    double potential = droneTimeSaved - truckTimeAdded;

    // 如果是严重无人机瓶颈，给转移更高的权重
    if (solution.diagnosis == "DRONE_BOTTLENECK") {
        potential *= 1.5;
    }

    return potential;
}


double TSPDSUtils::estimateTruckTimeIncrease(const TSPDSSolution& solution, int node) {
    // 找到插入到卡车路径中的最佳位置，返回时间增加量
    double minTimeIncrease = std::numeric_limits<double>::max();
    int n = solution.truck_route.size();

    if (n <= 1) {
        return graph.truck_time[graph.depot][node] + graph.truck_time[node][graph.depot];
    }

    // 尝试所有可能的插入位置
    for (int i = 0; i < n - 1; ++i) {
        int from = solution.truck_route[i];
        int to = solution.truck_route[i + 1];

        // 计算插入新节点增加的时间
        double originalTime = graph.truck_time[from][to];
        double newTime = graph.truck_time[from][node] + graph.truck_time[node][to];
        double timeIncrease = newTime - originalTime;

        if (timeIncrease < minTimeIncrease) {
            minTimeIncrease = timeIncrease;
        }
    }

    // 也考虑插入到末尾的情况（在最后一个节点和depot之间）
    int lastNode = solution.truck_route[n - 1];
    double timeIncrease = graph.truck_time[lastNode][node] + graph.truck_time[node][graph.depot]
        - graph.truck_time[lastNode][graph.depot];

    if (timeIncrease < minTimeIncrease) {
        minTimeIncrease = timeIncrease;
    }

    return std::max(0.0, minTimeIncrease); // 确保非负
}

int TSPDSUtils::findLeastLoadedDrone(const TSPDSSolution& solution) {
    int bestDrone = 0;
    double minLoad = std::numeric_limits<double>::max();

    for (int d = 0; d < graph.drone_count; ++d) {
        double load = 0.0;
        for (int node : solution.drone_assignments.at(d)) {
            load += graph.drone_time[graph.drone_station][node] * 2;
        }
        if (load < minLoad) {
            minLoad = load;
            bestDrone = d;
        }
    }
    return bestDrone;
}

// 将卡车节点重新分配给无人机
bool TSPDSUtils::reassignTruckToDrone(TSPDSSolution& solution, int node) {
    if (!graph.is_drone_eligible.empty() &&
        (node < 0 || node >= static_cast<int>(graph.is_drone_eligible.size()) ||
         !graph.is_drone_eligible[node])) {
        return false;
    }

    auto it = std::find(solution.truck_route.begin(), solution.truck_route.end(), node);
    if (it == solution.truck_route.end() || it == solution.truck_route.begin() ||
        it == solution.truck_route.end() - 1) {
        return false;
    }

    // 从卡车路径中移除
    int pos = it - solution.truck_route.begin();
    solution.truck_route.erase(it);
    solution.served_by_truck[node] = false;

    // 直接分配给无人机 evalute的时候会重新调度
    solution.served_by_drone[node] = true;
    return true;
}

bool TSPDSUtils::insertNodeToTruckRoute(TSPDSSolution& solution, int node) {
    if (solution.truck_route.empty()) {
        solution.truck_route.push_back(node);
        return true;
    }

    int bestPosition = -1;
    double minTimeIncrease = std::numeric_limits<double>::max();
    int n = solution.truck_route.size();

    // 找到最佳插入位置
    for (int i = 0; i < n - 1; ++i) {
        int from = solution.truck_route[i];
        int to = solution.truck_route[i + 1];

        double originalTime = graph.truck_time[from][to];
        double newTime = graph.truck_time[from][node] + graph.truck_time[node][to];
        double timeIncrease = newTime - originalTime;

        if (timeIncrease < minTimeIncrease) {
            minTimeIncrease = timeIncrease;
            bestPosition = i + 1; // 插入在i之后
        }
    }

    // 检查插入末尾的情况
    int lastNode = solution.truck_route[n - 1];
    double endTimeIncrease = graph.truck_time[lastNode][node] + graph.truck_time[node][graph.depot]
        - graph.truck_time[lastNode][graph.depot];

    if (endTimeIncrease < minTimeIncrease) {
        minTimeIncrease = endTimeIncrease;
        bestPosition = n; // 插入到末尾
    }

    if (bestPosition != -1) {
        // 执行插入
        if (bestPosition == n) {
            // 插入到最后一个节点之后，depot之前
            solution.truck_route.insert(solution.truck_route.end() - 1, node);
        }
        else {
            solution.truck_route.insert(solution.truck_route.begin() + bestPosition, node);
        }
        return true;
    }

    return false;
}

double TSPDSUtils::calculateTwoOptDelta(const vector<int>& route, int i, int j) {
    int n = route.size();
    int a = route[i - 1], b = route[i];
    int c = route[j], d = route[j + 1];

    double original = graph.truck_time[a][b] + graph.truck_time[c][d];
    double new_edges = graph.truck_time[a][c] + graph.truck_time[b][d];

    return new_edges - original;
}

// 实际移动无人机站到新位置
TSPDSSolution TSPDSUtils::tryMoveDroneStation(TSPDSSolution& solution, int oldPos, int newPos) {
    TSPDSSolution newSolution = solution;

    // 从原位置移除无人机站
    newSolution.truck_route.erase(newSolution.truck_route.begin() + oldPos);

    // 插入到新位置
    newSolution.truck_route.insert(newSolution.truck_route.begin() + newPos, graph.drone_station);


    return newSolution;
}

// 计算移除无人机节点对完成时间的实际影响
/***********************************************************************
 *  Estimate how much drone completion time can be reduced
 *  by removing "node" from its drone's assignment.
 *
 *  This is an UPPER BOUND used for pruning only.
 ***********************************************************************/
double TSPDSUtils::calculateActualDroneTimeReduction(
    const TSPDSSolution& S, int node, int droneId)
{
    // -----------------------------
    // Step 1: Compute all drone loads
    // -----------------------------
    const int D = graph.drone_count;

    std::vector<double> load(D, 0.0);

    for (int d = 0; d < D; ++d) {
        const auto& tasks = S.drone_assignments.at(d);
        for (int n : tasks) {
            load[d] += 2.0 * graph.drone_time[graph.drone_station][n];
        }
    }

    // -----------------------------
    // Step 2: Identify current bottleneck L_max
    // -----------------------------
    double Lmax = 0.0;
    for (double L : load) Lmax = std::max(Lmax, L);

    double Ld = load[droneId];  // load of the drone that serves node

    // If removing the node cannot reduce bottleneck (Ld < Lmax), prune quickly
    if (Ld + 1e-9 < Lmax) {
        return 0.0;   // not the bottleneck → removing node won't help
    }

    // -----------------------------
    // Step 3: Compute new load for droneId after removing node
    // -----------------------------
    double removed = 2.0 * graph.drone_time[graph.drone_station][node];
    double newLd = Ld - removed;
    if (newLd < 0.0) newLd = 0.0;

    // -----------------------------
    // Step 4: Compute the second highest load (among drones != droneId)
    // -----------------------------
    double secondMax = 0.0;
    for (int d = 0; d < D; ++d) {
        if (d == droneId) continue;
        secondMax = std::max(secondMax, load[d]);
    }

    // -----------------------------
    // Step 5: New overall drone completion time
    // -----------------------------
    double newLmax = std::max(newLd, secondMax);

    double reduction = Lmax - newLmax;

    // numerical stability
    if (reduction < 0.0) reduction = 0.0;

    return reduction;
}


TSPDSSolution TSPDSUtils::printSolution(TSPDSSolution& currentSolution1) {
    cout << "Makespan: " << currentSolution1.makespan << endl;
    cout << "Combine_score: " << currentSolution1.combined_score << endl;
    cout << "Truck route size: " << currentSolution1.truck_route.size() << endl;
    cout << "Truck completion time: " << currentSolution1.truck_completion_time << endl;
    cout << "Drone completion time: " << currentSolution1.drone_completion_time << endl;
    cout << "Station activation time: " << currentSolution1.station_activation_time << endl;
    cout << "Station ID: " << currentSolution1.drone_Id << endl;
    cout << "Station visit location: " << currentSolution1.dorne_visit_dep << endl;
    // 输出卡车路径详情
    int drone_task = 0;
    cout << "Truck route: ";
    for (int node : currentSolution1.truck_route) {
        cout << node << " ";
    }
    cout << endl;
    // 输出无人机分配详情
    for (int d = 0; d < graph.drone_count; ++d) {
        double time = 0;
        cout << "Drone " << d << " assignments: ";
        const auto& assignments = currentSolution1.drone_assignments[d];
        for (int node : assignments) {
            drone_task++;
			time += 2.0 * graph.drone_time[graph.drone_station][node];
            cout << node << " ";
        }
		cout << "|  drone"<<d <<" finish time: " << currentSolution1.station_activation_time + time;
        cout << endl;
    }
    cout << "drone_total_task: " << drone_task++<< endl;
    return currentSolution1;
}


/***********************************************************************
 *   Estimate how much truck time is saved if node is removed.
 *   (Used only for pruning; not exact makespan improvement.)
 ***********************************************************************/
double TSPDSUtils::estimateTruckRemovalSaving(const TSPDSSolution& S, int node)
{
    // 1. Locate the node on the truck route
    auto& route = S.truck_route;
    auto it = std::find(route.begin(), route.end(), node);

    if (it == route.end())
        return 0.0;              // Not a truck node → no saving

    // Node cannot be removed if it is the first or last (depot / station boundary)
    if (it == route.begin() || it == route.end() - 1)
        return 0.0;

    int prev = *(it - 1);
    int next = *(it + 1);

    // 2. Compute local removal saving:
    //    (prev → node → next)  vs  (prev → next)
    double original = graph.truck_time[prev][node] +
        graph.truck_time[node][next];

    double shortcut = graph.truck_time[prev][next];

    double saving = original - shortcut;

    // Saving must not be negative (removal never costs time in TSP-like graph)
    if (saving < 0.0)
        saving = 0.0;

    return saving;
}




void TSPDSUtils::optimizeTruckRouteWithLKH(TSPDSSolution& s)
{
    auto& route = s.truck_route;
    int n = static_cast<int>(route.size());
    if (n <= 3) return;

    evaluateSolution(s, /*needCalDrone=*/true);
    TSPDSSolution backup = s;
    double original_cost = s.truck_completion_time;

    // ---------- 生成唯一文件名 ----------
    const std::string temp_dir = "lkh_temp";
    std::filesystem::create_directories(temp_dir);

    auto ticks = std::chrono::high_resolution_clock::now().time_since_epoch().count();
    std::ostringstream oss;
    oss << "truck_" << ticks << "_" << std::this_thread::get_id();
    std::string base = oss.str();

    std::string tsp_path = temp_dir + "/" + base + ".tsp";
    std::string par_path = temp_dir + "/" + base + ".par";
    std::string tour_path = temp_dir + "/" + base + ".tour";

    auto cleanup_files = [&]() {
        std::error_code ec;
        std::filesystem::remove(tsp_path, ec);
        std::filesystem::remove(par_path, ec);
        std::filesystem::remove(tour_path, ec);
        };

    // ---------- 写 TSPLIB TSP 文件 ----------
    std::ofstream tsp(tsp_path);
    if (!tsp) {
        std::cerr << "Cannot write LKH TSP file\n";
        cleanup_files();
        return;
    }

    tsp << "NAME: TRUCK_TOUR\n";
    tsp << "TYPE: TSP\n";
    tsp << "COMMENT: Truck route optimization\n";
    tsp << "DIMENSION: " << n << "\n";
    tsp << "EDGE_WEIGHT_TYPE: EXPLICIT\n";
    tsp << "EDGE_WEIGHT_FORMAT: FULL_MATRIX\n";
    tsp << "EDGE_WEIGHT_SECTION\n";

    for (int i = 0; i < n; i++) {
        int ni = route[i];
        for (int j = 0; j < n; j++) {
            int nj = route[j];
            double dist = graph.truck_time[ni][nj];
            tsp << static_cast<long long>(std::llround(dist * 10.0)) << " ";
        }
        tsp << "\n";
    }
    tsp << "EOF\n";
    tsp.close();

    // ---------- 写 LKH 参数文件 ----------
    std::ofstream par(par_path);
    if (!par) {
        std::cerr << "Cannot write LKH PAR file\n";
        cleanup_files();
        return;
    }

    par << "PROBLEM_FILE = " << tsp_path << "\n";
    par << "TOUR_FILE    = " << tour_path << "\n";
    par << "RUNS = " << std::max(1, params.lkh_runs) << "\n";
    par << "TRACE_LEVEL = 0\n";
    par << "MOVE_TYPE = 5\n";
    par << "PATCHING_C = 3\n";
    par << "PATCHING_A = 2\n";
    par << "SEED = " << makeLkhSeed(base) << "\n";
    par.close();

    // ---------- 串行调用 LKH_main ----------
    {
        std::lock_guard<std::mutex> lock(g_lkh_mutex);

        char* args[] = { (char*)"LKH", (char*)par_path.c_str() };
        int argc = 2;
        int res = LKH_main(argc, args);
        if (res != 0) {
            std::cerr << "LKH failed\n";
            cleanup_files();
            return;
        }
    }

    // ---------- 读取 TOUR 文件并重建 truck_route ----------
    std::ifstream tour(tour_path);
    if (!tour) {
        std::cerr << "Cannot read tour\n";
        cleanup_files();
        return;
    }

    std::vector<int> new_order;
    std::string line;
    bool start = false;

    while (std::getline(tour, line)) {
        if (line.find("TOUR_SECTION") != std::string::npos) {
            start = true;
            continue;
        }
        if (!start) continue;
        if (line == "-1") break;

        int idx = std::stoi(line);
        if (idx >= 1 && idx <= n) {
            new_order.push_back(route[idx - 1]);  // 映射回真实节点编号
        }
    }
    tour.close();
    cleanup_files();

    if (static_cast<int>(new_order.size()) != n) {
        std::cerr << "Invalid LKH tour size\n";
        return;
    }

    s.truck_route = std::move(new_order);

    // ---------- 重新评估 ----------
    evaluateSolution(s, /*needCalDrone=*/true);
    double new_cost = s.truck_completion_time;

    if (new_cost < original_cost - 1e-6) {
        std::cout << "LKH improved truck path: " << (original_cost - new_cost) << "\n";
    }
    else {
        // 没有改善就回退
        s = backup;
    }
}




void TSPDSUtils::optimizeTruckRouteWithLKHFullPath(TSPDSSolution& s)
{
    auto& route = s.truck_route;
    const int n = static_cast<int>(route.size());
    if (n <= 4) return;
    if (s.makespan <= 0.0) evaluateSolution(s, /*needCalDrone=*/true);

    TSPDSSolution backup = s;
    const double originalMakespan = s.makespan;
    const int startNode = route.front();
    const int endNode = route.back();
    if (startNode != graph.depot || endNode != graph.depot) return;

    std::vector<int> internalNodes(route.begin() + 1, route.end() - 1);
    if (internalNodes.size() < 2) return;

    const int dummyNode = -1;
    std::vector<int> modelNodes;
    modelNodes.reserve(internalNodes.size() + 3);
    modelNodes.push_back(startNode);
    modelNodes.insert(modelNodes.end(), internalNodes.begin(), internalNodes.end());
    modelNodes.push_back(endNode);
    modelNodes.push_back(dummyNode);

    const int m = static_cast<int>(modelNodes.size());
    const int dummyIdx = m - 1;
    long long maxEdge = 1;
    for (int i = 0; i < m - 1; ++i) {
        for (int j = 0; j < m - 1; ++j) {
            maxEdge = std::max(maxEdge,
                static_cast<long long>(std::llround(graph.truck_time[modelNodes[i]][modelNodes[j]] * 10.0)));
        }
    }
    const long long bigM = std::max(10000000LL, maxEdge * static_cast<long long>(m + 1) * 100LL);

    const std::string temp_dir = "lkh_temp_fullpath";
    std::filesystem::create_directories(temp_dir);

    auto ticks = std::chrono::high_resolution_clock::now().time_since_epoch().count();
    std::ostringstream oss;
    oss << "full_path_" << ticks << "_" << std::this_thread::get_id();
    std::string base = oss.str();

    std::string tsp_path = temp_dir + "/" + base + ".tsp";
    std::string par_path = temp_dir + "/" + base + ".par";
    std::string tour_path = temp_dir + "/" + base + ".tour";

    auto cleanup_files = [&]() {
        std::error_code ec;
        std::filesystem::remove(tsp_path, ec);
        std::filesystem::remove(par_path, ec);
        std::filesystem::remove(tour_path, ec);
    };

    std::ofstream tsp(tsp_path);
    if (!tsp) {
        cleanup_files();
        return;
    }
    tsp << "NAME: TRUCK_FULL_PATH\n";
    tsp << "TYPE: TSP\n";
    tsp << "COMMENT: Fixed depot endpoints, station movable\n";
    tsp << "DIMENSION: " << m << "\n";
    tsp << "EDGE_WEIGHT_TYPE: EXPLICIT\n";
    tsp << "EDGE_WEIGHT_FORMAT: FULL_MATRIX\n";
    tsp << "EDGE_WEIGHT_SECTION\n";
    for (int i = 0; i < m; ++i) {
        for (int j = 0; j < m; ++j) {
            long long weight = 0;
            if (i == j) {
                weight = 0;
            } else if (i == dummyIdx || j == dummyIdx) {
                int other = (i == dummyIdx) ? j : i;
                weight = (other == 0 || other == m - 2) ? 0 : bigM;
            } else {
                weight = static_cast<long long>(std::llround(
                    graph.truck_time[modelNodes[i]][modelNodes[j]] * 10.0));
            }
            tsp << weight << " ";
        }
        tsp << "\n";
    }
    tsp << "EOF\n";
    tsp.close();

    std::ofstream par(par_path);
    if (!par) {
        cleanup_files();
        return;
    }
    par << "PROBLEM_FILE = " << tsp_path << "\n";
    par << "TOUR_FILE    = " << tour_path << "\n";
    par << "RUNS = " << std::max(1, params.lkh_runs) << "\n";
    par << "TRACE_LEVEL = 0\n";
    par << "MOVE_TYPE = 5\n";
    par << "PATCHING_C = 3\n";
    par << "PATCHING_A = 2\n";
    par << "SEED = " << makeLkhSeed(base) << "\n";
    par.close();

    {
        std::lock_guard<std::mutex> lock(g_lkh_mutex);
        char* args[] = { (char*)"LKH", (char*)par_path.c_str() };
        int argc = 2;
        int res = LKH_main(argc, args);
        if (res != 0) {
            cleanup_files();
            return;
        }
    }

    std::ifstream tour(tour_path);
    if (!tour) {
        cleanup_files();
        return;
    }
    std::vector<int> cycle;
    std::string line;
    bool start = false;
    while (std::getline(tour, line)) {
        if (line.find("TOUR_SECTION") != std::string::npos) {
            start = true;
            continue;
        }
        if (!start) continue;
        if (line == "-1") break;
        int idx = std::stoi(line);
        if (idx >= 1 && idx <= m) cycle.push_back(modelNodes[idx - 1]);
    }
    tour.close();
    cleanup_files();

    if (static_cast<int>(cycle.size()) != m) return;
    int dummyPos = -1;
    for (int i = 0; i < m; ++i) {
        if (cycle[i] == dummyNode) {
            dummyPos = i;
            break;
        }
    }
    if (dummyPos < 0) return;

    auto collectPath = [&](int step) {
        std::vector<int> path;
        int pos = (dummyPos + step + m) % m;
        while (cycle[pos] != dummyNode) {
            path.push_back(cycle[pos]);
            pos = (pos + step + m) % m;
        }
        return path;
    };

    std::vector<int> pathForward = collectPath(1);
    std::vector<int> pathBackward = collectPath(-1);
    std::vector<int> fixedPath;
    if (!pathForward.empty() && pathForward.front() == startNode && pathForward.back() == endNode) {
        fixedPath = std::move(pathForward);
    } else if (!pathBackward.empty() && pathBackward.front() == startNode && pathBackward.back() == endNode) {
        fixedPath = std::move(pathBackward);
    } else {
        return;
    }
    if (static_cast<int>(fixedPath.size()) != m - 1) return;

    s.truck_route = std::move(fixedPath);
    evaluateSolution(s, /*needCalDrone=*/true);
    if (s.makespan + 1e-6 < originalMakespan) {
        std::cout << "Full-path LKH improved makespan by " << (originalMakespan - s.makespan) << "\n";
    } else {
        s = backup;
    }
}

void TSPDSUtils::optimizeTruckRouteWithLKHIntern(TSPDSSolution& s)
{
    auto& route = s.truck_route;
    int n = static_cast<int>(route.size());
    if (n <= 4) return;
    if (s.makespan <= 0.0) evaluateSolution(s, /*needCalDrone=*/true);

    TSPDSSolution backup = s;
    double original_cost = s.truck_completion_time;

    int station = graph.drone_station;
    int stationPos = -1;
    for (int i = 0; i < n; ++i) {
        if (route[i] == station) {
            stationPos = i;
            break;
        }
    }
    if (stationPos <= 0 || stationPos >= n - 1) {
        std::cerr << "Station not in a valid middle position, skip LKH intern optimize.\n";
        return;
    }

    auto runSegmentLKH = [this](int startNode, const std::vector<int>& internalNodes,
        int endNode, const std::string& tag) -> std::vector<int>
        {
            if (internalNodes.size() < 2) {
                return internalNodes;
            }

            const int dummyNode = -1;
            std::vector<int> modelNodes;
            modelNodes.reserve(internalNodes.size() + 3);
            modelNodes.push_back(startNode);
            modelNodes.insert(modelNodes.end(), internalNodes.begin(), internalNodes.end());
            modelNodes.push_back(endNode);
            modelNodes.push_back(dummyNode);

            int m = static_cast<int>(modelNodes.size());
            const int dummyIdx = m - 1;
            long long maxEdge = 1;
            for (int i = 0; i < m - 1; ++i) {
                for (int j = 0; j < m - 1; ++j) {
                    maxEdge = std::max(maxEdge,
                        static_cast<long long>(std::llround(graph.truck_time[modelNodes[i]][modelNodes[j]] * 10.0)));
                }
            }
            const long long bigM = std::max(10000000LL, maxEdge * static_cast<long long>(m + 1) * 100LL);

            const std::string temp_dir = "lkh_temp_intern";
            std::filesystem::create_directories(temp_dir);

            auto ticks = std::chrono::high_resolution_clock::now().time_since_epoch().count();
            std::ostringstream oss;
            oss << "seg_path_" << tag << "_" << ticks << "_" << std::this_thread::get_id();
            std::string base = oss.str();

            std::string tsp_path = temp_dir + "/" + base + ".tsp";
            std::string par_path = temp_dir + "/" + base + ".par";
            std::string tour_path = temp_dir + "/" + base + ".tour";

            auto cleanup_files = [&]() {
                std::error_code ec;
                std::filesystem::remove(tsp_path, ec);
                std::filesystem::remove(par_path, ec);
                std::filesystem::remove(tour_path, ec);
                };

            std::ofstream tsp(tsp_path);
            if (!tsp) {
                std::cerr << "Cannot write LKH TSP file: " << tsp_path << "\n";
                cleanup_files();
                return internalNodes;
            }

            tsp << "NAME: TRUCK_PATH_SEGMENT\n";
            tsp << "TYPE: TSP\n";
            tsp << "COMMENT: Fixed-endpoint truck path optimization\n";
            tsp << "DIMENSION: " << m << "\n";
            tsp << "EDGE_WEIGHT_TYPE: EXPLICIT\n";
            tsp << "EDGE_WEIGHT_FORMAT: FULL_MATRIX\n";
            tsp << "EDGE_WEIGHT_SECTION\n";

            for (int i = 0; i < m; ++i) {
                for (int j = 0; j < m; ++j) {
                    long long weight = 0;
                    if (i == j) {
                        weight = 0;
                    }
                    else if (i == dummyIdx || j == dummyIdx) {
                        int other = (i == dummyIdx) ? j : i;
                        weight = (other == 0 || other == m - 2) ? 0 : bigM;
                    }
                    else {
                        weight = static_cast<long long>(std::llround(
                            graph.truck_time[modelNodes[i]][modelNodes[j]] * 10.0));
                    }
                    tsp << weight << " ";
                }
                tsp << "\n";
            }
            tsp << "EOF\n";
            tsp.close();

            std::ofstream par(par_path);
            if (!par) {
                std::cerr << "Cannot write LKH PAR file: " << par_path << "\n";
                cleanup_files();
                return internalNodes;
            }

            par << "PROBLEM_FILE = " << tsp_path << "\n";
            par << "TOUR_FILE    = " << tour_path << "\n";
            par << "RUNS = " << std::max(1, params.lkh_runs) << "\n";
            par << "TRACE_LEVEL = 0\n";
            par << "MOVE_TYPE = 5\n";
            par << "PATCHING_C = 3\n";
            par << "PATCHING_A = 2\n";
            par << "SEED = " << makeLkhSeed(base) << "\n";
            par.close();

            {
                std::lock_guard<std::mutex> lock(g_lkh_mutex);

                char* args[] = { (char*)"LKH", (char*)par_path.c_str() };
                int argc = 2;
                int res = LKH_main(argc, args);
                if (res != 0) {
                    std::cerr << "LKH failed on segment " << tag << "\n";
                    cleanup_files();
                    return internalNodes;
                }
            }

            std::ifstream tour(tour_path);
            if (!tour) {
                std::cerr << "Cannot read tour file: " << tour_path << "\n";
                cleanup_files();
                return internalNodes;
            }

            std::vector<int> cycle;
            std::string line;
            bool start = false;

            while (std::getline(tour, line)) {
                if (line.find("TOUR_SECTION") != std::string::npos) {
                    start = true;
                    continue;
                }
                if (!start) continue;
                if (line == "-1") break;

                int idx = std::stoi(line);
                if (idx >= 1 && idx <= m) {
                    cycle.push_back(modelNodes[idx - 1]);
                }
            }
            tour.close();
            cleanup_files();

            if (static_cast<int>(cycle.size()) != m) {
                std::cerr << "Invalid LKH tour size for segment " << tag << "\n";
                return internalNodes;
            }

            int dummyPos = -1;
            for (int i = 0; i < m; ++i) {
                if (cycle[i] == dummyNode) {
                    dummyPos = i;
                    break;
                }
            }
            if (dummyPos < 0) {
                std::cerr << "LKH path segment missing dummy node for " << tag << "\n";
                return internalNodes;
            }

            auto collectPath = [&](int step) {
                std::vector<int> path;
                int pos = (dummyPos + step + m) % m;
                while (cycle[pos] != dummyNode) {
                    path.push_back(cycle[pos]);
                    pos = (pos + step + m) % m;
                }
                return path;
            };

            std::vector<int> pathForward = collectPath(1);
            std::vector<int> pathBackward = collectPath(-1);
            std::vector<int> fixedPath;
            if (!pathForward.empty() && pathForward.front() == startNode && pathForward.back() == endNode) {
                fixedPath = std::move(pathForward);
            }
            else if (!pathBackward.empty() && pathBackward.front() == startNode && pathBackward.back() == endNode) {
                fixedPath = std::move(pathBackward);
            }
            else {
                std::cerr << "LKH path segment endpoints not preserved for " << tag << "\n";
                return internalNodes;
            }

            if (static_cast<int>(fixedPath.size()) != m - 1) {
                std::cerr << "Invalid LKH path size for segment " << tag << "\n";
                return internalNodes;
            }

            return std::vector<int>(fixedPath.begin() + 1, fixedPath.end() - 1);
        };

    // --------- 拆分成两段（只取内部节点） ----------
    std::vector<int> preInternal(route.begin() + 1, route.begin() + stationPos);
    std::vector<int> postInternal(route.begin() + stationPos + 1, route.end() - 1);

    std::vector<int> preOpt = runSegmentLKH(route.front(), preInternal, station, "pre");
    std::vector<int> postOpt = runSegmentLKH(station, postInternal, route.back(), "post");

    // --------- 重新拼装整条 truck_route ----------
    std::vector<int> newRoute;
    newRoute.reserve(n);

    newRoute.push_back(route.front());                 // depot
    newRoute.insert(newRoute.end(), preOpt.begin(), preOpt.end());
    newRoute.push_back(station);                       // station
    newRoute.insert(newRoute.end(), postOpt.begin(), postOpt.end());
    newRoute.push_back(route.back());                  // depot

    s.truck_route = std::move(newRoute);

    // --------- 重新评估 ----------
    evaluateSolution(s, /*needCalDrone=*/true);
    double new_cost = s.truck_completion_time;

    if (new_cost < original_cost - 1e-6) {
        std::cout << "LKH segment optimization improved truck path by "
            << (original_cost - new_cost) << "\n";
    }
    else {
        s = backup;
    }
}


double TSPDSUtils::estimateBestInsertionIncrease(const TSPDSSolution& solution, int node,
    InsertPolicy policy, int& bestPos) const
{
    bestPos = -1;
    const auto& r = solution.truck_route;
    int n = (int)r.size();
    if (n < 2) return std::numeric_limits<double>::infinity();

    // 找 station 位置（若不存在则退化为 ANYWHERE）
    int stationPos = -1;
    for (int i = 0; i < n; ++i) {
        if (r[i] == graph.drone_station) { stationPos = i; break; }
    }

    int edgeStart = 0;
    int edgeEnd = n - 2; // 遍历边 (i -> i+1)

    if (policy == InsertPolicy::AFTER_STATION && stationPos != -1) {
        // 允许插在 station 后（包括 station 与其后继之间）
        edgeStart = stationPos;
    }

    if (edgeStart > edgeEnd) return std::numeric_limits<double>::infinity();

    double bestInc = std::numeric_limits<double>::infinity();
    for (int i = edgeStart; i <= edgeEnd; ++i) {
        int from = r[i];
        int to = r[i + 1];

        double original = graph.truck_time[from][to];
        double replaced = graph.truck_time[from][node] + graph.truck_time[node][to];
        double inc = replaced - original;

        if (inc < bestInc) {
            bestInc = inc;
            bestPos = i + 1; // 插入位置
        }
    }

    return bestInc;
}


bool TSPDSUtils::insertNodeToTruckRoute(TSPDSSolution& solution, int node, InsertPolicy policy)
{
    auto& r = solution.truck_route;
    if (r.size() < 2) return false;

    int bestPos = -1;
    double bestInc = estimateBestInsertionIncrease(solution, node, policy, bestPos);

    if (bestPos < 0 || !std::isfinite(bestInc)) return false;

    // 直接插入（注意：这里不再写“末尾 end-case”，因为边遍历已覆盖最后客户->depot）
    r.insert(r.begin() + bestPos, node);
    return true;
}

// 无人机从 station 出发到 node 再回 station 的时间
double TSPDSUtils::getDroneRoundTripTime(int node) const
{
    return 2.0 * graph.drone_time[graph.drone_station][node];

}



