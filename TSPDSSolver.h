#pragma once
#include "InitialSolutionGenerator.h"
#include "DroneScheduler.h"
#include "PerturbationOperators.h"
#include "LocalSearchOperators.h"
#include "TSPDSUtils.h"
#include <iostream>
#include "VNSLogger.h"

class TSPDSSolver {
public:
    TSPDSGraph graph;
    TSPDSAlgorithmParams params;
    VNSLogger logger;  // 添加日志记录器成员



    void setRandomSeed(unsigned int seed);   // 新增

    unsigned int random_seed = 0;            // 新增

    // 功能模块实例
    InitialSolutionGenerator initGenerator;

    PerturbationOperators perturbOps;

    LocalSearchOperators localSearchOps;

    TSPDSUtils utils;

    // 日志相关的方法（委托给logger）
    void enableLogging(bool enabled) {
        logger.setLoggingEnabled(enabled);
    }
    void setLogFilename(const std::string& filename) {
        logger.setLogFilename(filename);
    }

    void verifyToyInstance();
    //LS
    std::vector<int> getNeighborhoodOrderByDiagnosis(const std::string& diagnosis);

    // 应用2-opt优化卡车路径
    // 2-opt优化卡车路径
    TSPDSSolution applyTwoOpt(TSPDSSolution& solution);

    // 重新分配节点（卡车和无人机之间）
    TSPDSSolution reassignNodes(TSPDSSolution& solution);


    TSPDSSolution optimizeTruckBottleneck(TSPDSSolution& solution);

    TSPDSSolution optimizeDroneBottleneck(TSPDSSolution& solution);

    TSPDSSolution optimizeBalance(TSPDSSolution& solution);

    TSPDSSolution adjustDroneStationPosition(TSPDSSolution& solution);

    TSPDSSolution tryMoveDroneStation(TSPDSSolution& solution, int oldPos, int newPos);

    bool reassignTruckToDrone(TSPDSSolution& solution, int node);

    TSPDSSolution swapTruckDroneNodes(TSPDSSolution& solution);


    //LS辅助函数
    int findLeastLoadedDrone(const TSPDSSolution& solution);

    TSPDSSolution rebalanceDroneTasks(TSPDSSolution& solution);

    double calculateTimeSaveIfReassign(const TSPDSSolution& solution, int node);

    TSPDSSolution transferDroneToTruck(TSPDSSolution& solution, const std::vector<int>& droneNodes);

    double calculateTransferPotential(const TSPDSSolution& solution, int node);

    double estimateTruckTimeIncrease(const TSPDSSolution& solution, int node);

    bool transferSingleNode(TSPDSSolution& solution, int node);

    bool insertNodeToTruckRoute(TSPDSSolution& solution, int node);

    void sortNodesBySwapPotential(vector<int>& truckNodes, vector<int>& droneNodes,
        const TSPDSSolution& solution);
    double calculateSwapPotential(int node, bool isTruckNode, const TSPDSSolution& solution);
    bool performNodeSwap(TSPDSSolution& solution, int truckNode, int droneNode);

    double calculateActualDroneTimeReduction(const TSPDSSolution& solution, int node, int droneId);

    double calculateMinDistanceToTruckRoute(const TSPDSSolution& solution, int node);

    //combine
    void scheduleDrones(TSPDSSolution& solution, const TSPDSGraph& graph);

    std::pair<std::vector<std::vector<int>>, double> combineAlgorithm(const std::vector<double>& processingTimes, int m);

    std::pair<std::vector<std::vector<int>>, double>
        lptScheduling(const std::vector<double>& processingTimes, int m);

    int ffdBinPacking(const std::vector<double>& items, double capacity, std::vector<std::vector<int>>& assignment);

    // VNS扰动函数
    TSPDSSolution perturbSolution(TSPDSSolution& solution, int k);

    // 随机交换卡车路径中的两个节点
    void randomSwap(TSPDSSolution& solution);

    // 随机反转卡车路径的一段
    void randomReverse(TSPDSSolution& solution);

    // 随机重新分配无人机节点
    void randomReassign(TSPDSSolution& solution);

    // 评估解，更新makespan等相关时间
    void evaluateSolution(TSPDSSolution& solution, bool needCalDrone = true);

    // 计算解的makespan
    double calculateMakespan(TSPDSSolution& solution);

    void randomMoveDroneStation(TSPDSSolution& solution);

    void batchNodeReassignment(TSPDSSolution& solution);

    void insertNodeToRandomPosition(TSPDSSolution& solution, int node);

    void segmentReshuffling(TSPDSSolution& solution);



    TSPDSSolution generateInitialSolutionPro();

    TSPDSSolution printSolution(TSPDSSolution& currentSolution1);
public:
    TSPDSSolver(const TSPDSGraph& graph);

    TSPDSSolver(const TSPDSGraph& graph, int runId);  // 新增

    int run_id = -1;   // 新增：记录这个 solver 对应第几次实验

    // 生成初始解
    TSPDSSolution generateInitialSolution();

    // 局部搜索改进解
    TSPDSSolution localSearch(TSPDSSolution& currentSolution);

    // 主求解方法
    TSPDSSolution solve(double F2Best);

    enum class CrossoverPartType {
        PreStation,
        PostStation,
        DroneSchedule
    };

    enum class TruckCompletionSide {
        Anywhere,
        BeforeStation,
        AfterStation
    };

    struct CrossoverPartSpec {
        const TSPDSSolution* solution = nullptr;
        CrossoverPartType type = CrossoverPartType::PreStation;
    };

    std::vector<TSPDSSolution> generateInitialPopulation();
    std::pair<int, int> selectParents(const std::vector<TSPDSSolution>& population, std::mt19937& rng);
    std::vector<TSPDSSolution> crossover(const TSPDSSolution& a, const TSPDSSolution& b);
    std::vector<TSPDSSolution> targetGuidedAssignmentChildren(const TSPDSSolution& seed,
        double incumbentBest,
        double knownBest);
    TSPDSSolution makeCrossoverChild(const CrossoverPartSpec& first,
        const CrossoverPartSpec& second,
        bool preferFirst);
    void insertMissingTruckNode(TSPDSSolution& solution, int node, TruckCompletionSide side);
    bool repairAndEvaluateCandidate(TSPDSSolution& solution);
    void updatePopulation(std::vector<TSPDSSolution>& population,
        const std::vector<TSPDSSolution>& candidates,
        const TSPDSSolution& bestSolution);
    void insertPostLocalSearchCandidate(std::vector<TSPDSSolution>& population,
        const TSPDSSolution& candidate,
        const TSPDSSolution& bestSolution,
        double preLocalSearchObjective);
    double solutionHammingDistance(const TSPDSSolution& a, const TSPDSSolution& b) const;
    std::string solutionSignature(const TSPDSSolution& solution) const;


    //tools

    double calculateTwoOptDelta(const vector<int>& route, int i, int j);

    bool isSolutionValid(const TSPDSSolution& solution, const TSPDSGraph& graph) {
        bool valid = true;

        // 1. 检查卡车路径的基本约束
        if (!validateTruckRoute(solution, graph)) {
            std::cerr << "卡车路径验证失败!" << std::endl;
            valid = false;
        }

        // 2. 检查节点服务分配约束
        if (!validateNodeAssignment(solution, graph)) {
            std::cerr << "节点分配验证失败!" << std::endl;
            valid = false;
        }

        // 3. 检查无人机任务约束
        if (!validateDroneAssignments(solution, graph)) {
            std::cerr << "无人机分配验证失败!" << std::endl;
            valid = false;
        }

        // 4. 检查时间约束
        if (!validateTimeConstraints(solution, graph)) {
            std::cerr << "时间约束验证失败!" << std::endl;
            valid = false;
        }

        // 5. 检查无人机站激活约束
        if (!validateDroneStationActivation(solution, graph)) {
            std::cerr << "无人机站激活验证失败!" << std::endl;
            valid = false;
        }

        // 6. 检查解的完整性
        if (!validateSolutionCompleteness(solution, graph)) {
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

    bool validateTruckRoute(const TSPDSSolution& solution, const TSPDSGraph& graph) {

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

    bool validateNodeAssignment(const TSPDSSolution& solution, const TSPDSGraph& graph) {

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
        // 3. 检查无人机可服务客户点的分配合理性
        for (int i = 0; i < graph.nodes.size(); ++i) {
            if (graph.is_drone_eligible[i] && solution.served_by_drone[i]) {
                // 检查无人机服务节点是否在无人机飞行范围内
                double droneTime = graph.drone_time[graph.drone_station][i];
                if (droneTime < 0 || droneTime > std::numeric_limits<double>::max() / 2) {
                    std::cerr << "错误: 节点 " << i << " 超出无人机飞行范围" << std::endl;
                    return false;
                }
            }
        }

        return true;
    }

    bool validateDroneAssignments(const TSPDSSolution& solution, const TSPDSGraph& graph) {

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

    bool validateTimeConstraints(const TSPDSSolution& solution, const TSPDSGraph& graph) {

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

    bool validateDroneStationActivation(const TSPDSSolution& solution, const TSPDSGraph& graph) {

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

    bool validateSolutionCompleteness(const TSPDSSolution& solution, const TSPDSGraph& graph) {


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
};
