#pragma once
#include "TSPDSGraph.h"
#include "TSPDSSolution.h"
#include "TSPDS_Params.h"
#include <vector>
#include <utility>

class DroneScheduler {
public:
    // 无人机任务调度主函数
    static void scheduleDrones(TSPDSSolution& solution, const TSPDSGraph& graph);
    static void polishDroneAssignment(TSPDSSolution& solution, const TSPDSGraph& graph);


    // COMBINE算法实现
    static std::pair<std::vector<std::vector<int>>, double>
        combineAlgorithm(const std::vector<double>& processingTimes, int m);

    // LPT调度算法
    static std::pair<std::vector<std::vector<int>>, double>
        lptScheduling(const std::vector<double>& processingTimes, int m);

    // FFD装箱算法
    static int ffdBinPacking(const std::vector<double>& items, double capacity,
        std::vector<std::vector<int>>& assignment);

    // 辅助函数
    static int findLeastLoadedDrone(const std::vector<double>& loads);
    static double calculateMakespan(const std::vector<std::vector<int>>& assignment,
        const std::vector<double>& processingTimes);
};