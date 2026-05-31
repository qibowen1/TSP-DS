#pragma once
#pragma once
#include "TSPDSGraph.h"
#include "TSPDSSolution.h"
#include "TSPDS_Params.h"
#include "DroneScheduler.h"
#include <vector>
#include <algorithm>
#include <numeric>
#include <random>
#include <iostream>

enum class InsertPolicy {
    ANYWHERE,
    AFTER_STATION
};

//truck->drone
struct DroneLoadCache {
    double minLoad = 0.0;
    double Cmax = 0.0;
};

//drone->truck
struct DroneLoadCacheDT {
    std::vector<double> loads;    // size = drone_count
    std::vector<int> node2drone;  // size = n, -1 if not on drone
    double max1 = 0.0;            // 最大负载
    double max2 = 0.0;            // 严格第二大（若不存在则=max1）
    int argmax = -1;              // 最大负载无人机 id（任取一个）
    int countMax = 0;             // 达到 max1 的无人机数量（考虑 eps）
};


class TSPDSUtils {
private:
    static std::random_device rd;
    static std::mt19937 gen;
    const TSPDSGraph& graph;
    const TSPDSAlgorithmParams& params;

	DroneScheduler droneScheduler;
public:
    TSPDSUtils(const TSPDSGraph& graph, const TSPDSAlgorithmParams& params);
    // 解评估函数
    void evaluateSolution(TSPDSSolution& solution,  bool needCalDrone = true);


    // 解验证函数
    bool isSolutionValid(const TSPDSSolution& solution);

	double calculateMakespan(TSPDSSolution& solution);

    //各种算子操作的评估
    double calculateTimeSaveIfReassign(const TSPDSSolution& solution, int node);

    void buildPosInTruck(TSPDSSolution& sol);

    void buildRemoveSaving(const TSPDSSolution& sol,
        std::vector<double>& removeSaving);

    DroneLoadCache buildDroneLoadCache(const TSPDSSolution& sol);

    double estimatePotentialTruckToDrone_O1(
        const TSPDSSolution& sol,
        int node,
        const std::vector<double>& removeSaving,
        const DroneLoadCache& cache);


    // 新增：无人机负载 cache
    DroneLoadCacheDT buildDroneLoadCacheDT(const TSPDSSolution& sol) const;

    // 新增：对 candNodes 批量预计算 best insertion（policy 支持 ANYWHERE/AFTER_STATION）
    void buildBestInsertionCacheForNodes(
        const TSPDSSolution& sol,
        const std::vector<int>& candNodes,
        InsertPolicy policy,
        std::vector<double>& bestInc,   // size=n
        std::vector<int>& bestPos       // size=n (插入位置)
    ) const;

    // 新增：O(1) potential（需要 insInc+insPos）
    double estimatePotentialDroneToTruck_O1(
        const TSPDSSolution& sol,
        int node,
        double insInc,
        int insPos,
        const DroneLoadCacheDT& cache) const;

    double estimateTruckEdgeSavingIfRemove(const TSPDSSolution& solution, int node);

    double calculateTransferPotential(const TSPDSSolution& solution, int node);

    double estimateTruckTimeIncrease(const TSPDSSolution& solution, int node);

    bool reassignTruckToDrone(TSPDSSolution& solution, int node);

	bool insertNodeToTruckRoute(TSPDSSolution& solution, int node);

    double calculateTwoOptDelta(const vector<int>& route, int i, int j);

    TSPDSSolution tryMoveDroneStation(TSPDSSolution& solution, int oldPos, int newPos);

	double calculateActualDroneTimeReduction(const TSPDSSolution& solution, int node, int droneId);

	TSPDSSolution printSolution(TSPDSSolution& currentSolution1);

    int findLeastLoadedDrone(const TSPDSSolution& solution);

    double estimateTruckRemovalSaving(const TSPDSSolution& S, int node);

    void optimizeTruckRouteWithLKH(TSPDSSolution& s);

    void optimizeTruckRouteWithLKHIntern(TSPDSSolution& s);

    bool insertNodeToTruckRoute(TSPDSSolution& solution, int node,
        InsertPolicy policy = InsertPolicy::ANYWHERE);

    double estimateBestInsertionIncrease(const TSPDSSolution& solution, int node,
        InsertPolicy policy, int& bestPos) const;

    double getDroneRoundTripTime(int node) const; // 需要你按实际数据实现

private:
    // 其他工具函数...
	bool validateTruckRoute(const TSPDSSolution& solution);

	bool validateNodeAssignment(const TSPDSSolution& solution);

	bool validateDroneAssignments(const TSPDSSolution& solution);

	bool validateTimeConstraints(const TSPDSSolution& solution);

    bool validateDroneStationActivation(const TSPDSSolution& solution);

    bool validateSolutionCompleteness(const TSPDSSolution& solution);

    

	
	
};