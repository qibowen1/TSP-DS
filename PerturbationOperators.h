#pragma once
#include "TSPDSUtils.h"

class PerturbationOperators {
private:
    const TSPDSGraph& graph;
    const TSPDSAlgorithmParams& params;
    std::mt19937 gen;
    TSPDSUtils utils;
public:
    PerturbationOperators(const TSPDSGraph& graph, const TSPDSAlgorithmParams& params);

    // 扰动主函数
    TSPDSSolution perturbSolution(TSPDSSolution& solution, int k);

    TSPDSSolution perturbSolution2(TSPDSSolution& solution, int k);

    TSPDSSolution perturbSolution3(TSPDSSolution& solution, int k);

    // 随机交换
    void randomSwap(TSPDSSolution& solution);

    // 随机移动无人机站
    void randomMoveDroneStation(TSPDSSolution& solution);

    void moveDroneStationLocalK(TSPDSSolution& solution, int k);

    // 批量节点重分配
    void batchNodeReassignment(TSPDSSolution& solution, int intensity);

    void batchNodeReassignment2(TSPDSSolution& solution, int intensity);

    // 路径段重排
    void segmentReshuffling(TSPDSSolution& solution);

    void doubleBridge(TSPDSSolution& s);

    void moveDroneStationBySegment(TSPDSSolution& solution, int k);


private:
    // 辅助函数
    void insertNodeToRandomPosition(TSPDSSolution& solution, int node);

    void insertNodeToBestPosition(TSPDSSolution& solution, int node);
};