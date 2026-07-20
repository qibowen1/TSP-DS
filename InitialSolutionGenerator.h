#pragma once
#include "TSPDSUtils.h"
#include <random>   // 新增
class InitialSolutionGenerator {
private:
    const TSPDSGraph& graph;
    const TSPDSAlgorithmParams& params;

	TSPDSUtils utils;

public:

    TSPDSSolution generateInitialSolution_DroneMain();
    TSPDSSolution generateInitialSolutionBackboneInsert();
    TSPDSSolution generateInitialSolutionTargetCapacity();
    InitialSolutionGenerator(const TSPDSGraph& graph, const TSPDSAlgorithmParams& params);
    void setRandomSeed(unsigned int seed) { rng.seed(seed); }   // 新增
    // 生成初始解
    TSPDSSolution generateInitialSolution();

    TSPDSSolution generateInitialSolution_paper();

    // 改进的初始解生成（无人机优先策略）
    TSPDSSolution generateInitialSolutionPro();

    TSPDSSolution generateInitialSolutionSmart();

private:
    std::mt19937 rng;   // 新增：随机数引擎
};