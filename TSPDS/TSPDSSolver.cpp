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

TSPDSSolution TSPDSSolver::solve(double F2Best) {
    std::mt19937 rng(random_seed ^ (0x9e3779b9u + static_cast<unsigned>(run_id)));
    std::uniform_real_distribution<double> uniDist(0.0, 1.0);

    auto lexBetter = [](const TSPDSSolution& a, const TSPDSSolution& b) -> bool {
        if (a.makespan < b.makespan - 1e-6) return true;
        return false;
        };

    using Clock = std::chrono::steady_clock;
    const auto start_time = Clock::now();

    auto elapsedSecs = [&]() -> double {
        return std::chrono::duration_cast<std::chrono::duration<double>>(Clock::now() - start_time).count();
        };

    auto elapsedMs = [&]() -> long long {
        return std::chrono::duration_cast<std::chrono::milliseconds>(Clock::now() - start_time).count();
        };

    // ---------- 初始解 ----------
    TSPDSSolution currentSolution;
    TSPDSSolution initSolution1;
    currentSolution = initGenerator.generateInitialSolution();
    TSPDSSolution bestSolution = currentSolution;

    // ---------- VNS ----------
    int k = 1;
    int iteration = 0;
    int noImprove = 0;

    const std::size_t tid = std::hash<std::thread::id>{}(std::this_thread::get_id());

    // 基于最大运行时间的终止条件
    while (elapsedSecs() < params.max_run_time) {
        const int k_used = k; // 本轮实际使用的扰动强度

        if (params.verbose) {
            std::cout << "\n[Run " << run_id << ", Thread " << tid << "] "
                << "-- VNS Iteration " << iteration
                << " , 扰动 k = " << k_used
                << " , elapsed = " << elapsedSecs() << "s / " << params.max_run_time << "s --\n";
            std::cout << "扰动前 makespan: " << currentSolution.makespan << "\n";
        }

        // 1) 扰动
        TSPDSSolution perturbedSolution = perturbOps.perturbSolution3(currentSolution, k_used);
        utils.evaluateSolution(perturbedSolution, /*needCalDrone=*/true);
        std::cout << "扰动之后 makespan: " << perturbedSolution.makespan << "\n";

        // 2) 局部搜索（VND）
        TSPDSSolution localSolution = localSearchOps.localSearch(perturbedSolution, true);
        utils.evaluateSolution(localSolution, /*needCalDrone=*/true);

        constexpr double EPS = 1e-9;
        bool accepted = false;

        const bool betterThanCurrent = lexBetter(localSolution, currentSolution);
        if (betterThanCurrent) {
            accepted = true;
        }

        // 4) 更新 current / best / k / noImprove
        if (accepted) {
            currentSolution = localSolution;

            if (currentSolution.makespan < bestSolution.makespan) {
                // 尝试进一步优化卡车路径和无人机调度
                utils.optimizeTruckRouteWithLKHIntern(currentSolution);
                TSPDSSolution temp = currentSolution;
                //utils.evaluateSolutionExactDroneByCplex(temp);
                if (temp.drone_completion_time < currentSolution.drone_completion_time) {
                    currentSolution = temp;
                }
                bestSolution = currentSolution;
                bestSolution.max_resAt = iteration;
                bestSolution.find_best_time = static_cast<double>(elapsedMs());

                if (params.verbose) {
                    std::cout << "★★★ 新全局最优解: " << bestSolution.makespan << "\n";
                    utils.printSolution(bestSolution);
                }
                k = 1;
                noImprove = 0;

                // 若已命中已知最优则提前结束
                if (bestSolution.makespan == F2Best) {
                    std::cout << "找到最优解，提前结束" << std::endl;
                    break;
                }
            }
        }
        else {
            noImprove++;
            k++;
            if (k > params.shaking_number) k = 1;
        }

        // 5) 多次未改进 -> restart
        if (noImprove >= params.restart_no_improve_iters) {
            if (params.verbose) {
                std::cout << "多次未改进，重新从当前最优解出发!" << "\n";
            }
            currentSolution = bestSolution;
            noImprove = 0;
        }

        iteration++;

        if (params.verbose) {
            std::cout << "Iter " << iteration << " done. best=" << bestSolution.makespan
                << " at iter " << bestSolution.max_resAt << "\n";
        }
    }

    if (params.verbose) {
        std::cout << "★★★ 全局最优解: " << bestSolution.makespan << "\n";
        utils.printSolution(bestSolution);
    }

    bestSolution.total_iter = iteration;
    // 最终对最优解做一次强化优化（时间允许范围内）
    utils.optimizeTruckRouteWithLKHIntern(bestSolution);
    //utils.evaluateSolutionExactDroneByCplex(bestSolution);
    return bestSolution;
}



