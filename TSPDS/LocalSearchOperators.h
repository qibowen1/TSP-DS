#pragma once
#include "TSPDSUtils.h"
#include <array>

class LocalSearchOperators {
private:
    const TSPDSGraph& graph;
    const TSPDSAlgorithmParams& params;

    TSPDSUtils utils;

    std::mt19937 gen;
    std::uniform_real_distribution<double> uni;

    struct Params {
        std::array<bool, 32> op_enabled;
        Params() {
            op_enabled.fill(true);
        }
    };


public:
    LocalSearchOperators(const TSPDSGraph& graph, const TSPDSAlgorithmParams& params);

    // 局部搜索主函数
    TSPDSSolution localSearch(TSPDSSolution& currentSolution);

    TSPDSSolution localSearch(const TSPDSSolution& start, bool afterShake);

    // 2-opt优化
    TSPDSSolution applyTwoOpt(TSPDSSolution& solution);

    // Truck route same-side node swap (TSwap)
    TSPDSSolution applyTruckRouteSwap(TSPDSSolution& solution);

    TSPDSSolution destroyRepairLNS(TSPDSSolution& solution);

    double bottleneckKeyBand(
        const TSPDSSolution& base,
        double estTt, double estTa, double estC);
    // 卡车瓶颈优化
    TSPDSSolution optimizeTruckBottleneck(TSPDSSolution& solution);

    // 无人机瓶颈优化
    TSPDSSolution optimizeDroneBottleneck(TSPDSSolution& solution);

    bool moveDroneToTruckAtPos(TSPDSSolution& sol, int node, int insPos);

    TSPDSSolution longEdgeDrivenLS(TSPDSSolution& solution,
        int topEdges,
        int win,
        bool firstImprove);
    // 无人机站位置调整
    TSPDSSolution adjustDroneStationPosition(TSPDSSolution& solution);

    TSPDSSolution applyLKHTruckRefinement(TSPDSSolution& solution);

    TSPDSSolution assignFarthestTruckToDrone(TSPDSSolution& solution,
        int maxAcceptedMoves,
        bool firstImprove);

    TSPDSSolution assignNearestDroneToTruck(TSPDSSolution& solution);

    TSPDSSolution batchCompensatedReassign(TSPDSSolution& solution,
        int numDroneToTruck,
        int numTruckToDrone,
        bool firstImprove);

    // 节点交换操作
    TSPDSSolution swapTruckDroneNodes(TSPDSSolution& solution);

    TSPDSSolution moveTruckNodeAcrossStation(TSPDSSolution& solution, bool firstImprove);

    TSPDSSolution applyTwoOptStar(const TSPDSSolution& solution);

    TSPDSSolution applyOrOpt(TSPDSSolution& solution, int segmentLen);

    TSPDSSolution optimizeTruckBlockToDrone(TSPDSSolution& solution, int blockSize);

    TSPDSSolution balanceDroneLoad(TSPDSSolution& solution);

    bool moveNodeBetweenDrones(TSPDSSolution& sol, int node, int from, int to);
    int getBusiestDrone( TSPDSSolution& S, const std::vector<double>& loads);
    int getIdlestDrone( TSPDSSolution& S, const std::vector<double>& loads);
    std::vector<int> getTopKNodesOfDrone( TSPDSSolution& S, int droneId, int K);
    double jobDurationForDrone(int node) ;
    void recomputeDroneLoads( TSPDSSolution& S, std::vector<double>& loads) ;

    TSPDSSolution swapBlocksAroundStation(TSPDSSolution& solution);

    TSPDSSolution nearStationLNSReassign(TSPDSSolution& solution);

    // (4) VND 卡住补覆盖：failRounds==1 做一次 boost pass（你在 localSearch 里用）
    TSPDSSolution localSearch_WithFailBoost(const TSPDSSolution& start, bool afterShake);

    struct StationAcceptCfg {
        // 允许 makespan 小幅变差的比例（铺路）
        double epsMkRatio = 0.01;          // 1%
        // 允许卡车完工时间绕路的比例（限制“直冲站点”）
        double epsDetourRatio = 0.01;       // 1% of Tt0
        // gap 至少改善比例
        double epsGapRatio = 0.10;          // 10%
        // 无人机完工至少提前阈值（按 Mk0 比例）
        double minAdvanceTdRatio = 0.005;   // 0.5%
        // drone bottleneck 时强倾向 Td 提前阈值
        double tdEpsRatio = 0.002;          // 0.2%
        // truck bottleneck 时允许 Td 略微超过 Tt 的 margin（按 Mk0 比例）
        double truckMarginRatio = 0.002;    // 0.2%

        // 结构窗口：station 左右最少节点数（避免太极端）
        int minSideLen = 1;

        // probe 配置
        bool enableProbe = true;
        int probeCandidates = 10;            // 只对前 probeCandidates 个“通过铺路接受”的候选做 probe
        int probeSteps = 10;                // 每个候选最多 probeSteps 步
    };

    bool acceptStationMove(TSPDSSolution& base,
        TSPDSSolution& cand,
        StationAcceptCfg& cfg);

    TSPDSSolution adjustDroneStationPosition2(TSPDSSolution& solution);


private:


    // 辅助函数
    std::vector<int> getNeighborhoodOrderByDiagnosis(const std::string& diagnosis);

    TSPDSSolution transferDroneToTruck(TSPDSSolution& solution, const std::vector<int>& droneNodes);

	bool transferSingleNode(TSPDSSolution& solution, int node);

    bool transferSingleNode(TSPDSSolution& solution, int node, InsertPolicy policy);

    void sortNodesBySwapPotential(vector<int>& truckNodes, vector<int>& droneNodes,
		const TSPDSSolution& solution);

	double calculateSwapPotential(int node, bool isTruckNode, const TSPDSSolution& solution);

	bool performNodeSwap(TSPDSSolution& solution, int truckNode, int droneNode);

	double estimateTruckTimeIncrease(const TSPDSSolution& solution, int node);

    void runSegmentTwoOpt(std::vector<int>& segment,
        bool protectFirst,
        bool protectLast);

    bool performSwap(TSPDSSolution& sol, int truckNode, int droneNode);

    double estimateQuickSwapBound(const TSPDSSolution& S, int t, int d);

    double estimateDroneToTruckBenefit(const TSPDSSolution& S, int node);

    double estimateTruckToDroneBenefit(const TSPDSSolution& S, int node);

    bool acceptMove(const TSPDSSolution& curr, const TSPDSSolution& cand);
    int  pickFromTopK_EpsGreedy(int topK); // 返回 [0, topK-1] 的下标

    // (1) 同侧小步：relocate(1) + or-opt(2/3)（pre/post 分段）
    TSPDSSolution sameSideRelocateOrOpt(TSPDSSolution& solution,
        int maxRounds = 3, bool firstImprove = true,
        bool enableRelocate1 = true, bool enableOrOpt2 = true, bool enableOrOpt3 = true);

    // (2) 批量 Truck->Drone
    TSPDSSolution batchTruckToDrone(TSPDSSolution& solution,
        int B = 5, bool firstImprove = true);

    // (2) 批量 Drone->Truck
    TSPDSSolution batchDroneToTruck(TSPDSSolution& solution,
        int B = 3, bool firstImprove = true);

    // (2.3) 组合：先批量 TD 再批量 DT（补偿式）
    TSPDSSolution batchCompensatedReassign2(TSPDSSolution& solution,
        int B_td = 5, int B_dt = 3, bool firstImprove = true);


    TSPDSSolution probeSearchStation(const TSPDSSolution& start,
        int steps);

    

    
};