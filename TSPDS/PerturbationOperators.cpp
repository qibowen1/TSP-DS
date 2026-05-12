#include "PerturbationOperators.h"
#include <cassert>

PerturbationOperators::PerturbationOperators(const TSPDSGraph& graph, const TSPDSAlgorithmParams& params)
	: graph(graph), params(params), utils(graph, params), gen(std::random_device{}()) {
}


TSPDSSolution PerturbationOperators::perturbSolution3(TSPDSSolution& solution, int k) {
    TSPDSSolution s = solution;

    batchNodeReassignment2(s, k);

    // depot 必须首尾
    assert(s.truck_route.front() == graph.depot);
    assert(s.truck_route.back() == graph.depot);
    // station 出现一次
    assert(std::count(s.truck_route.begin(), s.truck_route.end(),
        graph.drone_station) == 1);

    return s;
}

void PerturbationOperators::batchNodeReassignment2(TSPDSSolution& solution, int intensity) {
    std::cout << "Applying batch node reassignment shaking (intensity = " << intensity << ")...\n";

    // ---- A) 构造两类候选 ----
    std::vector<int> candTruckToDrone; // 从 truck_route 抽，且可给无人机
    candTruckToDrone.reserve(solution.truck_route.size());

    const auto& r = solution.truck_route;
    for (int k = 1; k + 1 < (int)r.size(); ++k) {
        int node = r[k];
        if (node == graph.depot || node == graph.drone_station) continue;
        if (!solution.served_by_truck.empty() && !solution.served_by_truck[node]) continue;
        if (!graph.is_drone_eligible.empty() && !graph.is_drone_eligible[node]) continue;
        candTruckToDrone.push_back(node);
    }

    std::vector<int> candDroneToTruck; // 从 served_by_drone 抽
    candDroneToTruck.reserve(64);
    for (int node = 0; node < (int)solution.served_by_drone.size(); ++node) {
        if (!solution.served_by_drone[node]) continue;
        if (node == graph.depot || node == graph.drone_station) continue;
        // 可选防御（一般 served_by_drone 已经保证合法）
        if (!graph.is_drone_eligible.empty() && !graph.is_drone_eligible[node]) continue;
        // 防御：避免 node 已经在 truck_route 里（flags 出错时）
        if (!solution.served_by_truck.empty() && solution.served_by_truck[node]) continue;
        candDroneToTruck.push_back(node);
    }

    int totalMovable = (int)candTruckToDrone.size() + (int)candDroneToTruck.size();
    if (totalMovable == 0) return;
    // ---- B) 根据强度决定比例----
    double reassignRatio;
	reassignRatio = intensity * 0.03; // 线性关系：强度越大，比例越大

    // 注意：按“可移动节点总数”算 numToReassign，而不是全图节点数
    int numToReassign = totalMovable * reassignRatio;

    // ---- C) 分配两类移动数量：先 50/50，不够就把剩余给另一侧 ----
    int wantTD = numToReassign / 2;                 // Truck->Drone
    int wantDT = numToReassign - wantTD;            // Drone->Truck

    int doTD = std::min(wantTD, (int)candTruckToDrone.size());
    int doDT = std::min(wantDT, (int)candDroneToTruck.size());

    int left = numToReassign - (doTD + doDT);
    if (left > 0) {
        int extraTD = std::min(left, (int)candTruckToDrone.size() - doTD);
        doTD += extraTD;
        left -= extraTD;
    }
    if (left > 0) {
        int extraDT = std::min(left, (int)candDroneToTruck.size() - doDT);
        doDT += extraDT;
        left -= extraDT;
    }

    if (doTD == 0 && doDT == 0) return;

    // ---- D) 抽样并执行 ----
    std::shuffle(candTruckToDrone.begin(), candTruckToDrone.end(), gen);
    std::shuffle(candDroneToTruck.begin(), candDroneToTruck.end(), gen);

    // 1) Truck -> Drone：从路径里删掉，flag 改为 drone
    for (int i = 0; i < doTD; ++i) {
        int node = candTruckToDrone[i];

        auto it = std::find(solution.truck_route.begin(), solution.truck_route.end(), node);
        if (it == solution.truck_route.end()) continue;
        if (it == solution.truck_route.begin() || it == solution.truck_route.end() - 1) continue;

        solution.truck_route.erase(it);
        solution.served_by_truck[node] = false;
        solution.served_by_drone[node] = true;
    }

    // 2) Drone -> Truck：flag 改为 truck，并 best-insertion 插回去
    for (int i = 0; i < doDT; ++i) {
        int node = candDroneToTruck[i];

        solution.served_by_drone[node] = false;
        solution.served_by_truck[node] = true;

        insertNodeToBestPosition(solution, node);
    }

}

void PerturbationOperators::insertNodeToBestPosition(TSPDSSolution& solution, int node)
{
    auto& r = solution.truck_route;
    if (r.size() < 2) return;


    int n = (int)r.size();

    int startEdge = 0;

    const double EPS = 1e-12;
    double bestInc = std::numeric_limits<double>::infinity();
    int bestPosition; // 存插入位置（index）

    for (int i = 0; i <= n - 2; ++i) {
        int from = r[i];
        int to = r[i + 1];

        double inc = graph.truck_time[from][node] + graph.truck_time[node][to] - graph.truck_time[from][to];

        if (inc + EPS < bestInc) {
            bestInc = inc;
            bestPosition = i+1;
        }
    }

    r.insert(r.begin() + bestPosition, node);
}


void PerturbationOperators::randomMoveDroneStation(TSPDSSolution& solution) {
    cout << "Applying random drone station movement perturbation..." << endl;

    // 1. 找到无人机站当前在路径中的位置
    int currentStationPos = -1;
    for (int i = 0; i < solution.truck_route.size(); ++i) {
        if (solution.truck_route[i] == graph.drone_station) {
            currentStationPos = i;
            break;
        }
    }

    if (currentStationPos == -1) {

        return;
    }

    // 2. 确定可移动的范围（避开depot起点和终点）
    int minPos = 1;  // 不能是第一个位置（depot）
    int maxPos = solution.truck_route.size() - 2;  // 不能是最后一个位置（depot）

    if (maxPos - minPos <= 1) {
        return;
    }

    // 3. 生成随机的新位置（确保与当前位置不同）
    uniform_int_distribution<> posDis(minPos, maxPos);
    int newPos;
    do {
        newPos = posDis(gen);
    } while (newPos == currentStationPos && (maxPos - minPos) > 1);

    // 4. 移动无人机站到新位置

    // 从原位置移除
    solution.truck_route.erase(solution.truck_route.begin() + currentStationPos);

    if (newPos > currentStationPos) newPos--;  // 关键修正

    // 插入到新位置
    solution.truck_route.insert(solution.truck_route.begin() + newPos, graph.drone_station);

}


void PerturbationOperators::moveDroneStationBySegment(TSPDSSolution& solution, int k)
{
    auto& r = solution.truck_route;
    const int n = (int)r.size();
    if (n <= 4) return; // depot ... station ... depot 至少要有点可动空间

    // 找 station 位置
    int oldPos = -1;
    for (int i = 0; i < n; ++i) {
        if (r[i] == graph.drone_station) { oldPos = i; break; }
    }
    if (oldPos <= 0 || oldPos >= n - 1) return;

    // 可插入位置范围：1..n-2（避开首尾 depot）
    const int L = 1;
    const int R = n - 2;
    const int len = R - L + 1;
    if (len <= 1) return;

    // ---- 1) 把 [L..R] 划分为 3 段（尽量均匀）----
    int a = len / 3;
    int b = len / 3;
    int c = len - a - b;
    // 保底：避免 a/b 为 0 导致段为空（len小的时候）
    if (a == 0) { a = 1; if (b > 0) b--; else c--; }
    if (b == 0) { b = 1; if (c > 0) c--; else a--; }
    if (c == 0) { c = 1; if (a > 1) a--; else if (b > 1) b--; }

    int frontL = L;
    int frontR = L + a - 1;
    int midL = frontR + 1;
    int midR = midL + b - 1;
    int backL = midR + 1;
    int backR = R; // 剩余全部给后段

    auto clampRange = [&](int& l, int& rr) {
        l = std::max(l, L);
        rr = std::min(rr, R);
        if (l > rr) { l = L; rr = R; }
        };
    clampRange(frontL, frontR);
    clampRange(midL, midR);
    clampRange(backL, backR);

    // ---- 2) k 决定目标段：1->前，2->中，0->后（循环）----
    int seg = ((k % 3) + 3) % 3;  // 0/1/2
    int targetL, targetR;
    if (seg == 1) { // k%3==1 -> 前段
        targetL = frontL; targetR = frontR;
    }
    else if (seg == 2) { // k%3==2 -> 中段
        targetL = midL; targetR = midR;
    }
    else { // k%3==0 -> 后段
        targetL = backL; targetR = backR;
    }
    clampRange(targetL, targetR);

    // 如果目标段只有一个位置且刚好等于 oldPos，就退化成全范围
    if (targetL == targetR && targetL == oldPos) {
        targetL = L; targetR = R;
    }

    // ---- 3) 选新位置（在目标段内，尽量 != oldPos）----
    std::uniform_int_distribution<int> dis(targetL, targetR);
    int newPos = dis(gen);


    // ---- 4) 执行 relocate：先删再插（注意删除后 index 修正）----
    r.erase(r.begin() + oldPos);

    // 删除后，如果 newPos 在 oldPos 之后，需要 -1 修正
    if (newPos > oldPos) newPos--;

    r.insert(r.begin() + newPos, graph.drone_station);
}
