#include "DroneScheduler.h"
#include <algorithm>
#include <numeric>
#include <queue>

#include <limits>

#include <vector>
#include <cmath>
#include <functional>

// ------------- helpers -------------
static inline int argmax_load(const std::vector<double>& loads) {
    int best = 0;
    for (int i = 1; i < (int)loads.size(); ++i)
        if (loads[i] > loads[best]) best = i;
    return best;
}
static inline int argmin_load(const std::vector<double>& loads) {
    int best = 0;
    for (int i = 1; i < (int)loads.size(); ++i)
        if (loads[i] < loads[best]) best = i;
    return best;
}

static inline void build_task_to_drone(
    const std::vector<std::vector<int>>& assign,
    int nTasks,
    std::vector<int>& taskToDrone
) {
    taskToDrone.assign(nTasks, -1);
    for (int d = 0; d < (int)assign.size(); ++d) {
        for (int t : assign[d]) taskToDrone[t] = d;
    }
}

// 取某个 drone 上处理时间最大的 TopK 个任务（返回 taskIndex）
static inline std::vector<int> topK_tasks_on_drone(
    const std::vector<std::vector<int>>& assign,
    int d,
    const std::vector<double>& p,
    int K
) {
    std::vector<int> tasks = assign[d];
    std::sort(tasks.begin(), tasks.end(), [&](int a, int b) { return p[a] > p[b]; });
    if ((int)tasks.size() > K) tasks.resize(K);
    return tasks;
}

// 取负载最小的 TopR 台无人机（返回 drone id 列表，按 load 升序）
static inline std::vector<int> topR_idlest_drones(
    const std::vector<double>& loads,
    int R
) {
    int m = (int)loads.size();
    std::vector<int> ids(m);
    std::iota(ids.begin(), ids.end(), 0);
    std::sort(ids.begin(), ids.end(), [&](int a, int b) { return loads[a] < loads[b]; });
    if ((int)ids.size() > R) ids.resize(R);
    return ids;
}

#include <random>
static inline double compute_makespan(const std::vector<double>& loads) {
    return *std::max_element(loads.begin(), loads.end());
}

static double bounded_exact_repack(
    const std::vector<double>& p,
    int m,
    std::vector<std::vector<int>>& bestAssignment,
    double bestMakespan,
    int nodeBudget = 1000,
    double eps = 1e-9
) {
    const int n = static_cast<int>(p.size());
    if (n == 0 || m <= 1) return bestMakespan;
    if (m > 5 || n > 70 || nodeBudget <= 0) return bestMakespan;

    std::vector<int> order(n);
    std::iota(order.begin(), order.end(), 0);
    std::sort(order.begin(), order.end(), [&](int a, int b) { return p[a] > p[b]; });

    std::vector<double> suffix(n + 1, 0.0);
    for (int i = n - 1; i >= 0; --i) suffix[i] = suffix[i + 1] + p[order[i]];

    std::vector<double> loads(m, 0.0);
    std::vector<std::vector<int>> current(m);
    int visited = 0;

    std::function<void(int)> dfs = [&](int idx) {
        if (visited >= nodeBudget) return;
        ++visited;
        if (idx == n) {
            double makespan = *std::max_element(loads.begin(), loads.end());
            if (makespan + eps < bestMakespan) {
                bestMakespan = makespan;
                bestAssignment = current;
            }
            return;
        }
        const double currentMax = *std::max_element(loads.begin(), loads.end());
        const double averageBound = (std::accumulate(loads.begin(), loads.end(), 0.0) + suffix[idx]) / m;
        if (std::max(currentMax, averageBound) >= bestMakespan - eps) return;
        const int task = order[idx];
        const double weight = p[task];
        std::vector<int> drones(m);
        std::iota(drones.begin(), drones.end(), 0);
        std::sort(drones.begin(), drones.end(), [&](int a, int b) { return loads[a] < loads[b]; });
        std::vector<double> triedLoads;
        for (int d : drones) {
            bool sameLoadTried = false;
            for (double tried : triedLoads) {
                if (std::fabs(tried - loads[d]) <= eps) { sameLoadTried = true; break; }
            }
            if (sameLoadTried) continue;
            triedLoads.push_back(loads[d]);
            if (loads[d] + weight >= bestMakespan - eps) continue;
            loads[d] += weight;
            current[d].push_back(task);
            dfs(idx + 1);
            current[d].pop_back();
            loads[d] -= weight;
        }
    };
    dfs(0);
    if (static_cast<int>(bestAssignment.size()) < m) bestAssignment.resize(m);
    return bestMakespan;
}

//枚举 move+swap（best-improve） + 卡住时随机扰动（shake）
static double improve_assignment_light(
    const std::vector<double>& p,
    std::vector<std::vector<int>>& assignment,
    int m,
    int maxIter = 200,
    double eps = 1e-9
) {
    const int nTasks = (int)p.size();
    if (nTasks == 0 || m <= 1) return 0.0;
    if ((int)assignment.size() < m) assignment.resize(m);

    // loads[d]
    std::vector<double> loads(m, 0.0);
    for (int d = 0; d < m; ++d) {
        double sum = 0.0;
        for (int t : assignment[d]) sum += p[t];
        loads[d] = sum;
    }

    // taskToDrone[t], taskPos[t]
    std::vector<int> taskToDrone(nTasks, -1);
    std::vector<int> taskPos(nTasks, -1);
    for (int d = 0; d < m; ++d) {
        for (int idx = 0; idx < (int)assignment[d].size(); ++idx) {
            int t = assignment[d][idx];
            if (t < 0 || t >= nTasks) continue;
            taskToDrone[t] = d;
            taskPos[t] = idx;
        }
    }

    auto makespan_if_change2 = [&](int dA, double newA, int dB, double newB) -> double {
        double ms = 0.0;
        for (int d = 0; d < m; ++d) {
            double ld = loads[d];
            if (d == dA) ld = newA;
            else if (d == dB) ld = newB;
            if (ld > ms) ms = ld;
        }
        return ms;
        };

    auto apply_move = [&](int t, int from, int to) {
        // remove t from assignment[from] via swap-back
        int pos = taskPos[t];
        auto& vFrom = assignment[from];
        int lastT = vFrom.back();
        vFrom[pos] = lastT;
        vFrom.pop_back();
        if (lastT != t) taskPos[lastT] = pos;

        // add t to assignment[to]
        auto& vTo = assignment[to];
        taskPos[t] = (int)vTo.size();
        vTo.push_back(t);
        taskToDrone[t] = to;

        // update loads
        loads[from] -= p[t];
        loads[to] += p[t];
        };

    auto apply_swap = [&](int ta, int da, int tb, int db) {
        int posa = taskPos[ta];
        int posb = taskPos[tb];

        assignment[da][posa] = tb;
        assignment[db][posb] = ta;

        taskToDrone[ta] = db;
        taskToDrone[tb] = da;

        taskPos[ta] = posb;
        taskPos[tb] = posa;

        loads[da] = loads[da] - p[ta] + p[tb];
        loads[db] = loads[db] - p[tb] + p[ta];
        };

    // ---------- 纯随机扰动（不区分最忙最闲） ----------
    static thread_local std::mt19937 rng(std::random_device{}());

    auto do_random_shake = [&](int kMoves, int kSwaps) {
        std::uniform_int_distribution<int> distTask(0, nTasks - 1);
        std::uniform_int_distribution<int> distDrone(0, m - 1);

        // 随机 MOVE：随机挑任务，移到随机其他无人机
        for (int i = 0; i < kMoves; ++i) {
            int t = -1, from = -1;
            // 尝试若干次找到有效任务
            for (int tries = 0; tries < 8; ++tries) {
                int cand = distTask(rng);
                int d = taskToDrone[cand];
                if (d >= 0 && d < m && !assignment[d].empty()) { t = cand; from = d; break; }
            }
            if (t < 0) continue;

            int to = distDrone(rng);
            if (to == from) to = (to + 1) % m;
            apply_move(t, from, to);
        }

        // 随机 SWAP：随机挑两台无人机各取一个任务交换
        for (int i = 0; i < kSwaps; ++i) {
            int da = distDrone(rng);
            int db = distDrone(rng);
            if (da == db) db = (db + 1) % m;

            if (assignment[da].empty() || assignment[db].empty()) continue;

            std::uniform_int_distribution<int> distA(0, (int)assignment[da].size() - 1);
            std::uniform_int_distribution<int> distB(0, (int)assignment[db].size() - 1);
            int ta = assignment[da][distA(rng)];
            int tb = assignment[db][distB(rng)];
            if (ta == tb) continue;

            apply_swap(ta, da, tb, db);
        }
        };

    double curMs = compute_makespan(loads);

    // 保底：保存全局最好（避免扰动把结果搞差）
    double bestGlobalMs = curMs;
    std::vector<std::vector<int>> bestGlobalAssign = assignment;

    const int maxShakes = 3;                 // 最多扰动几次
    const int kMovesBase = std::max(1, nTasks / 10); // 每次扰动做多少个随机 move（10%任务）
    const int kSwapsBase = (m >= 2 ? 1 : 0); // 每次扰动做多少个随机 swap
    int shakesUsed = 0;

    for (int it = 0; it < maxIter; ++it) {
        double bestMs = curMs;

        bool bestIsMove = false;
        int bestMoveTask = -1, bestMoveFrom = -1, bestMoveTo = -1;
        int bestSwapA = -1, bestSwapB = -1, bestSwapDA = -1, bestSwapDB = -1;

        // ---- enumerate all MOVEs ----
        for (int t = 0; t < nTasks; ++t) {
            int from = taskToDrone[t];
            if (from < 0 || from >= m) continue;
            double pt = p[t];

            for (int to = 0; to < m; ++to) {
                if (to == from) continue;
                double newFrom = loads[from] - pt;
                double newTo = loads[to] + pt;
                double ms = makespan_if_change2(from, newFrom, to, newTo);
                if (ms + eps < bestMs) {
                    bestMs = ms;
                    bestIsMove = true;
                    bestMoveTask = t;
                    bestMoveFrom = from;
                    bestMoveTo = to;
                }
            }
        }

        // ---- enumerate all SWAPs ----
        for (int da = 0; da < m; ++da) {
            for (int db = da + 1; db < m; ++db) {
                const auto& A = assignment[da];
                const auto& B = assignment[db];
                if (A.empty() || B.empty()) continue;

                for (int ta : A) {
                    double pta = p[ta];
                    for (int tb : B) {
                        if (ta == tb) continue;
                        double ptb = p[tb];

                        double newA = loads[da] - pta + ptb;
                        double newB = loads[db] - ptb + pta;
                        double ms = makespan_if_change2(da, newA, db, newB);

                        if (ms + eps < bestMs) {
                            bestMs = ms;
                            bestIsMove = false;
                            bestSwapA = ta; bestSwapB = tb;
                            bestSwapDA = da; bestSwapDB = db;
                        }
                    }
                }
            }
        }

        // ---- 如果本轮没有任何改进：尝试扰动而不是直接退出 ----
        if (!(bestMs + eps < curMs)) {
            if (shakesUsed < maxShakes) {
                // 纯扰动：随机 move/swap（不看最忙最闲）
                do_random_shake(kMovesBase, kSwapsBase);
                curMs = compute_makespan(loads);
                ++shakesUsed;
                continue; // 扰动后继续下一轮枚举
            }
            else {
                break; // 扰动次数用完，结束
            }
        }

        // ---- apply best move/swap ----
        if (bestIsMove) {
            if (bestMoveTask >= 0 && bestMoveFrom >= 0 && bestMoveTo >= 0) {
                apply_move(bestMoveTask, bestMoveFrom, bestMoveTo);
            }
            else break;
        }
        else {
            if (bestSwapA >= 0 && bestSwapB >= 0) {
                apply_swap(bestSwapA, bestSwapDA, bestSwapB, bestSwapDB);
            }
            else break;
        }

        curMs = bestMs;

        // 更新全局最好
        if (curMs + eps < bestGlobalMs) {
            bestGlobalMs = curMs;
            bestGlobalAssign = assignment;
        }
    }

    // 返回前恢复到全局最好（避免扰动把解弄差）
    assignment = std::move(bestGlobalAssign);
    return bestGlobalMs;
}





// 调度无人机任务：更新解决方案的无人机分配和完成时间

static double improve_assignment_deterministic(
    const std::vector<double>& p,
    std::vector<std::vector<int>>& assignment,
    int m,
    int maxIter = 50,
    double eps = 1e-9
) {
    const int nTasks = static_cast<int>(p.size());
    if (nTasks == 0 || m <= 1) return 0.0;
    if (static_cast<int>(assignment.size()) < m) assignment.resize(m);

    auto computeLoads = [&]() {
        std::vector<double> loads(m, 0.0);
        for (int d = 0; d < m; ++d) {
            for (int t : assignment[d]) {
                if (t >= 0 && t < nTasks) loads[d] += p[t];
            }
        }
        return loads;
    };

    auto eraseTask = [](std::vector<int>& tasks, int task) {
        auto it = std::find(tasks.begin(), tasks.end(), task);
        if (it != tasks.end()) tasks.erase(it);
    };

    std::vector<double> loads = computeLoads();
    double curMs = compute_makespan(loads);

    enum class MoveType { None, SingleMove, Swap, TwoForOne };

    for (int iter = 0; iter < maxIter; ++iter) {
        MoveType bestType = MoveType::None;
        double bestMs = curMs;
        int a = -1, b = -1, x = -1, y = -1, z = -1;

        for (int from = 0; from < m; ++from) {
            for (int task : assignment[from]) {
                const double pt = p[task];
                for (int to = 0; to < m; ++to) {
                    if (to == from) continue;
                    std::vector<double> trial = loads;
                    trial[from] -= pt;
                    trial[to] += pt;
                    double ms = compute_makespan(trial);
                    if (ms + eps < bestMs) {
                        bestMs = ms;
                        bestType = MoveType::SingleMove;
                        a = from; b = to; x = task;
                    }
                }
            }
        }

        for (int d1 = 0; d1 < m; ++d1) {
            for (int d2 = d1 + 1; d2 < m; ++d2) {
                for (int t1 : assignment[d1]) {
                    for (int t2 : assignment[d2]) {
                        std::vector<double> trial = loads;
                        trial[d1] += p[t2] - p[t1];
                        trial[d2] += p[t1] - p[t2];
                        double ms = compute_makespan(trial);
                        if (ms + eps < bestMs) {
                            bestMs = ms;
                            bestType = MoveType::Swap;
                            a = d1; b = d2; x = t1; y = t2;
                        }
                    }
                }
            }
        }

        for (int from = 0; from < m; ++from) {
            const auto& fromTasks = assignment[from];
            if (fromTasks.size() < 2) continue;
            for (int to = 0; to < m; ++to) {
                if (to == from || assignment[to].empty()) continue;
                for (int i = 0; i < static_cast<int>(fromTasks.size()); ++i) {
                    for (int j = i + 1; j < static_cast<int>(fromTasks.size()); ++j) {
                        int t1 = fromTasks[i];
                        int t2 = fromTasks[j];
                        const double pairLoad = p[t1] + p[t2];
                        for (int t3 : assignment[to]) {
                            std::vector<double> trial = loads;
                            trial[from] += p[t3] - pairLoad;
                            trial[to] += pairLoad - p[t3];
                            double ms = compute_makespan(trial);
                            if (ms + eps < bestMs) {
                                bestMs = ms;
                                bestType = MoveType::TwoForOne;
                                a = from; b = to; x = t1; y = t2; z = t3;
                            }
                        }
                    }
                }
            }
        }

        if (bestType == MoveType::None) break;

        if (bestType == MoveType::SingleMove) {
            eraseTask(assignment[a], x);
            assignment[b].push_back(x);
        }
        else if (bestType == MoveType::Swap) {
            eraseTask(assignment[a], x);
            eraseTask(assignment[b], y);
            assignment[a].push_back(y);
            assignment[b].push_back(x);
        }
        else if (bestType == MoveType::TwoForOne) {
            eraseTask(assignment[a], x);
            eraseTask(assignment[a], y);
            eraseTask(assignment[b], z);
            assignment[a].push_back(z);
            assignment[b].push_back(x);
            assignment[b].push_back(y);
        }

        loads = computeLoads();
        curMs = compute_makespan(loads);
    }

    return curMs;
}

void DroneScheduler::scheduleDrones(TSPDSSolution& solution, const TSPDSGraph& graph) {
    int stationIndex = graph.drone_station;
    if (stationIndex == -1) {
        return;
    }

    // 收集需要由无人机服务的客户点（排除无人机站本身）
    std::vector<int> droneNodes;
    for (int i = 0; i < solution.served_by_drone.size(); i++) {
        if (solution.served_by_drone[i] && i != stationIndex) {
            droneNodes.push_back(i);
        }
    }

    // 计算每个任务的处理时间：从无人机站到客户点的往返时间（2 * drone_time）
    std::vector<double> processingTimes;
    for (int node : droneNodes) {
        double time = 2 * graph.drone_time[stationIndex][node]; // 浠诲姟鏃堕棿
        processingTimes.push_back(time);
    }

    int numDrones = graph.drone_count;
    auto [assignment, makespan] = combineAlgorithm(processingTimes, numDrones);

    // 局部改进
    /*makespan = improve_assignment_light(
        processingTimes, assignment, numDrones
    );*/

    // 更新解决方案的无人机分配映射
    solution.drone_assignments.clear();
    solution.node_to_drone.clear();
    for (int d = 0; d < numDrones; d++) {
        solution.drone_assignments[d] = std::vector<int>(); // 初始化每个无人机的任务列表
        for (int taskIndex : assignment[d]) {
            int node = droneNodes[taskIndex]; // taskIndex是processingTimes列表的索引，映射回实际节点
            solution.drone_assignments[d].push_back(node);
            solution.node_to_drone[node] = d;
        }
    }


}


void DroneScheduler::polishDroneAssignment(TSPDSSolution& solution, const TSPDSGraph& graph) {
    const int stationIndex = graph.drone_station;
    const int m = graph.drone_count;
    if (stationIndex < 0 || m <= 1) return;

    std::vector<int> droneNodes;
    for (int i = 0; i < static_cast<int>(solution.served_by_drone.size()); ++i) {
        if (solution.served_by_drone[i] && i != stationIndex) droneNodes.push_back(i);
    }
    if (droneNodes.empty()) return;

    std::vector<double> processingTimes;
    processingTimes.reserve(droneNodes.size());
    for (int node : droneNodes) {
        processingTimes.push_back(2.0 * graph.drone_time[stationIndex][node]);
    }

    std::vector<std::vector<int>> currentAssignment(m);
    std::vector<double> currentLoads(m, 0.0);
    for (const auto& kv : solution.drone_assignments) {
        int d = kv.first;
        if (d < 0 || d >= m) continue;
        for (int node : kv.second) {
            auto it = std::find(droneNodes.begin(), droneNodes.end(), node);
            if (it == droneNodes.end()) continue;
            int taskIndex = static_cast<int>(std::distance(droneNodes.begin(), it));
            currentAssignment[d].push_back(taskIndex);
            currentLoads[d] += processingTimes[taskIndex];
        }
    }

    double currentMakespan = currentLoads.empty() ? 0.0 : *std::max_element(currentLoads.begin(), currentLoads.end());
    if (currentMakespan <= 0.0) {
        auto lpt = lptScheduling(processingTimes, m);
        currentAssignment = lpt.first;
        currentMakespan = lpt.second;
    }
    const double originalMakespan = currentMakespan;

    currentMakespan = improve_assignment_deterministic(
        processingTimes,
        currentAssignment,
        m,
        50
    );

    double polishedMakespan = bounded_exact_repack(
        processingTimes,
        m,
        currentAssignment,
        currentMakespan,
        100000
    );
    currentMakespan = std::min(currentMakespan, polishedMakespan);
    if (currentMakespan + 1e-9 >= originalMakespan) return;

    solution.drone_assignments.clear();
    solution.node_to_drone.clear();
    for (int d = 0; d < m; ++d) {
        solution.drone_assignments[d] = std::vector<int>();
        for (int taskIndex : currentAssignment[d]) {
            int node = droneNodes[taskIndex];
            solution.drone_assignments[d].push_back(node);
            solution.node_to_drone[node] = d;
        }
    }
}

std::pair<std::vector<std::vector<int>>, double>
DroneScheduler::combineAlgorithm(const std::vector<double>& processingTimes, int m) {
    // 空任务检查
    if (processingTimes.empty()) {
        std::vector<std::vector<int>> emptyAssignment(m);
        return { emptyAssignment, 0.0 };
    }

    // 1. 计算边界
    double maxTime = *std::max_element(processingTimes.begin(), processingTimes.end());
    double totalTime = std::accumulate(processingTimes.begin(), processingTimes.end(), 0.0);
    double lowerBound = std::max(maxTime, totalTime / m);

    // 2. LPT算法获取初始上界
    auto lptResult = lptScheduling(processingTimes, m);
    double upperBound = lptResult.second;
    std::vector<std::vector<int>> bestAssignment = lptResult.first;
    double bestMakespan = upperBound;

    // 如果LPT已经是最优，直接返回
    if (upperBound - lowerBound < 1e-5) {
        return { bestAssignment, bestMakespan };
    }

    // 3. MULTIFIT二分搜索
    const double epsilon = 1e-5;
    int iteration = 0;
    const int maxIterations = 50;

    while ((upperBound - lowerBound) > epsilon && iteration < maxIterations) {
        iteration++;
        double mid = (lowerBound + upperBound) / 2.0;

        std::vector<std::vector<int>> testAssignment;
        int binsNeeded = ffdBinPacking(processingTimes, mid, testAssignment);

        //std::cout << "Iteration " << iteration << ": mid=" << mid
        //    << ", binsNeeded=" << binsNeeded << ", m=" << m << std::endl;

        if (binsNeeded <= m) {
            // 可行解
            upperBound = mid;
            bestAssignment = testAssignment;
            bestMakespan = mid;
            //std::cout << "Feasible solution found, makespan: " << bestMakespan << std::endl;
        }
        else {
            // 不可行
            lowerBound = mid;
            //std::cout << "Infeasible, increasing lower bound to " << lowerBound << std::endl;
        }
    }

    // 确保返回m个箱子的分配
    if (bestAssignment.size() < m) {
        bestAssignment.resize(m);
    }

    //std::cout << "Final makespan: " << bestMakespan << std::endl;
    //std::cout << "Final assignment sizes: ";
    /*for (size_t i = 0; i < bestAssignment.size(); i++) {
        std::cout << bestAssignment[i].size() << " ";
    }
    std::cout << std::endl;*/


    return { bestAssignment, bestMakespan };

}

//LTP获取上界
std::pair<std::vector<std::vector<int>>, double>
DroneScheduler::lptScheduling(const std::vector<double>& processingTimes, int m) {
    std::vector<std::vector<int>> assignment(m);
    std::vector<double> loads(m, 0.0);

    // 创建索引并按时长降序排序
    std::vector<int> indices(processingTimes.size());
    std::iota(indices.begin(), indices.end(), 0);
    std::sort(indices.begin(), indices.end(),
        [&](int a, int b) { return processingTimes[a] > processingTimes[b]; });

    // LPT分配策略
    for (int idx : indices) {
        // 找到当前负载最小的机器
        int bestMachine = 0;
        double minLoad = loads[0];
        for (int i = 1; i < m; i++) {
            if (loads[i] < minLoad) {
                minLoad = loads[i];
                bestMachine = i;
            }
        }

        // 分配任务
        assignment[bestMachine].push_back(idx);
        loads[bestMachine] += processingTimes[idx];
    }

    double makespan = *std::max_element(loads.begin(), loads.end());
    return { assignment, makespan };
}

// 辅助函数：FFD bin packing测试，返回所需的bin数量和分配方案
int DroneScheduler::ffdBinPacking(const std::vector<double>& items, double capacity,
    std::vector<std::vector<int>>& assignment) {
    assignment.clear();

    if (items.empty()) {
        return 0;
    }

    // 创建索引并按处理时间降序排序
    std::vector<int> indices(items.size());
    std::iota(indices.begin(), indices.end(), 0);
    std::sort(indices.begin(), indices.end(),
        [&](int a, int b) { return items[a] > items[b]; });

    std::vector<double> binLoads;
    assignment.clear();

    // 浮点数比较容差
    const double tolerance = 1e-5;

    for (int idx : indices) {
        double itemSize = items[idx];
        bool placed = false;

        // 尝试放入现有箱子
        for (size_t i = 0; i < binLoads.size(); i++) {
            if (binLoads[i] + itemSize <= capacity + tolerance) {
                binLoads[i] += itemSize;

                // 确保assignment有足够的箱子
                if (i >= assignment.size()) {
                    assignment.resize(i + 1);
                }
                assignment[i].push_back(idx);
                placed = true;
                break;
            }
        }

        // 创建新箱子
        if (!placed) {
            binLoads.push_back(itemSize);
            assignment.push_back({ idx });
        }
    }

    return binLoads.size();
}