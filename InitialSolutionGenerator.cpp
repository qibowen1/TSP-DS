#include "InitialSolutionGenerator.h"
#include <algorithm>
#include <numeric>

InitialSolutionGenerator::InitialSolutionGenerator(const TSPDSGraph& graph,
    const TSPDSAlgorithmParams& params)
    : graph(graph), params(params) , utils(graph, params){
    std::random_device rd;
    rng.seed(rd());   // 默认随机器种子，后面 solver 会覆盖
}

TSPDSSolution InitialSolutionGenerator::generateInitialSolution() {
    TSPDSSolution solution;
    solution.initialize(graph.nodes.size());
    solution.truck_route.push_back(graph.depot); // 从 depot 开始
    solution.served_by_truck[graph.depot] = true;

    // 生成必须访问的节点列表：包括无人机站和所有顾客节点（不含 depot）
    vector<int> mustVisit;
    mustVisit.push_back(graph.drone_station);
    for (int i = 0; i < (int)graph.nodes.size(); ++i) {
        if (i != graph.depot) {
            mustVisit.push_back(i);
        }
    }

    // 最近邻构造初始 TSP 路径
    vector<bool> visited(graph.nodes.size(), false);
    visited[graph.depot] = true;

    int current = graph.depot;
    const int RNN_K = 10;  // 可以以后放到 params 中 从前 K 个最近的点里随机选一个

    while (true) {
        std::vector<std::pair<double, int>> cand;
        cand.reserve(mustVisit.size());

        // 收集所有未访问的候选点及其距离
        for (int node : mustVisit) {
            if (!visited[node]) {
                double t = graph.truck_time[current][node];
                cand.emplace_back(t, node);
            }
        }

        if (cand.empty()) break;  // 没有可选点了，结束

        // 按距离从小到大排序
        std::sort(cand.begin(), cand.end(),
            [](const auto& a, const auto& b) {
                return a.first < b.first;
            });

        // 只在前 K 个最近点里随机选一个
        int k = static_cast<int>(cand.size());
        if (k > RNN_K) k = RNN_K;

        std::uniform_int_distribution<int> dist(0, k - 1);
        int chosenIdx = dist(rng);
        int nextNode = cand[chosenIdx].second;

        solution.truck_route.push_back(nextNode);
        visited[nextNode] = true;
        current = nextNode;
    }

    solution.truck_route.push_back(graph.depot);

    // 初始化所有路径上的节点由卡车服务
    for (int node : solution.truck_route) {
        solution.served_by_truck[node] = true;
    }

    // 初始化无人机任务为空
    for (int d = 0; d < graph.drone_count; ++d) {
        solution.drone_assignments[d] = vector<int>();
    }

    // 初始评估
    utils.optimizeTruckRouteWithLKHIntern(solution);
    utils.evaluateSolution(solution, /*needCalDrone=*/true);
    double currentMakespan = solution.makespan;
    cout << "Initial makespan after truck route: " << currentMakespan << endl;

    // 改进循环：尝试把若干 truck 节点交给无人机
    TSPDSSolution testSolution = solution;
    vector<int> candidates;

    // 候选节点：路径中可由无人机服务的非 depot / station
    for (int node : solution.truck_route) {
        if (node != graph.depot &&
            node != graph.drone_station &&
            graph.is_drone_eligible[node]) {
            candidates.push_back(node);
        }
    }
    // 随机打乱候选顺序，让“先尝试给无人机的点”有多样性
    std::shuffle(candidates.begin(), candidates.end(), rng);
    for (int node : candidates) {
        if (!testSolution.served_by_truck[node]) continue;

        auto it = find(testSolution.truck_route.begin(), testSolution.truck_route.end(), node);
        if (it == testSolution.truck_route.end()) continue;
        if (it == testSolution.truck_route.begin() || it == testSolution.truck_route.end() - 1) {
            continue; // 不移除 depot 两端
        }

        // 备份当前解
        TSPDSSolution backup = testSolution;

        // 从卡车路径中移除，交给无人机
        testSolution.truck_route.erase(it);
        testSolution.served_by_truck[node] = false;
        testSolution.served_by_drone[node] = true;

        utils.evaluateSolution(testSolution, /*needCalDrone=*/true);

        if (testSolution.makespan < currentMakespan - 1e-6) {
            // 接受
            solution = testSolution;
            currentMakespan = solution.makespan;
            cout << "Improved by assigning node " << node
                << ", new makespan: " << currentMakespan << endl;
        }
        else {
            // 拒绝，恢复
            testSolution = backup;
        }
    }

    // 最终评估
    utils.evaluateSolution(solution, /*needCalDrone=*/true);

    cout << "Improved initial solution generated:" << endl;
    cout << "Makespan: " << solution.makespan << endl;
    cout << "Truck route size: " << solution.truck_route.size() << endl;
    cout << "Truck completion time: " << solution.truck_completion_time << endl;
    cout << "Drone completion time: " << solution.drone_completion_time << endl;
    cout << "Station activation time: " << solution.station_activation_time << endl;

    cout << "Truck route: ";
    for (int node : solution.truck_route) cout << node << " ";
    cout << endl;

    for (int d = 0; d < graph.drone_count; ++d) {
        if (!solution.drone_assignments[d].empty()) {
            cout << "Drone " << d << " tasks: ";
            for (int node : solution.drone_assignments[d]) {
                cout << node << "(" << graph.drone_time[graph.drone_station][node] << ") ";
            }
            cout << endl;
        }
    }
    return solution;
}

TSPDSSolution InitialSolutionGenerator::generateInitialSolution_paper() {
    TSPDSSolution solution;
    solution.initialize(graph.nodes.size());

    // =========================================================
    // 0) 先构造一个 truck-only 路径（包含 depot 和 station）
    // =========================================================
    solution.truck_route.clear();
    solution.truck_route.push_back(graph.depot);
    solution.served_by_truck[graph.depot] = true;

    // mustVisit：station + 所有客户（排除 depot/station，避免 station 重复）
    std::vector<int> mustVisit;
    mustVisit.reserve(graph.nodes.size());
    mustVisit.push_back(graph.drone_station);
    for (int i = 0; i < (int)graph.nodes.size(); ++i) {
        if (i == graph.depot) continue;
        if (i == graph.drone_station) continue;
        mustVisit.push_back(i);
    }

    std::vector<bool> visited(graph.nodes.size(), false);
    visited[graph.depot] = true;

    int current = graph.depot;
    const int RNN_K = 10;

    while (true) {
        std::vector<std::pair<double, int>> cand;
        cand.reserve(mustVisit.size());

        for (int node : mustVisit) {
            if (!visited[node]) {
                double t = graph.truck_time[current][node];
                cand.emplace_back(t, node);
            }
        }
        if (cand.empty()) break;

        std::sort(cand.begin(), cand.end(),
            [](const auto& a, const auto& b) { return a.first < b.first; });

        int k = (int)cand.size();
        if (k > RNN_K) k = RNN_K;

        std::uniform_int_distribution<int> dist(0, k - 1);
        int nextNode = cand[dist(rng)].second;

        solution.truck_route.push_back(nextNode);
        visited[nextNode] = true;
        current = nextNode;
    }

    // 回 depot
    solution.truck_route.push_back(graph.depot);

    // 全部先当作 truck-only
    std::fill(solution.served_by_truck.begin(), solution.served_by_truck.end(), false);
    std::fill(solution.served_by_drone.begin(), solution.served_by_drone.end(), false);

    for (int v : solution.truck_route) solution.served_by_truck[v] = true;
    solution.served_by_truck[graph.drone_station] = true;

    for (int d = 0; d < graph.drone_count; ++d) solution.drone_assignments[d].clear();

    // 先把 truck route 用 LKH 优化一下（可选但建议）
    utils.optimizeTruckRouteWithLKHIntern(solution);
    utils.evaluateSolution(solution, /*needCalDrone=*/true);

    // =========================================================
    // 1) Algorithm 2：从 truck-only 解出发改进
    // =========================================================

    auto segmentTime = [&](const std::vector<int>& R) -> double {
        double t = 0.0;
        for (int i = 0; i + 1 < (int)R.size(); ++i) {
            t += graph.truck_time[R[i]][R[i + 1]];
        }
        return t;
        };

    auto removeOne = [&](std::vector<int> R, int node) -> std::vector<int> {
        auto it = std::find(R.begin(), R.end(), node);
        if (it != R.end()) R.erase(it);
        return R;
        };

    auto roundTrip = [&](int node) -> double {
        int s = graph.drone_station;
        return graph.drone_time[s][node] + graph.drone_time[node][s];
        };

    // 计算 t3：对 D' 的无人机并行完成时间（用 LPT/贪心近似，和你 LTP 调度思想一致）
    auto droneDeliveryTime = [&](const std::vector<int>& droneSet) -> double {
        if (droneSet.empty()) return 0.0;
        int m = std::max(1, graph.drone_count);

        std::vector<double> jobs;
        jobs.reserve(droneSet.size());
        for (int v : droneSet) jobs.push_back(roundTrip(v));

        std::sort(jobs.begin(), jobs.end(), std::greater<double>()); // LPT

        std::vector<double> load(m, 0.0);
        for (double p : jobs) {
            auto itMin = std::min_element(load.begin(), load.end());
            *itMin += p;
        }
        return *std::max_element(load.begin(), load.end());
        };

    auto splitByStation = [&](const std::vector<int>& route,
        std::vector<int>& R1,
        std::vector<int>& R2) {
            int s = graph.drone_station;
            auto itS = std::find(route.begin(), route.end(), s);
            if (itS == route.end()) {
                throw std::runtime_error("station not found in truck_route");
            }
            int pos = (int)std::distance(route.begin(), itS);

            // route: depot ... station ... depot
            R1.assign(route.begin(), route.begin() + pos + 1);           // depot -> station
            R2.assign(route.begin() + pos, route.end());                 // station -> depot
        };

    auto swapSegmentsIfNeeded = [&](std::vector<int>& R1, std::vector<int>& R2,
        double& t1, double& t2) {
            // 你的场景 start=end=depot，所以 swap 就是 reverse + swap
            if (t1 <= t2) return;
            std::vector<int> oldR1 = R1;
            std::vector<int> oldR2 = R2;

            std::reverse(oldR2.begin(), oldR2.end()); // station->depot 反转成 depot->station
            std::reverse(oldR1.begin(), oldR1.end()); // depot->station 反转成 station->depot

            R1 = oldR2;
            R2 = oldR1;

            t1 = segmentTime(R1);
            t2 = segmentTime(R2);
        };

    // ---- (1) R1/R2, t1/t2 ----
    std::vector<int> R1, R2;
    splitByStation(solution.truck_route, R1, R2);

    double t1 = segmentTime(R1);
    double t2 = segmentTime(R2);

    // ---- (2) D：按“离 station 远”降序排列的 drone-eligible ----
    std::vector<int> D;
    D.reserve(graph.nodes.size());
    for (int v = 0; v < (int)graph.nodes.size(); ++v) {
        if (v == graph.depot) continue;
        if (v == graph.drone_station) continue;
        if (!graph.is_drone_eligible[v]) continue;

        // 若你有 range 约束，这里也可以过滤
        if (graph.drone_range < std::numeric_limits<double>::max()) {
            if (roundTrip(v) > graph.drone_range + 1e-9) continue;
        }
        D.push_back(v);
    }

    std::sort(D.begin(), D.end(), [&](int a, int b) {
        return graph.drone_time[graph.drone_station][a] > graph.drone_time[graph.drone_station][b];
        });

    std::vector<int> Dprime; // D'
    Dprime.reserve(D.size());

    double obj = t1 + t2; // 初始时 t3=0

    const double EPS = 1e-6;

    for (int i : D) {

        // i 可能已经不在卡车段里（之前被移走了），跳过即可
        bool inR1 = (std::find(R1.begin(), R1.end(), i) != R1.end());
        bool inR2 = (std::find(R2.begin(), R2.end(), i) != R2.end());
        if (!inR1 && !inR2) continue;

        // (Algorithm2 line 3-5)
        swapSegmentsIfNeeded(R1, R2, t1, t2);

        // 重算 membership（swap 之后段变了）
        inR1 = (std::find(R1.begin(), R1.end(), i) != R1.end());
        inR2 = (std::find(R2.begin(), R2.end(), i) != R2.end());
        if (!inR1 && !inR2) continue;

        // (line 6) t3 = drone delivery time on D' ∪ {i}
        std::vector<int> trialD = Dprime;
        trialD.push_back(i);
        double t3 = droneDeliveryTime(trialD);

        if (inR1) {
            // (line 7-11)
            auto R1n = removeOne(R1, i);
            // 不能把 depot/station 删除掉（理论上 i 不会是）
            if (R1n.size() < 2) continue;

            double t1n = segmentTime(R1n);
            double objn = t1n + std::max(t2, t3);

            if (objn + EPS < obj) {
                R1 = std::move(R1n);
                t1 = t1n;
                Dprime.push_back(i);
                obj = objn;
            }
        }
        else {
            // (line 12-17)
            auto R2n = removeOne(R2, i);
            if (R2n.size() < 2) continue;

            double t2n = segmentTime(R2n);
            double objn = t1 + std::max(t2n, t3);

            if (objn + EPS < obj) {
                R2 = std::move(R2n);
                t2 = t2n;
                Dprime.push_back(i);
                obj = objn;
            }
        }
    }

    // (line 19-21) 最后再确保 t1 <= t2
    swapSegmentsIfNeeded(R1, R2, t1, t2);

    // (line 22) 由 R1, R2, D' 生成 primal heuristic solution
    std::vector<int> newTruck;
    newTruck.reserve(R1.size() + R2.size());
    newTruck.insert(newTruck.end(), R1.begin(), R1.end());
    // 拼接 R2 时跳过第一个 station，避免重复
    if (!R2.empty()) newTruck.insert(newTruck.end(), R2.begin() + 1, R2.end());

    solution.truck_route = std::move(newTruck);

    // 更新服务关系
    std::fill(solution.served_by_truck.begin(), solution.served_by_truck.end(), false);
    std::fill(solution.served_by_drone.begin(), solution.served_by_drone.end(), false);

    for (int v : solution.truck_route) solution.served_by_truck[v] = true;
    solution.served_by_truck[graph.depot] = true;
    solution.served_by_truck[graph.drone_station] = true;

    for (int v : Dprime) {
        solution.served_by_truck[v] = false;
        solution.served_by_drone[v] = true;
    }
    solution.served_by_drone[graph.depot] = false;
    solution.served_by_drone[graph.drone_station] = false;

    for (int d = 0; d < graph.drone_count; ++d) solution.drone_assignments[d].clear();

    // 最终评估（会调用你的 droneScheduler 去算 makespan / 分配）
    utils.evaluateSolution(solution, /*needCalDrone=*/true);

    std::cout << "[Alg2 Init] makespan=" << solution.makespan
        << " Tt=" << solution.truck_completion_time
        << " Ta=" << solution.station_activation_time
        << " Td=" << solution.drone_completion_time
        << " |D'|=" << Dprime.size() << "\n";

    return solution;
}



TSPDSSolution InitialSolutionGenerator::generateInitialSolution_DroneMain() {
    TSPDSSolution sol;
    sol.initialize((int)graph.nodes.size());

    const int depot = graph.depot;
    const int station = graph.drone_station;

    auto isDroneEligible = [&](int node) -> bool {
        if (node == depot || node == station) return false;
        if (!graph.is_drone_eligible.empty()) return graph.is_drone_eligible[node];
        return true; // 如果没提供 eligible 信息，默认可无人机
        };

    auto findPos = [&](const std::vector<int>& r, int node) -> int {
        for (int i = 0; i < (int)r.size(); ++i) if (r[i] == node) return i;
        return -1;
        };

    auto bestInsertionPos = [&](const std::vector<int>& r, int node) -> int {
        // 在 r 的任意边 (r[i], r[i+1]) 中间插入 node，选增量最小的位置
        double bestInc = std::numeric_limits<double>::infinity();
        int bestPos = -1; // insert at begin()+bestPos
        for (int i = 0; i + 1 < (int)r.size(); ++i) {
            int a = r[i], b = r[i + 1];
            double inc = graph.truck_time[a][node] + graph.truck_time[node][b] - graph.truck_time[a][b];
            if (inc < bestInc) {
                bestInc = inc;
                bestPos = i + 1;
            }
        }
        return bestPos;
        };

    auto insertBest = [&](TSPDSSolution& S, int node) -> bool {
        if ((int)S.truck_route.size() < 2) return false;
        if (findPos(S.truck_route, node) != -1) return false;
        int pos = bestInsertionPos(S.truck_route, node);
        if (pos < 1 || pos >(int)S.truck_route.size() - 1) return false;
        S.truck_route.insert(S.truck_route.begin() + pos, node);
        return true;
        };

    // 1) 先构造卡车基本骨架：depot -> station -> depot
    sol.truck_route.clear();
    sol.truck_route.push_back(depot);
    sol.truck_route.push_back(station);
    sol.truck_route.push_back(depot);

    // 2) 初始化：除 depot/station 外，默认全部给无人机；不可行的再改给卡车
    for (int i = 0; i < (int)graph.nodes.size(); ++i) {
        sol.served_by_truck[i] = false;
        sol.served_by_drone[i] = false;
    }
    sol.served_by_truck[depot] = true;
    sol.served_by_truck[station] = true;

    // 3) 不可行节点 -> 卡车，并插入到卡车路径（best insertion）
    std::vector<int> truckOnly;
    truckOnly.reserve(graph.nodes.size());

    for (int i = 0; i < (int)graph.nodes.size(); ++i) {
        if (i == depot || i == station) continue;

        if (!isDroneEligible(i)) {
            sol.served_by_truck[i] = true;
            sol.served_by_drone[i] = false;
            truckOnly.push_back(i);
        }
        else {
            sol.served_by_truck[i] = false;
            sol.served_by_drone[i] = true;  // 可行的全部给无人机
        }
    }

    // 可选：打乱一下插入顺序（不想随机就删掉）
    std::shuffle(truckOnly.begin(), truckOnly.end(), rng);

    for (int node : truckOnly) {
        insertBest(sol, node);
    }

    // 4) 优化卡车路径（只会动 truck_route，不动你的 served 标志）
    utils.optimizeTruckRouteWithLKHIntern(sol);

    // 5) 评估（内部会重建 drone_assignments/node_to_drone）
    utils.evaluateSolution(sol, /*needCalDrone=*/true);

    return sol;
}







TSPDSSolution InitialSolutionGenerator::generateInitialSolutionBackboneInsert() {
    TSPDSSolution sol = generateInitialSolution_DroneMain();
    utils.evaluateSolution(sol, /*needCalDrone=*/true);

    const int depot = graph.depot;
    const int station = graph.drone_station;
    const int n = static_cast<int>(graph.nodes.size());
    const double EPS = 1e-6;

    auto isDroneEligible = [&](int node) -> bool {
        if (node == depot || node == station) return false;
        return graph.is_drone_eligible.empty() || graph.is_drone_eligible[node];
    };

    auto bestInsertion = [&](const std::vector<int>& route, int node) {
        double bestInc = std::numeric_limits<double>::infinity();
        int bestPos = -1;
        for (int i = 0; i + 1 < static_cast<int>(route.size()); ++i) {
            int a = route[i], b = route[i + 1];
            double inc = graph.truck_time[a][node] + graph.truck_time[node][b] - graph.truck_time[a][b];
            if (inc < bestInc) {
                bestInc = inc;
                bestPos = i + 1;
            }
        }
        return std::make_pair(bestInc, bestPos);
    };

    auto roundTrip = [&](int node) -> double {
        return graph.drone_time[station][node] + graph.drone_time[node][station];
    };

    double incumbent = sol.makespan;
    const int maxPasses = 2;
    for (int pass = 0; pass < maxPasses; ++pass) {
        struct Candidate { int node; double score; double insertCost; double droneTime; };
        std::vector<Candidate> candidates;
        candidates.reserve(n);

        for (int node = 0; node < n; ++node) {
            if (!isDroneEligible(node)) continue;
            if (node >= static_cast<int>(sol.served_by_drone.size()) || !sol.served_by_drone[node]) continue;
            auto [insertCost, pos] = bestInsertion(sol.truck_route, node);
            if (pos < 1 || pos > static_cast<int>(sol.truck_route.size()) - 1) continue;
            double droneTime = std::max(EPS, roundTrip(node));
            candidates.push_back({node, insertCost / droneTime, insertCost, droneTime});
        }

        if (candidates.empty()) break;
        std::sort(candidates.begin(), candidates.end(), [](const Candidate& a, const Candidate& b) {
            if (std::fabs(a.score - b.score) > 1e-12) return a.score < b.score;
            return a.insertCost < b.insertCost;
        });

        int limit = std::min(params.backbone_insert_candidates, static_cast<int>(candidates.size()));
        bool improved = false;
        for (int idx = 0; idx < limit; ++idx) {
            int node = candidates[idx].node;
            if (node >= static_cast<int>(sol.served_by_drone.size()) || !sol.served_by_drone[node]) continue;

            auto [insertCost, pos] = bestInsertion(sol.truck_route, node);
            if (pos < 1 || pos > static_cast<int>(sol.truck_route.size()) - 1) continue;

            TSPDSSolution trial = sol;
            trial.truck_route.insert(trial.truck_route.begin() + pos, node);
            trial.served_by_truck[node] = true;
            trial.served_by_drone[node] = false;
            trial.drone_assignments.clear();
            trial.node_to_drone.clear();
            for (int d = 0; d < graph.drone_count; ++d) trial.drone_assignments[d] = std::vector<int>();
            utils.evaluateSolution(trial, /*needCalDrone=*/true);

            if (trial.makespan + EPS < incumbent) {
                sol = std::move(trial);
                incumbent = sol.makespan;
                improved = true;
            }
        }
        if (!improved) break;
    }

    utils.optimizeTruckRouteWithLKHIntern(sol);
    utils.evaluateSolution(sol, /*needCalDrone=*/true);
    return sol;
}



TSPDSSolution InitialSolutionGenerator::generateInitialSolutionTargetCapacity() {
    if (params.target_value <= 0.0 || !std::isfinite(params.target_value)) {
        return generateInitialSolutionBackboneInsert();
    }

    TSPDSSolution sol = generateInitialSolution_DroneMain();
    utils.evaluateSolution(sol, /*needCalDrone=*/true);
    const double target = params.target_value;
    const int n = static_cast<int>(graph.nodes.size());
    const int m = std::max(1, graph.drone_count);
    const double EPS = 1e-6;

    auto scoreTarget = [&](const TSPDSSolution& s) {
        std::vector<double> loads(m, 0.0);
        double total = 0.0;
        double maxJob = 0.0;
        for (const auto& kv : s.drone_assignments) {
            int d = kv.first;
            if (d < 0 || d >= m) continue;
            for (int node : kv.second) {
                double p = 2.0 * graph.drone_time[graph.drone_station][node];
                loads[d] += p;
                total += p;
                maxJob = std::max(maxJob, p);
            }
        }
        const double cap = target - s.station_activation_time;
        double phi = std::max(0.0, s.truck_completion_time - target);
        for (double load : loads) phi += std::max(0.0, load - cap);
        phi += std::max(0.0, total - static_cast<double>(m) * cap);
        phi += std::max(0.0, maxJob - cap);
        return phi;
    };

    double bestPhi = scoreTarget(sol);
    for (int iter = 0; iter < n; ++iter) {
        struct MoveCand {
            TSPDSSolution sol;
            double phi;
            double makespan;
        };
        std::vector<MoveCand> moves;

        utils.buildPosInTruck(sol);
        for (int node = 0; node < n; ++node) {
            if (node == graph.depot || node == graph.drone_station) continue;
            if (node >= static_cast<int>(sol.served_by_drone.size()) || !sol.served_by_drone[node]) continue;
            int pos = -1;
            double inc = utils.estimateBestInsertionIncrease(sol, node, InsertPolicy::ANYWHERE, pos);
            if (!std::isfinite(inc) || pos < 0) continue;

            TSPDSSolution trial = sol;
            if (!utils.insertNodeToTruckRoute(trial, node, InsertPolicy::ANYWHERE)) continue;
            trial.served_by_truck[node] = true;
            trial.served_by_drone[node] = false;
            trial.drone_assignments.clear();
            trial.node_to_drone.clear();
            for (int d = 0; d < graph.drone_count; ++d) trial.drone_assignments[d] = std::vector<int>();
            utils.evaluateSolution(trial, /*needCalDrone=*/true);

            double phi = scoreTarget(trial);
            if (phi < bestPhi - EPS ||
                (std::fabs(phi - bestPhi) <= EPS && trial.makespan < sol.makespan - EPS)) {
                moves.push_back({std::move(trial), phi, moves.empty() ? 0.0 : 0.0});
                moves.back().makespan = moves.back().sol.makespan;
            }
        }

        if (moves.empty()) break;
        std::sort(moves.begin(), moves.end(), [](const MoveCand& a, const MoveCand& b) {
            if (std::fabs(a.phi - b.phi) > 1e-9) return a.phi < b.phi;
            return a.makespan < b.makespan;
        });
        int top = std::min(params.target_seed_random_top, static_cast<int>(moves.size()));
        std::uniform_int_distribution<int> pick(0, top - 1);
        int chosen = (top > 1) ? pick(rng) : 0;
        sol = std::move(moves[chosen].sol);
        bestPhi = moves[chosen].phi;
        if (bestPhi <= EPS && sol.makespan <= target + EPS) break;
    }

    utils.optimizeTruckRouteWithLKHIntern(sol);
    utils.evaluateSolution(sol, /*needCalDrone=*/true);
    return sol;
}


TSPDSSolution InitialSolutionGenerator::generateInitialSolutionPro() {
    TSPDSSolution solution;
    solution.initialize(graph.nodes.size());

    cout << "Generating improved initial solution with drone-first strategy..." << endl;

    // 1. 计算每个可无人机服务节点到无人机站的距离
    vector<pair<int, double>> nodeDistances;
    for (int i = 0; i < (int)graph.nodes.size(); ++i) {
        if (i != graph.depot && i != graph.drone_station && graph.is_drone_eligible[i]) {
            double distance = graph.drone_time[graph.drone_station][i];
            nodeDistances.push_back({ i, distance });
        }
    }

    sort(nodeDistances.begin(), nodeDistances.end(),
        [](const auto& a, const auto& b) {
            return a.second < b.second;
        });

    int maxDroneNodes = std::min((int)nodeDistances.size(), params.drone_node_k_number);

    vector<bool> isDroneNode(graph.nodes.size(), false);
    vector<int> droneNodes;

    for (int i = 0; i < maxDroneNodes; ++i) {
        int node = nodeDistances[i].first;
        isDroneNode[node] = true;
        droneNodes.push_back(node);
        solution.served_by_drone[node] = true;
        solution.served_by_truck[node] = false;
    }

    // 4. 构建卡车路径：尽早访问无人机站
    vector<int> truckNodes;
    vector<bool> visited(graph.nodes.size(), false);

    truckNodes.push_back(graph.depot);
    visited[graph.depot] = true;

    // 尽早加入 station
    if (!visited[graph.drone_station]) {
        truckNodes.push_back(graph.drone_station);
        visited[graph.drone_station] = true;
    }

    for (int i = 0; i < (int)graph.nodes.size(); ++i) {
        if (!visited[i] && !isDroneNode[i] && i != graph.depot && i != graph.drone_station) {
            truckNodes.push_back(i);
            visited[i] = true;
        }
    }

    // 5. 最近邻 + station 第二位
    vector<int> optimizedTruckRoute;
    optimizedTruckRoute.push_back(graph.depot);
    visited.assign(graph.nodes.size(), false);
    visited[graph.depot] = true;

    optimizedTruckRoute.push_back(graph.drone_station);
    visited[graph.drone_station] = true;

    int current = graph.drone_station;
    while (optimizedTruckRoute.size() < truckNodes.size()) {
        double minDist = numeric_limits<double>::max();
        int nextNode = -1;

        for (int node : truckNodes) {
            if (!visited[node] && graph.truck_time[current][node] < minDist) {
                minDist = graph.truck_time[current][node];
                nextNode = node;
            }
        }

        if (nextNode == -1) break;

        optimizedTruckRoute.push_back(nextNode);
        visited[nextNode] = true;
        current = nextNode;
    }

    optimizedTruckRoute.push_back(graph.depot);
    solution.truck_route = optimizedTruckRoute;

    for (int node : solution.truck_route) {
        solution.served_by_truck[node] = true;
    }
    solution.served_by_truck[graph.drone_station] = true;

    utils.evaluateSolution(solution, /*needCalDrone=*/true);

    // 输出略，同你原来
    return solution;
}


TSPDSSolution InitialSolutionGenerator::generateInitialSolutionSmart()
{
    TSPDSSolution solution;
    int n = graph.nodes.size();
    solution.initialize(n);

    const int depot = graph.depot;
    const int station = graph.drone_station;

    // -------------------- 1. 分类节点 --------------------
    vector<int> truckMandatory;
    vector<int> droneCandidates;

    truckMandatory.push_back(depot);

    for (int i = 0; i < n; ++i) {
        if (i == depot) continue;
        if (!graph.is_drone_eligible[i] || i == station)
            truckMandatory.push_back(i);
        else
            droneCandidates.push_back(i);
    }

    // -------------------- 2. 预选一批适合无人机的节点 --------------------
    struct Score { int node; double s; };
    vector<Score> scores;

    for (int node : droneCandidates) {
        double truckCost = graph.truck_time[depot][node] + graph.truck_time[node][depot];
        double droneCost = 2.0 * graph.drone_time[station][node];
        scores.push_back({ node, truckCost - droneCost });
    }

    sort(scores.begin(), scores.end(),
        [](auto& a, auto& b) { return a.s > b.s; });

    int K = min(params.drone_node_k_number, (int)scores.size());
    vector<bool> isDrone(n, false);

    for (int i = 0; i < K; ++i) {
        int node = scores[i].node;
        isDrone[node] = true;
        solution.served_by_drone[node] = true;
    }

    // 将 remaining drone candidates 作为 truck 访问对象
    for (auto& sc : scores) {
        if (!isDrone[sc.node])
            truckMandatory.push_back(sc.node);
    }

    // -------------------- 3. 构建 truck route (先 depot -> depot 基架) --------------------
    vector<int> route = { depot, depot }; // end depot is placeholder

    auto insertCheapest = [&](int node) {
        double bestDelta = 1e18;
        int bestPos = -1;

        for (int i = 0; i < (int)route.size() - 1; ++i) {
            int u = route[i];
            int v = route[i + 1];
            double oldCost = graph.truck_time[u][v];
            double newCost = graph.truck_time[u][node] + graph.truck_time[node][v];
            double delta = newCost - oldCost;

            if (delta < bestDelta) {
                bestDelta = delta;
                bestPos = i + 1;
            }
        }
        route.insert(route.begin() + bestPos, node);
        };

    // -------------------- 4. 先插入 station（智能位置，而不是固定第二个） --------------------
    insertCheapest(station);

    // -------------------- 5. 插入其它 truckMandatory 节点 --------------------
    for (int node : truckMandatory) {
        if (node == depot || node == station) continue;
        insertCheapest(node);
    }

    solution.truck_route = route;

    // -------------------- 6. 标记卡车服务 --------------------
    for (int v : route) {
        solution.served_by_truck[v] = true;
        solution.served_by_drone[v] = false;  // 优先 truck
    }

    // 恢复无人机服务形式
    for (int node = 0; node < n; ++node) {
        if (isDrone[node]) {
            solution.served_by_truck[node] = false;
            solution.served_by_drone[node] = true;
        }
    }

    // -------------------- 7. 调度无人机 & 评估 --------------------
    utils.evaluateSolution(solution, true);

    cout << "Smart initial solution built. Makespan = "
        << solution.makespan << endl;

    return solution;
}


