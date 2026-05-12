// CplexF1Solver.cpp
#include "CplexF1Solver.h"

ILOSTLBEGIN


bool CplexF1Solver::solve(const TSPDSGraph& g, TSPDSSolution& sol) {

    // ---------- Logger ----------
    if (!logger.initializeLogging()) {
        std::cout << "日志记录已禁用" << std::endl;
    }
    logger.logAlgorithmStart();

    const int nOrig = (int)g.truck_time.size();
    if (nOrig <= 2) return false;

    const int depot = g.depot;          // 论文默认 depot=0
    if (depot != 0) {
        std::cerr << "[F1] 当前实现假设 depot=0\n";
        return false;
    }

    const int s = g.drone_station;
    if (s < 0 || s >= nOrig) {
        std::cerr << "[F1] drone_station 未设置或越界\n";
        return false;
    }

    const int V = std::max(1, g.drone_count);

    // 论文：终点复制节点 c+1
    const int endDepot = nOrig;
    const int n = nOrig + 1;

    // truck time tau(i,j)
    auto tau = [&](int i, int j) -> double {
        if (i == endDepot) return 0.0;                // 禁止 endDepot 出边
        if (j == endDepot) return g.truck_time[i][depot];
        if (j == depot)    return g.truck_time[i][depot];
        return g.truck_time[i][j];
        };

    // round-trip time for drone: s -> i -> s
    auto rt = [&](int i) -> double {
        return g.drone_time[s][i] + g.drone_time[i][s];
        };

    // Big-M
    const double maxTau = maxTruckTime(g);
    const double M = (maxTau + 1.0) * (nOrig + 2);

    // 顾客集合：除 depot 与 station 之外的所有原始节点
    auto isCustomer = [&](int i) -> bool {
        return (i >= 0 && i < nOrig && i != depot && i != s);
        };

    IloEnv env;
    try {
        IloModel model(env);

        // -------------------------
        // 变量
        // -------------------------
        // x[i][j] ∈ {0,1}
        IloArray<IloNumVarArray> x(env, n);
        for (int i = 0; i < n; ++i) {
            x[i] = IloNumVarArray(env, n);
            for (int j = 0; j < n; ++j) {
                bool invalid = false;

                if (i == j) invalid = true;          // 禁止自环
                if (j == depot) invalid = true;       // 禁止进入 depot
                if (i == endDepot) invalid = true;    // 禁止从 endDepot 出发

                if (invalid) x[i][j] = IloNumVar(env, 0.0, 0.0, ILOBOOL);
                else         x[i][j] = IloNumVar(env, 0.0, 1.0, ILOBOOL);
            }
        }

        // y[i][v] ∈ {0,1} 仅对“可无人机服务的顾客”开放；其他固定 0
        IloArray<IloNumVarArray> y(env, nOrig);
        for (int i = 0; i < nOrig; ++i) {
            y[i] = IloNumVarArray(env, V);
            for (int v = 0; v < V; ++v) {
                bool eligible = false;
                if (isCustomer(i) &&
                    g.is_drone_eligible.size() == (size_t)nOrig &&
                    g.is_drone_eligible[i]) {
                    eligible = true;
                }
                if (!eligible) y[i][v] = IloNumVar(env, 0.0, 0.0, ILOBOOL);
                else           y[i][v] = IloNumVar(env, 0.0, 1.0, ILOBOOL);
            }
        }

        // a[i] >= 0
        IloNumVarArray a(env, n, 0.0, IloInfinity, ILOFLOAT);

        // makespan
        IloNumVar T(env, 0.0, IloInfinity, ILOFLOAT);

        // 目标
        model.add(IloMinimize(env, T));

        // a[depot] = 0
        model.add(a[depot] == 0.0);

        // -------------------------
        // 约束：站点必须由卡车访问一次（论文中 station ∈ C'）
        // sum_j x[s][j] = 1
        // -------------------------
        {
            IloExpr outS(env);
            for (int j = 0; j < n; ++j) outS += x[s][j];
            model.add(outS == 1);
            outS.end();
        }

        // -------------------------
        // 关键总约束：每个顾客恰好服务一次（你要求的）
        // sum_j x[i][j] + sum_v y[i][v] = 1,  ∀顾客 i
        // 对 drone-eligible=false 的点，y 已固定为0 => 自动变成 sum_j x[i][j]=1
        // -------------------------
        for (int i = 0; i < nOrig; ++i) {
            if (!isCustomer(i)) continue;

            IloExpr lhs(env);
            for (int j = 0; j < n; ++j) lhs += x[i][j];
            for (int v = 0; v < V; ++v) lhs += y[i][v];
            model.add(lhs == 1);
            lhs.end();
        }

        // -------------------------
        // 流守恒：对所有原始节点（除 depot）：
        // sum_j x[i][j] = sum_j x[j][i]
        // 这样无人机服务的顾客会被逼成 out=in=0；卡车访问的会 out=in=1
        // -------------------------
        for (int i = 0; i < nOrig; ++i) {
            if (i == depot) continue;

            IloExpr out(env), in(env);
            for (int j = 0; j < n; ++j) out += x[i][j];
            for (int j = 0; j < n; ++j) in += x[j][i];
            model.add(out == in);
            out.end();
            in.end();
        }

        // -------------------------
        // 起点出度=1：sum_j x[depot][j] = 1
        // 终点复制节点入度=1：sum_i x[i][endDepot] = 1
        // -------------------------
        {
            IloExpr out0(env);
            for (int j = 0; j < n; ++j) out0 += x[depot][j];
            model.add(out0 == 1);
            out0.end();
        }
        {
            IloExpr inEnd(env);
            for (int i = 0; i < n; ++i) inEnd += x[i][endDepot];
            model.add(inEnd == 1);
            inEnd.end();
        }

        // -------------------------
        // MTZ 时间传播（论文 F1 的核心）
        // a[i] + tau(i,j) <= a[j] + M(1 - x[i][j])
        // -------------------------
        for (int i = 0; i < n; ++i) {
            if (i == endDepot) continue;
            for (int j = 0; j < n; ++j) {
                if (j == depot) continue;
                if (i == j) continue;
                model.add(a[i] + tau(i, j) <= a[j] + M * (1 - x[i][j]));
            }
        }

        // -------------------------
        // makespan >= 卡车完成时刻：T >= a[endDepot]
        // -------------------------
        model.add(T >= a[endDepot]);

        // -------------------------
        // makespan >= 站点激活时刻 + 每架无人机总任务时长
        // T >= a[s] + sum_i rt(i)*y[i][v], ∀v
        // -------------------------
        for (int v = 0; v < V; ++v) {
            IloExpr rhs(env);
            rhs += a[s];
            for (int i = 0; i < nOrig; ++i) {
                if (!isCustomer(i)) continue;
                // 如果不可无人机服务，y 已经是 0..0，不影响
                rhs += rt(i) * y[i][v];
            }
            model.add(T >= rhs);
            rhs.end();
        }

        // -------------------------
        // 求解
        // -------------------------
        IloCplex cplex(model);
        cplex.setOut(params.verbose ? std::cout : env.getNullStream());

        if (params.time_limit_sec > 0) cplex.setParam(IloCplex::TiLim, params.time_limit_sec);
        if (params.threads > 0)        cplex.setParam(IloCplex::Threads, params.threads);
        if (params.mip_gap >= 0)       cplex.setParam(IloCplex::EpGap, params.mip_gap);

        bool ok = cplex.solve();
        if (!ok) {
            std::cerr << "[F1] CPLEX solve failed. status=" << cplex.getStatus() << "\n";
            env.end();
            return false;
        }

        // -------------------------
        // 回填解
        // -------------------------
        sol = TSPDSSolution();
        sol.initialize(nOrig);

        sol.makespan = cplex.getValue(T);
        sol.station_activation_time = cplex.getValue(a[s]);
        sol.truck_completion_time = cplex.getValue(a[endDepot]);

        // 无人机完成时间（含激活）：max_v (a[s] + sum rt*y)
        double droneFinish = 0.0;
        for (int v = 0; v < V; ++v) {
            double load = 0.0;
            for (int i = 0; i < nOrig; ++i) {
                if (!isCustomer(i)) continue;
                if (cplex.getValue(y[i][v]) > 0.5) load += rt(i);
            }
            droneFinish = std::max(droneFinish, sol.station_activation_time + load);
        }
        sol.drone_completion_time = droneFinish;

        // truck_route：从 depot 跟随 x 到 endDepot
        std::vector<int> route;
        route.push_back(depot);
        int cur = depot;

        std::vector<int> visited(n, 0);
        while (cur != endDepot) {
            visited[cur] = 1;
            int nxt = -1;
            for (int j = 0; j < n; ++j) {
                if (cplex.getValue(x[cur][j]) > 0.5) { nxt = j; break; }
            }
            if (nxt < 0) {
                std::cerr << "[F1] cannot trace route from node " << cur << "\n";
                env.end();
                return false;
            }
            if (nxt == endDepot) break;
            route.push_back(nxt);
            cur = nxt;
            if (visited[cur]) break; // 防护
        }
        route.push_back(depot);
        sol.truck_route = route;

        // served_by_truck
        for (int node : sol.truck_route) {
            if (node >= 0 && node < nOrig) sol.served_by_truck[node] = true;
        }

        // served_by_drone / assignments
        for (int v = 0; v < V; ++v) {
            for (int i = 0; i < nOrig; ++i) {
                if (!isCustomer(i)) continue;
                if (cplex.getValue(y[i][v]) > 0.5) {
                    sol.served_by_drone[i] = true;
                    sol.node_to_drone[i] = v;
                    sol.drone_assignments[v].push_back(i);
                }
            }
        }

        // pos_in_truck / pos_station_in_truck
        for (int idx = 0; idx < (int)sol.truck_route.size(); ++idx) {
            int node = sol.truck_route[idx];
            if (node >= 0 && node < nOrig) sol.pos_in_truck[node] = idx;
            if (node == s) sol.pos_station_in_truck = idx;
        }

        env.end();
        logger.logIterationData(1, 0, sol, sol, 1.0, /*accepted=*/true);
        logger.logAlgorithmEnd(1, 0);
        return true;
    }
    catch (IloException& e) {
        std::cerr << "[F1] IloException: " << e.getMessage() << "\n";
        env.end();
        return false;
    }
    catch (...) {
        std::cerr << "[F1] Unknown exception.\n";
        env.end();
        return false;
    }
}
