// CplexF2Solver.cpp
#include "CplexF2Solver.h"
#include <iostream>
#include <algorithm>
#include <numeric>
#include <vector>

ILOSTLBEGIN

struct GcsCand {
    std::vector<char> inU;  // size nMinus
    int k;
    double vio;
};




// -------------------------
// Tarjan SCC (0..nMinus-1)
// -------------------------
struct TarjanSCC {
    int n;
    const std::vector<std::vector<int>>& adj;
    std::vector<int> disc, low, st;
    std::vector<char> inSt;
    int timer = 0;
    std::vector<std::vector<int>> comps;

    TarjanSCC(int n_, const std::vector<std::vector<int>>& a)
        : n(n_), adj(a), disc(n_, -1), low(n_, -1), inSt(n_, 0) {
    }

    void dfs(int u) {
        disc[u] = low[u] = timer++;
        st.push_back(u);
        inSt[u] = 1;

        for (int v : adj[u]) {
            if (disc[v] == -1) {
                dfs(v);
                low[u] = std::min(low[u], low[v]);
            }
            else if (inSt[v]) {
                low[u] = std::min(low[u], disc[v]);
            }
        }

        if (low[u] == disc[u]) {
            std::vector<int> comp;
            while (true) {
                int w = st.back();
                st.pop_back();
                inSt[w] = 0;
                comp.push_back(w);
                if (w == u) break;
            }
            comps.push_back(std::move(comp));
        }
    }

    std::vector<std::vector<int>> run() {
        for (int i = 0; i < n; ++i)
            if (disc[i] == -1) dfs(i);
        return comps;
    }
};

// forward declare callbacks (so helper can use CB having addCutPublic)
class GCSUserCutCallback;
class GCSLazyCallback;

// -------------------------
// Build and add a single GCS cut for (U, k):
//   sum_{i in U} sum_{j notin U} x[i][j] >= sum_{j != k, j != depot} x[k][j]
// implemented as:  sum_{i in U, j notin U} x[i][j] - sum_{j != k, j != depot} x[k][j] >= 0
//
// IMPORTANT: We CANNOT call cb->add(...) here because add() is protected in CPLEX callbacks.
// So CB must provide a public wrapper: addCutPublic(IloRange).
// -------------------------
template<class CB>
static void addGcsCut(
    CB* cb,
    const IloArray<IloNumVarArray>& x,
    const std::vector<char>& inU,
    int k,
    int n,       // total nodes incl endDepot
    int nMinus,  // 0..nMinus-1 can depart (endDepot excluded)
    int depot
) {
    IloEnv env = cb->getEnv();
    IloExpr lhs(env);

    // delta+(U)
    for (int i = 0; i < nMinus; ++i) {
        if (!inU[i]) continue;
        for (int j = 0; j < n; ++j) {
            if (j == depot) continue;                    // depot不可入
            if (j < nMinus && inU[j]) continue;          // j in U
            if (i == j) continue;
            lhs += x[i][j];
        }
    }

    // - outdeg(k)
    for (int j = 0; j < n; ++j) {
        if (j == depot) continue;
        if (j == k) continue;
        lhs -= x[k][j];
    }

    IloRange cut(lhs >= 0.0);
    cb->addCutPublic(cut);   // ✅ public wrapper, not protected add()
    cut.end();
    lhs.end();
}

// -------------------------
// Common separation: find SCCs in support graph of xbar,
// add violated cuts for components not containing depot, size>=2
// -------------------------


template<class CB, typename GetValFn>
static int separateGcsCuts(
    CB* cb,
    const IloArray<IloNumVarArray>& x,
    int n, int nMinus, int depot,
    double cut_eps, double vio_eps,
    int maxCuts,
    GetValFn getXVal
) {
    // 1) build support digraph on nodes 0..nMinus-1
    std::vector<std::vector<int>> adj(nMinus);
    for (int i = 0; i < nMinus; ++i) {
        for (int j = 0; j < n; ++j) {
            if (j == depot) continue;
            if (i == j) continue;
            double v = getXVal(i, j);
            if (v > cut_eps) {
                // if j is endDepot (==nMinus), ignore in SCC graph
                if (j < nMinus) adj[i].push_back(j);
            }
        }
    }

    TarjanSCC tarjan(nMinus, adj);
    auto comps = tarjan.run();

    int added = 0;

    // outdeg(i)
    std::vector<double> outdeg(nMinus, 0.0);
    for (int i = 0; i < nMinus; ++i) {
        double s = 0.0;
        for (int j = 0; j < n; ++j) {
            if (j == depot) continue;
            if (i == j) continue;
            s += getXVal(i, j);
        }
        outdeg[i] = s;
    }

    for (auto& comp : comps) {
        if ((int)comp.size() < 2) continue;

        bool hasDepot = false;
        for (int v : comp) if (v == depot) { hasDepot = true; break; }
        if (hasDepot) continue;

        std::vector<char> inU(nMinus, 0);
        for (int v : comp) inU[v] = 1;

        double deltaPlus = 0.0;
        for (int i = 0; i < nMinus; ++i) if (inU[i]) {
            for (int j = 0; j < n; ++j) {
                if (j == depot) continue;
                if (i == j) continue;
                if (j < nMinus && inU[j]) continue;
                deltaPlus += getXVal(i, j);
            }
        }

        // choose most violated k in U
        int bestK = -1;
        double bestVio = 0.0;
        for (int k : comp) {
            double vio = outdeg[k] - deltaPlus; // should be <= 0
            if (vio > bestVio) {
                bestVio = vio;
                bestK = k;
            }
        }

        if (bestK >= 0 && bestVio > vio_eps) {
            addGcsCut(cb, x, inU, bestK, n, nMinus, depot);
            if (++added >= maxCuts) break;
        }
    }

    return added;
}

template<class CB, typename GetValFn>
static int separateGcsCutsStrongComp(
    CB* cb,
    const IloArray<IloNumVarArray>& x,
    int n, int nMinus, int depot,
    double cut_eps, double vio_eps,
    int maxCuts,
    GetValFn getXVal
) {
    // 1) build support digraph on 0..nMinus-1 (ignore endDepot in SCC graph)
    std::vector<std::vector<int>> adj(nMinus);
    for (int i = 0; i < nMinus; ++i) {
        for (int j = 0; j < n; ++j) {
            if (j == depot) continue;
            if (i == j) continue;
            double v = getXVal(i, j);
            if (v > cut_eps) {
                if (j < nMinus) adj[i].push_back(j);
            }
        }
    }

    TarjanSCC tarjan(nMinus, adj);
    auto comps = tarjan.run();

    // 2) precompute outdeg(i) = sum_{j != depot, j != i} x[i][j]
    std::vector<double> outdeg(nMinus, 0.0);
    for (int i = 0; i < nMinus; ++i) {
        double s = 0.0;
        for (int j = 0; j < n; ++j) {
            if (j == depot) continue;
            if (i == j) continue;
            s += getXVal(i, j);
        }
        outdeg[i] = s;
    }

    // 3) collect all violated (U,k) from SCCs
    std::vector<GcsCand> cands;
    cands.reserve(maxCuts * 4);

    for (auto& comp : comps) {
        if ((int)comp.size() < 2) continue;

        bool hasDepot = false;
        for (int v : comp) if (v == depot) { hasDepot = true; break; }
        if (hasDepot) continue;

        std::vector<char> inU(nMinus, 0);
        for (int v : comp) inU[v] = 1;

        // deltaPlus(U) = sum_{i in U} sum_{j notin U, j != depot} x[i][j]
        double deltaPlus = 0.0;
        for (int i = 0; i < nMinus; ++i) if (inU[i]) {
            for (int j = 0; j < n; ++j) {
                if (j == depot) continue;
                if (i == j) continue;
                if (j < nMinus && inU[j]) continue; // j in U
                deltaPlus += getXVal(i, j);
            }
        }

        // For all k in U, check violation: outdeg(k) - deltaPlus(U)
        // If > 0 (by eps), add candidate.
        for (int k : comp) {
            double vio = outdeg[k] - deltaPlus;
            if (vio > vio_eps) {
                cands.push_back(GcsCand{ inU, k, vio });
            }
        }
    }

    if (cands.empty()) return 0;

    // 4) pick first ν maximally violated
    std::sort(cands.begin(), cands.end(),
        [](const GcsCand& a, const GcsCand& b) { return a.vio > b.vio; });

    int added = 0;
    for (const auto& cand : cands) {
        addGcsCut(cb, x, cand.inU, cand.k, n, nMinus, depot);
        if (++added >= maxCuts) break;
    }
    return added;
}

// -------------------------
// UserCut callback (fractional)
// -------------------------
class GCSUserCutCallback : public IloCplex::UserCutCallbackI {
public:
    GCSUserCutCallback(
        IloEnv env,
        const IloArray<IloNumVarArray>& x_,
        int n_, int nMinus_, int depot_,
        double cut_eps_, double vio_eps_, int maxCuts_
    )
        : IloCplex::UserCutCallbackI(env),
        x(x_), n(n_), nMinus(nMinus_), depot(depot_),
        cut_eps(cut_eps_), vio_eps(vio_eps_), maxCuts(maxCuts_) {
    }

    // ✅ public wrapper to call protected add()
    void addCutPublic(const IloRange& rng) {
        this->add(rng); // protected in base, but accessible here (member)
    }

    void main() override {
        if (isAfterCutLoop()) return;

        auto getXVal = [&](int i, int j) -> double {
            return getValue(x[i][j]);
            };

        separateGcsCutsStrongComp(this, x, n, nMinus, depot, cut_eps, vio_eps, maxCuts, getXVal);
    }

    IloCplex::CallbackI* duplicateCallback() const override {
        return (new (getEnv()) GCSUserCutCallback(
            getEnv(), x, n, nMinus, depot, cut_eps, vio_eps, maxCuts));
    }

private:
    const IloArray<IloNumVarArray>& x;
    int n, nMinus, depot;
    double cut_eps, vio_eps;
    int maxCuts;
};

// -------------------------
// Lazy callback (integer feasible incumbents)
// -------------------------
class GCSLazyCallback : public IloCplex::LazyConstraintCallbackI {
public:
    GCSLazyCallback(
        IloEnv env,
        const IloArray<IloNumVarArray>& x_,
        int n_, int nMinus_, int depot_,
        double cut_eps_, double vio_eps_, int maxCuts_
    )
        : IloCplex::LazyConstraintCallbackI(env),
        x(x_), n(n_), nMinus(nMinus_), depot(depot_),
        cut_eps(cut_eps_), vio_eps(vio_eps_), maxCuts(maxCuts_) {
    }

    // ✅ public wrapper to call protected add()
    void addCutPublic(const IloRange& rng) {
        this->add(rng); // protected in base, but accessible here (member)
    }

    void main() override {
        auto getXVal = [&](int i, int j) -> double {
            return getValue(x[i][j]);
            };

        separateGcsCutsStrongComp(this, x, n, nMinus, depot, cut_eps, vio_eps, maxCuts, getXVal);
    }

    IloCplex::CallbackI* duplicateCallback() const override {
        return (new (getEnv()) GCSLazyCallback(
            getEnv(), x, n, nMinus, depot, cut_eps, vio_eps, maxCuts));
    }

private:
    const IloArray<IloNumVarArray>& x;
    int n, nMinus, depot;
    double cut_eps, vio_eps;
    int maxCuts;
};

// -------------------------
// CplexF2Solver::solve
// -------------------------
bool CplexF2Solver::solve(const TSPDSGraph& g, TSPDSSolution& sol) {
    const int nOrig = (int)g.truck_time.size();
    if (nOrig <= 2) return false;

    const int depot = g.depot;
    const int s = g.drone_station;
    const int V = std::max(1, g.drone_count);

    const int endDepot = nOrig;   // copy end node
    const int n = nOrig + 1;      // total nodes
    const int nMinus = nOrig;     // nodes that can depart (0..nOrig-1)

    auto tau = [&](int i, int j) -> double {
        if (i == endDepot) return 0.0;
        if (j == endDepot) return g.truck_time[i][depot];
        return g.truck_time[i][j];
        };
    auto rt = [&](int i) -> double {
        return g.drone_time[s][i] + g.drone_time[i][s];
        };

    IloEnv env;
    try {
        IloModel model(env);

        // variables: x (binary), xs (continuous), y (binary), T
        IloArray<IloNumVarArray> x(env, n);
        IloArray<IloNumVarArray> xs(env, n);
        for (int i = 0; i < n; ++i) {
            x[i] = IloNumVarArray(env, n);
            xs[i] = IloNumVarArray(env, n);
            for (int j = 0; j < n; ++j) {
                bool invalidX = false;
                if (i == j) invalidX = true;
                if (j == depot) invalidX = true;     // x: depot不可入
                if (i == endDepot) invalidX = true;  // x: endDepot不可出

                bool invalidXS = invalidX;
                // xs 更严格：前缀路径不能进 endDepot，也不能从 station 再出去
                if (j == endDepot) invalidXS = true; // xs: 不进入 endDepot（前缀到站前不可能到终点）
                if (i == s)        invalidXS = true; // xs: station 出边为 0（到站即终止）

                // build x
                if (invalidX) x[i][j] = IloNumVar(env, 0.0, 0.0, ILOBOOL);
                else          x[i][j] = IloNumVar(env, 0.0, 1.0, ILOBOOL);

                // build xs
                if (invalidXS) xs[i][j] = IloNumVar(env, 0.0, 0.0, ILOFLOAT);
                else           xs[i][j] = IloNumVar(env, 0.0, 1.0, ILOFLOAT);

            }
        }

        IloArray<IloNumVarArray> y(env, nOrig);
        for (int i = 0; i < nOrig; ++i) {
            y[i] = IloNumVarArray(env, V);
            for (int v = 0; v < V; ++v) {
                bool eligible = false;
                if (i != depot && i != s &&
                    (int)g.is_drone_eligible.size() == nOrig &&
                    g.is_drone_eligible[i]) {
                    eligible = true;
                }
                if (!eligible) y[i][v] = IloNumVar(env, 0.0, 0.0, ILOBOOL);
                else           y[i][v] = IloNumVar(env, 0.0, 1.0, ILOBOOL);
            }
        }

        IloNumVar T(env, 0.0, IloInfinity, ILOFLOAT);
        model.add(IloMinimize(env, T));

        // depot out = 1
        {
            IloExpr out0(env);
            for (int j = 0; j < n; ++j) out0 += x[depot][j];
            model.add(out0 == 1);
            out0.end();
        }
        // endDepot in = 1
        {
            IloExpr inEnd(env);
            for (int i = 0; i < n; ++i) inEnd += x[i][endDepot];
            model.add(inEnd == 1);
            inEnd.end();
        }
        // station must be visited by truck
        {
            IloExpr outS(env);
            for (int j = 0; j < n; ++j) outS += x[s][j];
            model.add(outS == 1);
            outS.end();
        }

        // each customer served exactly once (exclude depot and station)
        for (int i = 0; i < nOrig; ++i) {
            if (i == depot) continue;
            if (i == s) continue;

            bool isEligible = ((int)g.is_drone_eligible.size() == nOrig && g.is_drone_eligible[i]);

            IloExpr lhs(env);
            for (int j = 0; j < n; ++j) lhs += x[i][j];
            if (isEligible) {
                for (int v = 0; v < V; ++v) lhs += y[i][v];
            }
            model.add(lhs == 1);
            lhs.end();
        }

        // flow conservation on x for all original nodes except depot
        for (int i = 0; i < nOrig; ++i) {
            if (i == depot) continue;
            IloExpr out(env), in(env);
            for (int j = 0; j < n; ++j) out += x[i][j];
            for (int j = 0; j < n; ++j) in += x[j][i];
            model.add(out == in);
            out.end(); in.end();
        }

        // (19) xs <= x
        for (int i = 0; i < nMinus; ++i) {
    if (i == s) continue; // xs station 出边已固定，不需要约束
    for (int j = 0; j < n; ++j) {
        if (i == j) continue;
        if (j == depot) continue;
        if (j == endDepot) continue; // xs 不允许进入 endDepot
        if (i == endDepot) continue;
        model.add(xs[i][j] <= x[i][j]);
    }
}


        // (20) unit flow for xs from depot to station
        for (int i = 0; i < n; ++i) {
            IloExpr out(env), in(env);
            for (int j = 0; j < n; ++j) out += xs[i][j];
            for (int j = 0; j < n; ++j) in += xs[j][i];

            int b = 0;
            if (i == depot) b = 1;
            else if (i == s) b = -1;

            model.add(out - in == b);
            out.end(); in.end();
        }

        // activation = sum tau * xs
        IloExpr activation(env);
        for (int i = 0; i < nMinus; ++i) {
            if (i == s) continue; // station 出边为 0，跳过更干净
            for (int j = 0; j < n; ++j) {
                if (i == j || j == depot || j == endDepot || i == endDepot) continue;
                activation += tau(i, j) * xs[i][j];
            }
        }


        // (22) T >= activation + load_v
        for (int v = 0; v < V; ++v) {
            IloExpr rhs(env);
            rhs += activation;
            for (int i = 0; i < nOrig; ++i) {
                if (i == depot || i == s) continue;
                rhs += rt(i) * y[i][v];
            }
            model.add(T >= rhs);
            rhs.end();
        }
        activation.end();

        // (23) T >= sum tau * x
        {
            IloExpr truckTime(env);
            for (int i = 0; i < nMinus; ++i) {
                for (int j = 0; j < n; ++j) {
                    if (i == j || j == depot || i == endDepot) continue;
                    truckTime += tau(i, j) * x[i][j];
                }
            }
            model.add(T >= truckTime);
            truckTime.end();
        }

        // Solve with callbacks for GCS cuts (21)
        IloCplex cplex(model);

        // legacy callbacks need Traditional search
        cplex.setParam(IloCplex::Param::MIP::Strategy::Search, IloCplex::Traditional);

        cplex.setOut(params.verbose ? std::cout : env.getNullStream());
        if (params.time_limit_sec > 0) cplex.setParam(IloCplex::TiLim, params.time_limit_sec);
        if (params.threads > 0)        cplex.setParam(IloCplex::Threads, params.threads);
        if (params.mip_gap >= 0)       cplex.setParam(IloCplex::EpGap, params.mip_gap);

        cplex.use(new (env) GCSUserCutCallback(env, x, n, nMinus, depot,
            params.cut_eps, params.vio_eps, params.max_cuts_per_call));
        cplex.use(new (env) GCSLazyCallback(env, x, n, nMinus, depot,
            params.cut_eps, params.vio_eps, params.max_cuts_per_call));

        bool ok = cplex.solve();
        if (!ok) {
            std::cerr << "[F2] CPLEX solve failed. status=" << cplex.getStatus() << "\n";
            env.end();
            return false;
        }

        // fill solution
        sol = TSPDSSolution();
        sol.initialize(nOrig);
        sol.makespan = cplex.getValue(T);

        // activation time = sum tau * xs
        double act = 0.0;
        for (int i = 0; i < nMinus; ++i) {
            for (int j = 0; j < n; ++j) {
                if (i == j || j == depot || i == endDepot) continue;
                double v = cplex.getValue(xs[i][j]);
                if (v > 1e-9) act += tau(i, j) * v;
            }
        }
        sol.station_activation_time = act;

        // truck completion time = sum tau * x
        double ttruck = 0.0;
        for (int i = 0; i < nMinus; ++i) {
            for (int j = 0; j < n; ++j) {
                if (i == j || j == depot || i == endDepot) continue;
                double v = cplex.getValue(x[i][j]);
                if (v > 0.5) ttruck += tau(i, j);
            }
        }
        sol.truck_completion_time = ttruck;

        // drone completion time = max_v (act + sum rt*y)
        double droneFinish = 0.0;
        for (int v = 0; v < V; ++v) {
            double load = 0.0;
            for (int i = 0; i < nOrig; ++i) {
                if (i == depot || i == s) continue;
                if (cplex.getValue(y[i][v]) > 0.5) load += rt(i);
            }
            droneFinish = std::max(droneFinish, act + load);
        }
        sol.drone_completion_time = droneFinish;

        // reconstruct truck_route
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
            if (nxt < 0) break;
            if (nxt == endDepot) break;
            route.push_back(nxt);
            cur = nxt;
            if (visited[cur]) break;
        }
        route.push_back(depot);
        sol.truck_route = route;

        for (int node : sol.truck_route) {
            if (node >= 0 && node < nOrig) sol.served_by_truck[node] = true;
        }

        for (int v = 0; v < V; ++v) {
            for (int i = 0; i < nOrig; ++i) {
                if (i == depot || i == s) continue;
                if (cplex.getValue(y[i][v]) > 0.5) {
                    sol.served_by_drone[i] = true;
                    sol.node_to_drone[i] = v;
                    sol.drone_assignments[v].push_back(i);
                }
            }
        }

        for (int idx = 0; idx < (int)sol.truck_route.size(); ++idx) {
            int node = sol.truck_route[idx];
            if (node >= 0 && node < nOrig) sol.pos_in_truck[node] = idx;
            if (node == s) sol.pos_station_in_truck = idx;
        }

        // ✅CPLEX 信息回填
        sol.cplex_best_integer = cplex.getObjValue();     // 最优可行整数解目标（若有）
        sol.cplex_best_bound = cplex.getBestObjValue(); // 全局下界 Best Bound（日志里的那个）
		sol.cplex_mip_gap = cplex.getMIPRelativeGap() * 100; // 百分制 相对 gap，百分比
        sol.cplex_status = (int)cplex.getStatus();

        env.end();
        return true;
    }
    catch (IloException& e) {
        std::cerr << "[F2] IloException: " << e.getMessage() << "\n";
        env.end();
        return false;
    }
    catch (...) {
        std::cerr << "[F2] Unknown exception.\n";
        env.end();
        return false;
    }
}
