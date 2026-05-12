// CplexBendersSolver.cpp
#include "CplexBendersSolver.h"
#include "TSPDSSolutionValidator.h"
#include <iostream>
#include <algorithm>
#include <numeric>
#include <vector>
#include <cmath>
#include <limits>

ILOSTLBEGIN

struct GcsCand {
    std::vector<char> inU;  // size nMinus
    int k;
    double vio;
};

static std::string cplexStatusName(IloAlgorithm::Status st) {
    switch (st) {
    case IloAlgorithm::Optimal:
        return "Optimal";
    case IloAlgorithm::Feasible:
        return "Feasible";
    case IloAlgorithm::Infeasible:
        return "Infeasible";
    case IloAlgorithm::Unbounded:
        return "Unbounded";
    case IloAlgorithm::InfeasibleOrUnbounded:
        return "InfeasibleOrUnbounded";
    case IloAlgorithm::Unknown:
        return "Unknown";
    case IloAlgorithm::Error:
        return "Error";
    default:
        return "Other";
    }
}


// =========================
// =========================

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

class GCSUserCutCallback;
class GCSLazyCallback;

template<class CB>
static void addGcsCut(
    CB* cb,
    const IloArray<IloNumVarArray>& x,
    const std::vector<char>& inU,
    int k,
    int n,
    int nMinus,
    int depot
) {
    IloEnv env = cb->getEnv();
    IloExpr lhs(env);

    // delta+(U)
    for (int i = 0; i < nMinus; ++i) {
        if (!inU[i]) continue;
        for (int j = 0; j < n; ++j) {
            if (j == depot) continue;
            if (j < nMinus && inU[j]) continue;
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
    cb->addCutPublic(cut);
    cut.end();
    lhs.end();
}

template<class CB, typename GetValFn>
static int separateGcsCuts(
    CB* cb,
    const IloArray<IloNumVarArray>& x,
    int n, int nMinus, int depot,
    double cut_eps, double vio_eps,
    int maxCuts,
    GetValFn getXVal
) {
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

    int added = 0;

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

        int bestK = -1;
        double bestVio = 0.0;
        for (int k : comp) {
            double vio = outdeg[k] - deltaPlus;
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

    void addCutPublic(const IloRange& rng) { this->add(rng); }

    void main() override {
        if (isAfterCutLoop()) return;

        auto getXVal = [&](int i, int j) -> double { return getValue(x[i][j]); };
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

    void addCutPublic(const IloRange& rng) { this->add(rng); }

    void main() override {
        auto getXVal = [&](int i, int j) -> double { return getValue(x[i][j]); };
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


class BestIncumbentTimeCallback : public IloCplex::IncumbentCallbackI {
public:
    BestIncumbentTimeCallback(IloEnv env, double* bestObj, double* bestTime)
        : IloCplex::IncumbentCallbackI(env), bestObj_(bestObj), bestTime_(bestTime) {}

    void main() override {
        // 这是“新 incumbent”被接受时触发
        double obj = getObjValue();
        if (bestObj_ && bestTime_) {
            // 更稳：只在更优时更新（有时同值也会触发）
            if (obj + 1e-9 < *bestObj_) {
                *bestObj_ = obj;
                *bestTime_ = getCplexTime();   // 记录找到该 incumbent 的时间
            }
        }
    }

    IloCplex::CallbackI* duplicateCallback() const override {
        return new (getEnv()) BestIncumbentTimeCallback(getEnv(), bestObj_, bestTime_);
    }

private:
    double* bestObj_;
    double* bestTime_;
};



// =========================
// MULTIFIT：这里用“FFD 可装箱判定 + 二分找最小容量”
// 返回一个可行 makespan 上界 W_h （满足 W* <= W_h）
// =========================
static double multifitUpperBoundFFD(std::vector<double> p, int m, int iters) {
    if (p.empty()) return 0.0;
    if (m <= 0) return IloInfinity;

    std::sort(p.begin(), p.end(), std::greater<double>());
    double LB = *std::max_element(p.begin(), p.end());
    double UB = std::accumulate(p.begin(), p.end(), 0.0);

    auto canPack = [&](double T) -> bool {
        std::vector<double> rem(m, T);
        for (double job : p) {
            bool placed = false;
            for (int k = 0; k < m; ++k) {
                if (rem[k] + 1e-12 >= job) {
                    rem[k] -= job;
                    placed = true;
                    break;
                }
            }
            if (!placed) return false;
        }
        return true;
        };

    for (int it = 0; it < iters; ++it) {
        double mid = 0.5 * (LB + UB);
        if (canPack(mid)) UB = mid;
        else LB = mid;
    }
    return UB;
}

// =========================
// Helpers: route time / normalize / removal update
// =========================
struct UBResult {
    bool ok = false;
    double obj = IloInfinity; // total truck time
    double t1 = 0.0;          // depot->station along route
    double t2 = 0.0;          // station->end(depot) along route
    std::vector<int> route;   // [depot, ..., depot] (closed in original-index space)
};

struct PrimalHeuristicResult {
    bool ok = false;
    std::vector<int> route;       // truck route after removals, [depot,...,depot]
    std::vector<int> droneNodes;  // D' : assigned to drones
    std::vector<int> z01;         // size nOrig, 1 truck, 0 drone (for eligible)
    std::vector<int> assignDrone; // size |droneNodes|, drone id for each (for yhat MIP start)
    double t1 = 0.0;
    double t2 = 0.0;
    double t3 = 0.0;              // MULTIFIT makespan on droneNodes
    double W = 0.0;              // max(t2, t3)
    double obj = IloInfinity;      // t1 + max(t2, t3)
};

// NOTE: g.truck_time is on original nodes only, endDepot is virtual.
static inline double tau_truck(const TSPDSGraph& g, int i, int j, int depot) {
    // i,j are original node indices
    // return truck time i->j (Manhattan int already)
    return g.truck_time[i][j];
}

// compute t1,t2 for a closed route [depot,...,depot]
static void computeT1T2_fromClosedRoute(
    const TSPDSGraph& g,
    const std::vector<int>& route,
    int depot, int station,
    double& t1, double& t2
) {
    t1 = 0.0; t2 = 0.0;
    if (route.size() < 2) return;

    int posS = -1;
    for (int k = 0; k < (int)route.size(); ++k) {
        if (route[k] == station) { posS = k; break; }
    }
    if (posS <= 0) {
        // station not found or depot==station: treat t1=inf to force fix
        t1 = IloInfinity; t2 = IloInfinity;
        return;
    }

    // t1: sum edges from 0..posS-1
    for (int k = 0; k < posS; ++k) {
        int a = route[k], b = route[k + 1];
        t1 += tau_truck(g, a, b, depot);
    }
    // t2: sum edges from posS..end-2
    for (int k = posS; k < (int)route.size() - 1; ++k) {
        int a = route[k], b = route[k + 1];
        t2 += tau_truck(g, a, b, depot);
    }
}

// if t1>t2, reverse the tour direction (keep start depot), then recompute
static void normalizeActivationEarlier(
    const TSPDSGraph& g,
    std::vector<int>& route,
    int depot, int station,
    double& t1, double& t2
) {
    computeT1T2_fromClosedRoute(g, route, depot, station, t1, t2);
    if (!(t1 > t2 + 1e-12)) return;

    // reverse internal nodes (excluding both depots)
    if (route.size() <= 2) return;
    std::vector<int> mid(route.begin() + 1, route.end() - 1);
    std::reverse(mid.begin(), mid.end());

    std::vector<int> newRoute;
    newRoute.reserve(route.size());
    newRoute.push_back(depot);
    newRoute.insert(newRoute.end(), mid.begin(), mid.end());
    newRoute.push_back(depot);

    route.swap(newRoute);
    computeT1T2_fromClosedRoute(g, route, depot, station, t1, t2);
}

// remove node i from route (must appear once, not depot/station), update t1/t2 based on where i lies
// returns false if i not found or removal breaks (shouldn't)
static bool removeNodeAndUpdateT1T2(
    const TSPDSGraph& g,
    std::vector<int>& route,
    int depot, int station,
    int nodeToRemove,
    double& t1, double& t2
) {
    int pos = -1;
    for (int k = 1; k < (int)route.size() - 1; ++k) {
        if (route[k] == nodeToRemove) { pos = k; break; }
    }
    if (pos < 0) return false;

    // find station position (before removal)
    int posS = -1;
    for (int k = 0; k < (int)route.size(); ++k) {
        if (route[k] == station) { posS = k; break; }
    }
    if (posS < 0) return false;

    int prev = route[pos - 1];
    int next = route[pos + 1];

    double delta = -tau_truck(g, prev, nodeToRemove, depot)
        - tau_truck(g, nodeToRemove, next, depot)
        + tau_truck(g, prev, next, depot);

    // if node is in prefix depot->station (i in R1), affect t1; else affect t2
    if (pos < posS) t1 += delta;
    else            t2 += delta;

    // erase node
    route.erase(route.begin() + pos);
    return true;
}

// =========================
// 1) solveUB_TSP_route(...)
// Truck-only UB: visit ALL customers by truck (depot start, must pass station, return depot)
// Return closed route [depot,...,depot] and its t1/t2 split.
// =========================
static UBResult solveUB_TSP_route(
    const TSPDSGraph& g,
    const CplexBendersSolver::Params& params,
    double time_limit_sec = 30.0
) {
    UBResult res;
    const int nOrig = (int)g.truck_time.size();
    if (nOrig <= 2) return res;

    const int depot = g.depot;
    const int s = g.drone_station;

    const int endDepot = nOrig;
    const int n = nOrig + 1;
    const int nMinus = nOrig;

    auto tau = [&](int i, int j) -> double {
        if (i == endDepot) return 0.0;
        if (j == endDepot) return g.truck_time[i][depot];
        return g.truck_time[i][j];
        };

    IloEnv env;
    try {
        IloModel model(env);

        IloArray<IloNumVarArray> x(env, n);
        for (int i = 0; i < n; ++i) {
            x[i] = IloNumVarArray(env, n);
            for (int j = 0; j < n; ++j) {
                bool invalid = false;
                if (i == j) invalid = true;
                if (j == depot) invalid = true;     // no enter depot
                if (i == endDepot) invalid = true;  // endDepot no outgoing
                x[i][j] = invalid ? IloNumVar(env, 0, 0, ILOBOOL)
                    : IloNumVar(env, 0, 1, ILOBOOL);
            }
        }

        // objective min sum tau*x
        IloExpr obj(env);
        for (int i = 0; i < nMinus; ++i) {
            for (int j = 0; j < n; ++j) {
                if (i == j || j == depot || i == endDepot) continue;
                obj += tau(i, j) * x[i][j];
            }
        }
        model.add(IloMinimize(env, obj));
        obj.end();

        // depot out = 1
        {
            IloExpr out0(env);
            for (int j = 0; j < n; ++j) out0 += x[depot][j];
            model.add(out0 == 1);
            out0.end();
        }
        // endDepot in = 1
        {
            IloExpr inE(env);
            for (int i = 0; i < n; ++i) inE += x[i][endDepot];
            model.add(inE == 1);
            inE.end();
        }
        // station must be visited => outdeg(s)=1
        {
            IloExpr outS(env);
            for (int j = 0; j < n; ++j) outS += x[s][j];
            model.add(outS == 1);
            outS.end();
        }

        // every original customer (exclude depot) visited by truck: outdeg(i)=1
        for (int i = 0; i < nOrig; ++i) {
            if (i == depot) continue;
            IloExpr out(env);
            for (int j = 0; j < n; ++j) out += x[i][j];
            model.add(out == 1);
            out.end();
        }

        // flow conservation (all original except depot)
        for (int i = 0; i < nOrig; ++i) {
            if (i == depot) continue;
            IloExpr out(env), in(env);
            for (int j = 0; j < n; ++j) out += x[i][j];
            for (int j = 0; j < n; ++j) in += x[j][i];
            model.add(out == in);
            out.end(); in.end();
        }

        IloCplex cplex(model);
        cplex.setParam(IloCplex::Param::MIP::Strategy::Search, IloCplex::Traditional);
        cplex.setOut(params.verbose ? std::cout : env.getNullStream());
        if (time_limit_sec > 0) cplex.setParam(IloCplex::TiLim, time_limit_sec);
        if (params.threads > 0) cplex.setParam(IloCplex::Threads, params.threads);
        cplex.setParam(IloCplex::EpGap, 0.0);

        // reuse your GCS callbacks (integer + fractional)
        cplex.use(new (env) GCSUserCutCallback(env, x, n, nMinus, depot,
            params.cut_eps, params.vio_eps, params.max_cuts_per_call));
        cplex.use(new (env) GCSLazyCallback(env, x, n, nMinus, depot,
            params.cut_eps, params.vio_eps, params.max_cuts_per_call));

        if (!cplex.solve()) {
            env.end();
            return res;
        }

        // reconstruct route in original indices, close with depot
        std::vector<int> route;
        route.push_back(depot);
        int cur = depot;
        std::vector<int> seen(n, 0);

        while (cur != endDepot) {
            seen[cur] = 1;
            int nxt = -1;
            for (int j = 0; j < n; ++j) {
                if (j == depot) continue;
                if (j == cur) continue;
                if (cur == endDepot) break; // 防御
                if (cplex.getValue(x[cur][j]) > 0.5) { nxt = j; break; }
            }

            if (nxt < 0 || nxt == endDepot) break;
            route.push_back(nxt);
            cur = nxt;
            if (seen[cur]) break;
        }
        route.push_back(depot);

        // fill result
        res.ok = true;
        res.obj = cplex.getObjValue();
        res.route = route;
        computeT1T2_fromClosedRoute(g, res.route, depot, s, res.t1, res.t2);

        env.end();
        return res;
    }
    catch (...) {
        env.end();
        return res;
    }
}

// =========================
// 2) applyAlgorithm2_primalHeuristic(...)
// Strictly follow Algorithm 2 logic (MULTIFIT eval + optional reversal to make activation earlier)
// Input: UB route (truck serves all), output: improved TSP-DS incumbent (route + D')
// =========================
static PrimalHeuristicResult applyAlgorithm2_primalHeuristic(
    const TSPDSGraph& g,
    const UBResult& ub,
    const std::vector<int>& eligible_nodes, // C'' nodes
    const CplexBendersSolver::Params& params
) {
    PrimalHeuristicResult res;
    if (!ub.ok) return res;

    const int nOrig = (int)g.truck_time.size();
    const int depot = g.depot;
    const int s = g.drone_station;
    const int V = std::max(1, g.drone_count);

    auto rt = [&](int i) -> double {
        return g.drone_time[s][i] + g.drone_time[i][s];
        };

    // start from UB
    std::vector<int> route = ub.route;
    double t1 = ub.t1, t2 = ub.t2;

    // normalize if t1 > t2 (Algorithm2 lines 3-5)
    normalizeActivationEarlier(g, route, depot, s, t1, t2);

    // D: sort eligible by distance from station (use rt or station->i distance)
    struct Cand { int node; double key; };
    std::vector<Cand> D;
    D.reserve(eligible_nodes.size());
    for (int i : eligible_nodes) {
        // skip if not in current route (should be in UB route)
        if (i == depot || i == s) continue;
        // skip if out of range
        if (rt(i) > g.drone_range + 1e-12) continue;
        D.push_back({ i, rt(i) });
    }
    std::sort(D.begin(), D.end(), [](const Cand& a, const Cand& b) {
        return a.key > b.key; // descending
        });

    std::vector<int> Dprime;
    Dprime.reserve(D.size());

    // current obj (Algorithm2 line 1): obj = t1 + t2 (since no drones yet)
    double t3 = 0.0;
    double obj = t1 + std::max(t2, t3);

    // iterate each eligible i once
    for (const auto& c : D) {
        int i = c.node;

        // If i already removed (shouldn't happen), continue
        bool inRoute = false;
        for (int k = 1; k < (int)route.size() - 1; ++k) {
            if (route[k] == i) { inRoute = true; break; }
        }
        if (!inRoute) continue;

        // Again normalize if t1 > t2 BEFORE evaluating (Algorithm2 lines 3-5)
        normalizeActivationEarlier(g, route, depot, s, t1, t2);

        // compute t3 for D' U {i} using MULTIFIT (Algorithm2 line 6)
        std::vector<double> p;
        p.reserve(Dprime.size() + 1);
        for (int u : Dprime) p.push_back(rt(u));
        p.push_back(rt(i));
        double t3cand = multifitUpperBoundFFD(p, V, params.multifit_iters);

        // determine whether i in R1 or R2 (based on position vs station)
        int posI = -1, posS = -1;
        for (int k = 0; k < (int)route.size(); ++k) {
            if (route[k] == s) posS = k;
            if (route[k] == i) posI = k;
        }
        if (posI < 0 || posS < 0) continue;

        // simulate removing i (without changing visiting order)
        std::vector<int> route2 = route;
        double t1p = t1, t2p = t2;

        if (!removeNodeAndUpdateT1T2(g, route2, depot, s, i, t1p, t2p)) continue;

        // compute candidate objective (Algorithm2 lines 8-16)
        double objp = t1p + std::max(t2p, t3cand);

        if (objp + 1e-12 < obj) {
            // accept
            route.swap(route2);
            t1 = t1p; t2 = t2p;
            Dprime.push_back(i);
            t3 = t3cand;
            obj = objp;
        }
    }

    // final normalize (Algorithm2 lines 19-21)
    normalizeActivationEarlier(g, route, depot, s, t1, t2);

    // final t3 on D' (MULTIFIT)
    {
        std::vector<double> p;
        p.reserve(Dprime.size());
        for (int u : Dprime) p.push_back(rt(u));
        t3 = multifitUpperBoundFFD(p, V, params.multifit_iters);
    }

    // derive z (truck=1, drone=0 for D')
    std::vector<int> z01(nOrig, 1);
    z01[depot] = 1; z01[s] = 1;
    for (int u : Dprime) z01[u] = 0;

    // greedy assign drones for MIP start yhat (LPT)
    std::vector<int> order(Dprime.size());
    std::iota(order.begin(), order.end(), 0);
    std::sort(order.begin(), order.end(), [&](int a, int b) {
        return (g.drone_time[s][Dprime[a]] + g.drone_time[Dprime[a]][s])
        > (g.drone_time[s][Dprime[b]] + g.drone_time[Dprime[b]][s]);
        });
    std::vector<double> load(V, 0.0);
    std::vector<int> assign(Dprime.size(), 0);
    for (int idx : order) {
        int node = Dprime[idx];
        double p = rt(node);
        int bestV = 0;
        for (int v = 1; v < V; ++v) if (load[v] < load[bestV]) bestV = v;
        assign[idx] = bestV;
        load[bestV] += p;
    }

    res.ok = true;
    res.route = route;
    res.droneNodes = Dprime;
    res.z01 = z01;
    res.assignDrone = assign;
    res.t1 = t1;
    res.t2 = t2;
    res.t3 = t3;
    res.W = std::max(t2, t3);
    res.obj = t1 + res.W;
    return res;
}

// =========================
// LB-T computation for (59): shortest truck tour over truck-only nodes + station (paper 5.4.2)
// Implementation: solve a reduced UB on mandatory set M = {depot, station} U truck_only
// Return objective value T (lower bound).
// =========================
static double computeLB_T_value(
    const TSPDSGraph& g,
    const CplexBendersSolver::Params& params,
    double time_limit_sec = 20.0
) {
    const int nOrig = (int)g.truck_time.size();
    if (nOrig <= 2) return 0.0;

    const int depot = g.depot;
    const int s = g.drone_station;

    // build mandatory nodes list
    std::vector<int> M;
    M.reserve(nOrig);
    M.push_back(depot);
    if (s != depot) M.push_back(s);
    for (int i = 0; i < nOrig; ++i) {
        if (i == depot || i == s) continue;
        if ((int)g.is_truck_only.size() == nOrig && g.is_truck_only[i]) {
            M.push_back(i);
        }
    }
    // if only depot+station => trivial path depot->station->depot
    if ((int)M.size() <= 2) {
        double t = 0.0;
        if (s != depot) t = g.truck_time[depot][s] + g.truck_time[s][depot];
        return t;
    }

    // map original -> compact index
    std::vector<int> inv(nOrig, -1);
    for (int k = 0; k < (int)M.size(); ++k) inv[M[k]] = k;

    const int mOrig = (int)M.size();
    const int endDepot = mOrig;
    const int n = mOrig + 1;
    const int nMinus = mOrig;
    const int depotC = inv[depot];
    const int sC = inv[s];

    auto tau = [&](int i, int j) -> double {
        if (i == endDepot) return 0.0;
        if (j == endDepot) return g.truck_time[M[i]][depot];
        return g.truck_time[M[i]][M[j]];
        };

    IloEnv env;
    try {
        IloModel model(env);

        IloArray<IloNumVarArray> x(env, n);
        for (int i = 0; i < n; ++i) {
            x[i] = IloNumVarArray(env, n);
            for (int j = 0; j < n; ++j) {
                bool invalid = false;
                if (i == j) invalid = true;
                if (j == depotC) invalid = true;
                if (i == endDepot) invalid = true;
                x[i][j] = invalid ? IloNumVar(env, 0, 0, ILOBOOL)
                    : IloNumVar(env, 0, 1, ILOBOOL);
            }
        }

        IloExpr obj(env);
        for (int i = 0; i < nMinus; ++i) {
            for (int j = 0; j < n; ++j) {
                if (i == j || j == depotC || i == endDepot) continue;
                obj += tau(i, j) * x[i][j];
            }
        }
        model.add(IloMinimize(env, obj));
        obj.end();

        // depot out=1, endDepot in=1, station out=1
        {
            IloExpr out0(env);
            for (int j = 0; j < n; ++j) out0 += x[depotC][j];
            model.add(out0 == 1); out0.end();
        }
        {
            IloExpr inE(env);
            for (int i = 0; i < n; ++i) inE += x[i][endDepot];
            model.add(inE == 1); inE.end();
        }
        {
            IloExpr outS(env);
            for (int j = 0; j < n; ++j) outS += x[sC][j];
            model.add(outS == 1); outS.end();
        }

        // every mandatory node (exclude depotC) must be visited: outdeg=1
        for (int i = 0; i < mOrig; ++i) {
            if (i == depotC) continue;
            IloExpr out(env);
            for (int j = 0; j < n; ++j) out += x[i][j];
            model.add(out == 1);
            out.end();
        }

        // flow conservation for all original compact nodes except depot
        for (int i = 0; i < mOrig; ++i) {
            if (i == depotC) continue;
            IloExpr out(env), in(env);
            for (int j = 0; j < n; ++j) out += x[i][j];
            for (int j = 0; j < n; ++j) in += x[j][i];
            model.add(out == in);
            out.end(); in.end();
        }

        IloCplex cplex(model);
        cplex.setParam(IloCplex::Param::MIP::Strategy::Search, IloCplex::Traditional);
        cplex.setOut(params.verbose ? std::cout : env.getNullStream());
        if (time_limit_sec > 0) cplex.setParam(IloCplex::TiLim, time_limit_sec);
        if (params.threads > 0) cplex.setParam(IloCplex::Threads, params.threads);
        cplex.setParam(IloCplex::EpGap, 0.0);

        // GCS for compact model: reuse same callbacks
        cplex.use(new (env) GCSUserCutCallback(env, x, n, nMinus, depotC,
            params.cut_eps, params.vio_eps, params.max_cuts_per_call));
        cplex.use(new (env) GCSLazyCallback(env, x, n, nMinus, depotC,
            params.cut_eps, params.vio_eps, params.max_cuts_per_call));

        if (!cplex.solve()) {
            env.end();
            return 0.0;
        }

        double T = cplex.getObjValue();
        env.end();
        return T;
    }
    catch (...) {
        env.end();
        return 0.0;
    }
}

// =========================
// 3) computeLB_T_and_add59(...)
// - compute T (LB) per paper 5.4.2
// - add (59): t1 + W >= T
// - prepare MIP start from primal heuristic (Algorithm 2) for x/xs/z/W/yhat
// =========================
struct MIPStartData {
    bool has = false;
    IloNumVarArray vars;
    IloNumArray vals;
    MIPStartData(IloEnv env) : vars(env), vals(env) {}
};

static double computeLB_T_and_add59(
    const TSPDSGraph& g,
    const CplexBendersSolver::Params& params,
    IloEnv env,
    IloModel& model,
    const IloArray<IloNumVarArray>& x,   // BMP vars
    const IloArray<IloNumVarArray>& xs,  // BMP vars
    const IloNumVarArray& z,
    const IloNumVar& W,
    const IloArray<IloNumVarArray>& yhat,
    int nOrig, int n, int nMinus, int depot, int station, int endDepot,
    const std::vector<int>& eligible_nodes,
    const PrimalHeuristicResult& heur,
    MIPStartData& startOut,
    double lb_time_limit_sec = 20.0
) {
    // ---- compute T (LB) ----
    double Tlb = computeLB_T_value(g, params, lb_time_limit_sec);

    // ---- add (59): sum tau*xs + W >= Tlb ----
    auto tau = [&](int i, int j) -> double {
        if (i == endDepot) return 0.0;
        if (j == endDepot) return g.truck_time[i][depot];
        return g.truck_time[i][j];
        };

    IloExpr t1expr(env);
    for (int i = 0; i < nMinus; ++i) {
        for (int j = 0; j < n; ++j) {
            if (i == j || j == depot || i == endDepot) continue;
            t1expr += tau(i, j) * xs[i][j];
        }
    }
    model.add(t1expr + W >= Tlb);
    t1expr.end();

    // ---- build MIP start (paper 5.4.1): use primal heuristic ----
    if (!heur.ok) return Tlb;

    // Build full x start (0/1) to satisfy equalities robustly.
    // Route is closed [depot,...,depot] on original indices; map last depot to endDepot.
    std::vector<std::vector<int>> x01(n, std::vector<int>(n, 0));
    for (int k = 0; k < (int)heur.route.size() - 1; ++k) {
        int i = heur.route[k];
        int j = heur.route[k + 1];
        if (k == (int)heur.route.size() - 2 && j == depot) j = endDepot;
        x01[i][j] = 1;
    }

    // Build xs start: prefix from depot to station (inclusive arc into station)
    std::vector<std::vector<double>> xs01(n, std::vector<double>(n, 0.0));
    {
        int cur = depot;
        for (int k = 0; k < (int)heur.route.size() - 1; ++k) {
            int i = heur.route[k];
            int j = heur.route[k + 1];
            if (i != cur) cur = i;
            if (j == depot && k == (int)heur.route.size() - 2) break; // last arc to endDepot not in xs
            xs01[i][j] = 1.0;
            if (j == station) break;
        }
    }

    // z start
    std::vector<int> z01 = heur.z01;
    if ((int)z01.size() != nOrig) z01.assign(nOrig, 1);
    z01[depot] = 1; z01[station] = 1;

    // yhat start: for nodes with z=0, set yhat[i][assigned_v]=1 else 0
    // build node->assigned_v map
    std::vector<int> node2v(nOrig, -1);
    for (int k = 0; k < (int)heur.droneNodes.size(); ++k) {
        int node = heur.droneNodes[k];
        int v = 0;
        if (k < (int)heur.assignDrone.size()) v = heur.assignDrone[k];
        node2v[node] = v;
    }

    // pack vars/vals
    startOut.vars.clear();
    startOut.vals.clear();

    // all x binaries
    for (int i = 0; i < n; ++i) {
        for (int j = 0; j < n; ++j) {
            startOut.vars.add(x[i][j]);
            startOut.vals.add((double)x01[i][j]);
        }
    }
    // all xs continuous (0/1)
    for (int i = 0; i < n; ++i) {
        for (int j = 0; j < n; ++j) {
            startOut.vars.add(xs[i][j]);
            startOut.vals.add(xs01[i][j]);
        }
    }
    // all z binaries
    for (int i = 0; i < nOrig; ++i) {
        startOut.vars.add(z[i]);
        startOut.vals.add((double)z01[i]);
    }
    // W
    startOut.vars.add(W);
    startOut.vals.add(std::max(0.0, heur.W));

    // yhat (continuous)
    // only set for eligible nodes; others already fixed 0 in model, but setting is fine
    const int V = std::max(1, g.drone_count);
    for (int i : eligible_nodes) {
        for (int v = 0; v < V; ++v) {
            double val = 0.0;
            if (z01[i] == 0 && node2v[i] == v) val = 1.0;
            startOut.vars.add(yhat[i][v]);
            startOut.vals.add(val);
        }
    }


    startOut.has = true;
    return Tlb;
}


// =========================
// BSP：MILP (46)-(50)
// + Algorithm 1 的提前终止：如果找到 incumbent < W' 则 abort
// =========================
class BSPAbortIfIncBetter : public IloCplex::IncumbentCallbackI {
public:
    BSPAbortIfIncBetter(IloEnv env, double Wmaster, double eps, bool* hit)
        : IloCplex::IncumbentCallbackI(env), Wmaster_(Wmaster), eps_(eps), hit_(hit) {}

    void main() override {
        double inc = getObjValue();
        if (inc + eps_ < Wmaster_) {
            if (hit_) *hit_ = true;
            abort();
        }
    }

    IloCplex::CallbackI* duplicateCallback() const override {
        return (new (getEnv()) BSPAbortIfIncBetter(getEnv(), Wmaster_, eps_, hit_));
    }

private:
    double Wmaster_;
    double eps_;
    bool* hit_;
};

struct BSPResult {
    bool optimal = false;
    bool aborted_inc_better = false;
    double Wz = IloInfinity;
    std::vector<int> assign; // task k -> drone v
};

static BSPResult solveBSP_MILP(const TSPDSGraph& g,
    const std::vector<int>& tasks,
    const std::vector<double>& p,
    double Wmaster,
    const CplexBendersSolver::Params& params,
    bool enableAbort)
{
    BSPResult res;
    int m = std::max(1, g.drone_count);
    int K = (int)tasks.size();
    if (K == 0) { res.optimal = true; res.Wz = 0.0; return res; }

    IloEnv env;
    try {
        IloModel model(env);

        IloArray<IloBoolVarArray> y(env, m);
        for (int v = 0; v < m; ++v) {
            y[v] = IloBoolVarArray(env, K);
            for (int k = 0; k < K; ++k) y[v][k] = IloBoolVar(env);
        }
        IloNumVar Wz(env, 0.0, IloInfinity, ILOFLOAT);
        model.add(IloMinimize(env, Wz));

        // (47) each task assigned once
        for (int k = 0; k < K; ++k) {
            IloExpr sum(env);
            for (int v = 0; v < m; ++v) sum += y[v][k];
            model.add(sum == 1);
            sum.end();
        }

        // (48) load <= Wz
        for (int v = 0; v < m; ++v) {
            IloExpr load(env);
            for (int k = 0; k < K; ++k) load += p[k] * y[v][k];
            model.add(load <= Wz);
            load.end();
        }

        IloCplex cplex(model);
        cplex.setParam(IloCplex::Param::TimeLimit, params.bsp_time_limit_sec);
        cplex.setParam(IloCplex::Param::Threads, params.bsp_threads);
        cplex.setParam(IloCplex::Param::MIP::Tolerances::MIPGap, 0.0);
        cplex.setOut(params.verbose ? std::cout : env.getNullStream());

        bool hit = false;
        if (enableAbort) {
            cplex.use(new (env) BSPAbortIfIncBetter(env, Wmaster, params.vio_eps, &hit));
        }


        bool ok = cplex.solve();

        auto st = cplex.getStatus();
        if (hit) res.aborted_inc_better = true;

        // 只要有 incumbent（通常 st 会是 Feasible/Optimal，即使 abort）就尝试提取
        if (st == IloAlgorithm::Optimal || st == IloAlgorithm::Feasible) {
            res.optimal = (st == IloAlgorithm::Optimal);
            res.Wz = cplex.getObjValue();

            res.assign.assign(K, -1);
            for (int k = 0; k < K; ++k) {
                for (int v = 0; v < m; ++v) {
                    if (cplex.getValue(y[v][k]) > 0.5) { res.assign[k] = v; break; }
                }
                if (res.assign[k] < 0) res.assign[k] = 0;
            }
        }

        // 如果既没 ok 又没解，就按原逻辑返回（注意：hit=true 时也可能 ok=false）
        if (!ok && !(st == IloAlgorithm::Optimal || st == IloAlgorithm::Feasible)) {
            env.end();
            return res;
        }

        env.end();
        return res;
    }
    catch (...) {
        env.end();
        throw;
    }
}

// =========================
// Master Lazy Callback:
//   1) GCS subtour elimination (paper (35))  [integer enforce]
//   2) BMP cut (45) (paper (45))            [integer enforce]
//   3) Benders optimality cut (51)          [integer enforce]
// =========================
class MasterLazyCallback : public IloCplex::LazyConstraintCallbackI {
public:
    MasterLazyCallback(
        IloEnv env,
        const TSPDSGraph& g,
        const CplexBendersSolver::Params& params,

        // BMP x vars + dimensions
        const IloArray<IloNumVarArray>& x,
        int n, int nMinus, int depot,

        // (45) data
        int nOrig, int station,
        const std::vector<int>& Cprime,

        // Benders data
        const IloNumVarArray& z,
        const IloNumVar& W,
        const std::vector<int>& eligible_nodes
    )
        : IloCplex::LazyConstraintCallbackI(env),
        g_(g), params_(params),
        x_(x), n_(n), nMinus_(nMinus), depot_(depot),
        nOrig_(nOrig), station_(station), Cprime_(Cprime),
        z_(z), W_(W), eligible_(eligible_nodes) {
    }

    // for reuse by separateGcsCutsStrongComp(...)
    void addCutPublic(const IloRange& rng) { this->add(rng); }

    void main() override {
        // ---------- 1) GCS subtour elimination (paper (35)) ----------
        auto getXVal = [&](int i, int j) -> double { return getValue(x_[i][j]); };

        int addedGCS = separateGcsCutsStrongComp(
            this, x_, n_, nMinus_, depot_,
            params_.cut_eps, params_.vio_eps, params_.max_cuts_per_call,
            getXVal
        );
        if (addedGCS > 0) return; // 先把子环切干净，避免做BSP等重活

        // ---------- 2) BMP connectivity cut (paper (45)) ----------
        if (separate45_and_add_if_needed()) return;

        // ---------- 3) Benders optimality cut (paper (51)) ----------
        separateBenders_and_add_if_needed();
    }

    IloCplex::CallbackI* duplicateCallback() const override {
        return (new (getEnv()) MasterLazyCallback(
            getEnv(),
            g_, params_,
            x_, n_, nMinus_, depot_,
            nOrig_, station_, Cprime_,
            z_, W_, eligible_
        ));
    }

private:
    // ========== (45) separation BMP45LazyCallback ==========
    bool separate45_and_add_if_needed() {
        const double eps = std::max(1e-9, params_.vio_eps);

        auto getXSafe = [&](int a, int b) -> double {
            if (a == b) return 0.0;
            if (b == depot_) return 0.0;      // 进入 depot 的弧在模型里无效/通常未extract
            // if (a == endDepot) return 0.0; // 这里 a< nOrig_，用不到
            return getValue(x_[a][b]);
            };

        // 1) build undirected adjacency among original nodes (0..nOrig_-1) using x=1 arcs
        std::vector<std::vector<int>> adj(nOrig_);
        for (int i = 0; i < nOrig_; ++i) adj[i].clear();

        for (int i = 0; i < nOrig_; ++i) {
            for (int j = i + 1; j < nOrig_; ++j) {
                double xij = getXSafe(i, j);
                double xji = getXSafe(j, i);
                if (xij > 0.5 - eps || xji > 0.5 - eps) {
                    adj[i].push_back(j);
                    adj[j].push_back(i);
                }
            }
        }

        // 2) DFS/BFS to get component containing depot => U'
        std::vector<char> inU(nOrig_, 0);
        std::vector<int> stack;
        stack.push_back(depot_);
        inU[depot_] = 1;

        while (!stack.empty()) {
            int u = stack.back(); stack.pop_back();
            for (int v : adj[u]) {
                if (!inU[v]) {
                    inU[v] = 1;
                    stack.push_back(v);
                }
            }
        }

        // 3) check violation: station not in U OR some mandatory node not in U
        bool violated = false;
        if (station_ >= 0 && station_ < nOrig_ && !inU[station_]) violated = true;

        if (!violated) {
            for (int node : Cprime_) {
                if (node < 0 || node >= nOrig_) continue;
                if (!inU[node]) { violated = true; break; }
            }
        }
        if (!violated) return false;

        // 4) build cut (45): sum_{i in U} sum_{j in U, j!=i} x[i][j] <= |U|-2
        int Usize = 0;
        for (int i = 0; i < nOrig_; ++i) if (inU[i]) ++Usize;
        if (Usize < 2) return false;

        IloExpr lhs(getEnv());
        for (int i = 0; i < nOrig_; ++i) if (inU[i]) {
            for (int j = 0; j < nOrig_; ++j) if (inU[j] && j != i) {
                lhs += x_[i][j];
            }
        }

        add(lhs <= (double)Usize - 2.0).end();
        lhs.end();
        return true;
    }

    // ==========实现 Algorithm 1 + cut(51) Benders separation  BendersLazyCallback ==========
    void separateBenders_and_add_if_needed() {
        const double eps = std::max(1e-9, params_.vio_eps);

        // 当前主问题候选整数解的 W'
        double Wcand = getValue(W_);

        // 取 z'（只需要 eligible 节点）
        std::vector<int> z01(nOrig_, 1);
        for (int i : eligible_) {
            if (i < 0 || i >= nOrig_) continue;
            double zv = getValue(z_[i]);
            z01[i] = (zv > 0.5 ? 1 : 0);
        }

        // 构造 BSP tasks & processing times p_k = rt(i)
        std::vector<int> tasks;
        std::vector<double> p;
        tasks.reserve(eligible_.size());
        p.reserve(eligible_.size());

        int s = g_.drone_station;
        int m = std::max(1, g_.drone_count);

        for (int i : eligible_) {
            if (i < 0 || i >= nOrig_) continue;
            if (z01[i] == 0) {
                tasks.push_back(i);
                p.push_back(g_.drone_time[s][i] + g_.drone_time[i][s]);
            }
        }

        // Step 1 MULTIFIT -> W_h
        double Wh = multifitUpperBoundFFD(p, m, params_.multifit_iters);

        // Step 2: if Wh < W' then do not add cut
        if (Wh + eps < Wcand) return;

        // Step 5..16: solve BSP MILP with early abort
        BSPResult bsp = solveBSP_MILP(g_, tasks, p, Wcand, params_, /*enableAbort=*/true);

        // Step 7..11: if found incumbent < W', abort => no cut
        if (bsp.aborted_inc_better) return;

        // 为保证正确性：只有在 BSP 证明最优时才加 cut(51)
        if (!bsp.optimal) return;

        double Wz = bsp.Wz;
        if (!(Wz > Wcand + eps)) return; // Wz <= W' => cut 冗余

        // cut(51):
        // W >= Wz - Wz*( sum_{z'i=1}(1-z_i) + sum_{z'i=0} z_i )
        IloExpr dist(getEnv());
        for (int i : eligible_) {
            if (i < 0 || i >= nOrig_) continue;
            if (z01[i] == 1) dist += (1.0 - z_[i]);
            else             dist += z_[i];
        }

        IloExpr lhs(getEnv());
        lhs += W_;
        lhs += Wz * dist;
        add(lhs >= Wz).end();

        lhs.end();
        dist.end();
    }

private:
    // refs/handles
    const TSPDSGraph& g_;
    const CplexBendersSolver::Params& params_;

    const IloArray<IloNumVarArray>& x_;
    int n_, nMinus_, depot_;

    int nOrig_, station_;
    std::vector<int> Cprime_;

    const IloNumVarArray& z_;
    const IloNumVar& W_;

    std::vector<int> eligible_;
};

// =========================
// !!!主求解：BMP (28)-(40) + (41)-(44)
// =========================
bool CplexBendersSolver::solve(const TSPDSGraph& g, TSPDSSolution& sol) {
    const int nOrig = (int)g.truck_time.size();
    if (nOrig <= 2) return false;

    const int depot = g.depot;
    const int s = g.drone_station;
    const int V = std::max(1, g.drone_count);

    const int endDepot = nOrig;    // copy end node
    const int n = nOrig + 1;       // total nodes with endDepot
    const int nMinus = nOrig;      // 0..nOrig-1 can depart

    auto tau = [&](int i, int j) -> double {
        if (i == endDepot) return 0.0;
        if (j == endDepot) return g.truck_time[i][depot];
        return g.truck_time[i][j];
        };
    auto rt = [&](int i) -> double { // drone round trip time from station
        return g.drone_time[s][i] + g.drone_time[i][s];
        };

    const double EPS = 1e-12;

    // C' (truck-only) & C'' (drone-eligible) —— 保证 partition：C' ∪ C'' = C, 且 C' ∩ C'' = ∅
    // 约定：s ∈ C'； depot 不属于 C
    std::vector<int> eligible_nodes; // C''：可无人机服务（且在航程内）
    std::vector<int> Cprime;         // C'：其余必须卡车（含 s、不可无人机、超航程、flag缺失等）

    eligible_nodes.clear();
    Cprime.clear();
    Cprime.push_back(s);

    // 用于后面快速判断“是否属于 C''”
    std::vector<char> isC2(nOrig, 0); // is in C'' ?

    for (int i = 0; i < nOrig; ++i) {
        if (i == depot || i == s) continue;

        bool flagEligible = ((int)g.is_drone_eligible.size() == nOrig && g.is_drone_eligible[i]);
        bool inRange = (rt(i) <= g.drone_range + EPS);

        // 如果 truck_only flag 明确为 true，则强制进 C'
        bool flagTruckOnly = ((int)g.is_truck_only.size() == nOrig && g.is_truck_only[i]);

        if (!flagTruckOnly && flagEligible && inRange) {
            eligible_nodes.push_back(i);
            isC2[i] = 1;
        }
        else {
            Cprime.push_back(i);
            isC2[i] = 0;
        }
    }



    IloEnv env;
    try {
        IloModel model(env);

        // ========== 变量 ==========
        // x: binary, xs: continuous
        IloArray<IloNumVarArray> x(env, n);
        IloArray<IloNumVarArray> xs(env, n);

        for (int i = 0; i < n; ++i) {
            x[i] = IloNumVarArray(env, n);
            xs[i] = IloNumVarArray(env, n);
            for (int j = 0; j < n; ++j) {
                bool invalid = false;
                if (i == j) invalid = true;
                if (j == depot) invalid = true;     // do not enter depot
                if (i == endDepot) invalid = true;  // endDepot no outgoing

                if (invalid) {
                    x[i][j] = IloNumVar(env, 0.0, 0.0, ILOBOOL);
                    xs[i][j] = IloNumVar(env, 0.0, 0.0, ILOFLOAT);
                }
                else {
                    x[i][j] = IloNumVar(env, 0.0, 1.0, ILOBOOL);
                    xs[i][j] = IloNumVar(env, 0.0, 1.0, ILOFLOAT);
                }
            }
        }

        // z_i：只对原节点 0..nOrig-1 定义，endDepot 不需要
        IloNumVarArray z(env, nOrig);
        for (int i = 0; i < nOrig; ++i) {
            // 默认二进制
            z[i] = IloNumVar(env, 0.0, 1.0, ILOBOOL);
        }

        // W：T - t1
        IloNumVar W(env, 0.0, IloInfinity, ILOFLOAT);

        // 增强松弛变量 \hat y_{iv} in [0,1] 仅对 eligible
        IloArray<IloNumVarArray> yhat(env, nOrig);
        for (int i = 0; i < nOrig; ++i) {
            yhat[i] = IloNumVarArray(env, V);
            for (int v = 0; v < V; ++v) {
                bool eligible = isC2[i];  // 只对 C'' 开 yhat
                if (!eligible) yhat[i][v] = IloNumVar(env, 0.0, 0.0, ILOFLOAT);
                else           yhat[i][v] = IloNumVar(env, 0.0, 1.0, ILOFLOAT);
            }
        }

        // ========== 目标 (28)：min t1 + W ==========
        IloExpr t1(env);
        for (int i = 0; i < nMinus; ++i) {
            for (int j = 0; j < n; ++j) {
                if (i == j || j == depot || i == endDepot) continue;
                t1 += tau(i, j) * xs[i][j];
            }
        }
        model.add(IloMinimize(env, t1 + W));
        t1.end();

        // ========== 固定 z：只固定“非 C''”为 1（最小化 & 与 partition 一致） ==========
        model.add(z[depot] == 1);
        model.add(z[s] == 1);

        // 注意：z 只在 (30) 用到（i∈C''），但你定义了全体 z，所以我们统一把“非 C''”固定为 1
        for (int i = 0; i < nOrig; ++i) {
            if (i == depot || i == s) continue;
            if (!isC2[i]) {
                model.add(z[i] == 1);
            }
        }

        // ========== (29)-(33) 路由基本约束（含 endDepot） ==========
        // -------------------------
        // (29)  truck-only 节点 i∈C' : sum_j x_{i,j} = 1
        // -------------------------
        for (int i : Cprime) {
            if (i == depot) continue;             // depot 不在 (29)
            IloExpr out(env);
            for (int j = 0; j < n; ++j) {         // j ∈ N^+
                if (j == depot) continue;
                if (j == i) continue;
                out += x[i][j];
            }
            model.add(out == 1);
            out.end();
        }

        // -------------------------
        // (30)  drone-eligible 节点 i∈C'' : sum_j x_{i,j} = z_i
        // -------------------------
        for (int i : eligible_nodes) {            // eligible_nodes == C''
            IloExpr out(env);
            for (int j = 0; j < n; ++j) {         // j ∈ N^+
                if (j == depot) continue;
                if (j == i) continue;
                out += x[i][j];
            }
            model.add(out == z[i]);
            out.end();
        }

        // -------------------------
        // (31)  对所有客户节点 i∈C（论文：所有 customer；你这里用“所有原节点除 depot”）
        //       sum_j x_{i,j} = sum_j x_{j,i}
        // 注意：右边 j ∈ N^-（不含 endDepot），左边 j ∈ N^+（不含 depot）
        // -------------------------
        for (int i = 0; i < nOrig; ++i) {
            if (i == depot) continue;

            IloExpr out(env), in(env);

            // out: j in N^+ (0..n-1) 但排除 depot
            for (int j = 0; j < n; ++j) {
                if (j == depot) continue;
                if (j == i) continue;
                out += x[i][j];
            }

            // in: j in N^- (0..nOrig-1) 不含 endDepot
            for (int j = 0; j < nMinus; ++j) {
                if (j == i) continue;
                in += x[j][i];
            }

            model.add(out == in);
            out.end(); in.end();
        }

        // -------------------------
        // (32) depot out = 1; endDepot in = 1
        // -------------------------
        {
            IloExpr out0(env);
            for (int j = 0; j < n; ++j) {
                if (j == depot) continue;         // N^+ 不含 depot（你变量里也已固定为0）
                if (j == depot) continue;
                out0 += x[depot][j];
            }
            model.add(out0 == 1);
            out0.end();
        }
        {
            IloExpr inEnd(env);
            for (int i = 0; i < nMinus; ++i) {    // N^-：0..nOrig-1，不含 endDepot
                if (i == endDepot) continue;
                inEnd += x[i][endDepot];
            }
            model.add(inEnd == 1);
            inEnd.end();
        }

        // -------------------------
        // (33)  x^s_{i,j} <= x_{i,j},  ∀ i∈N^-, j∈N^+, i≠j
        // -------------------------
        for (int i = 0; i < nMinus; ++i) {        // i ∈ N^-
            for (int j = 0; j < n; ++j) {         // j ∈ N^+
                if (j == depot) continue;
                if (i == j) continue;
                model.add(xs[i][j] <= x[i][j]);
            }
        }

        // -------------------------
        // (34)  x^s 单位流：depot 为 +1，station s 为 -1，其余为 0
        // 论文写法：
        // sum_{j∈N^+, j≠i, i≠endDepot} x^s_{i,j} - sum_{j∈N^-, j≠i, i≠depot} x^s_{j,i} = b_i
        // -------------------------
        for (int i = 0; i < n; ++i) {             // i ∈ N (含 endDepot)
            IloExpr out(env), in(env);

            // out: j in N^+，且 i!=endDepot
            if (i != endDepot) {
                for (int j = 0; j < n; ++j) {
                    if (j == depot) continue;
                    if (j == i) continue;
                    out += xs[i][j];
                }
            }

            // in: j in N^-，且 i!=depot
            if (i != depot) {
                for (int j = 0; j < nMinus; ++j) {
                    if (j == i) continue;
                    in += xs[j][i];
                }
            }

            int b = 0;
            if (i == depot) b = 1;
            else if (i == s) b = -1;

            model.add(out - in == b);
            out.end(); in.end();
        }

        // -------------------------
        // (35)  GCS 子回路消除：指数约束，必须“分离”加入（callback）。
        // 这里不直接 add 全部约束，只保留你现有 Tarjan/GCS 分离逻辑即可。
        // 论文形式：
        //  sum_{i∈U} sum_{j∉U} x_{i,j} >= sum_{j∈N^+\{k}} x_{k,j},  ∀k∈U, U⊂N^-, |U|>=2
        // -------------------------
        // （这里留空：由你的 UserCut/LazyCut 回调中动态生成）

        // -------------------------
        // (36)  W >= sum τ_{i,j} x_{i,j} - sum τ_{i,j} x^s_{i,j}
        // -------------------------
        {
            IloExpr totalTruck(env), act(env);
            for (int i = 0; i < nMinus; ++i) {    // i ∈ N^-
                for (int j = 0; j < n; ++j) {     // j ∈ N^+
                    if (j == depot) continue;
                    if (i == j) continue;
                    totalTruck += tau(i, j) * x[i][j];
                    act += tau(i, j) * xs[i][j];
                }
            }
            model.add(W >= totalTruck - act);
            totalTruck.end();
            act.end();
        }


        // ========== (41)-(44) strengthened relaxation ==========
        // (41) sum_v yhat[i][v] = 1 - z_i
        for (int i : eligible_nodes) {
            IloExpr sum(env);
            for (int v = 0; v < V; ++v) sum += yhat[i][v];
            model.add(sum == 1.0 - z[i]);
            sum.end();
        }

        // (42)(43) W >= sum_i rt(i)*yhat[i][v]  for each v
        for (int v = 0; v < V; ++v) {
            IloExpr load(env);
            for (int i : eligible_nodes) load += rt(i) * yhat[i][v];
            model.add(W >= load);
            load.end();
        }

        // (44) paper-form: x[i][j] + x[j][i] <= z[i],  for i in C, j in C, i > j
        for (int i = 0; i < nOrig; ++i) {
            if (i == depot) continue;
            for (int j = 0; j < i; ++j) {           // i > j
                if (j == depot) continue;
                model.add(x[i][j] + x[j][i] <= z[i]);
            }
        }


        // ======= 5.4 preprocessing =======
        // 5.4.1: solve (UB) and apply Algorithm 2 to get primal heuristic incumbent
        cout << "执行UB得到全由卡车访问的路径" << endl;
        UBResult ub = solveUB_TSP_route(g, params, /*time_limit_sec=*/30.0);
        cout << "通过ub的路径，生成获得初始解 Algorithm2" << endl;
        PrimalHeuristicResult heur = applyAlgorithm2_primalHeuristic(g, ub, eligible_nodes, params);


        // ======= add (59) + prepare MIP start =======
        MIPStartData mipStart(env);
        double Tlb = computeLB_T_and_add59(
            g, params, env, model,
            x, xs, z, W, yhat,
            nOrig, n, nMinus, depot, s, endDepot,
            eligible_nodes,
            heur,
            mipStart,
            /*lb_time_limit_sec=*/20.0
        );


        // ========== 求解器与回调 ==========
        IloCplex cplex(model);
        cplex.extract(model);   

        // 传统回调必须 Traditional
        cplex.setParam(IloCplex::Param::MIP::Strategy::Search, IloCplex::Traditional);

        cplex.setOut(params.verbose ? std::cout : env.getNullStream());
        if (params.time_limit_sec > 0) cplex.setParam(IloCplex::TiLim, params.time_limit_sec);
        if (params.threads > 0)        cplex.setParam(IloCplex::Threads, params.threads);
        if (params.mip_gap >= 0)       cplex.setParam(IloCplex::EpGap, params.mip_gap);

        // GCS cuts（与 F2 一样）分数解用 usercut
        cplex.use(new (env) GCSUserCutCallback(env, x, n, nMinus, depot,
            params.cut_eps, params.vio_eps, params.max_cuts_per_call));


        // 只注册一个 MasterLazy
        cplex.use(new (env) MasterLazyCallback(env,
            g, params,
            x, n, nMinus, depot,
            nOrig, s, Cprime,
            z, W,
            eligible_nodes));


        double bestIncObj = IloInfinity;
        double bestIncTime = -1.0;

        cplex.use(new (env) BestIncumbentTimeCallback(env, &bestIncObj, &bestIncTime));

        // ======= inject primal heuristic MIP start =======
        if (mipStart.has) {
            // 过滤掉未被extract的变量，避免报错
            IloNumVarArray vars2(env);
            IloNumArray vals2(env);

            for (IloInt k = 0; k < mipStart.vars.getSize(); ++k) {
                const IloNumVar& var = mipStart.vars[k];
                if (cplex.isExtracted(var)) {
                    vars2.add(var);
                    vals2.add(mipStart.vals[k]);
                }
            }

            if (vars2.getSize() > 0) {
                cplex.addMIPStart(vars2, vals2, IloCplex::MIPStartAuto);
            }

            vars2.end();
            vals2.end();
        }



        bool ok = cplex.solve();

        // ===== 1. 记录 CPLEX 状态 =====
        IloAlgorithm::Status st = cplex.getStatus();
        sol.cplex_status = static_cast<int>(st);
        sol.cplex_status_name = cplexStatusName(st);

        // ===== 2. 尝试读取 bound =====
        try {
            sol.cplex_best_bound = cplex.getBestObjValue();
        }
        catch (...) {
            sol.cplex_best_bound = std::numeric_limits<double>::infinity();
        }

        // ===== 3. 尝试读取最好整数解 =====
        bool hasIntegerSolution = false;
        try {
            sol.cplex_best_integer = cplex.getObjValue();
            hasIntegerSolution = std::isfinite(sol.cplex_best_integer);
        }
        catch (...) {
            sol.cplex_best_integer = std::numeric_limits<double>::infinity();
            hasIntegerSolution = false;
        }

        // ===== 4. 尝试读取 gap =====
        try {
            sol.cplex_mip_gap = cplex.getMIPRelativeGap() * 100.0;
        }
        catch (...) {
            sol.cplex_mip_gap = std::numeric_limits<double>::infinity();
        }

        // ===== 5. 给一个简单结论 =====
        if (st == IloAlgorithm::Optimal) {
            sol.cplex_result_type = "OPTIMAL";
        }
        else if (hasIntegerSolution) {
            sol.cplex_result_type = "FEASIBLE_NOT_OPTIMAL";
        }
        else if (st == IloAlgorithm::Infeasible) {
            sol.cplex_result_type = "INFEASIBLE";
        }
        else {
            sol.cplex_result_type = "NO_SOLUTION";
        }

        std::cout << "[Benders CPLEX] "
            << "status=" << sol.cplex_status_name
            << ", result=" << sol.cplex_result_type
            << ", best_integer=" << sol.cplex_best_integer
            << ", best_bound=" << sol.cplex_best_bound
            << ", gap=" << sol.cplex_mip_gap << "%"
            << std::endl;

        // ===== 6. 如果没有可行整数解，不能继续取变量值 =====
        if (!hasIntegerSolution) {
            env.end();
            return false;
        }



        // ========== 回填 solution ==========
        sol = TSPDSSolution();
        sol.initialize(nOrig);

        sol.drone_assignments.clear();
        for (int v = 0; v < V; ++v) sol.drone_assignments[v] = {};


        // 目标值：t1 + W（注意：这不是 makespan T，但最终 makespan 是 max(truckTotal, t1 + Wz)）
        double obj = cplex.getObjValue();

        // 计算 act=t1 与 truckTotal
        double actTime = 0.0;
        double truckTime = 0.0;
        for (int i = 0; i < nMinus; ++i) {
            for (int j = 0; j < n; ++j) {
                if (i == j || j == depot || i == endDepot) continue;
                double xv = cplex.getValue(x[i][j]);
                if (xv > 0.5) truckTime += tau(i, j);

                double xsv = cplex.getValue(xs[i][j]);
                if (xsv > 1e-9) actTime += tau(i, j) * xsv;
            }
        }
        sol.station_activation_time = actTime;
        sol.truck_completion_time = truckTime;

        double masterObj = cplex.getObjValue();
        double Wcand = cplex.getValue(W);

        std::cout << "[MASTER] obj(t1+W)=" << masterObj
            << "  W=" << Wcand
            << "  truckTotal=" << truckTime
            << "  act(t1)=" << actTime
            << "\n";


        // 提取 z'
        std::vector<int> z01(nOrig, 1);
        for (int i : eligible_nodes) z01[i] = (cplex.getValue(z[i]) > 0.5 ? 1 : 0);

        // 最终解一次 BSP（精确）用于回填无人机 makespan
        std::vector<int> tasks;
        std::vector<double> p;
        for (int i : eligible_nodes) {
            if (z01[i] == 0) {
                tasks.push_back(i);
                p.push_back(rt(i));
            }
        }
        double Wcand2 = cplex.getValue(W);
        BSPResult bsp = solveBSP_MILP(g, tasks, p, Wcand2, params, /*enableAbort=*/false);
        double Wz = bsp.optimal ? bsp.Wz : multifitUpperBoundFFD(p, V, params.multifit_iters);

        sol.drone_completion_time = sol.station_activation_time + Wz;
        sol.makespan = std::max(sol.truck_completion_time, sol.drone_completion_time);

        std::cout << "[BSP] Wz=" << Wz
            << "  droneFinish=act+Wz=" << (actTime + Wz)
            << "  makespan=" << std::max(truckTime, actTime + Wz)
            << "\n";


        // reconstruct truck_route
        std::vector<int> route;
        route.push_back(depot);
        int cur = depot;
        std::vector<int> visited(n, 0);
        while (cur != endDepot) {
            visited[cur] = 1;
            int nxt = -1;
            for (int j = 0; j < n; ++j) {
                if (j == depot) continue;
                if (j == cur) continue;
                if (cur == endDepot) break; // 防御
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

        // served_by_truck
        for (int node : sol.truck_route) {
            if (node >= 0 && node < nOrig) sol.served_by_truck[node] = true;
        }

        std::vector<int> assign = bsp.assign;

        for (int k = 0; k < (int)tasks.size(); ++k) {
            int i = tasks[k];
            int v = assign[k];
            sol.served_by_drone[i] = true;
            sol.node_to_drone[i] = v;
            sol.drone_assignments[v].push_back(i);
        }

        // pos_in_truck
        for (int idx = 0; idx < (int)sol.truck_route.size(); ++idx) {
            int node = sol.truck_route[idx];
            if (node >= 0 && node < nOrig) sol.pos_in_truck[node] = idx;
            if (node == s) sol.pos_station_in_truck = idx;
        }

        // CPLEX 信息回填
        sol.cplex_best_integer = cplex.getObjValue();
        sol.cplex_best_bound = cplex.getBestObjValue();
        sol.cplex_mip_gap = cplex.getMIPRelativeGap() * 100;
        sol.cplex_status = (int)cplex.getStatus();
        sol.cplex_feasible_info = (int)cplex.getStatus();   // CPLEX 的字符串状态
        double totalTime = cplex.getCplexTime(); // 结束/终止时刻

        // bestIncTime = 第一次得到最终最优（或当前最优）incumbent 的时间
        sol.cplex_find_best_time = (bestIncTime >= 0.0 ? bestIncTime : totalTime);

        ValidationReport rep;
        bool ok2 = validateTSPDSolution(g, sol, rep, /*eps=*/1e-6, /*checkTimes=*/true, /*checkInternalFlags=*/true);
        std::cout << rep.toString() << "\n";
		sol.cplex_feasible_solution = ok2;
		sol.cplex_feasible_info = rep.toString();


        env.end();
        return true;
    }
    catch (IloException& e) {
        std::cerr << "[Benders] IloException: " << e.getMessage() << "\n";
        env.end();
        return false;
    }
    catch (...) {
        std::cerr << "[Benders] Unknown exception.\n";
        env.end();
        return false;
    }
}
