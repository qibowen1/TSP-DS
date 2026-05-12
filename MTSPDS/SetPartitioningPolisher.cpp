#include "SetPartitioningPolisher.h"
#include <unordered_set>
#include <unordered_map>
#include <algorithm>

#include <ilcplex/ilocplex.h>
ILOSTLBEGIN

struct RouteColumn {
    int from_solution = -1;
    int truck_id = -1;
    double cost = 0.0;
    std::vector<int> cover_customers;
    std::vector<int> used_stations;
    std::vector<int> tour; // keep the tour for reconstruction
    // for reconstruction: customer_station mapping snapshot
    std::vector<int> customer_station;
};

static inline bool contains(const std::vector<int>& a, int x) {
    return std::find(a.begin(), a.end(), x) != a.end();
}

bool SetPartitioningPolisher::solve(const std::vector<MTSPDSSolution>& pool,
    const MTSPDSSolution& incumbent,
    MTSPDSSolution& out) {
    // Build columns R from pool (and include incumbent as extra source)
    std::vector<MTSPDSSolution> all = pool;
    all.push_back(incumbent);

    std::vector<RouteColumn> cols;
    for (int si = 0; si < (int)all.size(); ++si) {
        const auto& S = all[si];
        for (int k = 0; k < graph.KN; ++k) {
            RouteColumn c;
            c.from_solution = si;
            c.truck_id = k;
            c.tour = S.truck_routes[k];
            c.customer_station = S.customer_station;

            // stations used in this tour
            std::unordered_set<int> st;
            for (int v : c.tour) if (graph.is_station[v]) st.insert(v);
            c.used_stations.assign(st.begin(), st.end());

            // coverage: truck-served customers in tour + customers served by stations used by this tour
            std::unordered_set<int> cov;
            for (int v : c.tour) if (graph.is_customer[v]) cov.insert(v);
            for (int j : graph.customers) {
                int s = S.customer_station[j];
                if (s != -1 && st.count(s)) cov.insert(j);
            }
            c.cover_customers.assign(cov.begin(), cov.end());

            // cost cr: max(truck_time, max station completion for stations in tour)
            double ttruck = 0.0;
            for (int i = 0; i + 1 < (int)c.tour.size(); ++i)
                ttruck += graph.truck_time[c.tour[i]][c.tour[i + 1]];

            double tstation = 0.0;
            for (int s : c.used_stations) {
                auto it = S.station_completion.find(s);
                if (it != S.station_completion.end()) tstation = std::max(tstation, it->second);
            }
            c.cost = std::max(ttruck, tstation);

            cols.push_back(std::move(c));
        }
    }

    int R = (int)cols.size();
    int J = (int)graph.customers.size();
    int Snum = (int)graph.stations.size();

    // Map customer id -> index 0..J-1
    std::unordered_map<int, int> cust2idx;
    for (int i = 0; i < J; ++i) cust2idx[graph.customers[i]] = i;

    // Map station id -> index
    std::unordered_map<int, int> st2idx;
    for (int i = 0; i < Snum; ++i) st2idx[graph.stations[i]] = i;

    IloEnv env;
    try {
        IloModel model(env);
        IloNumVar tau(env, 0.0, IloInfinity, ILOFLOAT);

        IloBoolVarArray x(env, R);
        for (int r = 0; r < R; ++r) x[r] = IloBoolVar(env);

        // min tau
        model.add(IloMinimize(env, tau));

        // tau >= c_r * x_r
        for (int r = 0; r < R; ++r) {
            model.add(tau >= cols[r].cost * x[r]);
        }

        // choose exactly KN routes
        {
            IloExpr expr(env);
            for (int r = 0; r < R; ++r) expr += x[r];
            model.add(expr == graph.KN);
            expr.end();
        }

        // each customer covered exactly once
        for (int jj = 0; jj < J; ++jj) {
            int cust = graph.customers[jj];
            IloExpr expr(env);
            for (int r = 0; r < R; ++r) {
                if (contains(cols[r].cover_customers, cust)) expr += x[r];
            }
            model.add(expr == 1);
            expr.end();
        }

        // each station used at most once across selected routes
        for (int si2 = 0; si2 < Snum; ++si2) {
            int stid = graph.stations[si2];
            IloExpr expr(env);
            for (int r = 0; r < R; ++r) {
                if (contains(cols[r].used_stations, stid)) expr += x[r];
            }
            model.add(expr <= 1);
            expr.end();
        }

        // total used stations <= C (linearize with y_s)
        {
            IloBoolVarArray y(env, Snum);
            for (int si2 = 0; si2 < Snum; ++si2) y[si2] = IloBoolVar(env);

            for (int si2 = 0; si2 < Snum; ++si2) {
                int stid = graph.stations[si2];
                IloExpr expr(env);
                for (int r = 0; r < R; ++r) {
                    if (contains(cols[r].used_stations, stid)) expr += x[r];
                }
                model.add(expr <= (double)graph.KN * y[si2]);
                expr.end();
            }

            IloExpr sumy(env);
            for (int si2 = 0; si2 < Snum; ++si2) sumy += y[si2];
            model.add(sumy <= graph.C);
            sumy.end();
        }

        IloCplex cplex(model);
        cplex.setOut(env.getNullStream());

        if (!cplex.solve()) {
            env.end();
            return false;
        }

        // reconstruct solution from selected columns
        MTSPDSSolution S;
        S.initialize((int)graph.nodes.size(), graph.KN);

        // choose columns
        std::vector<int> sel;
        for (int r = 0; r < R; ++r) {
            if (cplex.getValue(x[r]) > 0.5) sel.push_back(r);
        }
        if ((int)sel.size() != graph.KN) {
            env.end();
            return false;
        }

        // build truck routes
        for (int k = 0; k < graph.KN; ++k) {
            int r = sel[k];
            S.truck_routes[k] = cols[r].tour;
        }

        // build customer_station/customer_truck
        for (int j : graph.customers) {
            S.customer_station[j] = -1;
            S.customer_truck[j] = -1;
        }

        // fill truck-served customers from tours
        for (int k = 0; k < graph.KN; ++k) {
            for (int v : S.truck_routes[k]) {
                if (graph.is_customer[v]) {
                    S.customer_truck[v] = k;
                    S.customer_station[v] = -1;
                }
            }
        }

        // remaining customers served by stations: take station from selected column snapshot
        for (int j : graph.customers) {
            if (S.customer_truck[j] != -1) continue;
            for (int rSel : sel) {
                if (contains(cols[rSel].cover_customers, j)) {
                    int s = cols[rSel].customer_station[j];
                    S.customer_station[j] = s;
                    break;
                }
            }
        }

        out = S;
        env.end();
        return true;
    }
    catch (...) {
        env.end();
        return false;
    }
}
