#include "TwoPhaseMatheuristic.h"
#include "TSPHeuristics.h"
#include "SetPartitioningPolisher.h"
#include <algorithm>
#include <unordered_set>
#include <numeric>
#include <limits>
#include <iostream>
#include <chrono>
using namespace std;

TwoPhaseMatheuristic::TwoPhaseMatheuristic(const MTSPDSGraph& g, const TwoPhaseParams& p)
    : graph(g), params(p), rng(p.seed), scheduler(g) {
}

MTSPDSSolution TwoPhaseMatheuristic::emptySolution() const {
    MTSPDSSolution S;
    S.initialize((int)graph.nodes.size(), graph.KN);
    S.makespan = MTSPDSSolution::INF;
    return S;
}

void TwoPhaseMatheuristic::addToPool(const MTSPDSSolution& S) {
    if ((int)pool.size() < params.max_pool) pool.push_back(S);
}

bool TwoPhaseMatheuristic::feasibleBasic(const MTSPDSSolution& S) const {
    int n = (int)graph.nodes.size();
    std::vector<int> seen(n, 0);

    // check truck tours
    for (int k = 0;k < graph.KN;++k) {
        if (S.truck_routes[k].size() < 2) return false;
        if (S.truck_routes[k].front() != graph.depot) return false;
        if (S.truck_routes[k].back() != graph.depot) return false;
        for (int idx = 1; idx + 1 < (int)S.truck_routes[k].size(); ++idx) {
            int v = S.truck_routes[k][idx];
            if (graph.is_customer[v]) {
                seen[v]++;
                if (S.customer_truck[v] != k) return false;
                if (S.customer_station[v] != -1) return false;
            }
            if (graph.is_station[v]) {
                // station node can appear at most once overall (we check below)
            }
        }
    }

    // station uniqueness
    std::unordered_set<int> stSeen;
    for (int k = 0;k < graph.KN;++k) {
        for (int v : S.truck_routes[k]) {
            if (graph.is_station[v]) {
                if (stSeen.count(v)) return false;
                stSeen.insert(v);
            }
        }
    }
    if ((int)stSeen.size() > graph.C) return false;

    // drone-served customers
    for (int j : graph.customers) {
        if (S.customer_station[j] != -1) {
            int s = S.customer_station[j];
            if (!stSeen.count(s)) return false; // must be active
            if (!graph.droneEligible(s, j)) return false;
            seen[j]++;
        }
    }

    // every customer exactly once
    for (int j : graph.customers) {
        if (seen[j] != 1) return false;
    }
    return true;
}

void TwoPhaseMatheuristic::evaluate(MTSPDSSolution& S) {
    int n = (int)graph.nodes.size();
    int KN = graph.KN;

    // per truck time
    S.truck_completion.assign(KN, 0.0);
    for (int k = 0;k < KN;++k) {
        S.truck_completion[k] = tsp_heur::tourTime(graph, S.truck_routes[k]);
    }

    // station activation: arrival time along the truck route
    S.station_activation.clear();
    S.station_completion.clear();

    for (int k = 0;k < KN;++k) {
        double t = 0.0;
        for (int idx = 0; idx + 1 < (int)S.truck_routes[k].size(); ++idx) {
            int u = S.truck_routes[k][idx];
            int v = S.truck_routes[k][idx + 1];
            if (idx > 0 && graph.is_station[u]) {
                // arrival time at u already accumulated in t
                S.station_activation[u] = t;
            }
            t += graph.truck_time[u][v];
        }
        // also if station is first after depot
        if (S.truck_routes[k].size() >= 2) {
            int first = S.truck_routes[k][1];
            if (graph.is_station[first]) {
                S.station_activation[first] = graph.truck_time[graph.depot][first];
            }
        }
    }

    // rebuild station customer lists from customer_station
    std::unordered_map<int, std::vector<int>> stationCusts;
    for (int j : graph.customers) {
        int s = S.customer_station[j];
        if (s != -1) stationCusts[s].push_back(j);
    }

    // schedule each active station and compute completion
    for (auto& kv : S.station_activation) {
        int s = kv.first;
        auto it = stationCusts.find(s);
        std::vector<int> custs = (it == stationCusts.end() ? std::vector<int>{} : it->second);
        StationSchedule sch = scheduler.scheduleWithCache(s, custs);
        S.station_schedule[s] = sch;
        double act = kv.second;
        S.station_completion[s] = act + sch.makespan;
    }

    double maxTruck = 0.0;
    for (double x : S.truck_completion) maxTruck = std::max(maxTruck, x);

    double maxStation = 0.0;
    for (auto& kv : S.station_completion) maxStation = std::max(maxStation, kv.second);

    S.makespan = std::max(maxTruck, maxStation);
}

MTSPDSSolution TwoPhaseMatheuristic::constructInitialSolution() {
    MTSPDSSolution S;
    S.initialize((int)graph.nodes.size(), graph.KN);

    // (1) select C stations randomly (paper)
    std::vector<int> cand = graph.stations;
    std::shuffle(cand.begin(), cand.end(), rng);
    std::vector<int> selected;
    for (int i = 0;i < (int)cand.size() && (int)selected.size() < graph.C;++i) selected.push_back(cand[i]);

    // (2) assign customers in range to drones (random station if multiple)
    std::vector<int> remaining;
    for (int j : graph.customers) {
        if (graph.is_truck_only[j]) { remaining.push_back(j); continue; }
        std::vector<int> cover;
        for (int s : selected) if (graph.droneEligible(s, j)) cover.push_back(s);
        if (cover.empty()) {
            remaining.push_back(j);
        }
        else {
            std::uniform_int_distribution<int> dist(0, (int)cover.size() - 1);
            int s = cover[dist(rng)];
            S.customer_station[j] = s;
        }
    }

    // (3) sweep assigns remaining customers + selected stations to KN trucks
    auto buckets = tsp_heur::sweepAssign(graph, selected, remaining, rng);

    // (4) build each truck tour via NN + 2-opt; set customer_truck
    for (int k = 0;k < graph.KN;++k) {
        std::vector<int> nodes = buckets[k];
        auto tour = tsp_heur::tspNearestNeighbor(graph, nodes, graph.depot);
        tsp_heur::twoOptImprove(graph, tour);
        S.truck_routes[k] = tour;
    }

    // fill customer_truck for truck-served customers
    for (int k = 0;k < graph.KN;++k) {
        for (int v : S.truck_routes[k]) {
            if (graph.is_customer[v]) {
                S.customer_truck[v] = k;
                S.customer_station[v] = -1;
            }
        }
    }

    evaluate(S);
    return S;
}

// granular arcs = pi*(|VN|+1) shortest truck arcs (paper)
std::vector<std::pair<int, int>> TwoPhaseMatheuristic::buildGranularArcs(double pi) const {
    int VN = (int)graph.customers.size();
    int M = (int)std::ceil(pi * (VN + 1.0));

    // V = depot + customers + stations
    std::vector<int> V;
    V.reserve(1 + graph.customers.size() + graph.stations.size());
    V.push_back(graph.depot);
    V.insert(V.end(), graph.customers.begin(), graph.customers.end());
    V.insert(V.end(), graph.stations.begin(), graph.stations.end());

    std::vector<std::pair<double, std::pair<int, int>>> edges;
    edges.reserve((size_t)V.size() * (size_t)(V.size() - 1));

    for (int ii = 0; ii < (int)V.size(); ++ii) {
        int i = V[ii];
        for (int jj = 0; jj < (int)V.size(); ++jj) {
            int j = V[jj];
            if (i == j) continue;

            // ✅ 去掉 station-station 边（收缩边集，避免挤占）
            if (graph.is_station[i] && graph.is_station[j]) continue;

            edges.push_back({ graph.truck_time[i][j], {i, j} });
        }
    }

    if (edges.empty()) return {};

    if (M > (int)edges.size()) M = (int)edges.size();

    // nth_element 取前 M 小
    std::nth_element(edges.begin(), edges.begin() + (M - 1), edges.end(),
        [](const auto& a, const auto& b) { return a.first < b.first; });

    edges.resize(M);

    std::vector<std::pair<int, int>> gran;
    gran.reserve(edges.size());
    for (auto& e : edges) gran.push_back(e.second);
    return gran;
}


bool TwoPhaseMatheuristic::arcInGranular(int u, int v, const std::vector<std::pair<int, int>>& gran) const {
    // linear scan is OK for moderate size; you can optimize with unordered_set if needed
    for (auto& p : gran) if (p.first == u && p.second == v) return true;
    return false;
}

MTSPDSSolution TwoPhaseMatheuristic::rvndLocalSearch(MTSPDSSolution S, double pi) {
    while (true) {
        if (timeUp()) return S;

        bool improvedInRun = false;

        std::vector<int> neigh = { 0,1,2,3,4 };
        std::shuffle(neigh.begin(), neigh.end(), rng);

        for (int nt : neigh) {
            if (timeUp()) return S;

            bool improvedInThis = false;

            while (true) {
                if (timeUp()) return S;

                bool ok = false;
                if (nt == 0) ok = tryRelocate(S, pi);
                else if (nt == 1) ok = trySwap(S, pi);
                else if (nt == 2) ok = tryMultirangeDroneRelocate(S);
                else if (nt == 3) ok = tryDroneToTruckRelocate(S, pi);
                else if (nt == 4) ok = tryTruckToDroneRelocate(S);

                if (ok) {
                    improvedInThis = true;
                    improvedInRun = true;
                    continue; // first-improvement: restart current neighborhood
                }
                else {
                    break;
                }
            }

            if (!improvedInThis) {
                addToPool(S);
            }
        }

        if (!improvedInRun) break;
    }
    return S;
}


static inline bool isMovableNode(const MTSPDSGraph& g, int v) {
    return (g.is_customer[v] || g.is_station[v]) && (v != g.depot);
}


// ---- neighborhood operators (simple but faithful) ----
// Note: for speed, you can later replace full re-eval by delta evaluation.

bool TwoPhaseMatheuristic::tryRelocate(MTSPDSSolution& S, double pi) {
    auto gran = buildGranularArcs(pi);

    double bestCost = S.makespan;
    MTSPDSSolution best = S;

    // randomized order of trucks and nodes
    std::vector<int> trucks(graph.KN);
    std::iota(trucks.begin(), trucks.end(), 0);
    std::shuffle(trucks.begin(), trucks.end(), rng);

    for (int fromK : trucks) {
        auto& rt = S.truck_routes[fromK];
        // pick customer nodes only
        std::vector<int> idxs;
        for (int i = 1; i + 1 < (int)rt.size(); ++i) {
            int v = rt[i];
            if (isMovableNode(graph, v)) idxs.push_back(i);
        }

        std::shuffle(idxs.begin(), idxs.end(), rng);

        for (int posIdx : idxs) {
            int node = rt[posIdx];

            for (int toK : trucks) {
                auto& rt2 = S.truck_routes[toK];
                std::vector<int> ins;
                for (int j = 1;j < (int)rt2.size();++j) ins.push_back(j);
                std::shuffle(ins.begin(), ins.end(), rng);

                for (int insPos : ins) {
                    if (fromK == toK && (insPos == posIdx || insPos == posIdx + 1)) continue;

                    // granular filter: check at least one resulting arc in gran
                    int prev = rt2[insPos - 1];
                    int next = rt2[insPos];
                    if (!arcInGranular(prev, node, gran) && !arcInGranular(node, next, gran)) continue;

                    MTSPDSSolution cand = S;

                    // remove from fromK
                    auto& rta = cand.truck_routes[fromK];
                    int erasePos = posIdx;
                    // if moving within same route and insertion before erase shifts index
                    // do stable: remove first, then insert
                    rta.erase(rta.begin() + erasePos);

                    // insert into toK
                    auto& rtb = cand.truck_routes[toK];
                    int insertPos = insPos;
                    if (fromK == toK && insPos > posIdx) insertPos = insPos - 1;
                    rtb.insert(rtb.begin() + insertPos, node);

                    // update mappings
                    cand.customer_truck[node] = toK;

                    evaluate(cand);
                    if (cand.makespan + 1e-12 < bestCost) {
                        bestCost = cand.makespan;
                        best = cand;
                        S = best; // first-improvement
                        return true;
                    }
                }
            }
        }
    }
    return false;
}

bool TwoPhaseMatheuristic::trySwap(MTSPDSSolution& S, double pi) {
    auto gran = buildGranularArcs(pi);

    std::vector<int> trucks(graph.KN);
    std::iota(trucks.begin(), trucks.end(), 0);
    std::shuffle(trucks.begin(), trucks.end(), rng);

    for (int k1 : trucks) {
        for (int k2 : trucks) {
            auto& r1 = S.truck_routes[k1];
            auto& r2 = S.truck_routes[k2];

            std::vector<int> i1;
            for (int i = 1; i + 1 < (int)r1.size(); ++i) {
                int v = r1[i];
                if (isMovableNode(graph, v)) i1.push_back(i);
            }
            std::vector<int> i2;
            for (int j = 1; j + 1 < (int)r2.size(); ++j) {
                int v = r2[j];
                if (isMovableNode(graph, v)) i2.push_back(j);
            }


            std::shuffle(i1.begin(), i1.end(), rng);
            std::shuffle(i2.begin(), i2.end(), rng);

            for (int a : i1) {
                int na = r1[a];
                for (int b : i2) {
                    int nb = r2[b];
                    if (k1 == k2 && a == b) continue;

                    // granular filter: check arcs created by swap around positions
                    int pa = r1[a - 1], sa = r1[a + 1];
                    int pb = r2[b - 1], sb = r2[b + 1];
                    bool ok = arcInGranular(pa, nb, gran) || arcInGranular(nb, sa, gran) ||
                        arcInGranular(pb, na, gran) || arcInGranular(na, sb, gran);
                    if (!ok) continue;

                    MTSPDSSolution cand = S;
                    std::swap(cand.truck_routes[k1][a], cand.truck_routes[k2][b]);
                    cand.customer_truck[na] = k2;
                    cand.customer_truck[nb] = k1;

                    evaluate(cand);
                    if (cand.makespan + 1e-12 < S.makespan) {
                        S = cand;
                        return true;
                    }
                }
            }
        }
    }
    return false;
}

bool TwoPhaseMatheuristic::tryMultirangeDroneRelocate(MTSPDSSolution& S) {
    // pick a drone-served customer that is in range of multiple active stations and move it
    std::vector<int> js;
    for (int j : graph.customers) if (S.customer_station[j] != -1) js.push_back(j);
    std::shuffle(js.begin(), js.end(), rng);

    for (int j : js) {
        int curS = S.customer_station[j];

        // enumerate other active stations that cover j
        std::vector<int> active;
        for (auto& kv : S.station_activation) active.push_back(kv.first);
        std::shuffle(active.begin(), active.end(), rng);

        for (int s2 : active) {
            if (s2 == curS) continue;
            if (!graph.droneEligible(s2, j)) continue;

            MTSPDSSolution cand = S;
            cand.customer_station[j] = s2; // changes station assignment
            evaluate(cand);
            if (cand.makespan + 1e-12 < S.makespan) {
                S = cand;
                return true;
            }
        }
    }
    return false;
}

bool TwoPhaseMatheuristic::tryDroneToTruckRelocate(MTSPDSSolution& S, double pi) {
    auto gran = buildGranularArcs(pi);

    std::vector<int> js;
    for (int j : graph.customers) if (S.customer_station[j] != -1) js.push_back(j);
    std::shuffle(js.begin(), js.end(), rng);

    std::vector<int> trucks(graph.KN);
    std::iota(trucks.begin(), trucks.end(), 0);
    std::shuffle(trucks.begin(), trucks.end(), rng);

    for (int j : js) {
        // remove from station -> insert into some truck
        for (int k : trucks) {
            auto& rt = S.truck_routes[k];
            std::vector<int> ins;
            for (int pos = 1; pos < (int)rt.size(); ++pos) ins.push_back(pos);
            std::shuffle(ins.begin(), ins.end(), rng);

            for (int pos : ins) {
                int prev = rt[pos - 1], next = rt[pos];
                if (!arcInGranular(prev, j, gran) && !arcInGranular(j, next, gran)) continue;

                MTSPDSSolution cand = S;
                cand.customer_station[j] = -1;
                cand.customer_truck[j] = k;
                cand.truck_routes[k].insert(cand.truck_routes[k].begin() + pos, j);

                evaluate(cand);
                if (cand.makespan + 1e-12 < S.makespan) {
                    S = cand;
                    return true;
                }
            }
        }
    }
    return false;
}

bool TwoPhaseMatheuristic::tryTruckToDroneRelocate(MTSPDSSolution& S) {
    // move a truck-served customer to some active station that covers it
    std::vector<int> trucks(graph.KN);
    std::iota(trucks.begin(), trucks.end(), 0);
    std::shuffle(trucks.begin(), trucks.end(), rng);

    std::vector<int> activeStations;
    for (auto& kv : S.station_activation) activeStations.push_back(kv.first);
    std::shuffle(activeStations.begin(), activeStations.end(), rng);

    for (int k : trucks) {
        auto& rt = S.truck_routes[k];
        std::vector<int> idxs;
        for (int i = 1;i + 1 < (int)rt.size();++i) if (graph.is_customer[rt[i]]) idxs.push_back(i);
        std::shuffle(idxs.begin(), idxs.end(), rng);

        for (int idx : idxs) {
            int j = rt[idx];
            if (graph.is_truck_only[j]) continue;

            for (int s : activeStations) {
                if (!graph.droneEligible(s, j)) continue;

                MTSPDSSolution cand = S;
                // erase from truck route
                cand.truck_routes[k].erase(cand.truck_routes[k].begin() + idx);
                cand.customer_truck[j] = -1;
                cand.customer_station[j] = s;

                evaluate(cand);
                if (cand.makespan + 1e-12 < S.makespan) {
                    S = cand;
                    return true;
                }
            }
        }
    }
    return false;
}

MTSPDSSolution TwoPhaseMatheuristic::shake(MTSPDSSolution S) {
    // paper: delete 50% customers, reinsert with min insertion cost (truck or drones):contentReference[oaicite:15]{index=15}
    std::vector<int> cust = graph.customers;
    std::shuffle(cust.begin(), cust.end(), rng);
    int removeN = (int)cust.size() / 2;
    std::unordered_set<int> removed;
    for (int i = 0;i < removeN;++i) removed.insert(cust[i]);

    // remove from trucks
    for (int k = 0;k < graph.KN;++k) {
        auto& rt = S.truck_routes[k];
        for (int i = (int)rt.size() - 2;i >= 1;--i) {
            int v = rt[i];
            if (removed.count(v)) {
                rt.erase(rt.begin() + i);
                S.customer_truck[v] = -1;
            }
        }
    }
    // remove from drones
    for (int j : removed) {
        S.customer_station[j] = -1;
    }
    evaluate(S);

    // active stations list
    std::vector<int> activeStations;
    for (auto& kv : S.station_activation) activeStations.push_back(kv.first);

    // reinsert each removed customer at best place
    for (int j : removed) {
        double bestCost = std::numeric_limits<double>::infinity();
        MTSPDSSolution best = S;

        // try insert into trucks
        for (int k = 0;k < graph.KN;++k) {
            auto& rt = S.truck_routes[k];
            for (int pos = 1; pos < (int)rt.size(); ++pos) {
                MTSPDSSolution cand = S;
                cand.truck_routes[k].insert(cand.truck_routes[k].begin() + pos, j);
                cand.customer_truck[j] = k;
                cand.customer_station[j] = -1;
                evaluate(cand);
                if (cand.makespan < bestCost) {
                    bestCost = cand.makespan;
                    best = cand;
                }
            }
        }

        // try assign to drones (only active stations)
        if (!graph.is_truck_only[j]) {
            for (int s : activeStations) {
                if (!graph.droneEligible(s, j)) continue;
                MTSPDSSolution cand = S;
                cand.customer_station[j] = s;
                cand.customer_truck[j] = -1;
                evaluate(cand);
                if (cand.makespan < bestCost) {
                    bestCost = cand.makespan;
                    best = cand;
                }
            }
        }

        S = best;
    }

    return S;
}

MTSPDSSolution TwoPhaseMatheuristic::improveSolution(const MTSPDSSolution& start) {
    double pi = params.pi_base;
    int i = 0;
    MTSPDSSolution Sbest = start;
    MTSPDSSolution S = start;

    while (true) {
        if (timeUp()) return Sbest;
		cout << "  Improve iteration " << (i + 1) << ", pi=" << pi << "\n";
        S = rvndLocalSearch(S, pi);   // ✅ rvndLocalSearch 内部也会检查 timeUp()

        if (timeUp()) return Sbest;

        if (S.makespan + 1e-12 < Sbest.makespan) {
            i = 0;
            Sbest = S;
            pi = params.pi_base;
        }

        i++;
        if (i >= params.delta) break;

        int step = (int)std::round(params.phi * params.delta);
        if (step < 1) step = 1;
        if (i % step == 0) {
            pi = params.lambda * pi;
        }

        if (timeUp()) return Sbest;

        S = Sbest;
        S = shake(S);  // ⚠️ shake 也可能较慢（可选：在 shake 内也加 timeUp 检查）
    }
    return Sbest;
}


MTSPDSSolution TwoPhaseMatheuristic::polish(const MTSPDSSolution& bestFromPhase1) {
    // Phase 2: set-partitioning MILP polish from pool (paper line 11):contentReference[oaicite:17]{index=17}:contentReference[oaicite:18]{index=18}
    // If CPLEX available: solve; else fallback to best in pool.
    SetPartitioningPolisher pol(graph);
    MTSPDSSolution out;
    if (pol.solve(pool, bestFromPhase1, out)) { //退化成单卡车，单无人机站时，这个操作其实没有优化
        evaluate(out);
        return out; 
    }

    // fallback: best among pool and bestFromPhase1
    MTSPDSSolution best = bestFromPhase1;
    for (auto& s : pool) {
        if (s.makespan < best.makespan) best = s;
    }
    return best;
}

MTSPDSSolution TwoPhaseMatheuristic::solve(double opt) {
    pool.clear();
    MTSPDSSolution best = emptySolution();

    using Clock = std::chrono::steady_clock;
    const auto t0 = Clock::now();
    deadline_ = t0 + std::chrono::duration_cast<Clock::duration>(
        std::chrono::duration<double>(params.time_limit_sec) 
    );

    for (int r = 0; r < params.rho; ++r) {
        if (timeUp()) break;

        cout << "Two-Phase Matheuristic: Restart " << (r + 1) << "/" << params.rho << "\n";

        MTSPDSSolution S = constructInitialSolution();
        if (!feasibleBasic(S)) continue;

        if (timeUp()) break;
        S = improveSolution(S);            // ✅ improveSolution 内部也会随时检查 timeUp()

        if (S.makespan < best.makespan) best = S;
        cout << "current best makespan: " << best.makespan << "\n";

        if (best.makespan == opt) {
            cout << "※※※达到已知最优解: " << best.makespan << "\n";
            return best;
        }
    }

    // 超时就别 polish；没超时再 polish
    if (!timeUp()) {
        best = polish(best);
    }
    return best;
}

