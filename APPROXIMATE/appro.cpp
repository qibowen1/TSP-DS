#include <bits/stdc++.h>
using namespace std;

/*
Input format:

N q m s
customer_1 customer_2 ... customer_q
g
truck_only_1 ... truck_only_g
d
drone_eligible_1 ... drone_eligible_d
truck_time_matrix N x N
drone_time_matrix N x N

Node meaning:
- depot is node 0
- drone station is node s
- C is the customer set
- Cg is the truck-only customer set
- Cd is the drone-eligible customer set
- tg[i][j] is truck travel time
- td[i][j] is one-way drone travel time from i to j
- each drone delivery is a round trip, so its processing time is 2 * td[s][j]

Compile:
g++ -std=c++17 -O2 main.cpp -o main

Run:
./main < input.txt
*/

static const double INF = 1e100;
static const double EPS = 1e-9;

static bool isInf(double x) {
    return x > INF / 10.0;
}

static double addSafe(double a, double b) {
    if (isInf(a) || isInf(b)) return INF;
    if (a + b >= INF / 10.0) return INF;
    return a + b;
}

static double pathCost(const vector<int>& path, const vector<vector<double>>& cost) {
    double ans = 0.0;
    for (int i = 0; i + 1 < (int)path.size(); ++i) {
        ans += cost[path[i]][path[i + 1]];
    }
    return ans;
}

[[maybe_unused]] static void printVector(const vector<int>& v) {
    if (v.empty()) {
        cout << "(empty)\n";
        return;
    }
    for (int i = 0; i < (int)v.size(); ++i) {
        if (i) cout << ' ';
        cout << v[i];
    }
    cout << '\n';
}

struct PCPathResult {
    vector<int> path;       // s -> ... -> 0
    vector<int> visited;    // customers visited by truck in this PC-path
    double cost = INF;      // truck path cost
};


class GWPrizeCollectingPath {
private:
    struct Component {
        bool alive = true;
        bool containsRoot = false;
        bool infiniteBudget = false;
        double remainingBudget = 0.0;
        vector<int> vertices;
    };

    int n;
    int root;
    int dest;

    vector<vector<double>> cost;
    vector<double> penalty;
    vector<int> customers;
    vector<char> isCustomer;

    vector<Component> comps;
    vector<int> compId;
    vector<double> potential;
    vector<tuple<int, int, double>> forestEdges;

    bool active(int cid) const {
        if (cid < 0 || cid >= (int)comps.size()) return false;
        const Component& c = comps[cid];
        if (!c.alive) return false;
        if (c.containsRoot) return false;
        if (c.infiniteBudget) return true;
        return c.remainingBudget > EPS;
    }

    void mergeComponents(int u, int v) {
        int a = compId[u];
        int b = compId[v];

        if (a == b) return;
        if (!comps[a].alive || !comps[b].alive) return;

        forestEdges.push_back({ u, v, cost[u][v] });

        Component merged;
        merged.alive = true;
        merged.containsRoot = comps[a].containsRoot || comps[b].containsRoot;
        merged.infiniteBudget = comps[a].infiniteBudget || comps[b].infiniteBudget;

        if (merged.infiniteBudget) {
            merged.remainingBudget = INF;
        }
        else {
            merged.remainingBudget =
                max(0.0, comps[a].remainingBudget) +
                max(0.0, comps[b].remainingBudget);
        }

        for (int x : comps[a].vertices) merged.vertices.push_back(x);
        for (int x : comps[b].vertices) merged.vertices.push_back(x);

        comps[a].alive = false;
        comps[b].alive = false;

        int newId = (int)comps.size();
        comps.push_back(merged);

        for (int x : comps[newId].vertices) {
            compId[x] = newId;
        }
    }

    double dropPenaltyOfNode(int v) const {
        if (v == root || v == dest) return INF;
        if (!isCustomer[v]) return 0.0;
        return penalty[v];
    }


    vector<int> pruneAndGetKeptNodes() {
        vector<vector<pair<int, double>>> adj(n);

        for (auto [u, v, w] : forestEdges) {
            adj[u].push_back({ v, w });
            adj[v].push_back({ u, w });
        }

        vector<int> parent(n, -2);
        vector<double> parentEdge(n, 0.0);
        vector<int> order;

        stack<int> st;
        st.push(root);
        parent[root] = -1;

        while (!st.empty()) {
            int u = st.top();
            st.pop();
            order.push_back(u);

            for (auto [v, w] : adj[u]) {
                if (parent[v] != -2) continue;
                parent[v] = u;
                parentEdge[v] = w;
                st.push(v);
            }
        }


        if (parent[dest] == -2) {
            vector<int> forced;
            forced.push_back(root);
            for (int j : customers) {
                if (j != root && j != dest && isInf(penalty[j])) {
                    forced.push_back(j);
                }
            }
            forced.push_back(dest);
            return forced;
        }

        reverse(order.begin(), order.end());

        vector<double> keepDP(n, INF);
        vector<double> dropDP(n, INF);

        for (int u : order) {
            double drop = dropPenaltyOfNode(u);
            double keep = 0.0;

            for (auto [v, w] : adj[u]) {
                if (parent[v] != u) continue;

                drop = addSafe(drop, dropDP[v]);

                double keepChild = addSafe(w, keepDP[v]);
                double bestChild = min(keepChild, dropDP[v]);
                keep = addSafe(keep, bestChild);
            }

            keepDP[u] = keep;
            dropDP[u] = drop;
        }

        vector<int> kept(n, 0);

        function<void(int)> reconstruct = [&](int u) {
            kept[u] = 1;

            for (auto [v, w] : adj[u]) {
                if (parent[v] != u) continue;

                double keepChild = addSafe(w, keepDP[v]);
                double dropChild = dropDP[v];

                if (keepChild <= dropChild + EPS) {
                    reconstruct(v);
                }
            }
            };

        reconstruct(root);

        vector<int> keptNodes;
        for (int v = 0; v < n; ++v) {
            if (kept[v]) keptNodes.push_back(v);
        }

        /*
            Robustness: all infinite-penalty customers must be kept.
        */
        vector<char> already(n, false);
        for (int v : keptNodes) already[v] = true;

        for (int j : customers) {
            if (j == root || j == dest) continue;
            if (isInf(penalty[j]) && !already[j]) {
                keptNodes.push_back(j);
                already[j] = true;
            }
        }

        return keptNodes;
    }

    /*
        Convert the pruned tree into an s -> 0 path.
    */
    vector<int> shortcutTreeToPath(const vector<int>& keptNodes) {
        vector<char> kept(n, false);
        for (int v : keptNodes) kept[v] = true;

        vector<vector<int>> adj(n);

        for (auto [u, v, w] : forestEdges) {
            if (kept[u] && kept[v]) {
                adj[u].push_back(v);
                adj[v].push_back(u);
            }
        }

        vector<int> par(n, -1);
        queue<int> q;
        q.push(root);
        par[root] = root;

        while (!q.empty()) {
            int u = q.front();
            q.pop();

            for (int v : adj[u]) {
                if (par[v] != -1) continue;
                par[v] = u;
                q.push(v);
            }
        }

  
        if (par[dest] == -1) {
            vector<int> path;
            path.push_back(root);

            vector<int> mid;
            for (int v : keptNodes) {
                if (v != root && v != dest && isCustomer[v]) {
                    mid.push_back(v);
                }
            }

            vector<char> used(n, false);
            int cur = root;

            while (!mid.empty()) {
                int bestIdx = 0;
                double bestCost = cost[cur][mid[0]];

                for (int i = 1; i < (int)mid.size(); ++i) {
                    if (cost[cur][mid[i]] < bestCost) {
                        bestCost = cost[cur][mid[i]];
                        bestIdx = i;
                    }
                }

                cur = mid[bestIdx];
                path.push_back(cur);
                mid.erase(mid.begin() + bestIdx);
            }

            path.push_back(dest);
            return path;
        }

        vector<int> mainPath;
        for (int x = dest; x != root; x = par[x]) {
            mainPath.push_back(x);
        }
        mainPath.push_back(root);
        reverse(mainPath.begin(), mainPath.end());

        vector<char> onMain(n, false);
        for (int v : mainPath) onMain[v] = true;

        vector<int> walk;

        function<void(int, int)> visitSideSubtree = [&](int u, int p) {
            for (int v : adj[u]) {
                if (v == p || onMain[v]) continue;
                walk.push_back(v);
                visitSideSubtree(v, u);
                walk.push_back(u);
            }
            };

        for (int i = 0; i < (int)mainPath.size(); ++i) {
            int u = mainPath[i];

            if (walk.empty() || walk.back() != u) {
                walk.push_back(u);
            }

            visitSideSubtree(u, -1);

            if (i + 1 < (int)mainPath.size()) {
                walk.push_back(mainPath[i + 1]);
            }
        }

        /*
            Shortcut repeated vertices.
        */
        vector<char> seen(n, false);
        vector<int> path;

        path.push_back(root);
        seen[root] = true;

        for (int v : walk) {
            if (v == root || v == dest) continue;
            if (!kept[v]) continue;
            if (!isCustomer[v]) continue;

            if (!seen[v]) {
                path.push_back(v);
                seen[v] = true;
            }
        }

        path.push_back(dest);
        return path;
    }

public:
    GWPrizeCollectingPath(
        int n_,
        int root_,
        int dest_,
        const vector<vector<double>>& cost_,
        const vector<double>& penalty_,
        const vector<int>& customers_,
        const vector<char>& isCustomer_
    ) : n(n_),
        root(root_),
        dest(dest_),
        cost(cost_),
        penalty(penalty_),
        customers(customers_),
        isCustomer(isCustomer_) {
    }

    PCPathResult solve() {
        comps.clear();
        compId.assign(n, -1);
        potential.assign(n, 0.0);
        forestEdges.clear();

        /*
            Initialization:
        */
        for (int v = 0; v < n; ++v) {
            Component c;
            c.alive = true;
            c.containsRoot = (v == root);
            c.vertices = { v };

            if (v == root) {
                c.remainingBudget = 0.0;
                c.infiniteBudget = false;
            }
            else if (v == dest) {
                c.remainingBudget = INF;
                c.infiniteBudget = true;
            }
            else if (isInf(penalty[v])) {
                c.remainingBudget = INF;
                c.infiniteBudget = true;
            }
            else {
                c.remainingBudget = max(0.0, penalty[v]);
                c.infiniteBudget = false;
            }

            int id = (int)comps.size();
            comps.push_back(c);
            compId[v] = id;
        }

        /*
            Growth phase.
        */
        int maxIterations = 10 * n * n + 1000;

        while (maxIterations--) {
            vector<int> activeComps;
            for (int i = 0; i < (int)comps.size(); ++i) {
                if (active(i)) activeComps.push_back(i);
            }

            if (activeComps.empty()) break;

            double bestDelta = INF;
            int eventType = 0; // 1=edge tight, 2=budget exhausted
            int edgeU = -1;
            int edgeV = -1;
            int budgetComp = -1;

            /*
                Budget event.
            */
            for (int cid : activeComps) {
                if (!comps[cid].infiniteBudget) {
                    double delta = comps[cid].remainingBudget;
                    if (delta + EPS < bestDelta) {
                        bestDelta = delta;
                        eventType = 2;
                        budgetComp = cid;
                    }
                }
            }

            /*
                Tight edge event.

                For an edge between two different current components:
                slack = c(u,v) - potential[u] - potential[v].
                The slack decreases at rate:
                active(comp(u)) + active(comp(v)).
            */
            for (int u = 0; u < n; ++u) {
                for (int v = u + 1; v < n; ++v) {
                    int cu = compId[u];
                    int cv = compId[v];

                    if (cu == cv) continue;

                    int rate = 0;
                    if (active(cu)) rate++;
                    if (active(cv)) rate++;

                    if (rate == 0) continue;

                    double slack = cost[u][v] - potential[u] - potential[v];
                    if (slack < 0.0) slack = 0.0;

                    double delta = slack / rate;

                    if (delta + EPS < bestDelta) {
                        bestDelta = delta;
                        eventType = 1;
                        edgeU = u;
                        edgeV = v;
                    }
                }
            }

            if (bestDelta >= INF / 2) break;

            /*
                Grow all active components by bestDelta.
            */
            for (int cid : activeComps) {
                if (!active(cid)) continue;

                for (int v : comps[cid].vertices) {
                    potential[v] += bestDelta;
                }

                if (!comps[cid].infiniteBudget) {
                    comps[cid].remainingBudget -= bestDelta;
                    if (comps[cid].remainingBudget < EPS) {
                        comps[cid].remainingBudget = 0.0;
                    }
                }
            }

            if (eventType == 1) {
                if (edgeU != -1 && edgeV != -1) {
                    mergeComponents(edgeU, edgeV);
                }
            }
            else if (eventType == 2) {
                if (budgetComp != -1 && comps[budgetComp].alive &&
                    !comps[budgetComp].infiniteBudget &&
                    comps[budgetComp].remainingBudget <= EPS) {
                    /*
                        It becomes inactive automatically because active()
                        checks remainingBudget > EPS.
                    */
                }
            }
        }

        vector<int> keptNodes = pruneAndGetKeptNodes();
        vector<int> pcPath = shortcutTreeToPath(keptNodes);

        PCPathResult result;
        result.path = pcPath;
        result.cost = pathCost(pcPath, cost);

        vector<char> inPath(n, false);
        for (int v : pcPath) inPath[v] = true;

        for (int j : customers) {
            if (j == root || j == dest) continue;
            if (inPath[j]) result.visited.push_back(j);
        }

        return result;
    }
};

struct DroneSchedule {
    vector<vector<int>> jobs;
    vector<double> load;
};

struct Solution {
    double surrogateObjective = INF; // obj_T
    double actualMakespan = INF;     // real TSP-DS objective alpha(S)
    double threshold = 0.0;

    vector<int> pcPath;              // s -> ... -> 0
    vector<int> truckTour;           // 0 -> s -> ... -> 0
    vector<int> truckServed;
    vector<int> droneServed;

    DroneSchedule droneSchedule;

    double pcPathCost = INF;
    double truckTourCost = INF;
    double droneActivationTime = 0.0;
    double droneMakespan = 0.0;
};

class FastApproxTSPDS {
private:
    int n;
    int depot = 0;
    int station;
    int m;

    vector<int> C;
    vector<int> Cg;
    vector<int> Cd;

    vector<vector<double>> tg;
    vector<vector<double>> td;

    vector<char> isCustomer;
    vector<char> isTruckOnly;
    vector<char> isDroneEligible;

    double droneOneWayTime(int customer) const {
        return td[station][customer];
    }

    double droneRoundTripTime(int customer) const {
        return 2.0 * td[station][customer];
    }

    DroneSchedule lptSchedule(const vector<int>& droneCustomers) const {
        DroneSchedule ds;
        ds.jobs.assign(m, {});
        ds.load.assign(m, 0.0);

        vector<int> jobs = droneCustomers;

        sort(jobs.begin(), jobs.end(), [&](int a, int b) {
            return droneRoundTripTime(a) > droneRoundTripTime(b);
            });

        priority_queue<pair<double, int>,
            vector<pair<double, int>>,
            greater<pair<double, int>>> pq;

        for (int k = 0; k < m; ++k) {
            pq.push({ 0.0, k });
        }

        for (int j : jobs) {
            auto [curLoad, droneId] = pq.top();
            pq.pop();

            ds.jobs[droneId].push_back(j);
            ds.load[droneId] += droneRoundTripTime(j);

            pq.push({ ds.load[droneId], droneId });
        }

        return ds;
    }

    double maxDroneLoad(const DroneSchedule& ds) const {
        double ans = 0.0;
        for (double x : ds.load) ans = max(ans, x);
        return ans;
    }

public:
    FastApproxTSPDS(
        int n_,
        int station_,
        int m_,
        vector<int> C_,
        vector<int> Cg_,
        vector<int> Cd_,
        vector<vector<double>> tg_,
        vector<vector<double>> td_
    ) : n(n_),
        station(station_),
        m(m_),
        C(std::move(C_)),
        Cg(std::move(Cg_)),
        Cd(std::move(Cd_)),
        tg(std::move(tg_)),
        td(std::move(td_)) {

        isCustomer.assign(n, false);
        isTruckOnly.assign(n, false);
        isDroneEligible.assign(n, false);

        for (int x : C) {
            if (0 <= x && x < n) isCustomer[x] = true;
        }

        for (int x : Cg) {
            if (0 <= x && x < n) isTruckOnly[x] = true;
        }

        for (int x : Cd) {
            if (0 <= x && x < n) isDroneEligible[x] = true;
        }
    }

    Solution solve() {
        if (m <= 0) {
            throw runtime_error("number of drones m must be positive");
        }

        /*
            set beta_j.
        */
        vector<double> beta(n, 0.0);

        for (int j : C) {
            if (j == depot || j == station) continue;

            if (isTruckOnly[j] || !isDroneEligible[j]) {
                beta[j] = INF;
            }
            else {
                beta[j] = 2.0 * droneOneWayTime(j) / (double)m;
            }
        }

        /*
            enumerate T in {td[s][i] | i in Cd}.
        */
        vector<double> thresholds;

        for (int j : Cd) {
            if (!isCustomer[j]) continue;
            thresholds.push_back(droneOneWayTime(j));
        }

        if (thresholds.empty()) {
            thresholds.push_back(0.0);
        }

        sort(thresholds.begin(), thresholds.end());

        vector<double> uniqueThresholds;
        for (double x : thresholds) {
            if (uniqueThresholds.empty() ||
                fabs(x - uniqueThresholds.back()) > EPS) {
                uniqueThresholds.push_back(x);
            }
        }

        Solution best;

        for (double T : uniqueThresholds) {
            vector<double> betaPrime = beta;

            /*
                if td[s][j] > T, force truck to visit j.
            */
            for (int j : Cd) {
                if (!isCustomer[j]) continue;

                if (droneOneWayTime(j) > T + EPS) {
                    betaPrime[j] = INF;
                }
            }

            /*
                construct PC-path instance and call GW.
            */
            GWPrizeCollectingPath gw(
                n,
                station,
                depot,
                tg,
                betaPrime,
                C,
                isCustomer
            );

            PCPathResult pc = gw.solve();

            vector<char> visited(n, false);
            for (int v : pc.path) visited[v] = true;

            bool feasible = true;
            double penaltySum = 0.0;
            vector<int> truckServed;

            /*
                obj_T = path cost + penalties of unvisited customers + T.
            */
            for (int j : C) {
                if (j == depot || j == station) continue;

                if (visited[j]) {
                    truckServed.push_back(j);
                }
                else {
                    if (isInf(betaPrime[j])) {
                        feasible = false;
                        break;
                    }
                    penaltySum += betaPrime[j];
                }
            }

            if (!feasible) continue;

            double objT = pc.cost + penaltySum + T;

            if (objT + EPS < best.surrogateObjective) {
                best.surrogateObjective = objT;
                best.threshold = T;
                best.pcPath = pc.path;
                best.pcPathCost = pc.cost;
                best.truckServed = truckServed;
            }
        }

        if (best.pcPath.empty()) {
            throw runtime_error("failed to construct a feasible PC-path");
        }

        /*
            attach depot as first node.

            PC-path:    s -> ... -> 0
            Truck tour: 0 -> s -> ... -> 0
        */
        best.truckTour.clear();
        best.truckTour.push_back(depot);

        for (int v : best.pcPath) {
            best.truckTour.push_back(v);
        }

        best.truckTourCost = pathCost(best.truckTour, tg);

        /*
            Cd \ truckTour are served by drones using LPT.
        */
        vector<char> inTruckTour(n, false);
        for (int v : best.truckTour) inTruckTour[v] = true;

        for (int j : Cd) {
            if (!isCustomer[j]) continue;

            if (!inTruckTour[j]) {
                best.droneServed.push_back(j);
            }
        }

        best.droneSchedule = lptSchedule(best.droneServed);
        best.droneMakespan = maxDroneLoad(best.droneSchedule);

        /*
            Since Algorithm 1 forces the truck to visit station s first,
            drone station activation time is tg[0][s].
        */
        best.droneActivationTime = tg[depot][station];

        /*
            Real TSP-DS objective:
            alpha(S) = max{truck makespan, activation time + drone makespan}.
        */
        best.actualMakespan = max(
            best.truckTourCost,
            best.droneActivationTime + best.droneMakespan
        );

        return best;
    }
};

struct CsvNode {
    int id = -1;
    double x = 0.0;
    double y = 0.0;
    int type = -1;
};

static string trim(const string& s) {
    size_t l = 0;
    while (l < s.size() && isspace((unsigned char)s[l])) ++l;
    size_t r = s.size();
    while (r > l && isspace((unsigned char)s[r - 1])) --r;
    return s.substr(l, r - l);
}

static vector<CsvNode> readCsvNodes(const string& csvPath) {
    ifstream fin(csvPath);
    if (!fin) {
        throw runtime_error("cannot open CSV file: " + csvPath);
    }

    vector<CsvNode> rows;
    string line;
    int lineNo = 0;

    while (getline(fin, line)) {
        ++lineNo;
        line = trim(line);
        if (line.empty()) continue;


        for (char& ch : line) {
            if (ch == ',') ch = ' ';
        }

        stringstream ss(line);
        CsvNode node;
        if (!(ss >> node.id >> node.x >> node.y >> node.type)) {
            // If the CSV has a header, skip only the first non-empty line.
            if (rows.empty() && lineNo == 1) continue;
            throw runtime_error("invalid CSV row at line " + to_string(lineNo));
        }

        rows.push_back(node);
    }

    if (rows.size() < 3) {
        throw runtime_error("CSV must contain at least: start depot, one node, end depot");
    }

    return rows;
}

static double manhattanDistance(const CsvNode& a, const CsvNode& b) {
    return fabs(a.x - b.x) + fabs(a.y - b.y);
}

static double euclideanDistance(const CsvNode& a, const CsvNode& b) {
    double dx = a.x - b.x;
    double dy = a.y - b.y;
    return sqrt(dx * dx + dy * dy);
}

static vector<vector<double>> buildTruckTimeMatrix(
    const vector<CsvNode>& nodes,
    double truckTimeFactor
) {
    int N = (int)nodes.size();
    vector<vector<double>> tg(N, vector<double>(N, 0.0));

    for (int i = 0; i < N; ++i) {
        for (int j = 0; j < N; ++j) {
            tg[i][j] = manhattanDistance(nodes[i], nodes[j]) * truckTimeFactor;
        }
    }

    return tg;
}

static vector<vector<double>> buildDroneTimeMatrix(
    const vector<CsvNode>& nodes,
    double droneTimeFactor
) {
    int N = (int)nodes.size();
    vector<vector<double>> td(N, vector<double>(N, 0.0));

    for (int i = 0; i < N; ++i) {
        for (int j = 0; j < N; ++j) {
            td[i][j] = euclideanDistance(nodes[i], nodes[j]) * droneTimeFactor;
        }
    }

    return td;
}

[[maybe_unused]] static void printVectorByOriginalId(
    const vector<int>& internalNodes,
    const vector<int>& originalId
) {
    if (internalNodes.empty()) {
        cout << "(empty)\n";
        return;
    }

    for (int i = 0; i < (int)internalNodes.size(); ++i) {
        if (i) cout << ' ';
        int v = internalNodes[i];
        if (0 <= v && v < (int)originalId.size()) {
            cout << originalId[v];
        }
        else {
            cout << v;
        }
    }
    cout << '\n';
}


static string getBaseName(const string& path) {
    size_t pos1 = path.find_last_of('/');
    size_t pos2 = path.find_last_of('\\');
    size_t pos = string::npos;

    if (pos1 == string::npos) pos = pos2;
    else if (pos2 == string::npos) pos = pos1;
    else pos = max(pos1, pos2);

    if (pos == string::npos) return path;
    return path.substr(pos + 1);
}

static string csvEscape(const string& s) {
    bool needQuote = false;
    for (char c : s) {
        if (c == ',' || c == '"' || c == '\n' || c == '\r') {
            needQuote = true;
            break;
        }
    }

    string out;
    for (char c : s) {
        if (c == '"') out += "\"\"";
        else out.push_back(c);
    }

    if (needQuote) return "\"" + out + "\"";
    return out;
}

static string joinVectorByOriginalId(
    const vector<int>& internalNodes,
    const vector<int>& originalId,
    const string& sep = " "
) {
    string out;

    for (int i = 0; i < (int)internalNodes.size(); ++i) {
        if (i) out += sep;

        int v = internalNodes[i];
        if (0 <= v && v < (int)originalId.size()) {
            out += to_string(originalId[v]);
        }
        else {
            out += to_string(v);
        }
    }

    return out;
}

static string droneScheduleToString(
    const DroneSchedule& schedule,
    const vector<int>& originalId
) {
    string out;

    for (int k = 0; k < (int)schedule.jobs.size(); ++k) {
        if (k) out += " | ";

        out += "Drone";
        out += to_string(k);
        out += "(load=";

        ostringstream oss;
        oss << fixed << setprecision(6) << schedule.load[k];
        out += oss.str();

        out += ":";
        out += joinVectorByOriginalId(schedule.jobs[k], originalId, " ");
        out += ")";
    }

    return out;
}

static bool fileExistsAndNotEmpty(const string& filePath) {
    ifstream fin(filePath, ios::binary);
    if (!fin) return false;

    fin.seekg(0, ios::end);
    return fin.tellg() > 0;
}

static void printCsvHeader(ostream& out) {
    out << "Instance,"
        << "DepotPosition,"
        << "DroneCount,"
        << "RunId,"
        << "Seed,"
        << "Nodes,"
        << "Customers,"
        << "DroneEligibleCustomers,"
        << "TruckOnlyCustomers,"
        << "DepotOriginalId,"
        << "StationOriginalId,"
        << "TruckTimeFactor,"
        << "DroneTimeFactor,"
        << "BestThreshold,"
        << "SurrogateObjective,"
        << "PCPathCost,"
        << "TruckTourCost,"
        << "DroneActivationTime,"
        << "DroneMakespan,"
        << "Makespan,"
        << "PCPath,"
        << "TruckTour,"
        << "TruckServedCustomers,"
        << "DroneServedCustomers,"
        << "DroneSchedule,"
        << "RuntimeSeconds"
        << '\n';
}

static void printCsvRow(
    ostream& out,
    const string& csvPath,
    int m,
    int N,
    int customers,
    int droneEligibleCustomers,
    int truckOnlyCustomers,
    int depotOriginalId,
    int stationOriginalId,
    double truckTimeFactor,
    double droneTimeFactor,
    const Solution& sol,
    const vector<int>& originalId,
    double runtimeSeconds
) {

    string instance = getBaseName(csvPath);
    string depotPosition = "L";
    int runId = 0;
    int seed = 0;

    out << fixed << setprecision(6);

    out << csvEscape(instance) << ','
        << depotPosition << ','
        << m << ','
        << runId << ','
        << seed << ','
        << N << ','
        << customers << ','
        << droneEligibleCustomers << ','
        << truckOnlyCustomers << ','
        << depotOriginalId << ','
        << stationOriginalId << ','
        << truckTimeFactor << ','
        << droneTimeFactor << ','
        << sol.threshold << ','
        << sol.surrogateObjective << ','
        << sol.pcPathCost << ','
        << sol.truckTourCost << ','
        << sol.droneActivationTime << ','
        << sol.droneMakespan << ','
        << sol.actualMakespan << ','
        << csvEscape(joinVectorByOriginalId(sol.pcPath, originalId, " ")) << ','
        << csvEscape(joinVectorByOriginalId(sol.truckTour, originalId, " ")) << ','
        << csvEscape(joinVectorByOriginalId(sol.truckServed, originalId, " ")) << ','
        << csvEscape(joinVectorByOriginalId(sol.droneServed, originalId, " ")) << ','
        << csvEscape(droneScheduleToString(sol.droneSchedule, originalId)) << ','
        << runtimeSeconds
        << '\n';
}

static void printUsage(const char* programName) {
    cerr << "Usage:\n";
    cerr << "  " << programName
         << " <csv_file> <number_of_drones_m> [truck_time_factor] [drone_time_factor] [output_csv]\n\n";
    cerr << "Examples:\n";
    cerr << "  " << programName << " att48_0_80.csv 3 1 0.5\n";
    cerr << "  " << programName << " att48_0_80.csv 3 1 0.5 result.csv\n\n";
    cerr << "If output_csv is provided, the program appends one row to it.\n";
    cerr << "The header is written only when the file does not exist or is empty.\n\n";
    cerr << "CSV format, one node per row:\n";
    cerr << "  node_id,x,y,type\n\n";
    cerr << "Type meaning for non-depot rows:\n";
    cerr << "  0 = drone-eligible customer\n";
    cerr << "  1 = truck-only customer\n";
    cerr << "  2 = drone station\n\n";
    cerr << "The first row is treated as the depot.\n";
    cerr << "The last row is also depot coordinates and is ignored in this single-depot implementation.\n";
    cerr << "Truck time = Manhattan distance * truck_time_factor.\n";
    cerr << "Drone time = Euclidean distance * drone_time_factor.\n";
}

int main(int argc, char* argv[]) {
    ios::sync_with_stdio(false);
    cin.tie(nullptr);

    clock_t Start_time = clock();


    if (argc < 3 || argc > 6) {
        printUsage(argv[0]);
        return 1;
    }

    string csvPath = argv[1];
    int m = stoi(argv[2]);
    double truckTimeFactor = 1.0;
    double droneTimeFactor = 1.0;
    string outputCsvPath;

    if (argc >= 4) truckTimeFactor = stod(argv[3]);
    if (argc >= 5) droneTimeFactor = stod(argv[4]);
    if (argc >= 6) outputCsvPath = argv[5];

    if (m <= 0) {
        cerr << "Error: number_of_drones_m must be positive.\n";
        return 1;
    }
    if (truckTimeFactor < 0.0 || droneTimeFactor < 0.0) {
        cerr << "Error: time factors must be non-negative.\n";
        return 1;
    }

    try {
        vector<CsvNode> rows = readCsvNodes(csvPath);

        const CsvNode& firstDepot = rows.front();
        const CsvNode& lastDepot = rows.back();

        if (fabs(firstDepot.x - lastDepot.x) > EPS ||
            fabs(firstDepot.y - lastDepot.y) > EPS) {
            cerr << "Warning: first and last depot coordinates are different. "
                 << "This implementation uses the first row as the single depot and ignores the last row.\n";
        }


        vector<CsvNode> nodes;
        nodes.reserve(rows.size() - 1);
        for (int i = 0; i + 1 < (int)rows.size(); ++i) {
            nodes.push_back(rows[i]);
        }

        int N = (int)nodes.size();
        int station = -1;

        vector<int> originalId(N);
        for (int i = 0; i < N; ++i) {
            originalId[i] = nodes[i].id;
        }

        vector<int> C;
        vector<int> Cg;
        vector<int> Cd;

        for (int i = 1; i < N; ++i) {
            int type = nodes[i].type;

            if (type == 0) {
                C.push_back(i);
                Cd.push_back(i);
            }
            else if (type == 1) {
                C.push_back(i);
                Cg.push_back(i);
            }
            else if (type == 2) {
                if (station != -1) {
                    throw runtime_error("CSV contains more than one drone station, type = 2");
                }
                station = i;
            }
            else {
                throw runtime_error(
                    "invalid node type " + to_string(type) +
                    " at CSV node_id " + to_string(nodes[i].id)
                );
            }
        }

        if (station == -1) {
            throw runtime_error("CSV does not contain a drone station row with type = 2");
        }

        vector<vector<double>> tg = buildTruckTimeMatrix(nodes, truckTimeFactor);
        vector<vector<double>> td = buildDroneTimeMatrix(nodes, droneTimeFactor);

        FastApproxTSPDS solver(
            N,
            station,
            m,
            C,
            Cg,
            Cd,
            tg,
            td
        );

        Solution sol = solver.solve();

        clock_t End_time = clock();
        double runtimeSeconds = double(End_time - Start_time) / CLOCKS_PER_SEC;

        if (outputCsvPath.empty()) {
            // No output file is provided: print a complete CSV table to the terminal.
            printCsvHeader(cout);
            printCsvRow(
                cout,
                csvPath,
                m,
                N,
                (int)C.size(),
                (int)Cd.size(),
                (int)Cg.size(),
                originalId[0],
                originalId[station],
                truckTimeFactor,
                droneTimeFactor,
                sol,
                originalId,
                runtimeSeconds
            );
        }
        else {
            // Output file is provided: append one result row.
            // Header is written only once, when the file is new or empty.
            bool needHeader = !fileExistsAndNotEmpty(outputCsvPath);

            ofstream fout(outputCsvPath, ios::app);
            if (!fout) {
                throw runtime_error("cannot open output CSV file: " + outputCsvPath);
            }

            if (needHeader) {
                printCsvHeader(fout);
            }

            printCsvRow(
                fout,
                csvPath,
                m,
                N,
                (int)C.size(),
                (int)Cd.size(),
                (int)Cg.size(),
                originalId[0],
                originalId[station],
                truckTimeFactor,
                droneTimeFactor,
                sol,
                originalId,
                runtimeSeconds
            );
        }
    }
    catch (const exception& e) {
        cerr << "Error: " << e.what() << '\n';
        return 1;
    }

    return 0;
}
