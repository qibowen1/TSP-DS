#include "SetPartitioningPolisher.h"
#include <algorithm>
#include <cstdint>
#include <functional>
#include <limits>
#include <numeric>
#include <unordered_map>
#include <unordered_set>

using Bits = std::vector<std::uint64_t>;

struct RouteColumn {
    int from_solution = -1;
    int truck_id = -1;
    double cost = 0.0;
    std::vector<int> cover_customers;
    std::vector<int> used_stations;
    std::vector<int> tour;
    std::vector<int> customer_station;
    Bits customer_bits;
    Bits station_bits;
    int station_count = 0;
};

static inline bool contains(const std::vector<int>& a, int x) {
    return std::find(a.begin(), a.end(), x) != a.end();
}

static inline void setBit(Bits& bits, int idx) {
    bits[(std::size_t)idx / 64] |= (std::uint64_t{1} << (idx % 64));
}

static inline bool testBit(const Bits& bits, int idx) {
    return (bits[(std::size_t)idx / 64] & (std::uint64_t{1} << (idx % 64))) != 0;
}

static inline int popcount64(std::uint64_t x) {
    int c = 0;
    while (x) {
        x &= (x - 1);
        ++c;
    }
    return c;
}

static inline int bitCount(const Bits& bits) {
    int total = 0;
    for (std::uint64_t x : bits) total += popcount64(x);
    return total;
}

static inline bool bitsEmpty(const Bits& bits) {
    for (std::uint64_t x : bits) {
        if (x != 0) return false;
    }
    return true;
}

static inline bool bitsEqual(const Bits& a, const Bits& b) {
    return a == b;
}

static inline bool bitsIntersect(const Bits& a, const Bits& b) {
    for (std::size_t i = 0; i < a.size(); ++i) {
        if ((a[i] & b[i]) != 0) return true;
    }
    return false;
}

static inline Bits bitsMerged(Bits a, const Bits& b) {
    for (std::size_t i = 0; i < a.size(); ++i) a[i] |= b[i];
    return a;
}

bool SetPartitioningPolisher::solve(const std::vector<MTSPDSSolution>& pool,
    const MTSPDSSolution& incumbent,
    MTSPDSSolution& out) {
    const int J = (int)graph.customers.size();
    const int Snum = (int)graph.stations.size();
    const int customerWords = (J + 63) / 64;
    const int stationWords = (Snum + 63) / 64;

    std::unordered_map<int, int> cust2idx;
    for (int i = 0; i < J; ++i) cust2idx[graph.customers[i]] = i;

    std::unordered_map<int, int> st2idx;
    for (int i = 0; i < Snum; ++i) st2idx[graph.stations[i]] = i;

    Bits fullCustomers((std::size_t)customerWords, 0);
    for (int i = 0; i < J; ++i) setBit(fullCustomers, i);

    std::vector<MTSPDSSolution> all = pool;
    all.push_back(incumbent);

    std::vector<RouteColumn> cols;
    cols.reserve(all.size() * (std::size_t)graph.KN);

    for (int si = 0; si < (int)all.size(); ++si) {
        const auto& S = all[si];
        for (int k = 0; k < graph.KN; ++k) {
            RouteColumn c;
            c.from_solution = si;
            c.truck_id = k;
            if (k < (int)S.truck_routes.size()) c.tour = S.truck_routes[k];
            c.customer_station = S.customer_station;
            c.customer_bits.assign((std::size_t)customerWords, 0);
            c.station_bits.assign((std::size_t)stationWords, 0);

            std::unordered_set<int> st;
            for (int v : c.tour) {
                if (v >= 0 && v < (int)graph.is_station.size() && graph.is_station[v]) {
                    st.insert(v);
                }
            }
            c.used_stations.assign(st.begin(), st.end());
            for (int s : c.used_stations) {
                auto it = st2idx.find(s);
                if (it != st2idx.end()) setBit(c.station_bits, it->second);
            }
            c.station_count = bitCount(c.station_bits);

            std::unordered_set<int> cov;
            for (int v : c.tour) {
                if (v >= 0 && v < (int)graph.is_customer.size() && graph.is_customer[v]) {
                    cov.insert(v);
                }
            }
            for (int j : graph.customers) {
                int s = (j >= 0 && j < (int)S.customer_station.size()) ? S.customer_station[j] : -1;
                if (s != -1 && st.count(s)) cov.insert(j);
            }
            c.cover_customers.assign(cov.begin(), cov.end());
            for (int j : c.cover_customers) {
                auto it = cust2idx.find(j);
                if (it != cust2idx.end()) setBit(c.customer_bits, it->second);
            }

            double ttruck = 0.0;
            for (int i = 0; i + 1 < (int)c.tour.size(); ++i) {
                ttruck += graph.truck_time[c.tour[i]][c.tour[i + 1]];
            }

            double tstation = 0.0;
            for (int s : c.used_stations) {
                auto it = S.station_completion.find(s);
                if (it != S.station_completion.end()) tstation = std::max(tstation, it->second);
            }
            c.cost = std::max(ttruck, tstation);

            cols.push_back(std::move(c));
        }
    }

    const int R = (int)cols.size();
    if (R < graph.KN) return false;

    auto selectionFeasible = [&](const std::vector<int>& sel, double& maxCost) {
        if ((int)sel.size() != graph.KN) return false;

        Bits covered((std::size_t)customerWords, 0);
        Bits stations((std::size_t)stationWords, 0);
        int usedStations = 0;
        maxCost = 0.0;

        for (int idx : sel) {
            if (idx < 0 || idx >= R) return false;
            const auto& c = cols[idx];
            if (bitsIntersect(covered, c.customer_bits)) return false;
            if (bitsIntersect(stations, c.station_bits)) return false;
            if (usedStations + c.station_count > graph.C) return false;

            covered = bitsMerged(std::move(covered), c.customer_bits);
            stations = bitsMerged(std::move(stations), c.station_bits);
            usedStations += c.station_count;
            maxCost = std::max(maxCost, c.cost);
        }
        return bitsEqual(covered, fullCustomers);
    };

    std::vector<int> bestSel;
    double bestCost = std::numeric_limits<double>::infinity();

    for (int si = 0; si < (int)all.size(); ++si) {
        std::vector<int> sel;
        sel.reserve((std::size_t)graph.KN);
        for (int k = 0; k < graph.KN; ++k) sel.push_back(si * graph.KN + k);

        double cost = 0.0;
        if (selectionFeasible(sel, cost) && cost < bestCost) {
            bestCost = cost;
            bestSel = std::move(sel);
        }
    }

    std::vector<int> order(R);
    std::iota(order.begin(), order.end(), 0);
    std::stable_sort(order.begin(), order.end(), [&](int a, int b) {
        return cols[a].cost < cols[b].cost;
    });

    std::vector<char> include((std::size_t)R, 0);
    int candidateLimit = R;
    if (graph.KN >= 5) candidateLimit = std::min(R, 800);
    else if (graph.KN >= 3) candidateLimit = std::min(R, 1500);

    for (int i = 0; i < candidateLimit; ++i) include[order[i]] = 1;
    if (graph.KN <= 2) {
        std::fill(include.begin(), include.end(), 1);
    }
    for (int idx : bestSel) include[idx] = 1;

    std::vector<int> candidateIdx;
    candidateIdx.reserve((std::size_t)R);
    for (int idx : order) {
        if (include[idx]) candidateIdx.push_back(idx);
    }

    std::vector<std::vector<int>> custToCols((std::size_t)J);
    for (int idx : candidateIdx) {
        for (int j : cols[idx].cover_customers) {
            auto it = cust2idx.find(j);
            if (it != cust2idx.end()) custToCols[it->second].push_back(idx);
        }
    }

    std::vector<int> current;
    current.reserve((std::size_t)graph.KN);
    std::uint64_t nodes = 0;
    const std::uint64_t nodeLimit = (graph.KN <= 2) ? 1000000ULL : 250000ULL;

    auto alreadyChosen = [&](int idx) {
        return std::find(current.begin(), current.end(), idx) != current.end();
    };

    auto compatible = [&](int idx, const Bits& covered, const Bits& stations, int usedStations) {
        const auto& c = cols[idx];
        if (alreadyChosen(idx)) return false;
        if (bitsIntersect(covered, c.customer_bits)) return false;
        if (bitsIntersect(stations, c.station_bits)) return false;
        if (usedStations + c.station_count > graph.C) return false;
        return true;
    };

    auto updateBest = [&]() {
        double cost = 0.0;
        if (selectionFeasible(current, cost) && cost + 1e-12 < bestCost) {
            bestCost = cost;
            bestSel = current;
        }
    };

    std::function<void(std::size_t, Bits, int, double)> finishWithEmpty =
        [&](std::size_t start, Bits stations, int usedStations, double currentMax) {
            if (++nodes > nodeLimit) return;
            if (currentMax + 1e-12 >= bestCost) return;

            if ((int)current.size() == graph.KN) {
                updateBest();
                return;
            }

            for (std::size_t p = start; p < candidateIdx.size(); ++p) {
                int idx = candidateIdx[p];
                if (!bitsEmpty(cols[idx].customer_bits)) continue;
                if (!compatible(idx, fullCustomers, stations, usedStations)) continue;

                current.push_back(idx);
                finishWithEmpty(p + 1,
                    bitsMerged(stations, cols[idx].station_bits),
                    usedStations + cols[idx].station_count,
                    std::max(currentMax, cols[idx].cost));
                current.pop_back();
            }
        };

    std::function<void(Bits, Bits, int, double)> dfs =
        [&](Bits covered, Bits stations, int usedStations, double currentMax) {
            if (++nodes > nodeLimit) return;
            if (currentMax + 1e-12 >= bestCost) return;
            if ((int)current.size() > graph.KN) return;

            if (bitsEqual(covered, fullCustomers)) {
                if ((int)current.size() == graph.KN) {
                    updateBest();
                }
                else {
                    finishWithEmpty(0, stations, usedStations, currentMax);
                }
                return;
            }

            if ((int)current.size() == graph.KN) return;

            int chosenCustomer = -1;
            int bestCandidateCount = std::numeric_limits<int>::max();

            for (int jj = 0; jj < J; ++jj) {
                if (testBit(covered, jj)) continue;

                int count = 0;
                for (int idx : custToCols[jj]) {
                    double nextMax = std::max(currentMax, cols[idx].cost);
                    if (nextMax + 1e-12 >= bestCost) continue;
                    if (compatible(idx, covered, stations, usedStations)) ++count;
                }

                if (count == 0) return;
                if (count < bestCandidateCount) {
                    bestCandidateCount = count;
                    chosenCustomer = jj;
                    if (count == 1) break;
                }
            }

            if (chosenCustomer == -1) return;

            for (int idx : custToCols[chosenCustomer]) {
                double nextMax = std::max(currentMax, cols[idx].cost);
                if (nextMax + 1e-12 >= bestCost) continue;
                if (!compatible(idx, covered, stations, usedStations)) continue;

                current.push_back(idx);
                dfs(bitsMerged(covered, cols[idx].customer_bits),
                    bitsMerged(stations, cols[idx].station_bits),
                    usedStations + cols[idx].station_count,
                    nextMax);
                current.pop_back();

                if (nodes > nodeLimit) return;
            }
        };

    Bits emptyCustomers((std::size_t)customerWords, 0);
    Bits emptyStations((std::size_t)stationWords, 0);
    dfs(emptyCustomers, emptyStations, 0, 0.0);

    if ((int)bestSel.size() != graph.KN) return false;

    MTSPDSSolution S;
    S.initialize((int)graph.nodes.size(), graph.KN);

    for (int k = 0; k < graph.KN; ++k) {
        S.truck_routes[k] = cols[bestSel[k]].tour;
    }

    for (int j : graph.customers) {
        S.customer_station[j] = -1;
        S.customer_truck[j] = -1;
    }

    for (int k = 0; k < graph.KN; ++k) {
        for (int v : S.truck_routes[k]) {
            if (v >= 0 && v < (int)graph.is_customer.size() && graph.is_customer[v]) {
                S.customer_truck[v] = k;
                S.customer_station[v] = -1;
            }
        }
    }

    for (int j : graph.customers) {
        if (S.customer_truck[j] != -1) continue;

        for (int rSel : bestSel) {
            if (!contains(cols[rSel].cover_customers, j)) continue;
            int s = (j >= 0 && j < (int)cols[rSel].customer_station.size())
                ? cols[rSel].customer_station[j]
                : -1;
            if (s != -1) {
                S.customer_station[j] = s;
                break;
            }
        }

        if (S.customer_station[j] == -1) return false;
    }

    out = std::move(S);
    return true;
}
