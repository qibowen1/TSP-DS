#include "TSPHeuristics.h"
#include <algorithm>
#include <cmath>

namespace tsp_heur {

    static inline double angleFromDepot(const MTSPDSGraph& g, int node) {
        auto& d = g.nodes[g.depot];
        auto& p = g.nodes[node];
        return std::atan2(p.second - d.second, p.first - d.first);
    }

    std::vector<std::vector<int>> sweepAssign(
        const MTSPDSGraph& g,
        const std::vector<int>& selectedStations,
        const std::vector<int>& remainingCustomers,
        std::mt19937& rng
    ) {
        std::vector<int> all = remainingCustomers;
        all.insert(all.end(), selectedStations.begin(), selectedStations.end());

        std::vector<std::pair<double, int>> ang;
        ang.reserve(all.size());
        for (int v : all) ang.push_back({ angleFromDepot(g,v), v });
        std::sort(ang.begin(), ang.end(), [](auto& a, auto& b) { return a.first < b.first; });

        std::vector<int> seq;
        seq.reserve(ang.size());
        for (auto& it : ang) seq.push_back(it.second);

        std::uniform_int_distribution<int> dist(0, (int)seq.size() - 1);
        int start = seq.empty() ? 0 : dist(rng);
        std::rotate(seq.begin(), seq.begin() + start, seq.end());

        std::vector<std::vector<int>> bucket(g.KN);
        for (int i = 0;i < (int)seq.size();++i) {
            bucket[i % g.KN].push_back(seq[i]);
        }
        return bucket;
    }

    std::vector<int> tspNearestNeighbor(const MTSPDSGraph& g, const std::vector<int>& nodes, int depot) {
        std::vector<int> unvis = nodes;
        std::vector<int> tour;
        tour.push_back(depot);

        int cur = depot;
        while (!unvis.empty()) {
            int bestIdx = 0;
            double best = 1e100;
            for (int i = 0;i < (int)unvis.size();++i) {
                int v = unvis[i];
                double c = g.truck_time[cur][v];
                if (c < best) { best = c; bestIdx = i; }
            }
            int nxt = unvis[bestIdx];
            unvis.erase(unvis.begin() + bestIdx);
            tour.push_back(nxt);
            cur = nxt;
        }
        tour.push_back(depot);
        return tour;
    }

    double tourTime(const MTSPDSGraph& g, const std::vector<int>& tour) {
        double t = 0.0;
        for (int i = 0;i + 1 < (int)tour.size();++i) t += g.truck_time[tour[i]][tour[i + 1]];
        return t;
    }

    void twoOptImprove(const MTSPDSGraph& g, std::vector<int>& tour, int maxIters) {
        if (tour.size() <= 4) return;
        int n = (int)tour.size();
        int iters = 0;
        while (iters++ < maxIters) {
            bool improved = false;
            for (int i = 1;i < n - 2 && !improved;++i) {
                for (int k = i + 1;k < n - 1 && !improved;++k) {
                    int a = tour[i - 1], b = tour[i], c = tour[k], d = tour[k + 1];
                    double before = g.truck_time[a][b] + g.truck_time[c][d];
                    double after = g.truck_time[a][c] + g.truck_time[b][d];
                    if (after + 1e-12 < before) {
                        std::reverse(tour.begin() + i, tour.begin() + k + 1);
                        improved = true;
                    }
                }
            }
            if (!improved) break;
        }
    }

} // namespace
