#include "bits/stdc++.h"
#include "mcmf.cpp"

template <class Flow = int, class Cost = int>
struct MinCostFlow {
  const Flow INF_FLOW = 1000111000;
  const Cost INF_COST = 1000111000111000LL;

  int n, t, S, T;
  Flow totalFlow;
  Cost totalCost;
  std::vector<int> last, visited;
  std::vector<Cost> dis;
  struct Edge {
    int to;
    Flow cap;
    Cost cost;
    int next;
    int id;
    Edge(int _to, Flow _cap, Cost _cost, int _next, int _id)
        : to(_to), cap(_cap), cost(_cost), next(_next), id(_id) {}
  };
  std::vector<Edge> edges;

  MinCostFlow(int _n)
      : n(_n),
        t(0),
        totalFlow(0),
        totalCost(0),
        last(n, -1),
        visited(n, 0),
        dis(n, 0) {
    edges.clear();
  }

  int addEdge(int from, int to, Flow cap, Cost cost, int id) {
    edges.push_back(Edge(to, cap, cost, last[from], id));
    last[from] = t++;
    edges.push_back(Edge(from, 0, -cost, last[to], -1));
    last[to] = t++;
    return t - 2;
  }

  std::pair<Flow, Cost> minCostFlow(int _S, int _T) {
    S = _S;
    T = _T;
    SPFA();
    while (1) {
      while (1) {
        std::fill(visited.begin(), visited.end(), 0);
        if (!findFlow(S, INF_FLOW)) break;
      }
      if (!modifyLabel()) break;
    }
    return std::make_pair(totalFlow, totalCost);
  }

 private:
  void SPFA() {
    std::fill(dis.begin(), dis.end(), INF_COST);
    std::priority_queue<std::pair<Cost, int> > Q;
    Q.push(std::make_pair(dis[S] = 0, S));
    while (!Q.empty()) {
      int x = Q.top().second;
      Cost d = -Q.top().first;
      Q.pop();
      // For double: dis[x] > d + EPS
      if (dis[x] != d) continue;
      for (int it = last[x]; it >= 0; it = edges[it].next)
        if (edges[it].cap > 0 && dis[edges[it].to] > d + edges[it].cost)
          Q.push(std::make_pair(-(dis[edges[it].to] = d + edges[it].cost),
                           edges[it].to));
    }
    Cost disT = dis[T];
    for (int i = 0; i < n; i++) {
      dis[i] = disT - dis[i];
    }
  }

  Flow findFlow(int x, Flow flow) {
    if (x == T) {
      totalCost += dis[S] * flow;
      totalFlow += flow;
      return flow;
    }
    visited[x] = 1;
    Flow now = flow;
    for (int it = last[x]; it >= 0; it = edges[it].next)
      // For double: fabs(dis[edges[it].to] + edges[it].cost - dis[x]) < EPS
      if (edges[it].cap && !visited[edges[it].to] &&
          dis[edges[it].to] + edges[it].cost == dis[x]) {
        Flow tmp = findFlow(edges[it].to, std::min(now, edges[it].cap));
        edges[it].cap -= tmp;
        edges[it ^ 1].cap += tmp;
        now -= tmp;
        if (!now) break;
      }
    return flow - now;
  }

  bool modifyLabel() {
    Cost d = INF_COST;
    for (int i = 0; i < n; i++)
      if (visited[i])
        for (int it = last[i]; it >= 0; it = edges[it].next)
          if (edges[it].cap && !visited[edges[it].to])
            d = std::min(d, dis[edges[it].to] + edges[it].cost - dis[i]);

    // For double: if (d > INF_COST / 10)     INF_COST = 1e20
    if (d == INF_COST) return false;
    for (int i = 0; i < n; i++)
      if (visited[i]) dis[i] += d;
    return true;
  }
};

int main() {
  int s, t; std::cin >> s >> t;
  MinCostFlow<long long, long long> mcmf(t + 5);
  mincostmaxflow mcmf2; mcmf2.init(t + 5, s, t);
  int from, to, cap, cost;
  int e = 0;
  while (std::cin >> from >> to >> cap >> cost) {
    mcmf.addEdge(from, to, cap, cost, e++);
    mcmf2.addedge(from, to, cap, cost);
  }

  std::cout << mcmf.minCostFlow(s, t).second << '\n';
  for (auto it : mcmf.edges) {
    std::cerr << it.id << ' '<< it.cap << '\n';
  }
}