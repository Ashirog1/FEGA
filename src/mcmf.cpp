#include "bits/stdc++.h"

using namespace std;

struct mincostmaxflow {
  const int64_t inf = 1e18;

  struct edge {
    int from, to, cap, cost, flow, idx;
  };
  vector<vector<int>> adj;
  vector<int64_t> d;
  vector<int> p;
  vector<bool> inq;
  vector<edge> e;

  int S, T;
  int N;

  mincostmaxflow() {}
 void init(int _N, int _S, int _T) {
    N = _N;
    S = _S;
    T = _T;
    adj = vector<vector<int>>(N + 1);
    d = vector<int64_t>(N + 1);
    p = vector<int>(N + 1);
    inq = vector<bool>(N + 1);
  };

  void addedge(int u, int v, int cap, int cost, int id = -1) {
    edge e1 = {u, v, cap, -cost, 0, id};
    edge e2 = {v, u, 0, cost, 0, -1};
    adj[u].push_back(e.size());
    e.push_back(e1);
    adj[v].push_back(e.size());
    e.push_back(e2);
  }

  bool spfa(void) {
    for (int i = 1; i <= N; ++i)
      d[i] = inf;
    d[S] = 0;
    queue<int> q;
    q.push(S);
    while (q.size()) {
      auto u = q.front(); q.pop();
      inq[u] = false;
      for (int i : adj[u]) {
        int v = e[i].to;
        if (e[i].flow < e[i].cap && d[v] > d[u] + e[i].cost) {
          d[v] = d[u] + e[i].cost;
          p[v] = i;
          if (inq[v] == false) {
            inq[v] = true;
            q.push(v);
          }
        }
      }
    }
    return d[T] != inf;
  }

  int64_t mcmf(int K = 1e9) {
    int mfl = 0;
    int64_t mc = 0;
    while (spfa()) {
      int f = 1e9;
      for (int v = T; v != S;) {
        int id = p[v];
        f = min(f, e[id].cap - e[id].flow);
        v = e[id].from;
      }

      mfl += f;
      mc += 1LL * f * d[T];

      for (int v = T; v != S;) {
        int id = p[v];
        e[id].flow += f;
        e[id ^ 1].flow -= f;

        v = e[id].from;
      }
      if (mfl == K) break;
    }
    return mc;
  }

};