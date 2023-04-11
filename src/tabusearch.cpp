#include "bits/stdc++.h"
#include "constant.h"
#include "network_simplex.hpp"

const int MAX_ROUTE = 1000, MAX_CUSTOMER = 1000;

class TabuOperator {
  /*
  tabuList to check operator inside tabuList
  tabuQueue to update tabuList

  (route, customer)

  TODO: add removeTabu(i, j)
  */

  std::set<std::pair<int, int>> tabuList;
  std::queue<std::pair<int, int>> tabuQueue;
  double tabuSize;

 public:
  TabuOperator(int tabuSize) : tabuSize(tabuSize) {}
  TabuOperator() {}

  void normalize() {
    if (tabuList.size() > (int)tabuSize) {
      tabuList.erase(tabuQueue.front());
      tabuQueue.pop();
    }
  }
  /*add move i j to tabu list*/
  void addTabu(int i, int j) {
    if (not checkTabu(i, j)) {
      tabuQueue.emplace(i, j);
      tabuList.insert({i, j});
      normalize();
    }
  }
  /*check move is valid or not */
  bool checkTabu(int i, int j) {
    if (tabuList.find({i, j}) != tabuList.end()) {
      return true;
    }
    return false;
  }
  void remove(int i, int j) {
    if (tabuList.find({i, j}) != tabuList.end())
      tabuList.erase(tabuList.find({i, j}));
  }
  void incTabuSize(double delta=1.1) {
    tabuSize += delta;
  }
  int size() {
    return (int)tabuSize;
  }
} tabuList;

class Solution {
  int z[MAX_ROUTE][MAX_CUSTOMER];
  std::vector<int> route[MAX_ROUTE];

 public:
  void emptySolution() {
    for (int truck = 0; truck < num_truck; ++truck) {
      for (int customer = 0; customer < num_customer; ++customer)
        z[truck][customer] = 0;
      route[truck].clear();
    }
  }
  void emptyWeight() {
    for (int truck = 0; truck < num_truck; ++truck) {
      for (int customer = 0; customer < num_customer; ++customer)
        z[truck][customer] = 0;
    }
  }

 public:
  void educateNetworkSimplex() {
    emptyWeight();
    // reassign weight between route and customer

    int num_route = num_truck;

    int S = num_route + num_customer + 1, T = S + 1;
    network_simplex<long, long, int64_t> ns(T + 5);

    for (int truck = 0; truck < num_truck; ++truck) {
      for (auto customer : route[truck]) {
        if (customer == 0) continue;
        ns.add(truck + 1, num_route + customer, 0, INT_MAX, 0);
      }
      ns.add(S, truck + 1, 0, truck_capacity[truck], 0);
    }

    for (int i = 1; i < num_customer; ++i) {
      ns.add(num_route + i, T, customers[i].lower_weight,
             customers[i].upper_weight, -customers[i].cost);
    }
    ns.add(T, S, 0, INT_MAX, 0);
    auto result = ns.mincost_circulation();

    /// assign weight based on mcmf solution
    int e = 0;
    for (int truck = 0; truck < num_truck; ++truck) {
      for (auto customer : route[truck]) {
        if (customer == 0) continue;
        z[truck][customer] = ns.get_flow(e++);
      }
    }
  }
  void assignRoute(int route_id, const std::vector<int>& other_route) {
    route[route_id] = other_route;
  }
  void logging() {
    for (int truck = 0; truck < num_truck; ++truck) {
      std::cout << "[";
      for (auto customer : route[truck]) {
        std::cout << "[" << customer << ", " << z[truck][customer] << "], ";
      }
      std::cout << "]\n";
    }
  }
  std::vector<int> getCurrentWeight() {
    std::vector<int> current_weight(num_customer);
    for (int truck = 0; truck < num_truck; ++truck) {
      for (auto customer : route[truck]) {
        current_weight[customer] += z[truck][customer];
      }
    }
    return current_weight;
  }
  bool isFeasible() {
    auto current_weight = getCurrentWeight();
    for (int customer = 1; customer < num_customer; ++customer) {
      if (current_weight[customer] < customers[customer].lower_weight or
          current_weight[customer] > customers[customer].upper_weight) {
        return false;
      }
    }
    return true;
  }
  std::vector<int> getRoute(int route_id) {
    std::vector<int> current_route;
    for (auto customer : route[route_id]) {
      if (z[route_id][customer] != 0) current_route.push_back(customer);
    }
    return current_route;
  }
  /*route id*/
  std::vector<std::pair<int, int>> worstFeasbile() {
    std::vector<std::pair<double, int>> worst;
    for (int truck = 0; truck < num_truck; ++truck) {
      double D = 0;
      for (int i = 0; i + 1 < route[truck].size(); ++i) {
        D += customerDistance(i, i + 1);
      }
      double current_exceed =
          (D - time_limit_truck[truck]) / time_limit_truck[truck];
      if (current_exceed > 0)
        worst.emplace_back(current_exceed, truck);
    }
    std::sort(worst.rbegin(), worst.rend());
    std::vector<std::pair<int, int>> res;
    for (auto [_, truck] : worst) {
      res.emplace_back(truck, farthest(truck));
      for (auto c : route[truck]) res.emplace_back(truck, c);
    }
    return res;
  }
  /*find farthest*/
  int farthest(int route_id) {
    int farthest_customer = -1;
    for (auto customer : route[route_id]) {
      if (farthest_customer == -1 or customerDistance(0, farthest_customer) <
                                         customerDistance(0, customer)) {
        farthest_customer = customer;
      }
    }
    return farthest_customer;
  }
  void remove(int route_id, int customer) {
    route[route_id].erase(
        std::remove(route[route_id].begin(), route[route_id].end(), customer));
  }
  int64_t value() {
    int64_t res = 0;
    for (int t = 0; t < num_truck; ++t) {
      for (auto c : route[t]) {
        res += (int64_t)customers[c].cost * z[t][c];
      }
    }
    return res;
  }
  bool isValid() {
    return (isFeasible() and worstFeasbile().empty());
  }
} global_solution, best;

/*TSH in one route*/
std::vector<int> inRouteTSH(std::vector<int> route_order) {
  /*random a permutation*/
  std::shuffle(route_order.begin(), route_order.end(), rng);
  std::vector<int> route;
  route.push_back(0);
  route.push_back(0);

  for (auto customer : route_order) {
    if (route.size() - 2 < 3) {
      route.insert(route.begin() + route.size() - 1, customer);
    } else {
      /*find nearest customer*/
      int nearest = -1;
      for (int i = 0; i + 1 < route.size(); ++i) {
        if (nearest == -1 or customerDistance(route[i], customer) <
                                 customerDistance(nearest, customer)) {
          nearest = i;
        }
      }

      /*try to insert*/
      if (nearest == 0 or nearest == (int)route.size() - 1) {
        /// insert after nearest
        route.insert(route.begin() + nearest + 1, customer);
        continue;
      }
      if (customerDistance(nearest, nearest - 1) +
              customerDistance(customer, nearest + 1) <
          customerDistance(nearest - 1, customer) +
              customerDistance(nearest, nearest + 1)) {
        route.insert(route.begin() + nearest + 1, customer);
      } else {  
        if (nearest + 2 != route.size() - 1)
          route.insert(route.begin() + nearest + 2, customer);
        else
          route.insert(route.begin() + nearest + 1, customer);
      }
    }
  }
  return route;
}

void TSH() {
  for (int truck = 0; truck < num_truck; ++truck) {
    /// remove z[i][k] = 0 from current route
    global_solution.assignRoute(truck,
                                inRouteTSH(global_solution.getRoute(truck)));
  }
}

void readInput() {
  std::cin >> num_truck;
  
  time_limit_truck.resize(num_truck);
  truck_capacity.resize(num_truck);
  for (auto &it : time_limit_truck) std::cin >> it;
  for (auto &it : truck_capacity) std::cin >> it;

  Customer tmp;
  num_customer = 1;
  customers.emplace_back(Customer());
  while (std::cin >> tmp.x >> tmp.y >> tmp.lower_weight >> tmp.upper_weight >>
         tmp.cost) {
    customers.emplace_back(tmp);
  }
  num_customer = customers.size();
}

void initSolution() {
  /*
  Step 2 of paper
  Init weight between each route and each customer
  Don't add arc from tabulist
  */

  global_solution.emptySolution();

  for (int truck = 0; truck < num_truck; ++truck) {
    std::vector<int> all_customer;
    for (int c = 1; c < num_customer; ++c) {
      if (not tabuList.checkTabu(truck, c)) all_customer.push_back(c);
    }
    global_solution.assignRoute(truck, all_customer);
  }
  /// reassign weight
  global_solution.educateNetworkSimplex();
}

/*made feasible solution non tabu*/
void assignArc() {
  if (global_solution.isFeasible()) {
    for (int truck = 0; truck < num_truck; ++truck) {
      for (int customer : global_solution.getRoute(truck)) {
        tabuList.remove(truck, customer);
      }
    }
  }
}

void removeArcFromTabu() {
  for (int truck = 0; truck < num_truck; ++truck) {
    auto x = global_solution.getRoute(truck);
    for (int c : x) {
      // std::cout << "remove";
      tabuList.remove(truck, c);
    }
  }
}

void step4() {
  auto edges = global_solution.worstFeasbile();
  int cnt = 0;
  while (true) {
    bool good = false;
    for (auto [t, c] : edges) {
      tabuList.addTabu(t, c);
      initSolution();
      TSH();
      if (global_solution.isValid()) {
        good = true;
        break;
      }
    }
    if (good) break;
    tabuList.incTabuSize();
    std::cout << "still in step 4 " << tabuList.size() << '\n'; 
    if (tabuList.size() >= 1000) break;
  }
}

void localSearch() {
  for (int c = 0; c < num_customer; ++c) {
    for (int t = 0; t < num_truck; ++t) {
      ///  
    }
  }
}

void step6() {
  for (int iter = 0; iter < LOCAL_SEARCH_OP; ++iter) {

  }
}

void tabuSearch() {
  for (int iterator = 0; iterator < TABU_ITERATOR; ++iterator) {
    /// step 2 -> 4
    /// networkflow based on tabu list
    initSolution();
    /// tsp heuristic in every route
    TSH();
    /// step 4
    step4();
    /// step 5
    removeArcFromTabu();
    /// step 6
    step6();
  }
  global_solution.logging();
  std::cout << global_solution.isValid() << '\n';
  std::cout << global_solution.value();
}

int main(int argc, char* argv[]) {
  std::ios_base::sync_with_stdio(0);
  /// default should be 5 5 2
  TABU_ITERATOR = stoi(argv[1]);
  TABU_CYCLE = stoi(argv[2]);
  LOCAL_SEARCH_OP = stoi(argv[3]);


  tabuList = TabuOperator(TABU_CYCLE);

  readInput();
  tabuSearch();
}