#include "constant.h"
#include "customer.h"
#include "iostream"
#include "network_simplex.cpp"
#include "queue"
#include "set"
#include "vector"

const int MAX_ROUTE = 1000, MAX_CUSTOMER = 1000;

class TabuOperator {
  /*
  tabuList to check operator inside tabuList
  tabuQueue to update tabuList

  (route, customer)
  */

  std::set<std::pair<int, int>> tabuList;
  std::queue<std::pair<int, int>> tabuQueue;
  int tabuSize;

 public:
  TabuOperator(int tabuSize) : tabuSize(tabuSize) {}

  void normalize() {
    assert(tabuQueue.size() == tabuList.size());
    if (tabuList.size() > tabuSize) {
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
  bool educateNetworkSimplex() {
    emptyWeight();
    /// reassign weight between route and customer

    int num_route = num_truck;

    int S = num_route + num_customer + 1, T = S + 1;
    network_simplex<long, long, int64_t> ns(T + 5);

    for (int truck = 0; truck < num_truck; ++truck) {
      for (auto customer : route[truck]) {
        if (customer == 0) continue;
        ns.add(truck + 1, num_route + customer, 0, INT_MAX, 0);
      }
      ns.add(S, truck + 1, 0, capacity_truck, 0);
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
    return result;
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
  std::vector<int> currentWeight() {
    std::vector<int> current_weight(num_customer);
    for (int truck = 0; truck < num_truck; ++truck) {
      for (auto customer : route[truck]) {
        current_weight[customer] += z[truck][customer];
      }
    }
    return current_weight;
  }
  bool isFeasible() {
    auto current_weight = currentWeight();
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
  int worstFeasbile() {}
} global_solution;

/*TSH in one route*/
std::vector<int> inRouteTSH(std::vector<int> route_order) {
  /*random a permutation*/
  std::shuffle(route_order.begin(), route_order.end(), rng());
  std::vector<int> route;
  route.push_back(0);
  route.push_back(0);

  for (auto customer : route_order) {
    if (route.size() - 2 < 3) {
      route.push_back(customer);
    } else {
      /*find nearest customer*/
      int nearest = -1;
      for (int i = 0; i + 1 < route.size(); ++i) {
        if (nearest == -1 or customer_distance(route[i], customer) <
                                 customer_distance(nearest, customer)) {
          nearest = i;
        }
      }

      /*try to insert*/
      if (nearest == 0 or nearest == (int)route.size() - 1) {
        /// insert after nearest
        route.insert(route.begin() + nearest, customer);
        continue;
      }
      if (customer_distance(nearest, nearest - 1) +
              customer_distance(customer, nearest + 1) <
          customer_distance(nearest - 1, nearest + 1) +
              customer_distance(nearest, customer)) {
        
        ta
      }
    }
  }
}

void TSH() {
  for (int truck = 0; truck < num_truck; ++truck) {
    global_solution.assignRoute(truck,
                                inRouteTSH(global_solution.getRoute(truck)));
  }
}

void readInput() {
  std::cin >> num_truck >> num_drone >> time_limit;
  std::cin >> speed_truck >> speed_drone >> capacity_truck >> capacity_drone >>
      duration_drone;
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

/*add farthest node from a route in tabu*/
void removeArc() {}

/*made feasible solution non tabu*/
void assignArc() {}

void addArc() {}

void tabuSearch() {
  for (int iterator = 0; iterator < TABU_ITERATOR; ++iterator) {
    /// step 2 -> 4
    while (not global_solution.isFeasible()) {
      /// init based on tabu list
      initSolution();
      TSH();
      removeArc();
    }

    /// step 5
    /// I don't understand the meaning of goto step 2?
    /* remove feasible arc from tabulist */
    assignArc();
    /* step 6 consider what arc add to tabulist*/
    addArc();
    /* step 7 check stop condition*/
  }
}

int main(int argc, char* argv[]) {
  std::ios_base::sync_with_stdio(0);

  TABU_ITERATOR = stoi(argv[1]);
  TABU_CYCLE = stoi(argv[2]);
  tabuList = TabuOperator(TABU_CYCLE);
  readInput();
  ///
  initSolution();

  tabuSearch();
}