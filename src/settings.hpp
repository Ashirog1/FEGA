#include "climits"
#include "iostream"
#include "list"
#include "numeric"
#include "random"
#include "vector"
#include "cassert"
#include "fstream"

/*
TODO: try to split to different module.
For example:
route.hpp
solution.hpp
customer.hpp
vehicle.hpp

chromesome
*/

const int TTRUCK = 0, TDRONE = 1;

int num_customer, num_truck, num_drone, time_limit;
int speed_truck, speed_drone, capacity_truck, capacity_drone, duration_drone;
std::ofstream log_debug("debug.log");

template <class T>
std::vector<T> create(size_t size, T initialValue) {
  return std::vector<T>(size, initialValue);
}

template <class T, class... Args>
auto create(size_t head, Args &&...tail) {
  auto inner = create<T>(tail...);
  return std::vector<decltype(inner)>(head, inner);
}

class Customer {
 public:
  Customer() {
    lower_weight = upper_weight = cost = 0;
    x = y = 0;
  }
  int lower_weight, upper_weight, cost;
  double x = 0, y = 0;
};

double euclid_distance(const Customer &A, const Customer &B) {
  return (A.x - B.x) * (A.x - B.x) + (A.y - B.y) * (A.y - B.y);
}

double time_travel(const Customer &A, const Customer &B, int type) {
  return sqrt(euclid_distance(A, B)) /
         (type == TTRUCK ? speed_truck : speed_drone);
}

std::vector<Customer> customers;

class Route {
  class Node {
   public:
    int weight, customer_id;
    Node *prev_node = nullptr, *next_node = nullptr;
    Node() {
      weight = 0;
      customer_id = -1;
      prev_node = nullptr, next_node = nullptr;
    }
    ~Node() {
      delete prev_node;
      delete next_node;
    }
  };

 public:
  Route() {
    total_time = 0;
    total_weight = 0;
    route.clear();
    route.push_back(new Node());
    route.back()->customer_id = 0;
  }
  /*
  0 is truck
  1 is drone
  */
  int vehicle_type;
  /*
  id of truck/drone that manage that trip

  not neccessery?
  */
  int owner;
  std::vector<Node *> route;
  double total_time = 0;
  int total_weight = 0;
  /* info = {customer_id, weight};
   */
  int size() { return route.size(); }
  bool append(std::pair<int, int> info) {
    auto [customer_id, weight] = info;
    total_time -= time_travel(customers[route.back()->customer_id],
                              customers[0], vehicle_type);
    total_time += time_travel(customers[route.back()->customer_id],
                              customers[customer_id], vehicle_type);
    total_time +=
        time_travel(customers[customer_id], customers[0], vehicle_type);

    total_weight += weight;

    Node *tmp = new Node();

    tmp->customer_id = customer_id;
    tmp->prev_node = route.back();
    route.back()->next_node = tmp;
    route.push_back(tmp);

    total_weight += weight;
    if (not valid_route()) {
      pop();
      return false;
    }
    return true;
  }
  void pop() {
    total_time -= time_travel(customers[route.back()->customer_id],
                              customers[0], vehicle_type);
    total_time -= time_travel(customers[route.back()->customer_id],
                              customers[route.back()->prev_node->customer_id],
                              vehicle_type);
    total_time += time_travel(customers[route.back()->prev_node->customer_id],
                              customers[0], vehicle_type);
    total_weight -= route.back()->weight;

    route.pop_back();
  }
  bool valid_route() {
    if (total_time > time_limit) return false;

    if (vehicle_type == TTRUCK) {
      return total_weight <= capacity_truck;
    } else {
      return (total_weight <= capacity_drone and total_time <= duration_drone);
    }
  }
  /*
    set before route building
  */
  void set_vehicle_type(int type) { vehicle_type = type; }
};

class routeSet {
 public:
  std::vector<Route> multiRoute;
  double total_time = 0;
  int total_weight = 0;
  int vehicle_type = 0;
  void new_route() {
    multiRoute.emplace_back(Route());
    multiRoute.back().set_vehicle_type(vehicle_type);
  }
  routeSet() {
    multiRoute.emplace_back(Route());
    multiRoute.back().set_vehicle_type(vehicle_type);
  }
  /*{customer_id, weight}*/
  bool append(std::pair<int, int> info, int trip_id) {
    assert(trip_id < multiRoute.size());
    total_time -= multiRoute[trip_id].total_time;
    total_weight -= multiRoute[trip_id].total_weight;
    multiRoute[trip_id].append(info);
    total_time += multiRoute[trip_id].total_time;
    total_weight += multiRoute[trip_id].total_weight;

    if (not valid_route() or not multiRoute[trip_id].valid_route()) {
      pop(trip_id);
      return false;
    }
    return true;
  }
  void pop(int trip_id) {
    total_time -= multiRoute[trip_id].total_time;
    total_weight -= multiRoute[trip_id].total_weight;
    multiRoute[trip_id].pop();
    total_time += multiRoute[trip_id].total_time;
    total_weight += multiRoute[trip_id].total_weight;
  }
  bool valid_route() {
    if (total_time > time_limit) return false;
    return true;
  }
  void set_vehicle_type(int type) { vehicle_type = type; }
};

class Solution {
 public:
  std::vector<routeSet> truck_trip, drone_trip;
  Solution() {
    truck_trip.clear();
    drone_trip.clear();
  }
  /*
  greedy algo to maximize objective function
  */
  void educate() {}
  bool valid_solution() {
    std::vector<int> total_weight(num_customer);
    for (auto truck : truck_trip) {
      if (not truck.valid_route()) return false;
      for (auto route : truck.multiRoute) {
        for (auto loc : route.route) {
          total_weight[loc->customer_id] += loc->weight;
        }
      }
    }
    for (auto drone : drone_trip) {
      if (not drone.valid_route()) return false;
      for (auto route : drone.multiRoute) {
        for (auto loc : route.route) {
          total_weight[loc->customer_id] += loc->weight;
        }
      }
    }
    for (int i = 0; i < num_customer; ++i) {
      if (total_weight[i] < customers[i].lower_weight) return false;
    }
    return true;
  }
  int evaluate() {
    /// make sure valid_solution() return true
    std::vector<int> total_weight(num_customer);
    for (auto truck : truck_trip) {
      for (auto route : truck.multiRoute) {
        for (auto loc : route.route) {
          total_weight[loc->customer_id] += loc->weight;
        }
      }
    }
    for (auto drone : drone_trip) {
      for (auto route : drone.multiRoute) {
        for (auto loc : route.route) {
          total_weight[loc->customer_id] += loc->weight;
        }
      }
    }
    int ans = 0;
    for (int i = 0; i < num_customer; ++i) {
      ans += customers[i].cost * total_weight[i];
    }
    return ans;
  }
  int penalty(int alpha=1, int beta=1) {
    double truck_pen = 0, drone_pen = 0;
    for (auto truck : truck_trip) {
      for (auto route : truck.multiRoute) {
        int benefit = 0;
        for (auto loc : route.route) {
          benefit += loc->weight * customers[loc->customer_id].cost;
        }
        truck_pen += 1.0 * route.total_time / (double)time_limit
                    * benefit;
      } 
    }
    for (auto drone : drone_trip) {
      for (auto route : drone.multiRoute) {
        int benefit = 0;
        for (auto loc : route.route) {
          benefit += loc->weight * customers[loc->customer_id].cost;
        }
        drone_pen += 1.0 * route.total_time / (double)duration_drone
                      * benefit;
      }
    }
    return alpha * truck_pen + beta * drone_pen;
  }
  int fitness(int alpha=1, int beta=1) {
    return evaluate() - penalty();
  }
};

/* a constant seed random interger generator */
std::mt19937 rng(64);

int rand(int l, int r) { return l + rng() % (r - l + 1); }

/*
return a real value in defined range
*/
double random_number_in_range(double l, double r) {
  std::uniform_real_distribution<double> unif(l, r);
  std::random_device rand_dev;
  std::mt19937 rand_engine(rand_dev());
  double x = unif(rand_engine);
  return x;
}

/*
1
*/

std::vector<double> build_partial_sum(const std::vector<double> &prob) {
  std::vector<double> partial(prob.size(), 0);
  for (int i = 0; i < (int)prob.size(); ++i) {
    partial[i] = (i == 0 ? 0 : partial[i - 1]) + prob[i];
  }
  return partial;
}

/*
A random number generator that sastisfy
P(i) = prob(i) / (sigma(prob))
*/

int random_number_with_probability(const std::vector<double> &partial) {
  double dice = random_number_in_range(0, partial.back());
  return std::lower_bound(partial.begin(), partial.end(), dice) -
         partial.begin();
}

template <class T>
bool minimize(T &x, const T &y) {
  if (x > y) {
    x = y;
    return true;
  }
  return false;
}

/*
for genetic part
use std::list for O(1) mutation operation
*/

class Chromosome {
 public:
  /*
  {customer id, weight}
  kind of conflict? right
  */
  std::list<std::pair<int, int>> chr;
  /*
  decode to a genetic
  */
  /*
  truck trip
  */
  int size() { return (int)chr.size(); }
  Chromosome() { chr.clear(); }
  Chromosome(int size) { chr.resize(size); }
  Chromosome(Solution sol) {
    chr.clear();

    /*
      for each vehicle
        for each routez
          for each customer on route
    */
    /*
    TODO:
    format to more readable code
    */
    for (auto truck : sol.truck_trip) {
      for (auto route : truck.multiRoute) {
        for (auto loc : route.route) {
          chr.emplace_back(loc->customer_id, loc->weight);
        }
      }
    }
    for (auto drone : sol.drone_trip) {
      for (auto route : drone.multiRoute) {
        for (auto loc : route.route) {
          chr.emplace_back(loc->customer_id, loc->weight);
        }
      }
    }
  }

  /*
  encode to a solution use greedy algo
  try to push customer on current route until the condition hold true
  */
  Solution encode() {
    Solution sol; 
    int current_vehicle = 0, trip_id = 0, current_pushed = 0;
    for (auto customer_info : chr) {
      /// try to push current route
      /// if fail, create new route -> check condition
      /// if fail, update current_vehicle
      const auto push = [&]() {
        if (current_vehicle < num_truck) {
          if (not sol.truck_trip[current_vehicle].append(customer_info,
                                                         trip_id)) {
            return false;
          } else {
            return true;
          }
        } else {
          if (not sol.drone_trip[current_vehicle - num_truck].append(
                  customer_info, trip_id)) {
            return false;
          } else {
            return true;
          }
        }
      };
      while (current_vehicle < num_truck + num_drone) {
        if (not push()) {
          if (current_vehicle < num_truck) {
            ++current_vehicle;
            trip_id = 0;
          } else {
            /*success last drone trip?*/
            if (current_pushed == 0) {
              ++current_vehicle;
              ++trip_id;
            } else {
              ++trip_id;
              current_pushed = 0;
            }
          }
        } else {
          ++current_pushed;
        }
      }
    }
    return sol;
  }
  /*
  TODO: create dynamic Chromosome support append and pop
  */
};
std::vector<Chromosome> Population;

/// C = A[:pivot] + B[pivot+1:] (plus some edge case checking)

Chromosome crossover(const Chromosome &a, const Chromosome &b) {
  /*

  */
  int pivot = rand(1, (int)a.chr.size() - 1);
  Chromosome c;
  std::vector<int> current_lowerbound(num_customer);
  for (int i = 0; i < num_customer; ++i) {
    current_lowerbound[i] = customers[i].lower_weight;
  }
  // need to make sure that a don't contradict with the constraints
  std::vector<std::pair<int, int>> A, B, C;
  for (auto it : a.chr) A.emplace_back(it);
  for (auto it : b.chr) B.emplace_back(it);
  for (int i = 0; i < pivot; ++i) C.emplace_back(A[i]);
  const auto push = [&](int ind) {
    if (ind >= B.size()) return;
    auto [customer_id, weight] = B[ind];
    if (current_lowerbound[customer_id] == 0) return;
    int push_weight = std::min(current_lowerbound[customer_id], weight);
    if (push_weight) C.emplace_back(customer_id, push_weight);
    current_lowerbound[customer_id] -= push_weight;
  };
  for (int i = pivot; i < B.size(); ++i) {
    push(i);
  }
  for (int i = 0; i < pivot; ++i) {
    push(i);
  }
  for (auto it : C) c.chr.emplace_back(it);
  return c;
}

class settings {
 public:
  int POPULATION_SIZE = 100;
  int OFFSPRING_PERCENT = 70;
} general_setting;