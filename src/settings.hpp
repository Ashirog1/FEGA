#include "fstream"
#include "iostream"
#include "list"
#include "mcmf.cpp"
#include "numeric"
#include "random"
#include "vector"

/*
TODO: try to split to different module.
For example:
route.hpp
solution.hpp
customer.hpp
vehicle.hpp

chromosome
*/

const int TTRUCK = 0, TDRONE = 1;

/// @brief default constraint parameter
int num_customer, num_truck, num_drone, time_limit;
double speed_truck, speed_drone;
int capacity_truck, capacity_drone, duration_drone;

std::ofstream log_debug("debug.log");
std::ofstream log_result("result.log");
csvfile log_csv_result("result.csv");

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
  return sqrt((A.x - B.x) * (A.x - B.x) + (A.y - B.y) * (A.y - B.y));
}

double time_travel(const Customer &A, const Customer &B, int type) {
  return (1.0 * euclid_distance(A, B)) /
         (type == TTRUCK ? (double)speed_truck : (double)speed_drone);
}

std::vector<Customer> customers;

class Route {
  class Node {
   public:
    int weight, customer_id;
    Node() {
      weight = 0;
      customer_id = -1;
    }
  };

 public:
  Route() {
    total_time = 0;
    total_weight = 0;
    route.clear();
    route.push_back(Node());
    route.back().customer_id = 0;
  }
  /*
  0 is truck
  1 is drone
  */
  int vehicle_type;
  /*
  id of truck/drone that manage that trip

  not necessary?
  */
  int owner;
  std::vector<Node> route;
  double total_time = 0;
  int total_weight = 0;
  /* info = {customer_id, weight};
   */
  int size() { return route.size(); }
  bool append(std::pair<int, int> info) {
    assert(route.size() >= 1);
    auto [customer_id, weight] = info;
    assert(0 <= customer_id && customer_id < num_customer);
    total_time -= time_travel(customers[route.back().customer_id], customers[0],
                              vehicle_type);
    total_time += time_travel(customers[route.back().customer_id],
                              customers[customer_id], vehicle_type);
    total_time +=
        time_travel(customers[customer_id], customers[0], vehicle_type);

    Node tmp = Node();

    tmp.customer_id = customer_id;
    tmp.weight = weight;
    route.push_back(tmp);

    total_weight += weight;
    if (not valid_route()) {
      pop();
      return false;
    }
    return true;
  }
  void pop() {
    if (route.size() <= 1) return;
    total_time -= time_travel(customers[route.back().customer_id], customers[0],
                              vehicle_type);
    total_time -= time_travel(customers[route.back().customer_id],
                              customers[route[route.size() - 2].customer_id],
                              vehicle_type);
    total_time += time_travel(customers[route[route.size() - 2].customer_id],
                              customers[0], vehicle_type);
    total_weight -= route.back().weight;
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
  int rem_weight() {
    return (vehicle_type == TTRUCK ? capacity_truck : capacity_drone) -
           total_weight;
  }
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
  routeSet(int _vehicle_type) {
    set_vehicle_type(_vehicle_type);
    multiRoute.emplace_back(Route());
    multiRoute.back().set_vehicle_type(vehicle_type);
  }
  /*{customer_id, weight}*/
  bool append(std::pair<int, int> info, int trip_id) {
    assert(trip_id < multiRoute.size());
    total_time -= multiRoute[trip_id].total_time;
    total_weight -= multiRoute[trip_id].total_weight;
    bool flag = multiRoute[trip_id].append(info);
    total_time += multiRoute[trip_id].total_time;
    total_weight += multiRoute[trip_id].total_weight;
    if (not flag)
      return false;
    if (flag and not valid_route() or not multiRoute[trip_id].valid_route()) {
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
    if (vehicle_type == TDRONE) {
      if (total_time > duration_drone) return false;
    }
    return true;
  }
  void set_vehicle_type(int type) { vehicle_type = type; }
};

class Solution {
 public:
  std::vector<routeSet> truck_trip, drone_trip;
  std::vector<int> current_lowerbound, total_weight;
  static int educate_call, evaluate_call;
  Solution() {
    truck_trip.clear();
    drone_trip.clear();

    current_lowerbound.assign(num_customer, 0);
    total_weight.assign(num_customer, 0);
    for (int i = 0; i < num_customer; ++i)
      current_lowerbound[i] = customers[i].lower_weight;
    truck_trip.resize(num_truck, routeSet(TTRUCK));
    drone_trip.resize(num_drone, routeSet(TDRONE));
    // debug("complete init solution");
    for (int i = 0; i < num_truck; ++i) {
      truck_trip[i].set_vehicle_type(TTRUCK);
    }
    for (int i = 0; i < num_drone; ++i) {
      drone_trip[i].set_vehicle_type(TDRONE);
    }
    // debug("complete set vehicle type");
  }
  /*
  greedy algo to maximize objective function
  */
  routeSet *route_at(int index) {
    return (index < num_truck ? &truck_trip[index]
                              : &drone_trip[index - num_truck]);
  }
  int find_pushed_weight(int route_id, int trip_id, int customer_id) {
    if (route_id > num_drone + num_truck) return 0;
    /// debug(route_id, trip_id, customer_id,
    /// route_at(route_id)->multiRoute.size());
    if (trip_id > route_at(route_id)->multiRoute.size()) {
      return 0;
    }
    return std::min(current_lowerbound[customer_id],
                      find_remaining_weight(route_id, trip_id));
  }
  int find_remaining_weight(int route_id, int trip_id) {
    if (route_id > num_drone + num_truck) return 0;
    /// debug(route_id, trip_id, customer_id,
    /// route_at(route_id)->multiRoute.size());
    if (trip_id > route_at(route_id)->multiRoute.size()) {
      return 0;
    }
    return (route_id < num_truck)
               ? truck_trip[route_id].multiRoute[trip_id].rem_weight()
               : drone_trip[route_id - num_truck]
                     .multiRoute[trip_id]
                     .rem_weight();
  }
  int push_cus(int route_id, int trip_id, pair<int, int> customer_info) {
    std::pair<int, int> cus = {
        customer_info.first,
        find_pushed_weight(route_id, trip_id, customer_info.first)};
    cus.second = min(customer_info.second, cus.second);
    if (cus.second == 0) return -1;
    if (route_at(route_id)->append(cus, trip_id)) {
      current_lowerbound[cus.first] -= cus.second;
      total_weight[cus.first] += cus.second;
      return cus.second;
    } else {
      return -1;
    }
  }
  void split_process() {}
  void educate() {
    /// push process? push remaining weight to cus

    if (not valid_solution()) return;
    std::vector<int> total_weight(num_customer);
    for (auto truck : truck_trip) {
      for (auto r : truck.multiRoute)
        for (auto loc : r.route) total_weight[loc.customer_id] += loc.weight;
    }
    for (auto drone : drone_trip) {
      for (auto r : drone.multiRoute)
        for (auto loc : r.route) total_weight[loc.customer_id] += loc.weight;
    }
    mincostmaxflow mcmf;
    int N = num_truck, M = num_customer;
    for (int i = 0; i < num_drone; ++i) {
      N += drone_trip[i].multiRoute.size();
    }
    int S = N + M + 1, T = S + 1;
    mcmf.init(T, S, T);

    int cur_route = 0;
    for (int i = 0; i < num_truck; ++i) {
      for (auto route : truck_trip[i].multiRoute) {
        ++cur_route;
        mcmf.addedge(S, cur_route, route.rem_weight(), 0);
        for (auto cus : route.route) {
          if (cus.customer_id == 0) continue;
          mcmf.addedge(cur_route, N + cus.customer_id, INT_MAX, 0);
          //    debug(cur_route, N + cus.customer_id);
        }
      }
    }
    for (int i = 0; i < num_drone; ++i) {
      for (auto route : drone_trip[i].multiRoute) {
        ++cur_route;
        mcmf.addedge(S, cur_route, route.rem_weight(), 0);
        for (auto cus : route.route) {
          if (cus.customer_id == 0) continue;
          mcmf.addedge(cur_route, N + cus.customer_id, INT_MAX, 0);
        }
      }
    }
    for (int i = 1; i < num_customer; ++i) {
      mcmf.addedge(N + i, T, customers[i].upper_weight - (total_weight[i]),
                   customers[i].cost);
    }
    for (auto e : mcmf.e) {
      //   debug(e.from, e.to, e.cap, e.cost);
    }
    mcmf.mcmf();

    for (auto e : mcmf.e) {
      //      debug(e.from, e.to, e.cap, e.cost, e.flow);
    }

    /// assign weight based on mcmf solution
    map<pair<int, int>, int> assign_weight;

    for (auto e : mcmf.e) {
      assign_weight[{e.from, e.to}] = e.flow;
    }
    cur_route = 0;
    for (int i = 0; i < num_truck; ++i) {
      for (auto route : truck_trip[i].multiRoute) {
        ++cur_route;
        for (auto &cus : route.route) {
          if (cus.customer_id == 0) continue;

          cus.weight += assign_weight[{cur_route, N + cus.customer_id}];
          current_lowerbound[cus.customer_id] -=
              assign_weight[{cur_route, N + cus.customer_id}];
        }
      }
    }
    for (int i = 0; i < num_drone; ++i) {
      for (auto route : drone_trip[i].multiRoute) {
        ++cur_route;
        mcmf.addedge(S, cur_route, route.rem_weight(), 0);
        for (auto &cus : route.route) {
          if (cus.customer_id == 0) continue;
          cus.weight += assign_weight[{cur_route, N + cus.customer_id}];
          current_lowerbound[cus.customer_id] -=
              assign_weight[{cur_route, N + cus.customer_id}];
        }
      }
    }
  }
  void educate2() {
    /// heuristic
    /// 1. trip with remaining weight -> push remaining customer
    /// 2. do sth heuristic
  }
  bool valid_solution() {
    for (auto truck : truck_trip) {
      if (not truck.valid_route()) return false;
    }
    for (auto drone : drone_trip) {
      if (not drone.valid_route()) return false;
    }
    fill(total_weight.begin(), total_weight.end(), 0);
    for (auto truck : truck_trip) {
      for (auto r : truck.multiRoute)
        for (auto loc : r.route) total_weight[loc.customer_id] += loc.weight;
    }
    for (auto drone : drone_trip) {
      double tot = 0;
      for (auto r : drone.multiRoute) {
        for (auto loc : r.route) total_weight[loc.customer_id] += loc.weight;
        tot += r.total_time;
      }
    }

    for (int i = 1; i < num_customer; ++i) {
      //  log_debug << "weight " << total_weight[i] << ' ' <<
      //  customers[i].lower_weight << '\n';
      if (total_weight[i] < customers[i].lower_weight) return false;
    }
    return true;
  }
  int evaluate() {
    /// make sure valid_solution() return true
    ++evaluate_call;
    int ans = 0;
    for (int i = 0; i < num_customer; ++i) {
      ans += customers[i].cost * total_weight[i];
    }
    return ans;
  }
  int penalty(int alpha = 1, int beta = 1) {
    if (valid_solution()) return 0;
    double truck_pen = 0, drone_pen = 0;
    for (auto truck : truck_trip) {
      for (auto route : truck.multiRoute) {
        int benefit = 0;
        for (auto loc : route.route) {
          benefit += loc.weight * customers[loc.customer_id].cost;
        }
        truck_pen += 1.0 * route.total_time / (double)time_limit * benefit;
      }
    }
    for (auto drone : drone_trip) {
      for (auto route : drone.multiRoute) {
        int benefit = 0;
        for (auto loc : route.route) {
          benefit += loc.weight * customers[loc.customer_id].cost;
        }
        drone_pen += 1.0 * route.total_time / (double)duration_drone * benefit;
      }
    }
    return alpha * truck_pen + beta * drone_pen;
  }
  int fitness(int alpha = 1, int beta = 1) { return evaluate() - penalty(); }
  void print_out() {
    log_debug << "solution debug\n";
    log_debug << "customer weight\n";
    for (int i = 1; i < num_customer; ++i) log_debug << total_weight[i] << ' ';
    log_debug << '\n';
    int tot_weight = 0;
    for (auto truck : truck_trip) {
      for (auto route : truck.multiRoute) {
        for (auto loc : route.route) {
          log_debug << "[" << loc.customer_id << ' ' << loc.weight << "],"
                    << ' ';
          tot_weight += loc.weight;
        }
        log_debug << '\n';
      }
    }
    log_debug << "complete truck with total weight is " << tot_weight << '\n';
    tot_weight = 0;
    for (auto drone : drone_trip) {
      for (auto route : drone.multiRoute) {
        for (auto loc : route.route) {
          log_debug << "[" << loc.customer_id << ' ' << loc.weight << ']'
                    << ' ';
          tot_weight += loc.weight;
        }
        log_debug << '\n';
      }
    }

    log_debug << "complete drone with total weight is " << tot_weight << '\n';
    log_debug << '\n';
  }
};
/* a constant seed random integer generator */
std::mt19937 rng(64);

int rand(int l, int r) { return l + rng() % (r - l + 1); }

/*
return a real value in defined range
*/
double random_number_in_range(double l, double r) {
  std::uniform_real_distribution<double> unif(l, r);
  std::default_random_engine re;
  return unif(re);
}

std::vector<double> build_partial_sum(const std::vector<double> &prob) {
  std::vector<double> partial(prob.size(), 0);
  for (int i = 0; i < (int)prob.size(); ++i) {
    partial[i] = (i == 0 ? 0 : partial[i - 1]) + prob[i];
  }
  debug(prob, partial);
  return partial;
}

/*
A random number generator that satisfy
P(i) = prob(i) / (sigma(prob))
*/

int random_number_with_probability(const std::vector<double> &partial) {
  double dice = random_number_in_range(0, partial.back());
  debug(dice, partial.back());
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
  std::vector<std::pair<int, int>> chr;
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
        for each routes
          for each customer on route
    */
    /*
    TODO:
    format to more readable code
    */
    for (auto truck : sol.truck_trip) {
      for (auto route : truck.multiRoute) {
        for (auto loc : route.route) {
          if (loc.customer_id == 0 or loc.weight == 0) continue;
          chr.emplace_back(loc.customer_id, loc.weight);
        }
      }
    }
    for (auto drone : sol.drone_trip) {
      for (auto route : drone.multiRoute) {
        for (auto loc : route.route) {
          if (loc.customer_id == 0 or loc.weight == 0) continue;
          chr.emplace_back(loc.customer_id, loc.weight);
        }
      }
    }
  }

  /*
  encode to a solution use greedy algo
  try to push customer on current route until the valid_solution false
  */
  Solution encode() {
    /// merge adj cus

    Solution sol;
    for (int i = 1; i < num_customer; ++i)
      sol.current_lowerbound[i] = customers[i].upper_weight;
    int current_vehicle = 0, trip_id = 0, current_pushed = 0;
    log_debug << "[";
    for (auto it : chr) {
      log_debug << "[" << it.first << "," << it.second << "]";
    }
    log_debug << "]";

    for (auto customer_info : chr) {
      auto [customer_id, customer_weight] = customer_info;
      if (customer_id == 0 or customer_id >= num_customer) continue;

      const auto pushable = [&]() {
        int dec_weight = sol.push_cus(current_vehicle, trip_id, customer_info);
        if (dec_weight <= 0) return false;
        customer_weight -= dec_weight;
        return true;
      };
      const auto push_route = [&]() {
        while (current_vehicle < num_truck) {
          if (not pushable()) {
            ++current_vehicle;
            trip_id = 0;
          } else {
            ++current_pushed;
            break;
          }
        }
        /// @brief drone route
        if (current_vehicle >= num_truck) {
          while (current_vehicle < num_truck + num_drone) {
            if (not pushable()) {
              if (current_pushed == 0) {
                ++current_vehicle;
                current_pushed = 0;
                trip_id = 0;
              } else {
                ++trip_id;
                sol.drone_trip[current_vehicle - num_truck].new_route();
                current_pushed = 0;
              }
            } else {
              ++current_pushed;
              break;
            }
          }
        }
      };
      while (customer_weight > 0) {
        auto tmp = customer_weight;
        push_route();
        if (tmp == customer_weight) break;
      }
    }
    sol.print_out();
    return sol;
  }
  /*
  TODO: create dynamic Chromosome support append and pop
  */
  void normalize() {
    /// reduce weight to satisfy weight upperbound
  }
};
std::vector<Chromosome> Population;

/// C = A[:pivot] + B[pivot+1:] (plus some edge case checking)

Chromosome crossover(const Chromosome &a, const Chromosome &b) {
  /*
   */
  int pivot = rand(0, (int)a.chr.size());
  // debug(a.chr.size(), pivot);
  Chromosome c;
  std::vector<int> current_lowerbound(num_customer);
  for (int i = 0; i < num_customer; ++i) {
    current_lowerbound[i] = customers[i].upper_weight;
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
  for (int i = pivot; i < (int)B.size(); ++i) {
    push(i);
  }
  c.chr.clear();
  // debug(A, B, C);
  for (auto it : C) c.chr.emplace_back(it);
  return c;
}

class settings {
 public:
  int POPULATION_SIZE = 200;
  int OFFSPRING_PERCENT = 70;
  int MUTATION_ITER = 10;
  int NUM_GENERATION = 100;
} general_setting;

/// @brief log result variable

/*
- objective func
        - average, min, max in each generation
- educate call
- evaluation call
- infeasible solution in each generation
- running time
*/

vector<double> average_in_generation, best_in_generation, worst_in_generation;
int Solution::educate_call = 0;
int Solution::evaluate_call = 0;
vector<int> num_infeasible_solution;
