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

int num_customer, num_truck, num_drone, time_limit;
int speed_truck, speed_drone, capacity_truck, capacity_drone, duration_drone;
std::ofstream log_debug("debug.log");
std::ofstream log_result("result.log");

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
  return sqrt(1.0 * euclid_distance(A, B)) /
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
    total_time -= time_travel(customers[route.back().customer_id], customers[0],
                              vehicle_type);
    total_time += time_travel(customers[route.back().weight],
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
    ///debug(route_id, trip_id, customer_id, route_at(route_id)->multiRoute.size());
    if (trip_id > route_at(route_id)->multiRoute.size()) {
      return 0;
    }
    return std::min(current_lowerbound[customer_id],
                    (route_id < num_truck)
                        ? truck_trip[route_id].multiRoute[trip_id].rem_weight()
                        : drone_trip[route_id - num_truck].multiRoute[trip_id].rem_weight());
  }
  int push_cus(int route_id, int trip_id, int cus_id) {
    std::pair<int, int> cus = {cus_id,
                               find_pushed_weight(route_id, trip_id, cus_id)};
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
    /// TODO: change to mincostmaxflow
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
  bool valid_solution() {
    for (auto truck : truck_trip) {
      if (not truck.valid_route()) return false;
    }
    for (auto drone : drone_trip) {
      if (not drone.valid_route()) return false;
    }
    std::vector<int> total_weight(num_customer);
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
    int ans = 0;
    for (int i = 0; i < num_customer; ++i) {
      ans += customers[i].cost * total_weight[i];
    }
    return ans;
  }
  int penalty(int alpha = 1, int beta = 1) {
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

    for (int i = 1; i < num_customer; ++i) log_debug << total_weight[i] << ' ';
    log_debug << '\n';
    for (auto truck : truck_trip) {
      for (auto route : truck.multiRoute) {
        for (auto loc : route.route) {
          log_debug << "[" << loc.customer_id << ' ' << loc.weight << "]," << ' ';
        }
        log_debug << '\n';
      }
    }
    log_debug << "complete truck\n";
    for (auto drone : drone_trip) {
      for (auto route : drone.multiRoute) {
        for (auto loc : route.route) {
          log_debug << "[" << loc.customer_id << ' ' << loc.weight << ']' <<  ' ';
        }
        log_debug << '\n';
      }
    }
    log_debug << "complete drone\n";
    log_debug << '\n';
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
    //debug(chr);
    for (auto [customer_id, customer_weight] : chr) {
      if (customer_id == 0 or customer_id >= num_customer) continue;

      const auto push = [&]() {
        int dec_weight = sol.push_cus(current_vehicle, trip_id, customer_id);
        if (dec_weight <= 0) return false;
        customer_weight -= dec_weight;
        return true;
      };
      const auto push_route = [&]() {
        int unsuccesd_push = 0;
        while (current_vehicle < num_truck + num_drone and unsuccesd_push < 2) {
          if (not push()) {
            if (current_vehicle < num_truck) {
              ++current_vehicle;
              trip_id = 0;
            } else {
              if (current_pushed == 0) {
                ++current_vehicle;
                trip_id = 0;
              } else {
                ++trip_id;
                current_pushed = 0;
              }
            }
            if (current_vehicle < num_truck) {
            } else if (trip_id > 0) {
              if (current_vehicle - num_truck < num_drone)
                sol.drone_trip[current_vehicle - num_truck].new_route();
              else
                break;
            }
            ++unsuccesd_push;
          } else {
            ++current_pushed;
            break;
          }
        }
      };
      while (customer_weight > 0) {
        auto tmp = customer_weight;
        push_route();
        if (tmp == customer_weight) break;
      }
    }
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
} general_setting;