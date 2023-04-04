#include "fstream"
#include "iostream"
#include "list"
#include "mcmf.cpp"
#include "numeric"
#include "random"
#include "vector"
#include "network_simplex.cpp"

constexpr int TTRUCK = 0, TDRONE = 1;
/// @brief default constraint parameter
int num_customer, num_truck, num_drone, time_limit;
double speed_truck, speed_drone;
int capacity_truck, capacity_drone, duration_drone;

std::ofstream log_debug("debug.log");
std::ofstream log_result("result.log");
csvfile log_csv_result("result.csv");


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
template <class T>
bool minimize(T &x, const T &y) {
  if (x > y) {
    x = y;
    return true;
  }
  return false;
}

std::mt19937 rng(chrono::steady_clock::now().time_since_epoch().count());

int rand(int l, int r) { return l + rng() % (r - l + 1); }

/*
return a real value in defined range
*/
double random_number_in_range(double l, double r) {
  std::uniform_real_distribution<double> unif(l, r);
  std::default_random_engine re;
  return unif(rng);
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

std::vector<double> init_piority_matrix(
    const std::vector<int>& current_lowerbound) {
  std::vector<double> priority(num_customer, 0);
  for (int j = 1; j < num_customer; ++j) {
    priority[j] = 1.0 * (double)current_lowerbound[j] * customers[j].cost /
                  (double)euclid_distance(customers[0], customers[j]);
  }
  debug(current_lowerbound, priority);
  return priority;
}

class settings {
 public:
  int POPULATION_SIZE = 200;
  int OFFSPRING_PERCENT = 90;
  int MUTATION_ITER = 10;
  int NUM_GENERATION = 100;
} general_setting;

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

    bool existed = false;
    for (auto it : route) {
      if (it.customer_id == customer_id) {
        existed = true;
        break;
      }
    }

    if (existed) {
      for (auto &it : route) {
        if (it.customer_id == customer_id) {
          it.weight += weight;
          total_weight += weight;
          if (valid_route()) {
            return true;
          } else {
            for (auto it : route) debug(it.customer_id, it.weight);

            it.weight -= weight;
            total_weight -= weight;

            for (auto it : route) debug(it.customer_id, it.weight);
            return false;
          }
        }
      }
    }

    /*     debug("route");
        // /for (auto it : route) debug(it.customer_id, it.weight);
        debug(route.size());
        debug(route.back().customer_id);
        debug(vehicle_type);
        debug(customers[route.back().customer_id].x,
       customers[route.back().customer_id].y); debug(info); */
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
    if (not flag) return false;
    if (flag) {
      if (not valid_route()) {
        // debug(total_time);
        pop(trip_id);
        return false;
      }
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
      truck_trip[i].new_route();
    }
    for (int i = 0; i < num_drone; ++i) {
      drone_trip[i].set_vehicle_type(TDRONE);
      drone_trip[i].new_route();
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
    // debug(current_lowerbound[customer_id]);
    return min(current_lowerbound[customer_id],
               find_remaining_weight(route_id, trip_id));
  }
  int find_remaining_weight(int route_id, int trip_id) {
    if (route_id > num_drone + num_truck) return 0;
    /// debug(route_id, trip_id, customer_id,
    /// route_at(route_id)->multiRoute.size());
    if (trip_id > route_at(route_id)->multiRoute.size()) {
      return 0;
    }
    if (route_id < num_truck)
      return truck_trip[route_id].multiRoute[trip_id].rem_weight();
    else {
      return drone_trip[route_id - num_truck].multiRoute[trip_id].rem_weight();
    }
  }
  int push_cus(int route_id, int trip_id, pair<int, int> customer_info) {
    std::pair<int, int> cus = {
        customer_info.first,
        find_pushed_weight(route_id, trip_id, customer_info.first)};
    cus.second = min(customer_info.second, cus.second);
    // debug(customer_info, trip_id);
    // debug(cus);
    if (cus.second == 0) return -1;
    if (route_at(route_id)->append(cus, trip_id)) {
      current_lowerbound[cus.first] -= cus.second;
      total_weight[cus.first] += cus.second;
      // debug(customer_info);
      return cus.second;
    } else {
      return -1;
    }
  }
  void split_process() {}
  void educate_with_lowerbound() {
    int N = 0, M = num_customer;
    for (int i = 0; i < num_truck; ++i) N += truck_trip[i].multiRoute.size();
    for (int i = 0; i < num_drone; ++i) {
      N += drone_trip[i].multiRoute.size();
    }
    int S = N + M + 1, T = S + 1;
    network_simplex<long, long, int64_t> ns(T + 5);

    int cur_route = 0;
    for (int i = 0; i < num_truck; ++i) {
      for (auto &route : truck_trip[i].multiRoute) {
        ++cur_route;
        route.total_weight = 0;
        for (auto cus : route.route) {
          if (cus.customer_id == 0) continue;
          ns.add(cur_route, N + cus.customer_id, 0, INT_MAX, 0);
          route.total_weight += cus.weight;
        }
        ns.add(S, cur_route, 0, capacity_truck, 0);
      }
    }
    for (int i = 0; i < num_drone; ++i) {
      for (auto &route : drone_trip[i].multiRoute) {
        route.total_weight = 0;
        ++cur_route;
        for (auto cus : route.route) {
          if (cus.customer_id == 0) continue;
          ns.add(cur_route, N + cus.customer_id, 0, INT_MAX, 0);
          route.total_weight += cus.weight;
          // debug(cur_route, N + cus.customer_id, N);
        }
        ns.add(S, cur_route, 0, capacity_drone, 0);
      }
    }
    for (int i = 1; i < num_customer; ++i) {
      ns.add(N + i, T, customers[i].lower_weight, customers[i].upper_weight,
                -customers[i].cost);
    }
    ns.add(T, S, 0, INT_MAX, 0);
    ns.mincost_circulation();
    
    debug("go mcmf");

    /// assign weight based on mcmf solution
    cur_route = 0;
    int e = 0;
    for (int i = 0; i < num_truck; ++i) {
      for (auto &route : truck_trip[i].multiRoute) {
        for (auto &cus : route.route) {
          if (cus.customer_id == 0) continue;
          cus.weight = ns.get_flow(e++);
        }
        e++;
      }
    }
    for (int i = 0; i < num_drone; ++i) {
      for (auto &route : drone_trip[i].multiRoute) {
        for (auto &cus : route.route) {
          if (cus.customer_id == 0) continue;
          cus.weight = ns.get_flow(e++);
        }
        e++;
      }
    }
  }
  void educate() {
    /// push process? push remaining weight to cus

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
    int N = 0, M = num_customer;
    for (int i = 0; i < num_truck; ++i) N += truck_trip[i].multiRoute.size();
    for (int i = 0; i < num_drone; ++i) {
      N += drone_trip[i].multiRoute.size();
    }
    int S = N + M + 1, T = S + 1;
    mcmf.init(T, S, T);

    int cur_route = 0;
    for (int i = 0; i < num_truck; ++i) {
      for (auto &route : truck_trip[i].multiRoute) {
        ++cur_route;
        route.total_weight = 0;
        for (auto cus : route.route) {
          if (cus.customer_id == 0) continue;
          mcmf.addedge(cur_route, N + cus.customer_id, INT_MAX, 0);

          // debug(cur_route, N + cus.customer_id, N);
          route.total_weight += cus.weight;
        }
        mcmf.addedge(S, cur_route, route.rem_weight(), 0);
      }
    }
    for (int i = 0; i < num_drone; ++i) {
      for (auto &route : drone_trip[i].multiRoute) {
        route.total_weight = 0;
        ++cur_route;
        for (auto cus : route.route) {
          if (cus.customer_id == 0) continue;
          mcmf.addedge(cur_route, N + cus.customer_id, INT_MAX, 0);
          route.total_weight += cus.weight;
          // debug(cur_route, N + cus.customer_id, N);
        }
        mcmf.addedge(S, cur_route, route.rem_weight(), 0);
      }
    }
    for (int i = 1; i < num_customer; ++i) {
      mcmf.addedge(N + i, T, customers[i].upper_weight - total_weight[i],
                   -customers[i].cost);
    }
    mcmf.mcmf();
    debug("go mcmf");
    for (auto e : mcmf.e) {
    }

    /// assign weight based on mcmf solution
    map<pair<int, int>, int> assign_weight;
    for (auto e : mcmf.e) {
      assign_weight[{e.from, e.to}] = e.flow;
    }
    cur_route = 0;
    for (int i = 0; i < num_truck; ++i) {
      for (auto &route : truck_trip[i].multiRoute) {
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
      for (auto &route : drone_trip[i].multiRoute) {
        ++cur_route;
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
    /// reduce to satisfy
    std::vector<int> total_weight(num_customer);
    for (auto truck : truck_trip) {
      for (auto r : truck.multiRoute)
        for (auto loc : r.route) total_weight[loc.customer_id] += loc.weight;
    }
    for (auto drone : drone_trip) {
      for (auto r : drone.multiRoute)
        for (auto loc : r.route) total_weight[loc.customer_id] += loc.weight;
    }
    const auto correct = [&](auto &r, auto &loc) {
      if (total_weight[loc.customer_id] >
          customers[loc.customer_id].lower_weight) {
        int dec = min(loc.weight, total_weight[loc.customer_id] -
                                      customers[loc.customer_id].lower_weight);
        loc.weight -= dec;
        total_weight[loc.customer_id] -= dec;
        r.total_weight -= dec;
        debug(dec);
      }
    };
    for (int i = 0; i < num_truck; ++i) {
      for (auto &r : truck_trip[i].multiRoute) {
        for (auto &loc : r.route) {
          correct(r, loc);
        }
      }
    }
    for (int i = 0; i < num_drone; ++i) {
      for (auto &r : drone_trip[i].multiRoute) {
        for (auto &loc : r.route) {
          correct(r, loc);
        }
      }
    }
  }
  void educate3() {
    std::vector<int> current_lowerbound(num_customer);
    std::fill(total_weight.begin(), total_weight.end(), 0);
    for (auto truck : truck_trip) {
      for (auto r : truck.multiRoute)
        for (auto loc : r.route) total_weight[loc.customer_id] += loc.weight;
    }
    for (auto drone : drone_trip) {
      for (auto r : drone.multiRoute) {
        for (auto loc : r.route) total_weight[loc.customer_id] += loc.weight;
      }
    }
    for (int i = 0; i < num_customer; ++i)
      current_lowerbound[i] = customers[i].upper_weight - total_weight[i];

    const auto find_pushed_weight = [&](routeSet& routeSet, int trip_id,
                                        int customer_id) {
      return min(current_lowerbound[customer_id],
                routeSet.multiRoute[trip_id].rem_weight());
    };
    const auto push_cus = [&](routeSet& routeSet, int trip_id, int cus_id) {
      std::pair<int, int> cus = {cus_id,
                                find_pushed_weight(routeSet, trip_id, cus_id)};

      if (routeSet.append(cus, trip_id)) {
        current_lowerbound[cus.first] -= cus.second;
      }
    };

    const auto find_next_cus = [&](routeSet& routeSet, int trip_id) {
      std::pair<int, int> next_customer = {-1, 0};
      double min_dist = std::numeric_limits<double>::max();
      for (int i = 1; i < num_customer; ++i) {
        /// check if successful append?
        int pushed_weight =
            min(current_lowerbound[i], routeSet.multiRoute[trip_id].rem_weight());
        // debug(current_lowerbound[i],
        // routeSet.multiRoute[trip_id].rem_weight());
        if (pushed_weight <= 0) continue;
        /*       debug(i, pushed_weight);
              debug("route");
              for (auto it : routeSet.multiRoute[trip_id].route)
                debug(it.customer_id, it.weight); */
        bool flag = routeSet.append({i, pushed_weight}, trip_id);
        if (flag)
          routeSet.pop(trip_id);
        else
          continue;
        if (minimize<double>(
                min_dist,
                euclid_distance(
                    customers
                        [routeSet.multiRoute[trip_id].route.back().customer_id],
                    customers[i]))) {
          next_customer = {i, pushed_weight};
        }
      }
      debug(next_customer);
      return next_customer;
    };

    const auto build_route = [&](routeSet& routeSet, int trip_id) {
      while (routeSet.valid_route()) {
        auto next_cus = find_next_cus(routeSet, trip_id);
        debug(next_cus);
        if (next_cus.first == -1) break;
        // debug("before");
        // for (auto it : routeSet.multiRoute[trip_id].route)
        // debug(it.customer_id, it.weight);
        push_cus(routeSet, trip_id, next_cus.first);
        // debug("after");
        // for (auto it : routeSet.multiRoute[trip_id].route)
        // debug(it.customer_id, it.weight); debug(current_lowerbound);
      }
    };

    const auto build_random_route = [&](routeSet& routeSet, int trip_id) {
      int tmp = random_number_with_probability(
          build_partial_sum(init_piority_matrix(current_lowerbound)));
      debug("first customer is", tmp);
      if (current_lowerbound[tmp] == 0 or tmp == 0) return;

      push_cus(routeSet, trip_id, tmp);
      build_route(routeSet, trip_id);
    };


    for (int i = 0; i < num_drone; ++i) {
      int route_id = 0;
      /// @brief find current route_id
      /// @return

      for (int j = 0; j < drone_trip[i].multiRoute.size(); ++j) {
        if (drone_trip[i].multiRoute[j].route.size() <= 1) {
          route_id = j;
          break;
        }
      }

      while (drone_trip[i].valid_route()) {
        build_random_route(drone_trip[i], route_id);
        /// @brief only depot
        if (drone_trip[i].multiRoute[route_id].route.size() <= 1)
          break;
        /// create new route for drone
        drone_trip[i].new_route();
        ++route_id;
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
  pair<int, int> find_remain_cus() {
    std::fill(total_weight.begin(), total_weight.end(), 0);
    for (auto truck : truck_trip) {
      for (auto r : truck.multiRoute)
        for (auto loc : r.route) total_weight[loc.customer_id] += loc.weight;
    }
    for (auto drone : drone_trip) {
      for (auto r : drone.multiRoute)
        for (auto loc : r.route) total_weight[loc.customer_id] += loc.weight;
    }
    vector<pair<int, int>> rem;
    for (int i = 1; i < num_customer; ++i) {
      if (total_weight[i] < customers[i].upper_weight)
        rem.emplace_back(i, customers[i].upper_weight - total_weight[i]);
    }
    if (rem.empty()) return {-1, -1};
    return rem[random_number_in_range(0, (int)rem.size() - 1)];
  }
  void print_out() {
    std::fill(total_weight.begin(), total_weight.end(), 0);
    for (auto truck : truck_trip) {
      for (auto r : truck.multiRoute)
        for (auto loc : r.route) total_weight[loc.customer_id] += loc.weight;
    }
    for (auto drone : drone_trip) {
      for (auto r : drone.multiRoute)
        for (auto loc : r.route) total_weight[loc.customer_id] += loc.weight;
    }
    log_debug << "solution debug\n";
    log_debug << "objective function\n";
    log_debug << evaluate() << '\n';
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
    log_debug << "complete truck with total time is "
              << truck_trip[0].total_time << '\n';
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
    log_debug << "complete drone with total time is "
              << drone_trip[0].total_time << '\n';
    log_debug << '\n';
  }
};
/* a constant seed random integer generator */

/*
for genetic part
use std::list for O(1) mutation operation
*/

class Chromosome {
 public:
  std::vector<std::pair<int, int>> chr;
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
    int current_vehicle = 0, trip_id = 0;
    // log_debug << "[";
    // for (auto it : chr) {
    //   log_debug << "[" << it.first << "," << it.second << "]";
    // }
    // log_debug << "]";

    for (auto customer_info : chr) {
      auto [customer_id, customer_weight] = customer_info;
      if (sol.current_lowerbound[customer_id] == 0) continue;
      // debug(customer_info);
      if (customer_id == 0 or customer_id >= num_customer) continue;
      const auto pushable = [&]() {
        int dec_weight = sol.push_cus(current_vehicle, trip_id,
                                      {customer_id, customer_weight});
        if (dec_weight <= 0) return false;
        customer_weight -= dec_weight;
        return true;
      };
      const auto push_route = [&]() {
        /// @brief truck route
        if (current_vehicle < num_truck) {
          while (current_vehicle < num_truck) {
            if (not pushable()) {
              ++current_vehicle;
              trip_id = 0;
            } else {
              return;
            }
          }
        }
        /// @brief drone route
        if (current_vehicle >= num_truck) {
          while (current_vehicle < num_truck + num_drone) {
            if (not pushable()) {
              // debug(trip_id, customer_id, customer_weight);
              if (sol.drone_trip[current_vehicle - num_truck]
                      .multiRoute[trip_id]
                      .route.back()
                      .customer_id == 0) {
                ++current_vehicle;
                trip_id = 0;
              } else {
                ++trip_id;
                sol.drone_trip[current_vehicle - num_truck].new_route();
              }
            } else {
              break;
            }
          }
        }
      };
      while (customer_weight > 0) {
        auto tmp = customer_weight;
        push_route();
        if (sol.current_lowerbound[customer_id] == 0) break;
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

void sort_population() {
  vector<pair<int, Chromosome>> val;
  for (auto gen : Population) {
    if (gen.encode().valid_solution())
      val.emplace_back(gen.encode().fitness(), Chromosome(gen.encode()));
    else
      val.emplace_back(gen.encode().fitness() - 10000,
                       Chromosome(gen.encode()));
  }
  sort(val.begin(), val.end(),
       [&](auto x, auto y) { return x.first > y.first; });
  Population.clear();
  for (auto [w, v] : val) Population.emplace_back(v);
}

void mutation1() {
  for (int iter = 0;
       iter <
       (general_setting.POPULATION_SIZE * general_setting.MUTATION_ITER) / 100;
       ++iter) {
    int i = random_number_in_range(1, (int)Population.size() - 1);
    int maxsize = Population[i].chr.size();
    int len =
        random_number_in_range(1, max(1, (int)Population[i].chr.size() / 10));

    int s1 = random_number_in_range(0, maxsize - 2 * len);
    int s2 = random_number_in_range(s1 + len, min(maxsize - len, s1 + 2 * len));

    Chromosome new_gen;

    for (int j = 0; j < s1; ++j) new_gen.chr.push_back(Population[i].chr[j]);
    // for (int j = s2; j < min(maxsize, s2 + len); ++j)
    //   new_gen.chr.push_back(Population[i].chr[j]);
    for (int j = min(maxsize, s2 + len) - 1; j >= s2; --j)
      new_gen.chr.push_back(Population[i].chr[j]);
    for (int j = s1; j < s2; ++j) new_gen.chr.push_back(Population[i].chr[j]);
    // for (int j = s1; j < min(maxsize, s1 + len); ++j)
    //   new_gen.chr.push_back(Population[i].chr[j]);
    for (int j = min(maxsize, s1 + len) - 1; j >= s1; --j)
      new_gen.chr.push_back(Population[i].chr[j]);
    for (int j = s2 + len; j < maxsize; ++j)
      new_gen.chr.push_back(Population[i].chr[j]);
    // debug(Population[i].chr);
    // debug(new_gen.chr);
    Population[i] = new_gen;
  }
}

void mutation2() {
  for (int iter = 0; iter < general_setting.MUTATION_ITER; ++iter) {
    int i = random_number_in_range(1, (int)Population.size() - 1);
    int j = random_number_in_range(0, Population[i].chr.size());

    Solution sol = Population[i].encode();
    
    pair<int, int> cus = sol.find_remain_cus();
    if (cus.first == -1) continue;

    debug("before", Population[i].chr);
    auto copy_gene = Population[i].chr;
    copy_gene.clear();
    for (int k = 0; k < j; ++k) copy_gene.emplace_back(Population[i].chr[k]);
    copy_gene.emplace_back(cus);
    for (int k = j; k < Population[i].chr.size(); ++k)
      copy_gene.emplace_back(Population[i].chr[k]);

    Population[i].chr = copy_gene;
    debug("after", Population[i].chr);
  }
}

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
