#include "bits/stdc++.h"
#include "debug.hpp"
#include "settings.hpp"

void read_input() {
  std::ifstream read_file("input.txt");
  /*
  tip for input reading

  depot is customers[0] = {all value equal to 0}
  */
  read_file >> numTruck >> numDrone >> timeLimit;
  read_file >> speedTruck >> speedDrone >> capacityTruck >> capacityDrone >>
      durationDrone;
  Customer tmp;
  numCustomer = 1;
  customers.emplace_back(Customer());
  while (read_file >> tmp.x >> tmp.y >> tmp.lower_weight >> tmp.upper_weight >>
         tmp.cost) {
    customers.emplace_back(tmp);
  }
  read_file.close();
  numCustomer = customers.size();

  debug(numTruck, numDrone, timeLimit);
  debug(speedTruck, speedDrone, capacityTruck, capacityDrone, durationDrone);
  for (auto x : customers)
    debug(x.x, x.y, x.lower_weight, x.upper_weight, x.cost);

  /*
  assert limit of constraint
  */
  assert(speedTruck != 0);
  assert(speedDrone != 0);
}

std::vector<double> init_piority_matrix(
    const std::vector<int>& current_loewrbound) {
  std::vector<double> piority(numCustomer, 0);
  double totDistance = 0;
  for (int j = 0; j < numCustomer; ++j) {
    totDistance += euclid_distance(customers[0], customers[j]);
  }
  for (int j = 0; j < numCustomer; ++j) {
    piority[j] = 1.0 * euclid_distance(customers[0], customers[j]) *
                 (double)current_loewrbound[j] * customers[j].cost /
                 totDistance;
  }
  return piority;
}

Solution init_random_solution() {
  Solution first_solution;
  first_solution.drone_trip.resize(numDrone);
  first_solution.truck_trip.resize(numDrone);

  /*
  set vehicle type for every trip
  try to dynamic calculate remaining time
  */
  for (int i = 0; i < numDrone; ++i) {
    first_solution.drone_trip[i].set_vehicle_type(TDRONE);
  }
  for (int i = 0; i < numDrone; ++i) {
    first_solution.truck_trip[i].set_vehicle_type(TTRUCK);
  }

  /*
  3.1. Create first customer for every trip
  */

  /*
  3.2. Init first route for every vehicle
  */
  std::vector<int> current_lowerbound(numCustomer);
  for (int i = 0; i < numCustomer; ++i)
    current_lowerbound[i] = customers[i].lower_weight;

  for (int i = 0; i < numTruck; ++i) {
    double tot_time = 0;
    int now_weight = capacityTruck;
    int tmp = random_number_with_probability(
        build_partial_sum(init_piority_matrix(current_lowerbound)));
    first_solution.truck_trip[i].new_route();
    first_solution.truck_trip[i].append({0, tmp}, 0);
    if (current_lowerbound[tmp] == 0) break;
    for (int j = 1; j < (int)first_solution.truck_trip[i].multiRoute[0].size();
         ++j) {
      auto current_loc = first_solution.truck_trip[i].multiRoute[0].route[j];
      debug("truck ", i, current_loc->customer_id);
      if (current_loc->prev_node != nullptr)
        tot_time +=
            time_travel(customers[current_loc->customer_id],
                        customers[current_loc->prev_node->customer_id], TTRUCK);
      int push_weight =
          std::min(current_lowerbound[current_loc->customer_id], now_weight);
      first_solution.truck_trip[i].multiRoute[0].route.back()->weight =
          push_weight;
      now_weight -= push_weight;
      current_lowerbound[current_loc->customer_id] -= push_weight;
      debug(i, push_weight);
      if (push_weight == 0) {
        first_solution.truck_trip[i].pop(0);
        break;
      }
      if (now_weight == 0) break;
      double min_dist = std::numeric_limits<double>::max();
      int next_customer = -1;
      for (int i = 1; i < numCustomer; ++i) {
        /*
        check condition for time_limit and availablity for customer i
        */
        if (i == current_loc->customer_id) continue;
        if (tot_time + time_travel(customers[current_loc->customer_id],
                                   customers[i], TTRUCK) >
            timeLimit)
          continue;
        if (current_lowerbound[i] == 0) continue;
        if (minimize<double>(
                min_dist, euclid_distance(customers[current_loc->customer_id],
                                          customers[i]))) {
          next_customer = i;
        }
      }
      debug("truck", next_customer);
      if (next_customer != -1) {
        first_solution.truck_trip[i].append({0, next_customer}, 0);
      }
    }
  }

  for (int i = 0; i < numDrone; ++i) {
    double tot_time = 0;
    int now_weight = capacityDrone;
    int route_id = 0;

    while (first_solution.drone_trip[i].total_time < timeLimit) {
      int tmp = random_number_with_probability(
          build_partial_sum(init_piority_matrix(current_lowerbound)));
      first_solution.drone_trip[i].new_route();
      first_solution.drone_trip[i].append({0, tmp}, route_id);
      /// split new route on drone
      debug("build for ", route_id);
      int pushed_cus = 0;
      for (int j = 1;
           j < (int)first_solution.drone_trip[i].multiRoute[0].size(); ++j) {
        auto current_loc = first_solution.drone_trip[i].multiRoute[0].route[j];
        debug("drone", current_loc->customer_id);
        if (current_loc->prev_node != nullptr)
          tot_time +=
              euclid_distance(customers[current_loc->customer_id],
                              customers[current_loc->prev_node->customer_id]);

        int push_weight =
            std::min(current_lowerbound[current_loc->customer_id], now_weight);
        first_solution.drone_trip[i].multiRoute[route_id].route.back()->weight =
            push_weight;
        current_lowerbound[i] -= push_weight;
        now_weight -= push_weight;
        if (push_weight == 0) {
          first_solution.drone_trip[i].pop(route_id);
          --pushed_cus;
          break;
        }
        if (now_weight == 0) break;
        double min_dist = std::numeric_limits<double>::max();
        int next_customer = -1;
        for (int i = 1; i < numCustomer; ++i) {
          /// check condition for time_limit and availablity for customer i
          if (i == current_loc->customer_id) continue;
          if (tot_time + time_travel(customers[current_loc->customer_id],
                                     customers[i], TDRONE) >
              durationDrone)
            continue;
          if (tot_time + time_travel(customers[current_loc->customer_id],
                                     customers[i], TDRONE) >
              timeLimit)
            continue;
          if (current_lowerbound[i] == 0) continue;
          if (minimize<double>(
                  min_dist, euclid_distance(customers[current_loc->customer_id],
                                            customers[i]))) {
            next_customer = i;
          }
        }
        debug(next_customer);
        if (next_customer != -1) {
          first_solution.drone_trip[i].append({0, next_customer}, route_id);
          ++pushed_cus;
        }
      }
      if (pushed_cus)
        ++route_id;
      else
        break;
    }
  }

  // log first_solution
  log_debug << "first_solution\n";
  log_debug << first_solution.valid_solution() << '\n';
  log_debug << "current lowerber\n";
  for (int i = 0; i < numCustomer; ++i) {
    log_debug << "cus " << i << ' ' << current_lowerbound[i] << '\n';
  }
  log_debug << '\n';
  for (int i = 0; i < numTruck; ++i) {
    log_debug << "truck" << ' ' << i << '\n';
    for (auto loc : first_solution.truck_trip[i].multiRoute[0].route) {
      log_debug << loc->customer_id << ' ' << loc->weight << '\n';
    }
  }

  for (int i = 0; i < numDrone; ++i) {
    log_debug << "drone " << i << '\n';
    for (auto trip : first_solution.drone_trip[i].multiRoute) {
      for (auto loc : trip.route) {
        log_debug << loc->customer_id << ' ' << loc->weight << '\n';
      }
    }
  }
  log_debug << '\n';

  /*
  3.2. Split tour for every trip

  use greedy to assaign remain customer for remain trip
  */
  /*
  skip it because better solution?
  and i just tired
  */

  /*
  TODO:
  is it feasible solution?
  */
  return first_solution;
}

void random_init_population() {
  int valid = 0;
  for (int iter = 0; iter < 50; ++iter) {
    auto sol = init_random_solution();
    valid += sol.valid_solution();
    Population.emplace_back(Chromosome(sol));
  }

  log_debug << "valid solution is " << valid << '\n';
}
void ga_process() {
  Solution best;
  std::vector<Chromosome> offsprings;
  const auto init = [&]() { offsprings.clear(); };
  const auto evaluate = [&]() {
    for (auto Sol : Population) {
      if (best.evaluate() < Sol.encode().evaluate()) {
        best = Sol.encode();
      }
    }
  };
  const auto mutation = [&]() {
    int cnt = 0;
    while (cnt <= 30) {
      int i = rand(0, Population.size() - 1);
      int j = rand(0, Population.size() - 1);
      if (i == j) {
        continue;
      }
      ++cnt;
      offsprings.emplace_back();
    }
  };
  const auto educate = [&]() {

  };
  for (int iter = 0; iter <= 10000; ++iter) {
    init();
    evaluate();
    mutation();
    educate();
  }
  log_debug << "Solution is" << best.evaluate();
}

int main() {
  read_input();
  random_init_population();
  ga_process();
}