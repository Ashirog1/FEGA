#include "bits/stdc++.h"
#include "debug.hpp"
#include "settings.hpp"

void read_input() {
  std::ifstream read_file("input.txt");
  /*
  tip for input reading

  depot is customers[0] = {all value equal to 0}
  */
  read_file >> num_truck >> num_drone >> time_limit;
  read_file >> speed_truck >> speed_drone >> capacity_truck >> capacity_drone >>
      duration_drone;
  Customer tmp;
  num_customer = 1;
  customers.emplace_back(Customer());
  while (read_file >> tmp.x >> tmp.y >> tmp.lower_weight >> tmp.upper_weight >>
         tmp.cost) {
    customers.emplace_back(tmp);
  }
  read_file.close();
  num_customer = customers.size();

  debug(num_truck, num_drone, time_limit);
  debug(speed_truck, speed_drone, capacity_truck, capacity_drone,
        duration_drone);
  for (auto x : customers)
    debug(x.x, x.y, x.lower_weight, x.upper_weight, x.cost);

  /*
  assert limit of constraint
  */
  assert(speed_truck != 0);
  assert(speed_drone != 0);
}

std::vector<double> init_piority_matrix(
    const std::vector<int>& current_loewrbound) {
  std::vector<double> piority(num_customer, 0);
  double totDistance = 0;
  for (int j = 0; j < num_customer; ++j) {
    totDistance += euclid_distance(customers[0], customers[j]);
  }
  for (int j = 0; j < num_customer; ++j) {
    piority[j] = 1.0 * euclid_distance(customers[0], customers[j]) *
                 (double)current_loewrbound[j] * customers[j].cost /
                 totDistance;
  }
  return piority;
}

Solution init_random_solution() {
  Solution first_solution;
  first_solution.drone_trip.resize(num_drone);
  first_solution.truck_trip.resize(num_drone);

  /*
  set vehicle type for every trip
  try to dynamic calculate remaining time
  */
  for (int i = 0; i < num_drone; ++i) {
    first_solution.drone_trip[i].set_vehicle_type(TDRONE);
  }
  for (int i = 0; i < num_drone; ++i) {
    first_solution.truck_trip[i].set_vehicle_type(TTRUCK);
  }

  /*
  3.1. Create first customer for every trip
  */

  /*
  3.2. Init first route for every vehicle
  */
  std::vector<int> current_lowerbound(num_customer);
  for (int i = 0; i < num_customer; ++i)
    current_lowerbound[i] = customers[i].lower_weight;

  for (int i = 0; i < num_truck; ++i) {
    double tot_time = 0;
    int now_weight = capacity_truck;
    int tmp = random_number_with_probability(
        build_partial_sum(init_piority_matrix(current_lowerbound)));
    first_solution.truck_trip[i].new_route();
    first_solution.truck_trip[i].append({tmp, 0}, 0);
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
      debug(current_loc->customer_id, push_weight);
      if (push_weight == 0) {
        first_solution.truck_trip[i].pop(0);
        break;
      }
      if (now_weight == 0) break;
      double min_dist = std::numeric_limits<double>::max();
      int next_customer = -1;
      for (int i = 1; i < num_customer; ++i) {
        /*
        check condition for time_limit and availablity for customer i
        */
        if (i == current_loc->customer_id) continue;
        if (tot_time + time_travel(customers[current_loc->customer_id],
                                   customers[i], TTRUCK) >
            time_limit)
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
        first_solution.truck_trip[i].append({next_customer, 0}, 0);
      }
    }
  }

  for (int i = 0; i < num_drone; ++i) {
    double tot_time = 0;
    int now_weight = capacity_drone;
    int route_id = 0;

    while (first_solution.drone_trip[i].total_time < time_limit) {
      int tmp = random_number_with_probability(
          build_partial_sum(init_piority_matrix(current_lowerbound)));
      first_solution.drone_trip[i].new_route();
      first_solution.drone_trip[i].append({tmp, 0}, route_id);
      debug(i, route_id, tmp);
      /// split new route on drone
      if (current_lowerbound[tmp] == 0) break;
      debug("build for ", route_id);
      int pushed_cus = 0;
      for (int j = 1;
           j < (int)first_solution.drone_trip[i].multiRoute[route_id].size();
           ++j) {
        auto current_loc =
            first_solution.drone_trip[i].multiRoute[route_id].route[j];
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
        for (int i = 1; i < num_customer; ++i) {
          /// check condition for time_limit and availablity for customer i
          /// TODO: move the condition checking process to solution method?
          /// use append is enough? and pop if success?
          if (i == current_loc->customer_id) continue;
          if (tot_time + time_travel(customers[current_loc->customer_id],
                                     customers[i], TDRONE) >
              duration_drone)
            continue;
          if (tot_time + time_travel(customers[current_loc->customer_id],
                                     customers[i], TDRONE) >
              time_limit)
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
          first_solution.drone_trip[i].append({next_customer, 0}, route_id);
          ++pushed_cus;
        }
      }
      if (pushed_cus >= 1)
        ++route_id;
      else
        break;
    }
  }

  // log first_solution
  log_debug << "after 3.2\n";
  log_debug << "first_solution\n";
  log_debug << first_solution.valid_solution() << '\n';
  log_debug << "current lowerbound\n";
  for (int i = 0; i < num_customer; ++i) {
    log_debug << "customer " << i << ' ' << current_lowerbound[i] << '\n';
  }
  log_debug << '\n';
  for (int i = 0; i < num_truck; ++i) {
    log_debug << "truck route" << ' ' << i << '\n';
    for (auto loc : first_solution.truck_trip[i].multiRoute[0].route) {
      log_debug << loc->customer_id << ' ' << loc->weight << '\n';
    }
  }

  for (int i = 0; i < num_drone; ++i) {
    log_debug << "drone route" << i << '\n';
    for (auto trip : first_solution.drone_trip[i].multiRoute) {
      for (auto loc : trip.route) {
        log_debug << loc->customer_id << ' ' << loc->weight << '\n';
      }
      log_debug << '\n';
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
  for (int iter = 0; iter < 1; ++iter) {
    auto sol = init_random_solution();
    valid += sol.valid_solution();
    Population.emplace_back(Chromosome(sol));
  }

  log_debug << "valid solution is " << valid << '\n';
}
void ga_process() {
  Solution best;
  std::vector<Chromosome> offsprings;
  const auto init = [&]() {
    offsprings.clear();
    vector
  };
  const auto evaluate = [&]() {
    for (auto Sol : Population) {
      if (best.evaluate() < Sol.encode().evaluate()) {
        best = Sol.encode();
      }
    }
  };
  const auto mutation = [&]() {
    int cnt = 0;
    int off_springs_size = general_setting.POPULATION_SIZE *
                           general_setting.OFFSPRING_PERCENT / 100;
    while (cnt <= off_springs_size) {
      int i = rand(0, Population.size() - 1);
      int j = rand(0, Population.size() - 1);
      if (i == j) {
        continue;
      }
      ++cnt;
      offsprings.emplace_back(crossover(Population[i], Population[j]));
    }
  };
  const auto educate = [&]() {
    /*
     */
  };
  const auto choose_next_population = [&]() {
    std::vector<Chromosome> next_population = offsprings;
    /// TODO: use addition vector to reduce the fitness computation
    const int one_hundred_percent = 100;
    int keep = general_setting.POPULATION_SIZE *
               (one_hundred_percent - general_setting.OFFSPRING_PERCENT) / 100;
    next_population.insert(next_population.end(), Population.begin(),
                           Population.begin() + keep);
    std::swap(Population, next_population);
  };
  for (int iter = 0; iter <= 10000; ++iter) {
    debug("generation", iter);
    init();
    evaluate();
    mutation();
    educate();
    choose_next_population();
  }
  log_debug << "Solution is" << best.evaluate();
}

int main() {
  read_input();
  random_init_population();
  ga_process();
}
