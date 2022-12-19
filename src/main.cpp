#include "bits/stdc++.h"
#include "csvfile.h"
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
    const std::vector<int>& current_lowerbound) {
  std::vector<double> piority(num_customer, 0);
  double totDistance = 0;
  for (int j = 0; j < num_customer; ++j) {
    totDistance += euclid_distance(customers[0], customers[j]);
  }
  for (int j = 0; j < num_customer; ++j) {
    piority[j] = 1.0 * euclid_distance(customers[0], customers[j]) *
                 (double)current_lowerbound[j] * customers[j].cost /
                 totDistance;
  }
  return piority;
}

Solution init_random_solution() {
  Solution first_solution;
  first_solution.drone_trip.resize(num_drone, routeSet(TDRONE));
  first_solution.truck_trip.resize(num_truck, routeSet(TTRUCK));

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
    current_lowerbound[i] = max(customers[i].lower_weight, 20);

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
      debug(i, pushed_weight);
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

      routeSet.append(next_cus, trip_id);
      current_lowerbound[next_cus.first] -= next_cus.second;
    }
  };

  for (int i = 0; i < num_truck; ++i) {
    int tmp = random_number_with_probability(
        build_partial_sum(init_piority_matrix(current_lowerbound)));
    if (current_lowerbound[tmp] == 0) break;
    push_cus(first_solution.truck_trip[i], 0, tmp);

    build_route(first_solution.truck_trip[i], 0);
  }
  for (int i = 0; i < num_drone; ++i) {
    int route_id = 0;
    while (first_solution.drone_trip[i].valid_route()) {
      int tmp = random_number_with_probability(
          build_partial_sum(init_piority_matrix(current_lowerbound)));
      if (current_lowerbound[tmp] == 0) break;
      first_solution.drone_trip[i].new_route();
      push_cus(first_solution.drone_trip[i], route_id, tmp);
      build_route(first_solution.drone_trip[i], route_id);
      ++route_id;
    }
  }
  // log first_solution
  /*
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
      log_debug << loc.customer_id << ' ' << loc.weight << '\n';
    }
  }

  for (int i = 0; i < num_drone; ++i) {
    log_debug << "drone route" << i << '\n';
    for (auto trip : first_solution.drone_trip[i].multiRoute) {
      for (auto loc : trip.route) {
        log_debug << loc.customer_id << ' ' << loc.weight << '\n';
      }
      log_debug << '\n';
    }
  }
  */
  debug(first_solution.evaluate());

  first_solution.educate();

  debug(first_solution.evaluate());
  /*
  3.2. Split tour for every trip

  use greedy to assaign remain customer for remain trip
  */
  /*
  skip it because better solution?
  and i just tired
  */
  /*
    log_debug << "after educate\n";
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
        log_debug << loc.customer_id << ' ' << loc.weight << '\n';
      }
    }

    for (int i = 0; i < num_drone; ++i) {
      log_debug << "drone route" << i << '\n';
      for (auto trip : first_solution.drone_trip[i].multiRoute) {
        for (auto loc : trip.route) {
          log_debug << loc.customer_id << ' ' << loc.weight << '\n';
        }
        log_debug << '\n';
      }
    }
    log_debug << '\n';
  */
  return first_solution;
}

void random_init_population() {
  int valid = 0;
  int best = 0;
  for (int iter = 0; iter < general_setting.POPULATION_SIZE; ++iter) {
    auto sol = init_random_solution();
    valid += sol.valid_solution();
    Population.emplace_back(Chromosome(sol));
    if (sol.valid_solution())
      best = max(best, sol.evaluate());
    // sol.print_out();
    //    log_debug << "valid" << sol.valid_solution() << '\n';
  }
  log_debug << "best solution evaluate is " << best << '\n';
  log_debug << "valid solution is " << valid << '\n';
}

void ga_process() {
  Solution best;
  std::vector<Chromosome> offsprings;
  int best_generation = 0;
  const auto init = [&]() {
    offsprings.clear();
    vector<pair<int, Chromosome>> val;
    for (auto gen : Population) {
      if (gen.encode().valid_solution())
        val.emplace_back(gen.encode().fitness(), gen);
      else
        val.emplace_back(gen.encode().fitness() - 10000, gen);
    }
    sort(val.begin(), val.end(),
         [&](auto x, auto y) { return x.first > y.first; });
    Population.clear();
    for (auto [w, v] : val) Population.emplace_back(v);
  };
  const auto evaluate = [&](int gen_id) {
    for (auto Sol : Population) {
      if (Sol.encode().valid_solution())
        if (best.evaluate() < Sol.encode().evaluate()) {
          best = Sol.encode();
          best_generation = gen_id;
        }
    }
  };
  const auto create_offspring = [&]() {
    int cnt = 0;
    int off_springs_size = general_setting.POPULATION_SIZE *
                           general_setting.OFFSPRING_PERCENT / 100;
    // debug("mutation", off_springs_size, Population.size());
    while (cnt <= off_springs_size) {
      int i = rand(0, Population.size() - 1);
      int j = rand(0, Population.size() - 1);
      if (i == j) {
        continue;
      }
      cnt++;
      offsprings.emplace_back(crossover(Population[i], Population[j]));
    }
  };
  const auto educate = [&]() {
    /*

    */
    for (auto& gen : offsprings) {
      auto sol = gen.encode();
      sol.educate();
      gen = Chromosome(sol);
    }
  };
  const auto choose_next_population = [&]() {
    /// @brief sort Population

    vector<pair<int, Chromosome>> val;
    for (auto gen : Population) {
      if (gen.encode().valid_solution())
        val.emplace_back(gen.encode().fitness(), gen);
      else
        val.emplace_back(gen.encode().fitness() - 10000, gen);
    }
    sort(val.begin(), val.end(),
         [&](auto x, auto y) { return x.first > y.first; });
    Population.clear();
    for (auto [w, v] : val) Population.emplace_back(v);

    std::vector<Chromosome> next_population = offsprings;
    /// TODO: use addition vector to reduce the fitness computation
    const int one_hundred_percent = 100;
    int keep = general_setting.POPULATION_SIZE *
               (one_hundred_percent - general_setting.OFFSPRING_PERCENT) / 100;
    keep = min(keep, (int)Population.size());
    for (int i = 0; i < keep; ++i) next_population.push_back(Population[i]);

    //  for (auto it : Population) log_result << it.encode().evaluate() << ' ';
    std::swap(Population, next_population);
    //  for (auto it : Population) log_result << it.encode().evaluate() << ' ';
  };
  const auto mutation = [&]() {
    for (int iter = 0; iter < (general_setting.POPULATION_SIZE *
                               general_setting.MUTATION_ITER) /
                                  100;
         ++iter) {
      int i = random_number_in_range(10, (int)Population.size());
      int maxsize = Population[i].chr.size();
      int len =
          random_number_in_range(1, max(1, (int)Population[i].chr.size() / 5));

      int s1 = random_number_in_range(0, len);
      int s2 =
          random_number_in_range(s1 + len, min(maxsize - len, s1 + 2 * len));

      Chromosome new_gen;

      for (int j = 0; j < s1; ++j) new_gen.chr.push_back(Population[i].chr[j]);
      for (int j = s2; j < min(maxsize, s2 + len); ++j)
        new_gen.chr.push_back(Population[i].chr[j]);
      for (int j = s1; j < s2; ++j) new_gen.chr.push_back(Population[i].chr[j]);
      for (int j = s1; j < min(maxsize, s1 + len); ++j)
        new_gen.chr.push_back(Population[i].chr[j]);
      for (int j = s2 + len; j < maxsize; ++j)
        new_gen.chr.push_back(Population[i].chr[j]);
      // debug(Population[i].chr);
      //debug(new_gen.chr);
      Population[i] = new_gen;
    }
  };
  log_debug << "start ga_process\n";
  for (int iter = 0; iter <= general_setting.NUM_GENERATION; ++iter) {
    init();

    evaluate(iter);
    create_offspring();
    educate();
    choose_next_population();
    mutation();

    /*for debug*/
    log_debug << Population[0].encode().evaluate() << '\n';
    Population[0].encode().print_out();

    /*logging process*/
    best_in_generation.push_back(Population[0].encode().evaluate());
    worst_in_generation.push_back(Population.back().encode().evaluate());
    int64_t total_results = 0;
    int total_infeasibility = 0;
    for (auto gene : Population) {
      total_results += gene.encode().evaluate();
      total_infeasibility += gene.encode().valid_solution();
    }
    average_in_generation.push_back(1.0 * (double)total_results /
                                    (int)Population.size());
    num_infeasible_solution.push_back(total_infeasibility);
  }
  debug(best.evaluate());
  log_result << "complete running\n";
  log_result << "convergence after " << best_generation << '\n';
  log_result << "Solution is " << best.evaluate() << '\n';
  best.print_out();
  //assert(best.valid_solution());
}

void logging_to_csv() {
  /// @brief convert all generation result to string format
  string best, worst, average, infeasible;
  const auto vector_to_string = [&](const auto& v) {
    string ans = "[";
    for (const auto& it : v) {
      ans += to_string(it);
      ans += ',';
    }
    ans += ']';
    return ans;
  };
  best = vector_to_string(best_in_generation);
  worst = vector_to_string(worst_in_generation);
  average = vector_to_string(average_in_generation);
  infeasible = vector_to_string(num_infeasible_solution);

  log_csv_result << best << worst << average << infeasible;

  log_csv_result << Solution::evaluate_call;
  log_csv_result << Solution::educate_call;
}

int main() {
  read_input();
  random_init_population();
  ga_process();
  logging_to_csv();

  cerr << "\nTime elapsed: " << 1000 * clock() / CLOCKS_PER_SEC << "ms\n";
}