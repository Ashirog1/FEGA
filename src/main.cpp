#include "bits/stdc++.h"
#include "csvfile.h"
#include "debug.hpp"
#include "settings.hpp"

void read_input() {
  // std::ifstream read_file("input.txt");
  /*
  tip for input reading

  depot is customers[0] = {all value equal to 0}
  */
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
  // read_file.close();
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

  /// time_travel logging
  /// for (int i = 0; i < num_customer; ++i) {
  //   log_debug << "distance from" << i << '\n';
  //   for (int j = 1; j < num_customer; ++j) {
  //     log_debug << (time_travel(customers[i], customers[j], TDRONE)) << ' ';
  //   }
  //   log_debug << '\n';
  // }
  //exit(0);
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
    current_lowerbound[i] = max(customers[i].lower_weight, 1);

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
      debug(cus.first, current_lowerbound[cus.first]);
      assert(current_lowerbound[cus.first] >= 0);
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
              euclidDistance(
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

  for (int i = 0; i < num_truck; ++i) {
    debug("build truck trip", i);
    build_random_route(first_solution.truck_trip[i], 0);
  }

  for (int i = 0; i < num_drone; ++i) {
    int route_id = 0;
    while (first_solution.drone_trip[i].valid_route()) {
      debug("build drone trip", i, "trip_id", route_id);

      build_random_route(first_solution.drone_trip[i], route_id);

      for (auto it : first_solution.drone_trip[i].multiRoute[route_id].route)
        debug(it.customer_id, it.weight);

      if (first_solution.drone_trip[i].multiRoute[route_id].route.size() <= 1)
        break;
      /// create new route for drone
      first_solution.drone_trip[i].new_route();
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

  /// std::chrono::steady_clock::time_point begin =
  /// std::chrono::steady_clock::now();
  first_solution.educate3();
  first_solution.educate_with_lowerbound();
  /// std::chrono::steady_clock::time_point end =
  /// std::chrono::steady_clock::now();

  /// std::cout << "Time difference = " <<
  /// std::chrono::duration_cast<std::chrono::microseconds>(end - begin).count()
  /// << "[µs]" << std::endl;

  /*
  3.2. Split tour for every trip

  use greedy to assaign remain customer for remain trip
  skip it because better solution?
  and i just tired
  */

  return first_solution;
}

Solution init_with_permutation() {
  vector<int> p(num_customer);
  iota(p.begin(), p.end(), 1);
  shuffle(p.begin(), p.end(), rng);

  Chromosome chr;
  for (auto cus : p) chr.chr.emplace_back(cus, customers[cus].lower_weight);
  Solution sol = chr.encode();
  sol.educate3();
  sol.educate_with_lowerbound();
  return sol;
}

void random_init_population() {
  int valid = 0;
  int best = 0;
  for (int iter = 0; iter < general_setting.POPULATION_SIZE; ++iter) {
    /// @brief
    Solution sol = (rand(0, general_setting.GEN_PERM) == 0 ?  init_random_solution() : init_with_permutation());
    Population.emplace_back(Chromosome(sol));

    /// @brief logging process
    if (sol.valid_solution()) best = max(best, sol.evaluate());
    sol.print_out();
    valid += sol.valid_solution();
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
    sort_population();
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
    while (cnt < off_springs_size) {
      int i = rand(0, Population.size() - 1);
      int j = rand(0, Population.size() - 1);
      if (i == j) {
        continue;
      }
      if (rand(0, general_setting.CROSSOVER2) == 0) {
        cnt++;
        offsprings.emplace_back(crossover(Population[i], Population[j]));

        if (cnt < off_springs_size) {
          ++cnt;
          offsprings.emplace_back(crossover(Population[j], Population[i]));
        }
      } else {
        auto next = crossover2(Population[i], Population[j]);
        for (auto it : next.first.chr) debug(it);
        cnt++;
        offsprings.emplace_back(next.first);
        if (cnt < off_springs_size) {
          ++cnt;
          offsprings.emplace_back(next.second);
        }
      }
    }
  };
  const auto educate = [&]() {
    /// @brief: educate Population
    for (auto& gen : Population) {
      auto sol = gen.encode();
      sol.educate3();
      sol.educate_with_lowerbound();
      // sol.educate2();
      // sol.educate();
      gen = Chromosome(sol);
    }
  };
  const auto choose_next_population = [&]() {
    /// @brief sort Population
    sort_population();

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
  log_debug << "start ga_process\n";
  for (int iter = 0; iter <= general_setting.NUM_GENERATION; ++iter) {
    educate();
    init();
 
    evaluate(iter);
    create_offspring();
    choose_next_population();

    sort_population();
    mutation1();
    //mutation2();

    log_debug << "eval of generation " << iter + 1 << '\n';
    for (auto gene : Population) log_debug << gene.encode().evaluate() << ' ';
    log_debug << '\n';

    /*logging best solution*/
    log_debug << "best solution of generation " << iter + 1 << "\n";
    log_debug << "gene is\n";
    log_debug << "[";
    for (auto it : Population[0].chr) {
      log_debug << "[" << it.first << ", " << it.second << "], ";
    }
    log_debug << "]\n";

    Population[0].encode().print_out();

    best_in_generation.push_back(Population[0].encode().evaluate());
    worst_in_generation.push_back(Population.back().encode().evaluate());
    int64_t total_results = 0;
    int num_feasible = 0;
    for (auto gene : Population) {
      total_results += gene.encode().evaluate();
      num_feasible += gene.encode().valid_solution();
    }
    average_in_generation.push_back(1.0 * (double)total_results /
                                    (int)Population.size());
    num_infeasible_solution.push_back(num_feasible);
  }
  debug(best.evaluate());
  // assert(best.valid_solution());
  // log_result << "complete running\n";
  // log_result << "convergence after " << best_generation << '\n';
  // log_result << "Solution is " << best.evaluate() << '\n';
  std::cout << best.evaluate() << ';';
  int max_drone_trip = 0;
  for (int i = 0; i < num_drone; ++i) {
    max_drone_trip = max(max_drone_trip, (int)best.drone_trip[i].multiRoute.size());
  }
  std::cout << max_drone_trip << ";";
  // log_debug << "best solution overall is\n" << "with objective function " << best.evaluate() << '\n';
  best.print_out();
  // assert(best.valid_solution());
}

void logging_to_csv() {
  /// @brief convert all generation result to string format
  string best, worst, average, infeasible;
  const auto vector_to_string = [&](const auto& v) {
    string ans = "[";
    for (const auto& it : v) {
      ans += to_string((int)it);
      ans += ',';
    }
    ans += ']';
    return ans;
  };
  best = vector_to_string(best_in_generation);
  worst = vector_to_string(worst_in_generation);
  average = vector_to_string(average_in_generation);
  infeasible = vector_to_string(num_infeasible_solution);

  std::cout << best << '\n';

  // log_csv_result << best << worst << average << infeasible;

  // log_csv_result << Solution::evaluate_call;
  // log_csv_result << Solution::educate_call;
}

namespace testing {
  void test_mcmf() {
    Solution sol;
    Chromosome chr;
    
    chr.chr.emplace_back(4, 675);
    chr.chr.emplace_back(3, 150);
    chr.chr.emplace_back(6, 350);
    chr.chr.emplace_back(5, 275);
    chr.chr.emplace_back(1, 50);

    for (int i = 1; i <= 6; ++i) chr.chr.emplace_back(6, 40);
    for (int i = 1; i <= 2; ++i) chr.chr.emplace_back(2, 40);

    sol = chr.encode();
    sol.print_out();
    sol.educate_with_lowerbound();
    sol.print_out();
  }
  void test_encode() {
    Chromosome chr;
    chr.encode().print_out();
  }
}  // namespace testing

int main(int argc, char*argv[]) {
  /// assign 
  general_setting.GEN_PERM = stoi(argv[1]);
  general_setting.CROSSOVER2 = stoi(argv[2]);
  general_setting.MUTATION_OP = stoi(argv[3]);
  
  read_input();
  random_init_population();

  ga_process();
  logging_to_csv();

  // testing::test_mcmf();

  cerr << "\nTime elapsed: " << 1000 * clock() / CLOCKS_PER_SEC << "ms\n";
}
