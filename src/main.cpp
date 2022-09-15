#include "bits/stdc++.h"
#include "settings.hpp"
#include "debug.hpp"

std::vector<std::vector<double>> piority;

void read_input() {
  std::ifstream read_file("input.txt");
  /*
  tip for input reading

  depot is customers[0] = {all value equal to 0}
  */
  read_file >> numTruck >> numDrone >> timeLimit;
  read_file >> speedTruck >> speedDrone >> capacityTruck >> capacityDrone >> durationDrone;
  Customer tmp;
  numCustomer = 1;
  customers.emplace_back(Customer());
  while (
    read_file >> tmp.x >> tmp.y >> tmp.lower_weight >> tmp.upper_weight >> tmp.cost
  ) {
    customers.emplace_back(tmp);
  }
  read_file.close();
  numCustomer = customers.size();

  debug(numTruck, numDrone, timeLimit);
  debug(speedTruck, speedDrone, capacityTruck, capacityDrone, durationDrone); 
  for (auto x : customers) debug(x.x, x.y, x.lower_weight, x.upper_weight, x.cost);

  /*
  assert limit of constraint
  */
  assert(speedTruck != 0); assert(speedDrone != 0);
}

void init_piority_matrix() {
  piority.assign(numCustomer, std::vector<double>(numCustomer, 0));
  for (int i = 0; i < numCustomer; ++i) {
    double totDistance = 0;
    for (int j = 0; j < numCustomer; ++j) {
      totDistance += euclid_distance(customers[i], customers[j]);
    }
    debug(totDistance);
    for (int j = 0; j < numCustomer; ++j) {
      piority[i][j] = 1.0 * euclid_distance(customers[i], customers[j]) * (double)customers[j].lower_weight
                        * customers[j].cost / totDistance;
    }
  }
  debug(piority);
}

Solution init_random_solution() {
  Solution first_solution;
  first_solution.drone_trip.resize(numDrone); first_solution.truck_trip.resize(numDrone);

  /*
  set vehicle type for every trip
  try to dynamic calculate remaining time
  */
  for (int i = 0; i < numDrone; ++i) {
    first_solution.drone_trip[i].set_vehicle_type(TDRONE);
  }

  auto partial_sum = build_partial_sum(piority[0]);

  /*
  3.1. Create first customer for every trip
  */

  for (int i = 0; i < numDrone; ++i) {
    int tmp = random_number_with_probability(partial_sum);
    debug(tmp); first_solution.drone_trip[i].new_route();
    first_solution.drone_trip[i].append({0, tmp}, 0); 
  }
  for (int i = 0; i < numTruck; ++i) {
    int tmp = random_number_with_probability(partial_sum);
    first_solution.truck_trip[i].new_route();
    first_solution.truck_trip[i].append({0, tmp}, 0); 
  }

  /*
  3.2. Init first route for every vehicle
  */
  std::vector<int> current_lowerbound(numCustomer);
  for (int i = 0; i < numCustomer; ++i) 
    current_lowerbound[i] = customers[i].lower_weight;
  
  for (int i = 0; i < numTruck; ++i) {
    double tot_time = 0;
    int now_weight = capacityTruck;
    for (int j = 1; j < (int)first_solution.truck_trip[i].multiRoute[0].size(); ++j) {
      auto current_loc = first_solution.truck_trip[i].multiRoute[0].route[j];
      debug("truck", current_loc->customer_id);
      if (current_loc->prev_node != nullptr)
        tot_time += time_travel(customers[current_loc->customer_id], 
                                  customers[current_loc->prev_node->customer_id], TTRUCK);
      int push_weight = min(current_lowerbound[current_loc->customer_id], now_weight);
      first_solution.truck_trip[i].multiRoute[0].route.back()->weight = push_weight;
      now_weight -= push_weight;
      current_lowerbound[current_loc->customer_id] -= push_weight;
      debug(i, push_weight);
      if (push_weight == 0) {
        first_solution.truck_trip[i].multiRoute[0].pop(); 
        break;
      }
      if (now_weight == 0) 
        break;
      double min_dist = std::numeric_limits<double>::max();
      int next_customer = -1;
      for (int i = 1; i < numCustomer; ++i) {
        /*
        check condition for time_limit and availablity for customer i
        */
        if (tot_time + time_travel(customers[current_loc->customer_id], customers[i], TTRUCK) > timeLimit)
          continue;
        if (current_lowerbound[i] == 0) 
          continue;
        if (minimize<double>(min_dist, euclid_distance(customers[current_loc->customer_id], customers[i]))) {
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
      /*
      split new route on drone
      */
      for (int j = 1; j < (int)first_solution.drone_trip[i].multiRoute[0].size(); ++j) {
        auto current_loc = first_solution.drone_trip[i].multiRoute[0].route[j];
        debug("drone", current_loc->customer_id);
        if (current_loc->prev_node != nullptr)
          tot_time += euclid_distance(customers[current_loc->customer_id], 
                                    customers[current_loc->prev_node->customer_id]);
        
        int push_weight = min(current_lowerbound[current_loc->customer_id], now_weight);
        first_solution.drone_trip[i].multiRoute[route_id].route.back()->weight = push_weight;
        if (push_weight == 0) {
          first_solution.drone_trip[i].route[0].pop_back(); 
          break;
        }
        double min_dist = std::numeric_limits<double>::max();
        int next_customer = -1;
        for (int i = 1; i < numCustomer; ++i) {
          /*
          check condition for time_limit and availablity for customer i
          */
          if (tot_time + time_travel(customers[current_loc->customer_id], customers[i], TDRONE) > durationDrone)
            continue;
          if (tot_time + time_travel(customers[current_loc->customer_id], customers[i], TDRONE) > timeLimit)
            continue;
          if (current_lowerbound[i] == 0) 
            continue;
          if (minimize<double>(min_dist, euclid_distance(customers[current_loc->customer_id], customers[i]))) {
            next_customer = i;
          }
        }
        debug(next_customer);
        if (next_customer != -1) {
          first_solution.drone_trip[i].append({0, next_customer}, route_id);
        }
      }
      ++route_id;
    }
 }
  // log first_solution
  log_debug << "first_solution\n";
  for (int i = 0; i < numTruck; ++i) {
    log_debug << "truck" << ' ' << i << '\n';
    for (auto loc : first_solution.truck_trip[i].route[0]) {
      log_debug << loc->customer_id << ' ' << loc->weight << '\n';
    }
  }

  for (int i = 0; i < numDrone; ++i) {
    log_debug << "drone " << i << '\n';
    for (auto loc : first_solution.drone_trip[i].route[0]) {
      log_debug << loc->customer_id << ' ' << loc->weight << '\n';
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
  for (int iter = 0; iter < 1; ++iter) {
    Population.emplace_back(init_random_solution());
  } 
}

void evaluate() {

}

void mutation() {

}

void educate() {

}

void ga_process() {
  for (int iter = 0; iter <= 10000; ++iter) {
    evaluate();
    mutation();
    educate();
  }
}

int main() {
  read_input();
  init_piority_matrix();
  random_init_population();
  ga_process();
}