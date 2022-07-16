#include "bits/stdc++.h"
#include "settings.hpp"
#include "string"
#include "debug.hpp"

std::vector<std::vector<double>> piority;

void read_input() {
  std::ifstream read_file("input.txt");
  read_file >> numCustomer >> numTruck >> numDrone >> timeLimit;
  for (int i = 0; i < numCustomer; ++i) {
    Customer tmp;
    read_file >> tmp.x >> tmp.y >> tmp.lower_weight >> tmp.upper_weight >> tmp.cost;
    customers.emplace_back(tmp);
  }
  for (int i = 0; i < numTruck; ++i) {
    Vehicle tmp;
    read_file >> tmp.weight_limit;
    trucks.emplace_back(tmp);
  }
  for (int i = 0; i < numDrone; ++i) {
    Vehicle tmp;
    read_file >> tmp.weight_limit;
    drones.emplace_back(tmp);
  }
  for (auto x : customers) debug(x.x, x.y);
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
      piority[i][j] = 1.0 * euclid_distance(customers[i], customers[j]) * (double)customers[i].lower_weight
                        * customers[i].cost / totDistance;
    }
  }
  debug(piority);
}

void random_init_solution() {
  Solution first_solution;
  first_solution.drone_trip.resize(numDrone); first_solution.truck_trip.resize(numDrone);

  auto partial_sum = build_partial_sum(piority[0]);

  for (int i = 0; i < numDrone; ++i) {
    int tmp = random_number_with_probability(partial_sum);
    first_solution.drone_trip[i].new_route();
    first_solution.drone_trip[i].append({0, tmp}, 0); 
  }
  for (int i = 0; i < numTruck; ++i) {
    int tmp = random_number_with_probability(partial_sum);
    first_solution.truck_trip[i].new_route();
    first_solution.truck_trip[i].append({0, tmp}, 0); 
  }
  std::vector<int> current_lowerbound(numCustomer);
  for (int i = 0; i < numTruck; ++i) {
    double totTime = 0, now_weight = trucks[i].weight_limit;
    for (int j = 0; j < (int)first_solution.truck_trip[i].route[0].size(); ++j) {
      auto current_loc = first_solution.truck_trip[i].route[0][i];
      totTime += euclid_distance(customers[current_loc->customer_id], 
                                customers[current_loc->prev_node->customer_id]);
      int push_weight = min(current_lowerbound[current_loc->customer_id], now_weight);
      first_solution.truck_trip[i].route[0].back()->weight = push_weight;

      for (int i = 0; i < numCustomer; ++i) {
        
      }
    }
    
  }
  return first_solution;
}

int main() {
  read_input();
  init_piority_matrix();
  random_init_solution();
}