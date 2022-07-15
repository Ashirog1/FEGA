#include "bits/stdc++.h"
#include "settings.hpp"

std::vector<std::vector<double>> piority;

void read_input() {
  std::ifstream read_file("input.txt");
  read_file >> numCustomer >> numTruck >> numDrone;
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
}

void init_piority_matrix() {
  piority.assign(numCustomer + 5, std::vector<double>(numCustomer + 5, 0));
  for (int i = 0; i < numCustomer; ++i) {
    double totDistance = 0;
    for (int j = 0; j < numCustomer; ++j) {
      totDistance += euclid_distance(customers[i], customers[j]);
    }
    for (int j = 0; j < numCustomer; ++j) {
      piority[i][j] = euclid_distance(customers[i], customers[j]) * customers[i].lower_weight
                        * customers[i].cost / totDistance;
    }
  }
}

void random_init_solution() {
  Solu 
}

int main() {
  read_input();
  init_piority_matrix();
  random_init_solution();
}