#pragma once

#include "customer.h"
#include "vector"
#include "random"


std::mt19937 rng(std::chrono::steady_clock::now().time_since_epoch().count());

int num_customer, num_truck, num_drone, time_limit;
double speed_truck, speed_drone;
int capacity_truck, capacity_drone, duration_drone;
std::vector<Customer> customers;
std::vector<int> time_limit_truck;

const int TTRUCK = 0, TDRONE = 1;


int TABU_ITERATOR = 0;
int TABU_CYCLE = 0;


/*helper func*/
Customer customerInfo(int customer_id) {
  return customers[customer_id];
}
double customer_distance(int i, int j) {
  return euclid_distance(customerInfo(i), customerInfo(j));
}