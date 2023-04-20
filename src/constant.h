#pragma once

#include "vector"
#include "random"
#include "chrono"
#include "math.h"

class Customer {
 public:
  Customer() {
    lower_weight = upper_weight = cost = 0;
    x = y = 0;
  }
  int lower_weight, upper_weight, cost;
  double x = 0, y = 0;
};

double euclidDistance(const Customer &A, const Customer &B) {
  return sqrt((A.x - B.x) * (A.x - B.x) + (A.y - B.y) * (A.y - B.y));
}

std::mt19937 rng(64);

int num_customer, num_truck, num_drone;
double speed_truck, speed_drone;
std::vector<Customer> customers;
std::vector<int> time_limit_truck, truck_capacity;

const int TTRUCK = 0, TDRONE = 1;


int TABU_ITERATOR = 0;
int TABU_CYCLE = 0;
int LOCAL_SEARCH_OP = 0;


/*helper func*/
Customer customerInfo(int customer_id) {
  return customers[customer_id];
}
double customerDistance(int i, int j) {
    return euclidDistance(customerInfo(i), customerInfo(j));
}

/*
for checking and logging process
*/
int valid_solution_called = 0;