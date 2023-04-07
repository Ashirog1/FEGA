#pragma once
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

double euclid_distance(const Customer &A, const Customer &B);
double time_travel(const Customer &A, const Customer &B, int type);
