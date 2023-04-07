#pragma once

#include "customer.h"
#include "constant.h"
#include "math.h"

double euclid_distance(const Customer &A, const Customer &B) {
  return sqrt((A.x - B.x) * (A.x - B.x) + (A.y - B.y) * (A.y - B.y));
}

double time_travel(const Customer &A, const Customer &B, int type) {
  return (1.0 * euclid_distance(A, B)) /
         (type == TTRUCK ? (double)speed_truck : (double)speed_drone);
}