#include "customer.h"
#include "constant.h"
#include "math.h"

double euclidDistance(const Customer &A, const Customer &B) {
  return sqrt((A.x - B.x) * (A.x - B.x) + (A.y - B.y) * (A.y - B.y));
}
