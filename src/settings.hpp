#include "vector"
#include "random"
#include "numeric"
#include "climits"
#include "iostream"

int numCustomer, numTruck, numDrone, timeLimit;

template <class T>
std::vector<T> create(size_t size, T initialValue) {
  return std::vector<T>(size, initialValue);
}

template <class T, class... Args>
auto create(size_t head, Args&&... tail) {
  auto inner = create<T>(tail...);
  return std::vector<decltype(inner)>(head, inner);
}

class Customer {
public:
  Customer() {lower_weight = upper_weight = cost = 0;
              x = y = 0;}
  int lower_weight, upper_weight, cost;
  double x = 0, y = 0;
};

double euclid_distance(const Customer&A, const Customer&B) {
  return (A.x - B.x) * (A.x - B.x) + (A.y - B.y) * (A.y - B.y);
}

std::vector<Customer> customers;

class Route {
  class Node {
  public:
    int weight, customer_id;
    Node *prev_node = nullptr, *next_node = nullptr;
    Node() {weight = 0; customer_id = -1; prev_node = nullptr, next_node = nullptr;}
  };
public:
  /*
  0 is truck
  1 is drone
  */
  int vehicle_type;
  /*
  id of truck/drone that manage that trip
  */
  int owner;
  std::vector<std::vector<Node*>> route;
  /* info = {weight, customer_id};
  */
  void new_route() {
    std::vector<Node*> tmp;
    tmp.push_back(new Node());
    tmp.back()->customer_id = 0;
    route.emplace_back(tmp);
  }
  void append(std::pair<int, int> info, int trip_id = 0) {
    assert(trip_id < route.size());
    Node *tmp = new Node();

    tmp->customer_id = info.second;
    tmp->prev_node = route[trip_id].back();
    route[trip_id].back()->next_node = tmp;
    route[trip_id].push_back(tmp); 
  }
};

class Solution {
public:
  std::vector<Route> truck_trip, drone_trip;
  Solution() {
    truck_trip.clear();
    drone_trip.clear();
  }
};

std::vector<Solution> Population;


class Vehicle {
public:
  int speed, weight_limit;
};
std::vector<Vehicle> drones, trucks;

const int POPULATION_SIZE = 50;

std::mt19937 rng(64);
/*
return a real value in defined range 
*/
double random_number_in_range(double l, double r) {
  std::uniform_real_distribution<double> unif(l, r);
  std::random_device rand_dev; 
  std::mt19937 rand_engine(rand_dev()); 
  double x = unif(rand_engine);
  return x;
}

/*

*/

std::vector<double> build_partial_sum(const std::vector<double>&prob) {
  std::vector<double> partial(prob.size(), 0);
  for (int i = 0; i < (int)prob.size(); ++i) {
    partial[i] = (i == 0 ? 0 : partial[i - 1]) + prob[i];
  }
  return partial;
}

/*
A random number generator that sastisfy
P(i) = prob(i) / (sigma(prob))
*/

int random_number_with_probability(const std::vector<double>&partial) {
  double dice = random_number_in_range(0, partial.back());
  return std::lower_bound(partial.begin(), partial.end(), dice) - partial.begin();
}