#include "vector"
#include "random"
#include "numeric"
#include "climits"
#include "iostream"
#include "list"

/*
TODO: try to split to different module.
For example:
route.hpp
solution.hpp
customer.hpp
vehicle.hpp

chromesome
*/

const int TTRUCK = 0, TDRONE = 1;

int numCustomer, numTruck, numDrone, timeLimit;
int speedTruck, speedDrone, capacityTruck, capacityDrone, durationDrone;
std::ofstream log_debug("debug.log");

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

double time_travel(const Customer&A, const Customer&B, int type) {
  return sqrt(euclid_distance(A, B)) /  (type == TTRUCK ? speedTruck : speedDrone);
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
  Route() {
    total_time = 0;
    total_weight = 0;
  }
  /*
  0 is truck
  1 is drone
  */
  int vehicle_type;
  /*
  id of truck/drone that manage that trip

  not neccessery?
  */
  int owner;
  std::vector<std::vector<Node*>> route;
  double total_time = 0;
  int total_weight = 0;
  void new_route() {
    std::vector<Node*> tmp;
    tmp.push_back(new Node());
    tmp.back()->customer_id = 0;
    route.emplace_back(tmp);
  }
  /* info = {weight, customer_id};
  */
  void append(std::pair<int, int> info, int trip_id = 0) {
    assert(trip_id < route.size());
    total_time -= time_travel(customers[route[trip_id].back()->customer_id], 
                                customers[0], vehicle_type);
    total_time += time_travel(customers[route[trip_id].back()->customer_id],
                                customers[info.second], vehicle_type);
    total_time += time_travel(customers[info.second], customers[0], vehicle_type);
    Node *tmp = new Node();

    tmp->customer_id = info.second;
    tmp->prev_node = route[trip_id].back();
    route[trip_id].back()->next_node = tmp;
    route[trip_id].push_back(tmp); 
  }
  /*
    set before route building
  */
  void set_vehicle_type(int type) {
    vehicle_type = type;
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

const int POPULATION_SIZE = 50;
/* a constant seed random interger generator
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


template<class T>
bool minimize(T&x, const T& y) {
  if (x > y) {
    x = y;
    return true;
  }
  return false;
}

/*
for genetic part
use std::list for O(1) mutation operation
*/

class Chromosome {
  public:
  
  /*
  {customer id, weight}
  kind of conflict? right
  */
  std::list<std::pair<int, int>> chr;
  /*
  decode to a genetic
  */
  /*
  truck trip
  */
  Chromosome(Solution sol) {
    chr.clear();

    /*
      for each vehicle
        for each route
          for each customer on route
    */
    /*
    TODO:
    format to more readable code
    */
    for (int r = 0; r < (int)sol.truck_trip.size(); ++r) {
      for (int j = 0; j < (int)sol.truck_trip[r].route.size(); ++j) {
        for (auto cus : sol.truck_trip[r].route[j]) {
          chr.emplace_back(cus->customer_id, cus->weight);
        }  
      }
    }
    for (int r = 0; r < (int)sol.drone_trip.size(); ++r) {
      for (int j = 0; j < (int)sol.drone_trip[r].route.size(); ++j) {
        for (auto cus : sol.drone_trip[r].route[j]) {
          chr.emplace_back(cus->customer_id, cus->weight);
        }  
      }
    }
  }

  /*
  encode to a solution use greedy algo
  try to push customer on current route until the condition hold true

  */
  Solution encode() {
    Solution sol;
    int vehicle_idx = 0;
    for (auto [customer_id, weight] : chr) {
      /*
      try to push
      */
      auto tmp = (vehicle_idx >= numTruck ? sol.drone_trip[vehicle_idx - numTruck]:
                        sol.truck_trip[vehicle_idx]);
      sol.truck_trip[route.append({weight, customer_id}, 0);
      /*
      condition checking
      */
      if (route.total_time <= timeLimit and route.total_weight <= capacityTruck) {
        continue;
      }

      /*
      rollback
      */
      route = tmp;
    }
  }
}