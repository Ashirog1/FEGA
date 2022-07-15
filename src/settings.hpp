class Customer {
public:
  int lower_weight, upper_weight, piority;
};

class Route {
private:
  class Node {
    int weight, customer_id;
    Node *prev_node = nullptr, *next_node = nullptr;
    Node() {weight = 0; customer_id = -1; prev_node = nullptr, next_node = nullptr;}
  };
public:
  std::vector<Node*> trips;
};

class Truck {
  int speed, weight_limit;
};

class Drone {
  int speed, weight_limit;
};

class {

};

int numCustomer, numTruck, numDrone;