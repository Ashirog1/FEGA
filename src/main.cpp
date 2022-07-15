#include "bits/stdc++.h"
#include "settings.hpp"

std::vector<Customer> customers;
/*
read number of truck, number of drone, customer information from local file
*/
void read_input() {
  std::ifstream read_file("input.txt");
  read_file >> numCustomer >> numTruck >> numDrone;

}

int main() {
  read_input();
}