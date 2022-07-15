#include "string"

std::string to_string(std::string s) {
  return '"' + s + '"';
}
 
std::string to_string(const char* s) {
  return to_string((std::string) s);
}
 
std::string to_string(bool b) {
  return (b ? "true" : "false");
}
 
template <typename A, typename B>
std::string to_string(std::pair<A, B> p) {
  return "(" + to_string(p.first) + ", " + to_string(p.second) + ")";
}
 
template <typename A>
std::string to_string(A v) {
  bool first = true;
  std::string res = "{";
  for (const auto &x : v) {
    if (!first) {
      res += ", ";
    }
    first = false;
    res += to_string(x);
  }
  res += "}";
  return res;
}
 
void debug_out() { std::cerr << std::endl; }
 
template <typename Head, typename... Tail>
void debug_out(Head H, Tail... T) {
  std::cerr << " " << to_string(H);
  debug_out(T...);
}
 
#ifdef LOCAL
#define debug(...) std::cerr << "[" << #__VA_ARGS__ << "]:", debug_out(__VA_ARGS__)
#else
#define debug(...) 42
#endif