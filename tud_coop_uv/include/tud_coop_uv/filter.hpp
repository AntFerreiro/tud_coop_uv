#ifndef FILTER_HPP
#define FILTER_HPP
#include <vector>
class Filter {
 public:
  Filter();
  double filter(double new_value);

 private:
  double m_size_filter;
  std::vector<double> m_coeffs;
  std::vector<double> m_input_buffer;
};

#endif  // FILTER_HPP
