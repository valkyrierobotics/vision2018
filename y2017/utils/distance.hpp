#ifndef DISTANCE_HPP
#define DISTANCE_HPP

#include <math.h>

// Returns the Euclidean distance between two points
template <class T1, class T2>
double distance(T1 a, T2 b)
{
  return std::sqrt((a.x - b.x)*(a.x - b.x) + (a.y - b.y)*(a.y - b.y));
}

#endif // DISTANCE_HPP
