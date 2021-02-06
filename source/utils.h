#pragma once
#include <vector>
#include <limits>
#include <random>
#define PI 3.14159265359

const double infinity = std::numeric_limits<double>::infinity();

inline double deg2rad(double d)
{
  return d * PI / 180;
}

inline double rand_double() {
    // Returns a random real in [0,1).
         return rand() / (RAND_MAX + 1.0);
         }

inline double rand_double(double min, double max)
{
  return min + (max-min)*rand_double();
}

inline double clamp(double x, double min, double max) {
    if (x < min) return min;
    if (x > max) return max;
    return x;
}
