#ifndef FLOCKEVO_RANDOM_HPP_INCLUDED
#define FLOCKEVO_RANDOM_HPP_INCLUDED

#include <random>

typedef std::mt19937 rnd_eng_type;

// Seeding the random number engine(s)
// Call this once before any call to rnd_eng()
void rnd_seed(unsigned long);

// Returns thread local random number engine
rnd_eng_type& rnd_eng();

inline float randomFloat(float minValue, float maxValue)
{
  std::uniform_real_distribution<float> d(minValue, maxValue);
  return d(rnd_eng());
}

inline int randomInt(int minValue, int maxValue)
{
  std::uniform_int_distribution<int> d(minValue, maxValue);
  return d(rnd_eng());
}

#endif