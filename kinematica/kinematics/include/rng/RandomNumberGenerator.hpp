#ifndef RNG_RANDOMNUMBERGENERATOR_HPP
#define RNG_RANDOMNUMBERGENERATOR_HPP

#include <random>

namespace rng
{

  class RandomNumberGenerator
  {
      public:
    // template <typename T>
    inline static double GenerateInRange(double min, double max)
    {
      static std::random_device rd;
      static std::mt19937 gen(rd());
      std::uniform_real_distribution<> distribution(min, max);
      return distribution(gen);
    }
  };
} // namespace rng

#endif // RNG_RANDOMNUMBERGENERATOR_HPP
