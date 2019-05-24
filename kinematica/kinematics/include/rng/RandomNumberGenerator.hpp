#ifndef RNG_RANDOMNUMBERGENERATOR_HPP
#define RNG_RANDOMNUMBERGENERATOR_HPP

#include <cstdint>
#include <random>

namespace rng
{
  class RandomNumberGenerator
  {
      public:
    /**
     * @brief Generates a random real number between min and max
     *
     * Uses mSeed for random number generation
     *
     * @param aMin
     * @param aMax
     * @return double
     */
    inline static double GenerateInRange(double aMin, double aMax)
    {
      static std::mt19937 lGenerator(mSeed);
      std::uniform_real_distribution<> lDistribution(aMin, aMax);
      return lDistribution(lGenerator);
    }
    static const uint64_t mSeed = 8099959656190;
  };
} // namespace rng

#endif // RNG_RANDOMNUMBERGENERATOR_HPP
