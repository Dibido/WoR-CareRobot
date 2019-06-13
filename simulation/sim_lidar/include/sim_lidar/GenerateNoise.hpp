/**
 * @file GenerateNoise.hpp
 * @author Stein Zwerink
 * @brief
 * @version 0.1
 * @date 2019-06-22
 *
 * @copyright Copyright (c) 2019
 *
 */

#ifndef GENERATE_NOISE_HPP
#define GENERATE_NOISE_HPP

#include <math.h>
#include <numeric>
#include <random>
#include <vector>

#include "sim_lidar/CalculateData.hpp"
#include "sim_lidar/LidarConst.hpp"

namespace generate
{
  class GenerateNoise
  {
      public:
    GenerateNoise() = default;
    //
    /**
     * @brief generates noise based on a mean and a devation
     * @param aMean the mean of a dataset
     * @param aDevation the devation of a dataset
     */
    GenerateNoise(double aMean, double aDevation);

    virtual ~GenerateNoise() = default;
    /**
     * @brief generates noise sample based on a mean and a devation
     * @param aMean the mean of a dataset
     * @param aDevation the devation of a dataset
     */
    void generateNoiseSample(double aMean, double aDeviation);

    double mMean;
    double mDeviation;
    double mStep;
    std::random_device mRd;
    std::vector<double> mNoise;
  };

} // namespace generate
#endif // GENERATE_NOISE_HPP