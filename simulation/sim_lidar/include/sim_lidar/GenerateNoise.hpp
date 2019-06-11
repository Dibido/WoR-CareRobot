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
    GenerateNoise(double aMean, double aDevation);
    //
    /**
     * @brief Load the robot controller plugin, overrides the Load from
     * ModelPlugin
     * @param aParent: parent model
     * @param aSdf: the sdf (xml) in the robot model, within the <plugin>
     * element
     */
    virtual ~GenerateNoise() = default;

    void GenerateNoiseSample(double aMean, double aDeviation);

    double mMean;
    double mDeviation;
    double mStep;
    std::random_device mrd;
    std::vector<double> mNoise;
  };

} // namespace generate
#endif // GENERATE_NOISE_HPP