#ifndef CALCULATE_DATA_HPP
#define CALCULATE_DATA_HPP

#include <math.h>
#include <numeric>
#include <vector>

#include <fstream>
#include <iostream>
#include <ostream>
#include <string>

namespace calculate
{

  class Calculatedata
  {
      public:
    Calculatedata();
    Calculatedata(std::vector<double> aMeasurements);
    /**
     * @brief Load the robot controller plugin, overrides the Load from
     * ModelPlugin
     * @param aParent: parent model
     * @param aSdf: the sdf (xml) in the robot model, within the <plugin>
     * element
     */
    virtual ~Calculatedata() = default;

    void fillVector(std::string aFile);

    void calculateStepSize();
    void calculateAverage();
    void calculateDeviation();
    void processData();

    std::vector<double> mMeasurements;
    std::vector<double> mStepSize;

    double mMean;
    double mDeviation;
    int mDefectiveMeasurement;
  };

} // namespace calculate
#endif