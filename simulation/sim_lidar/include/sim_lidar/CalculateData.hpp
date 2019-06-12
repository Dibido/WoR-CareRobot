/**
 * @file Calculatedata.hpp
 * @author Stein Zwerink
 * @brief
 * @version 0.1
 * @date 2019-06-22
 *
 * @copyright Copyright (c) 2019
 *
 */

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
    /**
     * @brief calculates a mean and deviation from a provided dataset
     * @param aMeasurements: the aMeasurements who needs te be processed.
     */
    Calculatedata(std::vector<double> aMeasurements);

    virtual ~Calculatedata() = default;
    /**
     * @brief fillVector fills a vector with Measurements taken from a .txt file
     *
     * @param aFile the destination of the .txt file
     */
    void fillVector(std::string aFile);
    /**
     * @brief calculateStepSize calculates the increment between two steps
     *
     */
    void calculateStepSize();

    /**
     * @brief calculates the mean of all the Measurements taken
     *
     */
    void calculateAverage();

    /**
     * @brief calculates the deviation of all the Measurements taken
     *
     */
    void calculateDeviation();

    /**
     * @brief processes all of the Measurements and produces a mean and
     * deviation
     *
     */
    void processData();

    std::vector<double> mMeasurements;
    std::vector<double> mStepSize;

    double mMean;
    double mDeviation;
    int mDefectiveMeasurement;
  };

} // namespace calculate
#endif