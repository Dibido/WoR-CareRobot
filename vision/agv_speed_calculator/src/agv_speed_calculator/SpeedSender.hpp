#ifndef _SPEEDSENDER_HPP_
#define _SPEEDSENDER_HPP_

/**
 * @brief Start the radio
 *
 */
void initialiseRadio();

/**
 * @brief Sends the estimated speed using the radio
 *
 * @param aEstimatedSpeed
 */
void sendEstimatedSpeed(double aEstimatedSpeed);

#endif