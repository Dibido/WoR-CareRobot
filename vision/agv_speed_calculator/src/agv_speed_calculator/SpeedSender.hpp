#ifndef _SPEEDSENDER_HPP_
#define _SPEEDSENDER_HPP_

void initialiseRadio();

/*
 * Sends the estimated value over the NRF to the AGV gateway.
 */
void sendEstimatedSpeed(double aEstimatedSpeed);

#endif