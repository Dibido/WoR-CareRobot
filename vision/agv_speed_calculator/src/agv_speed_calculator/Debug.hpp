#ifndef _DEBUG_HPP_
#define _DEBUG_HPP_

// #define USE_DEBUGLN

#ifdef USE_DEBUGLN
#define DEBUG(x) Serial.print(x)
#define DEBUGLN(x) Serial.println(x)
#define DEBUGLNFLOAT(x, y) Serial.println(x, y)
#else
#define DEBUG(x)
#define DEBUGLN(x)
#define DEBUGLNFLOAT(x, y)
#endif

#endif