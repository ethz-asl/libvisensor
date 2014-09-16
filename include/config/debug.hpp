/*
 * debug.hpp
 *
 *  Created on: Jan 2, 2014
 *      Author: skybotix
 */

#ifndef DEBUG_HPP_
#define DEBUG_HPP_

#include <stdio.h>

#ifdef NDEBUG
  #define DEBUG_TEST 0
#else
  #define DEBUG_TEST 1
#endif


/***************************************************/
/* DEBUG TERMINAL PRINT                            */
/***************************************************/

struct Sink { template<typename ...Args> Sink(Args const& ... ) {} };
#if DEBUG_TEST
  #define VISENSOR_DEBUG(...) \
      do { printf("[libvisensor]: "); printf(__VA_ARGS__); } while (0)
#else
  #define VISENSOR_DEBUG(...) Sink { __VA_ARGS__ }
#endif




#endif /* DEBUG_HPP_ */
