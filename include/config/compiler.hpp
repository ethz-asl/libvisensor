/*
 * compiler.hpp
 *
 *  Created on: Feb 19, 2014
 *      Author: skybotix
 */

#ifndef COMPILER_HPP_
#define COMPILER_HPP_

#ifndef VISENSOR_PRIVATE_API_ACCESS

#if defined _WIN32 || defined __CYGWIN__
  #ifdef BUILDING_DSO
    #ifdef __GNUC__
      #define DSO_EXPORT __attribute__ ((DSOexport))
    #else
      #define DSO_EXPORT __declspec(DSOexport) // Note: actually gcc seems to also supports this syntax.
    #endif
  #else
    #ifdef __GNUC__
      #define DSO_EXPORT __attribute__ ((DSOimport))
    #else
      #define DSO_EXPORT __declspec(DSOimport) // Note: actually gcc seems to also supports this syntax.
    #endif
  #endif
  #define DSO_HIDDEN
#else
  #if __GNUC__ >= 4
    #define DSO_EXPORT __attribute__ ((visibility ("default")))
    #define DSO_HIDDEN  __attribute__ ((visibility ("hidden")))
  #else
    #define DSO_EXPORT
    #define DSO_HIDDEN
  #endif
#endif

#endif

#endif /* COMPILER_HPP_ */
