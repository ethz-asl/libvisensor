/*
 * assert.hpp
 *
 *  Created on: Feb 19, 2014
 *      Author: skybotix
 */

#ifndef ASSERT_HPP_
#define ASSERT_HPP_

/***************************************************/
/* ASSERT MACROS                                   */
/***************************************************/
//asserts if cond is true
#define VISENSOR_ASSERT_COND(cond, ...) \
  do { \
    if ((cond)) { \
      printf("ASSERTION FAILED\n\tfile = %s\n\tline = %d\n\tcond = %s\n", __FILE__, __LINE__, #cond); \
      do { printf("\tmsg:   "); printf(__VA_ARGS__); printf("\n");} while (0); \
      exit(1); \
    } \
  } while (0)


#define VISENSOR_ASSERT(...) \
  do { \
      printf("ASSERTION FAILED\n\tfile = %s\n\tline = %d\n", __FILE__, __LINE__); \
      do { printf("\tmsg:   "); printf(__VA_ARGS__); printf("\n");} while (0); \
      exit(1); \
  } while (0)



#endif /* ASSERT_HPP_ */
