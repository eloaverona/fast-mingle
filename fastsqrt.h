/*
 * fastsqrt.h
 *
 *  Created on: Jan 21, 2017
 *      Author: alan
 */

#ifndef FASTSQRT_H_
#define FASTSQRT_H_

#include <math.h>

/**
* Fast square root calculation adapted from
* http://stackoverflow.com/questions/19611198/finding-square-root-without-using-sqrt-function
*/
static double fastsqrt(double x) {
  int MAX_ITER = 2;
  if (x <= 0)
    return 0; // if negative number throw an exception?
  int exp = 0;
  x = frexp(x, &exp); // extract binary exponent from x
  if (exp & 1) {      // we want exponent to be even
    exp--;
    x *= 2;
  }
  double y = (1 + x) / 2; // first approximation
  double z = 0;
  int i;
  while (y != z && i < MAX_ITER) { // yes, we CAN compare doubles here!
    z = y;
    y = (y + x / y) / 2;
    i++;
  }
  return ldexp(y, exp / 2); // multiply answer by 2^(exp/2)
}

#endif /* FASTSQRT_H_ */
