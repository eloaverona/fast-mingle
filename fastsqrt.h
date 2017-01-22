/*
 * fastsqrt.h
 *
 *  Created on: Jan 21, 2017
 *      Author: alan
 */

#ifndef FASTSQRT_H_
#define FASTSQRT_H_

/**
* Fast square root calculation taken from
* https://www.codeproject.com/Articles/69941/Best-Square-Root-Method-Algorithm-Function-Precisi
*/
static double fastsqrt(const double number) {
  const double ACCURACY = 0.001;
  double lower, upper, guess;

  if (number < 1) {
    lower = number;
    upper = 1;
  } else {
    lower = 1;
    upper = number;
  }

  while ((upper - lower) > ACCURACY) {
    guess = (lower + upper) / 2;
    if (guess * guess > number)
      upper = guess;
    else
      lower = guess;
  }
  return (lower + upper) / 2;
}

#endif /* FASTSQRT_H_ */
