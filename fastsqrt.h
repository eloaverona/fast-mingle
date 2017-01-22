/*
 * fastsqrt.h
 *
 *  Created on: Jan 21, 2017
 *      Author: alan
 */

#ifndef FASTSQRT_H_
#define FASTSQRT_H_

/**
* Fast square root calculation taken from https://www.codeproject.com/Articles/69941/Best-Square-Root-Method-Algorithm-Function-Precisi
*/
  static double Abs(double Nbr)
 {
  if( Nbr >= 0 )
   return Nbr;
  else
   return -Nbr;
 }

 static double fastsqrt(double Nbr)
 {
  double Number = Nbr / 2;
  const double Tolerance = 1.0e-7;
  do
  {
   Number = (Number + Nbr / Number) / 2;
  }while( Abs(Number * Number - Nbr) > Tolerance);

  return Number;
 }




#endif /* FASTSQRT_H_ */
