/*
 * Point.h
 *
 *  Created on: Oct 20, 2016
 *      Author: alan
 */

#ifndef POINT_H_
#define POINT_H_

struct Point {
  double x, y;

  Point operator-(Point& p);
  Point operator*(int k);
  double norm();
};

#endif /* POINT_H_ */
