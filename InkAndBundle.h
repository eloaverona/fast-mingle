/*
 * Contains the ink savings of a calculation and the bundle that gives the ink
 * savings for two edges.
 */

#ifndef INKANDBUNDLE_H_
#define INKANDBUNDLE_H_

struct InkAndBundle {
  double ink;
  Edge *bundle;
};

#endif /* INKANDBUNDLE_H_ */
