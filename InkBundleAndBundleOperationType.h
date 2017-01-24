/*
 * Contains the ink savings of a calculation and the bundle that gives the ink
 * savings for two edges. It also contains the operation type used to arrive to
 * this ink savings.
 */

#ifndef INKBUNDLEANDBUNDLEOPERATIONTYPE_H_
#define INKBUNDLEANDBUNDLEOPERATIONTYPE_H_

#include "BundleOperation.h"

struct InkBundleAndBundleOperationType {
  double ink;
  Edge bundle;
  BundleOperationType bundleOperationType;
};

#endif /* INKBUNDLEANDBUNDLEOPERATIONTYPE_H_ */
