/*
 * A structure that contains an Edge object and a BundleOperationType object.
 * The
 * edge variable contains a bundle edge, and type variable contains how this
 * bundled
 * edge was created.
 */

#ifndef EDGEANDBUNDLEOPERATIONTYPE_H_
#define EDGEANDBUNDLEOPERATIONTYPE_H_

#include "BundleOperation.h"

struct EdgeAndBundleOperationType {
  Edge edge;
  BundleOperationType type;
};

#endif /* EDGEANDBUNDLEOPERATIONTYPE_H_ */
