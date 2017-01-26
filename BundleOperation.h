/*
 * Contains a bundle operation that describes a bundle operation to put an edge and it's
 * best neighbor together on the same bundle. It has an operation type member variable
 * that describes how the two should be combined.
 */

#ifndef BUNDLEOPERATION_H_
#define BUNDLEOPERATION_H_

enum BundleOperationType {
	NEW_BUNDLE, MERGE_BUNDLES, ADD_NEIGHBOR_TO_PARENT_OF_EDGE, ADD_EDGE_TO_PARENT_OF_NEIGHBOR
};

struct BundleOperation {
	BundleOperationType operationType;
	Edge edge;
	Edge *neighborPointer;
	Edge *bundle;
	double inkSaved;
};

#endif /* BUNDLEOPERATION_H_ */
