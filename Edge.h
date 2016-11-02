/*
 * A directed edge. This is also a node in the four-dimensional graph Γ
 * mentioned in the paper.
 * This edge can have a parent and children. On the final mingled layout, the
 * source point of
 * an edge is connected to the source point of its parent. The target point of
 * an edge is connected
 * to the target point of its parent.
 */

#ifndef EDGE_H_
#define EDGE_H_

#include "Point.h"
#include <vector>

class Edge {
public:
  /**
   * Build an edge given a source point and a target point.
   */
  Edge(Point source, Point target);

  /**
   * Get the source point of the edge.
   */
  Point getSource() { return _source; }

  /**
   * Get the target point of an edge.
   */
  Point getTarget() { return _target; }

  /**
   * Get the ink used by this edge and its children.
   */
  double getInk() { return _ink; }

  /**
   * Get the weight of this edge. The weight of this edge is equal to the number
   * of children it
   * has + 1.
   */
  int getWeight() { return _weight; }

  /**
   * Get the edge's children. In the final mingled layout, each one of its
   * children's source points is connected to this edge's source point.
   * Each one of its children's target points is connected to this edge's
   * target point.
   */
  std::vector<Edge *> getChildren() { return _children; }

  /**
   * Adds a child edge to this edge.
   */
  void addChild(Edge *child);

  /**
   * Gets the parent edge of this edge.
   */
  Edge *getParent() { return _parent; }

  /**
   * Clears the parent pointer in this edge. As a consequence of clearing
   * the parent, the hasParent() method will return false.
   */
  void clearParent();

  /**
   * Sets the parent edge of this edge.
   */
  void setParent(Edge *edge) { _parent = edge; }

  /**
   * Whether this edge has been assigned a parent pointer or not.
   */
  bool hasParent() { return _parent != nullptr; }

  /**
   * Whether this edge has children or not.
   */
  bool hasChildren() { return _children.size() > 0; }

private:
  int _weight;
  double _ink;
  Point _source;
  Point _target;
  std::vector<Edge *> _children;
  Edge *_parent;
};

#endif /* EDGE_H_ */
