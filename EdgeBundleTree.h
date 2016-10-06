#ifndef MINGLEC_EDGEBUNDLETREE_H
#define MINGLEC_EDGEBUNDLETREE_H

#include <vector>
#include <unordered_set>
#include <cmath>
#include <cassert>
#include <list>
#include <deque>
#include "Util.h"
#include "EdgeGraph.h"

class BaseNode;
class BundleNode;
class EdgeNode;

/*
 * Represents a node in the 4-dimensional graph, Γ, that represents the edges
 * in a graph. The four coordinates of this node are the x and y coordinates
 * of both the source and the target points in the original graph.
 */
class BaseNode {
public:
    virtual ~BaseNode() {}

    /**
     * @return The source endpoint
     */
    virtual Point *getS() = 0;
    /**
     * @return The destination endpoint
     */
    virtual Point *getT() = 0;

    /**
     * @return The centroid of all childrens' S endpoints
     */
    virtual Point *getSCentroid() = 0;

    /**
     * @return The centroid of all childrens' T endpoints
     */
    virtual Point *getTCentroid() = 0;

    /**
     * @return Number of children who run through this node.
     */
    virtual int getWeight() = 0;

    /**
     * @return Children of the node
     */
    virtual std::vector<BaseNode *> *getChildren() = 0;

    /**
     * @return If this node is, itself, a bundle
     */
    virtual bool isBundle() = 0;

    /**
     * @return Total amount of ink used through this bundle.
     */
    virtual double getInk() = 0;

    /**
     * Bundles this node into other's bundle. If this node already has
     * a bundle, its bundle is merged with that of other. If no bundle
     * exists, one is created.
     *
     * @param other The node this node should be bundled with.
     */
    void bundleWith(BaseNode *other);

    /**
     * @return Whether this node has a root bundle.
     */
    bool hasBundle() { return _b != nullptr; }

    /**
     * @return The root bundle associated with this node.
     */
    BundleNode* getBundle() { return _b; }

    /**
     * Calculates the amount of ink saved by bundling with other.
     * A positive return value indicates a net savings.
     *
     * @param other
     * @return
     */
    double calculateBundleInkSavings(BaseNode *other);

    static void ReadEdges(const char *path, std::vector<Point> &points, std::vector<EdgeNode> &edges);

    double calculateBundle(BaseNode *other,
                           Point *s, Point *t,
                           Point *sCentroid, Point *tCentroid);
    BundleNode *_b = nullptr;   // Top level bundle
};


/**
 * Represents a bundle of underlying nodes.
 *
 * Note that after a node is coalesced, all the "weight" shifts to S.
 * e.g. getSCentroid returns the point S. This is useful for recursive
 * bundling, because it allows a BundleNode to act as an edge node.
 */
class BundleNode : public BaseNode {
public:
    BundleNode(BaseNode *e1, BaseNode *e2);
    ~BundleNode() {}

    Point* getS() { return &_s; };
    Point* getT() { return &_t; };
    Point* getSCentroid() { return _isFinal ? &_s : &_sCentroid; };
    Point* getTCentroid() { return _isFinal ? &_t : &_tCentroid; };
    std::vector<BaseNode *> *getChildren() { return &_children; }
    int getWeight() { return _weight; }
    bool isBundle() { return true; }
    double getInk() { return _ink; }
    void annexBundle(BaseNode *otherBundle);
    void addChild(BaseNode *child);
    void coalesce() { _isFinal = true; }

private:
    Point _s;
    Point _t;
    Point _sCentroid;
    Point _tCentroid;
    std::vector<BaseNode *> _children;
    int _weight = 0;
    float _ink;
    bool _isFinal = false;
    friend class EdgeNode;
};

static std::vector<BaseNode *> NO_POINTS;

class EdgeNode : public BaseNode {
public:
    EdgeNode(Point *s, Point *t) : _s(s), _t(t) {}
    ~EdgeNode() {}
    Point* getS() { return _s; };
    Point* getT() { return _t; };
    Point* getSCentroid() { return _s; };
    Point* getTCentroid() { return _t; };
    int getWeight() { return 1; }
    std::vector<BaseNode *> *getChildren() { return &NO_POINTS; }
    bool isBundle() { return false; }
    double getInk() { return sqrt(_t->sqDist(*_s)); }

private:
    Point *_s = nullptr;
    Point *_t = nullptr;
    friend class BundleNode;
};

/**
 * Fixme: Convert to range-based iterator
 * Usage: call next() over and over until it returns nullptr.
 */
class EdgeBundleIterator {
public:
    EdgeBundleIterator(std::vector<EdgeNode> *edges)
            : _edges(edges) {}
    EdgeBundleIterator(std::vector<EdgeNode> *edges, bool justRoots)
            : _edges(edges), _justRoots(justRoots) {}

    BaseNode *next() {
        while (true) {
            if (!_frontier.empty()) {
                // Something on the frontier.
                // Pop it, push its children (if any), return it.
                BaseNode *n = _frontier.back();
                _frontier.pop_back();
                if (!_justRoots) {
                    for (auto child : *n->getChildren()) {
                        _frontier.push_back(child);
                    }
                }
                return n;
            } else if (_edgeIndex < _edges->size()) {
                // More raw edges remain. Check the next...
                BaseNode *n = &(*_edges)[_edgeIndex++];
                BaseNode *b = n->getBundle();

                // If it's an orphaned edge, visit it
                if (b == nullptr) {
                  return n;
                }

                // If there's a root bundle, check to see if it's
                // already visited or not. Add it to the frontier
                // if not and cycle around to the next iteration.
                if (_visitedBundles.find(b) == _visitedBundles.end()) {
                    _visitedBundles.insert(b);
                    _frontier.push_back(b);
                }
            } else {
                return nullptr;
            }
        }
    }
private:
    int _edgeIndex = 0;
    std::vector<EdgeNode> *_edges;        // Raw, unbundled edges
    std::unordered_set<BaseNode *> _visitedBundles;
    std::deque<BaseNode *> _frontier;
    bool _justRoots = false;

};

class EdgeBundleTree {
public:
    EdgeBundleTree(std::vector<Point> *points, std::vector<EdgeNode> *edges) : _edges(edges), _points(points) {
    }

    std::vector<Point> *_points;
    std::vector<EdgeNode> *_edges;        // Raw, unbundled edges

    EdgeBundleIterator *iterator() {
        return new EdgeBundleIterator(_edges);
    }

    EdgeBundleIterator *rootIterator() {
        return new EdgeBundleIterator(_edges, true);
    }

    void write(char *pathNodes, char *pathEdges) {
        FILE *nf = fopen(pathNodes, "w");
        assert(nf != nullptr);
        FILE *ef = fopen(pathEdges, "w");
        assert(ef != nullptr);

        std::unordered_map<PointId, PointId> idMap;
        std::unordered_set<PointId> seen;
        EdgeBundleIterator *iter = rootIterator();

        _maxPointId = 0;
        for (auto &p : *_points) {
            idMap[p.id] = p.id;
            if (p.id > _maxPointId)  _maxPointId = p.id;
        }

        while (true) {
            BaseNode *n = iter->next();
            if (n == nullptr) break;
            writeBundle(nf, ef, seen, idMap, n, nullptr, nullptr);
        }
        delete(iter);
        PointId maxPointId = 0;
        GraphNodeId numNodes = 0;

        fclose(nf);
        fclose(ef);
    }

private:
    PointId _maxPointId;     // The largest original point id.

    /**
     * Writes a bundle to the output file.
     * @param nf The nodes file to write to.
     * @param ef The edges file to write to.
     * @param seen A set of points that have been already seen.
     * @param idMap A map of points with the point id as key and the point as value.
     * @param n The node to write a bundle from.
     * @param s2 The source point in the node. This is one of the two points in the edge.
     * @param t2 The target point in the node. This is one of the two points in the edge.
     */
    void writeBundle(FILE *nf, FILE *ef,
                     std::unordered_set<PointId> &seen, std::unordered_map<PointId, PointId> &idMap,
                     BaseNode *n, Point *s, Point *t) {
        Point *s2 = (s == nullptr) ? n->getS() : s;
        Point *t2 = (t == nullptr) ? n->getT() : t;

        // Write out each point...
        for (auto p : {s2, t2}) {
        	// If the point hasn't been seen.
            if (seen.find(p->id) == seen.end()) {
                seen.insert(p->id);
                PointId id = idMap[p->id];
                if (id == POINT_ID_NONE) {
                    id = ++_maxPointId;
                    idMap[p->id] = id;
                }
                fprintf(nf, "%u %.4f %.4f\n", id, p->x, p->y);
            }
        }
        fprintf(ef, " %u:%u:%d", idMap[s2->id], idMap[t2->id], n->getWeight());
        // Write the edges of this bundle.
        for (auto c : *n->getChildren()) {
            writeBundle(nf, ef, seen, idMap, c, nullptr, s2);
            writeBundle(nf, ef, seen, idMap, c, t2, nullptr);
        }
    }
};



#endif //MINGLEC_EDGEBUNDLETREE_H
