/*
 * PointOrStar.h
 *
 *  Created on: Oct 19, 2016
 *      Author: alan
 */

#ifndef POINTORSTAR_H_
#define POINTORSTAR_H_

#include <vector>

class PointOrStar {
public:
	PointOrStar(double x, double y);
	~PointOrStar();
	bool isStar() {
		return _children.size() != 0;
	}
	bool isPoint() {
		return _children.size() == 0;
	}

	std::vector<PointOrStar> getChildren() {
		return _children;
	}

	void addChild(PointOrStar child) {
		_children.push_back(child);
	}

	double getX() {
		return _x;
	}

	void setX(double x) {
		_x = x;
	}

	double getY() {
		return _y;
	}

	void setY(double y) {
		_y = y;
	}

private:
	std::vector<PointOrStar> _children;
	double _x;
	double _y;
};

#endif /* POINTORSTAR_H_ */
