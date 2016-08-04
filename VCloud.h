#pragma once

#include <vector>

#include "VPoint.h"

class VCloud {
private:
	std::vector<VPoint> _points;

public:
	size_t size() {
		return _points.size();
	}

	void push_back(VPoint& point) {
		_points.push_back(point);
	}

	VPoint operator[](int i) {
		return _points[i];
	}

	VPoint* at(int i) {
		return &_points[i];
	}

	void clear() {
		_points.clear();
		_points.resize(0);
	}
};