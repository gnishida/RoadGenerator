#include "RoadEdge.h"
#include "Util.h"

RoadEdge::RoadEdge(unsigned int type, unsigned int lanes, bool oneWay) {
	this->type = type;
	this->lanes = lanes;
	this->oneWay = oneWay;

	this->color = QColor(255, 255, 255);

	// initialize other members
	this->valid = true;
	this->shapeType = SHAPE_DEFAULT;
	this->group = -1;
	this->gridness = 0.0f;
	this->seed = false;
	this->fullyPaired = false;
}

RoadEdge::~RoadEdge() {
}

float RoadEdge::getLength() {
	float length = 0.0f;
	for (int i = 0; i < polyLine.size() - 1; i++) {
		length += (polyLine[i + 1] - polyLine[i]).length();
	}

	return length;
}

std::vector<QVector2D> RoadEdge::getPolyLine() {
	return polyLine;
}

/**
 * Add a point to the polyline of the road segment.
 *
 * @param pt		new point to be added
 */
void RoadEdge::addPoint(const QVector2D &pt) {
	polyLine.push_back(pt);
}

float RoadEdge::getWidth(float widthPerLane) {
	if (type == 1) { // local street
		return widthPerLane;
	} else if (type == 2) { // avenue
		return widthPerLane * 1.5f;
	} else if (type == 3) { // high way
		return widthPerLane * 2.0f;
	} else {
		return 0.0f;
	}
}

/**
 * Check whether the point resides in this road segment.
 * Return true if so, false otherwise.
 *
 * @param pos		the point
 * @return			true if the point is inside the road segment, false otherwise
 */
bool RoadEdge::containsPoint(const QVector2D &pos, float widthPerLane, int& index) {
	for (int i = 0; i < polyLine.size() - 1; i++) {
		QVector2D p0 = polyLine[i];
		QVector2D p1 = polyLine[i + 1];
		if (Util::pointSegmentDistanceXY(p0, p1, pos) <= getWidth(widthPerLane)) {
			// find the closest point
			float min_dist = std::numeric_limits<float>::max();
			for (int j = 0; j < polyLine.size(); j++) {
				float dist = (polyLine[j] - pos).length();
				if (dist < min_dist) {
					min_dist = dist;
					index = j;
				}
			}

			return true;
		}
	}

	return false;
}