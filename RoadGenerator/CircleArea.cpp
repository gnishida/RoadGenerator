#include "CircleArea.h"
#include "GraphUtil.h"

CircleArea::CircleArea(const QVector2D& center, float radius) {
	this->center = center;
	this->radius = radius;

	minPt.setX(center.x() - radius);
	minPt.setY(center.y() - radius);
	maxPt.setX(center.x() + radius);
	maxPt.setY(center.y() + radius);
}

CircleArea::~CircleArea() {
}

bool CircleArea::contains(const QVector2D& pt) const {
	if ((pt - center).length() < radius) return true;
	else return false;
}

QVector2D CircleArea::midPt() const {
	return center;
}

float CircleArea::dx() const {
	return radius;
}

float CircleArea::dy() const {
	return radius;
}

void CircleArea::translate(float x, float y) {
	center.setX(center.x() + x);
	center.setX(center.y() + y);
	minPt.setX(minPt.x() + x);
	minPt.setY(minPt.y() + y);
	maxPt.setX(maxPt.x() + x);
	maxPt.setY(maxPt.y() + y);
}

void CircleArea::resize(const QVector2D& pt) {
	switch (resizingType) {
	case RESIZING_TOP_LEFT:
		minPt.setX(pt.x());
		maxPt.setY(pt.y());
		break;
	case RESIZING_TOP_RIGHT:
		maxPt.setX(pt.x());
		maxPt.setY(pt.y());
		break;
	case RESIZING_BOTTOM_LEFT:
		minPt.setX(pt.x());
		minPt.setY(pt.y());
		break;
	case RESIZING_BOTTOM_RIGHT:
		maxPt.setX(pt.x());
		minPt.setY(pt.y());
		break;
	}

	center = (minPt + maxPt) / 2.0f;
}

bool CircleArea::hitTest(const QVector2D& pt) const {
	if ((pt - center).length() < radius * 1.1f) return true;
	else return false;
}

bool CircleArea::hitTestResizingPoint(const QVector2D& pt) const {
	if (fabs(pt.x() - center.x() - radius) < dx() * 0.1f && fabs(pt.y() - center.y() + radius) < dy() * 0.1f) return true;
	else return false;
}

std::vector<QVector2D> CircleArea::polyline() const {
	std::vector<QVector2D> ret;

	int n = 16;
	for (int i = 0; i < n; i++) {
		float theta = M_PI * 2 * (float)i / (float)n;
		ret.push_back(QVector2D(center.x() + cosf(theta) * radius, center.y() + sinf(theta) * radius));
	}

	return ret;
}

