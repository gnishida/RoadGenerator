#pragma once

#include "AbstractArea.h"
#include <QVector2D>
#include <boost/shared_ptr.hpp>

class CircleArea : public AbstractArea {
private:
	QVector2D minPt;
	QVector2D maxPt;
	QVector2D center;
	float radius;

public:
	CircleArea(const QVector2D& center, float radius);
	~CircleArea();

	bool contains(const QVector2D& pt) const;
	QVector2D midPt() const;
	float dx() const;
	float dy() const;
	void translate(float x, float y);
	void resize(const QVector2D& pt);
	bool hitTest(const QVector2D& pt) const;
	bool hitTestResizingPoint(const QVector2D& pt) const;
	std::vector<QVector2D> polyline() const;
};

typedef boost::shared_ptr<CircleArea> CircleAreaPtr;