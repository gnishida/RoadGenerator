#pragma once

#include "AbstractArea.h"
#include <QVector2D>
#include <vector>
#include <boost/shared_ptr.hpp>

class BBox : public AbstractArea {
public:
	QVector2D minPt;
	QVector2D maxPt;

public:
	BBox();
	BBox(const QVector2D& pt);
	~BBox();

	void reset();
	void combineWithBBox(const BBox& other);
	void addPoint(const QVector2D& newPt);
	bool overlapsWithBBoxXY(const BBox& other);
	QVector2D midPt() const;
	float dx() const;
	float dy() const;
	void translate(float x, float y);
	void resize(const QVector2D& pt);
	std::vector<QVector2D> polyline() const;
	bool hitTest(const QVector2D& pt) const;
	bool hitTestResizingPoint(const QVector2D& pt) const;
	bool contains(const QVector2D &pt) const;
};

typedef boost::shared_ptr<BBox> BBoxPtr;