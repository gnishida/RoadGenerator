#pragma once

#include <QVector2D>
#include <vector>
#include <boost/shared_ptr.hpp>

class AbstractArea {
public:
	static enum {
		RESIZING_TOP_LEFT = 0,
		RESIZING_TOP_RIGHT,
		RESIZING_BOTTOM_LEFT,
		RESIZING_BOTTOM_RIGHT
	};

	int resizingType;

public:
	AbstractArea();
	~AbstractArea();

	virtual bool contains(const QVector2D& pt) const = 0;
	virtual QVector2D midPt() const = 0;
	virtual float dx() const = 0;
	virtual float dy() const = 0;
	virtual void translate(float x, float y) = 0;
	virtual void resize(const QVector2D& pt) = 0;
	virtual bool hitTest(const QVector2D& pt) const = 0;
	virtual bool hitTestResizingPoint(const QVector2D& pt) const = 0;
	virtual std::vector<QVector2D> polyline() const = 0;
};

typedef boost::shared_ptr<AbstractArea> AbstractAreaPtr;