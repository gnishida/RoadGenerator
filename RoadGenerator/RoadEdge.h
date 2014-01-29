#pragma once

#include <Qvector2D>
#include <QColor>
#include <vector>
#include <boost/shared_ptr.hpp>

class RoadEdge {
public:
	static enum { SHAPE_DEFAULT = 0, SHAPE_GRID, SHAPE_RADIAL, SHAPE_PLAZA };

public:
	int type;
	int lanes;
	bool oneWay;
	std::vector<QVector2D> polyLine;
	QColor color;

	bool valid;			// if this edge is valid
	bool seed;			// if this edge is used as a seed of a forest

	int shapeType;		// 0 - default / 1 - grid / 2 - radial / 3 - plaza
	int group;			// to which tree in the forest this edge belongs to
	float gridness;		// how much it looks like grid
	bool fullyPaired;	// if this edge has a corresponding edge

public:
	RoadEdge(unsigned int type, unsigned int lanes, bool oneWay);
	~RoadEdge();
	
	float getLength();
	std::vector<QVector2D> getPolyLine();

	void addPoint(const QVector2D &pt);
	float getWidth(float widthPerLane);

	bool containsPoint(const QVector2D &pos, float widthPerLane, int& index);
};

typedef boost::shared_ptr<RoadEdge> RoadEdgePtr;