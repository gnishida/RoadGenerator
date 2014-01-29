#pragma once

#include "RoadGraph.h"
#include <QMap>
#include <vector>

class AbstractForest {
public:
	RoadGraph* roads;
	QMap<RoadVertexDesc, std::vector<RoadVertexDesc> > children;
	QList<RoadVertexDesc> roots;

public:
	AbstractForest(RoadGraph* roads);
	~AbstractForest();

	std::vector<RoadVertexDesc>& getChildren(RoadVertexDesc node);
	void addChild(RoadVertexDesc parent, RoadVertexDesc child);
	QList<RoadVertexDesc> getParent(RoadVertexDesc node);
	QList<RoadVertexDesc> getRoots();
	int getDepth(RoadVertexDesc);

	virtual void buildForest(int maxDepth = 0) = 0;
};

