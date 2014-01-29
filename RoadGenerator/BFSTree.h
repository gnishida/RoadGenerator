#pragma once

#include "AbstractForest.h"
#include "RoadGraph.h"
#include <vector>

class BFSTree : public AbstractForest {
public:
	int depth;

public:
	BFSTree(RoadGraph* roads, RoadVertexDesc root, int maxDepth = 0);
	~BFSTree();
	
	void buildForest(int maxDepth = 0);
};

