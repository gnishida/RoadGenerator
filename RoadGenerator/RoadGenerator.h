#pragma once

#include "RoadGraph.h"
#include "BBox.h"
#include "GridFeature.h"

class RoadGenerator {
public:
	RoadGenerator();
	~RoadGenerator();

	void generateRoadNetwork(RoadGraph& roads, const BBox& area, const GridFeature& gf);

private:
	void generateHorizontalAvenues(RoadGraph& roads, const BBox& area, const GridFeature& gf, std::list<RoadVertexDesc>& seeds);
	std::list<RoadVertexDesc> expandHorizontalAvenues(RoadGraph& roads, const BBox& area, const GridFeature& gf, std::list<RoadVertexDesc>& seeds, int dir, float length);
};

