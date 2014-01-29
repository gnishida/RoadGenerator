#pragma once

#include "RoadGraph.h"
#include "AbstractArea.h"
#include "GridFeature.h"
#include <opencv/cv.h>
#include <opencv/highgui.h>

class RoadSegmentationUtil {
protected:
	RoadSegmentationUtil() {}
	~RoadSegmentationUtil() {}

public:
	static void detectGrid(RoadGraph& roads, AbstractArea& area, int roadType, int maxIteration, float numBins, float minTotalLength, float minMaxBinRatio, float angleThreshold, float votingRatioThreshold, float extendingDistanceThreshold);
	static bool detectOneGrid(RoadGraph& roads, AbstractArea& area, int roadType, GridFeature& gf, int numBins, float minTotalLength, float minMaxBinRatio, float angleThreshold, float votingRatioThreshold, float extendingDistanceThreshold);
	static int traverseConnectedEdges(RoadGraph& roads, RoadEdgeDesc e, QMap<RoadEdgeDesc, int>& edges, int segment_id);
	static void reduceGridGroup(RoadGraph& roads, GridFeature& gf, QMap<RoadEdgeDesc, float>& edges);
	static void extendGridGroup(RoadGraph& roads, AbstractArea& area, int roadType, GridFeature& gf, QMap<RoadEdgeDesc, float>& edges, float angleThreshold, float votingRatioThreshold, float distanceThreshold);

	static void detectPlaza(RoadGraph& roads, AbstractArea& area);

	static void detectRadial(RoadGraph& roads, AbstractArea& area, int roadType, int maxIteration, float scale1, float scale2, float centerErrorTol2, float angleThreshold2, float scale3, float centerErrorTol3, float angleThreshold3, float votingRatioThreshold, float seedDistance, float extendingAngleThreshold);
	static bool detectOneRadial(RoadGraph& roads, AbstractArea& area, int roadType, int group_id, float scale1, float scale2, float centerErrorTol2, float angleThreshold2, float scale3, float centerErrorTol3, float angleThreshold3, float votingRatioThreshold, float seedDistance, float extendingAngleThreshold);
	static QVector2D detectRadialCenterInScaled(RoadGraph& roads, AbstractArea& area, int roadType, float scale);
	static QVector2D refineRadialCenterInScaled(RoadGraph& roads, AbstractArea& area, int roadType, float scale, QVector2D& centerApprox, float distanceThreshold, float angleThreshold);
	static bool reduceRadialGroup(RoadGraph& roads, QVector2D& center, int group_id, float distanceThreshold, int minNumSeeds);
	static void extendRadialGroup(RoadGraph& roads, AbstractArea& area, int roadType, QVector2D& center, int group_id, float angleThreshold, float dirCheckRatio);

	static void detectRoundabout(RoadGraph& roads, AbstractArea& area);
};

