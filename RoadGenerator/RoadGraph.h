#pragma once

#include "RoadVertex.h"
#include "RoadEdge.h"
#include "Renderable.h"
#include <stdio.h>
#include <qvector2d.h>
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/properties.hpp>
#include <boost/graph/graph_traits.hpp>
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/topological_sort.hpp>
#include <boost/graph/graph_utility.hpp>
#include <boost/shared_ptr.hpp>

using namespace boost;

typedef adjacency_list<vecS, vecS, undirectedS, RoadVertexPtr, RoadEdgePtr> BGLGraph;
typedef graph_traits<BGLGraph>::vertex_descriptor RoadVertexDesc;
typedef graph_traits<BGLGraph>::edge_descriptor RoadEdgeDesc;
typedef graph_traits<BGLGraph>::vertex_iterator RoadVertexIter;
typedef graph_traits<BGLGraph>::edge_iterator RoadEdgeIter;
typedef graph_traits<BGLGraph>::out_edge_iterator RoadOutEdgeIter;
typedef graph_traits<BGLGraph>::in_edge_iterator RoadInEdgeIter;

class RoadGraph {
public:
	BGLGraph graph;
	bool modified;
	std::vector<RenderablePtr> renderables;

	// for rendering (These variables should be updated via setZ() function only!!
	float highwayHeight;
	float avenueHeight;
	float widthBase;
	float curbRatio;
	bool showLocalStreets;

public:
	RoadGraph();
	~RoadGraph();

	void generateMesh();
	void RoadGraph::addMeshFromEdge(RenderablePtr renderable, RoadEdgePtr edge, float widthBase, QColor color, float height);

	bool getModified();
	void setModified();
	void clear();
	void setZ(float z);
};

typedef boost::shared_ptr<RoadGraph> RoadGraphPtr;
