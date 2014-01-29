#include "BFSTree.h"
#include "GraphUtil.h"

/**
 * Constructor
 * root is the root node to be used to create the tree.
 */
BFSTree::BFSTree(RoadGraph* roads, RoadVertexDesc root, int maxDepth) : AbstractForest(roads) {
	this->roots.push_back(root);
	depth = 0;

	buildForest(maxDepth);
}

BFSTree::~BFSTree() {
}

void BFSTree::buildForest(int maxDepth) {
	QList<RoadVertexDesc> seeds;

	QMap<RoadEdgeDesc, bool> visitedEdge;
	QMap<RoadVertexDesc, bool> visitedVertex;

	RoadVertexDesc root = roots[0];

	// register the root as a seed
	seeds.push_back(root);

	// mark the root as visited
	visitedVertex[root] = true;

	// starting from the roots, traverse all the nodes in the BFS manner
	while (!seeds.empty()) {
		RoadVertexDesc parent = seeds.front();
		seeds.pop_front();

		// update the depth of this tree
		int d = getDepth(parent);
		if (d > depth) depth = d;

		// check the depth limit
		if (maxDepth > 0 && d >= maxDepth) continue;

		std::vector<RoadVertexDesc> children;

		// list up all the neighbor nodes
		std::vector<RoadVertexDesc> nodes;
		std::vector<RoadEdgeDesc> edges;
		RoadOutEdgeIter ei, eend;
		for (boost::tie(ei, eend) = boost::out_edges(parent, roads->graph); ei != eend; ++ei) {
			if (!roads->graph[*ei]->valid) continue;
			if (visitedEdge[*ei]) continue;

			// retrieve the neighbor
			RoadVertexDesc child = boost::target(*ei, roads->graph);
			if (!roads->graph[child]->valid) continue;

			if (getParent(parent).contains(child)) continue;

			nodes.push_back(child);
			edges.push_back(*ei);

			// mark the edge as passed
			visitedEdge[*ei] = true;
		}

		// visit each neighbor node
		for (int i = 0; i < nodes.size(); i++) {
			RoadVertexDesc child = nodes[i];

			if (visitedVertex.contains(child)) { // if it is already visited
				/*
				RoadEdgeDesc orig_e_desc = GraphUtil::getEdge(roads, parent, child);

				// もともとのエッジを無効にする
				roads->graph[orig_e_desc]->valid = false;

				// 対象ノードが訪問済みの場合、対象ノードをコピーして子ノードにする
				RoadVertexDesc child2 = GraphUtil::addVertex(roads, roads->graph[child]);
				roads->graph[child2]->virt = false;

				// エッジ作成
				RoadEdgeDesc e_desc = GraphUtil::addEdge(roads, parent, child2, roads->graph[orig_e_desc]);

				children.push_back(child2);
				*/

				children.push_back(child);
			} else { // if it is not visited
				visitedVertex[child] = true;

				children.push_back(child);

				seeds.push_back(child);
			}
		}

		this->children.insert(parent, children);
	}
}