#include "BFSForest.h"
#include "GraphUtil.h"

/**
 * Constructor
 * 
 * roots contains some pairs of two vertices.
 * Each pair contains two vertices that are corresponding each other.
 */
BFSForest::BFSForest(RoadGraph* roads, QList<RoadVertexDesc> roots, int maxDepth) : AbstractForest(roads) {
	this->roots = roots;

	buildForest(maxDepth);
}

BFSForest::~BFSForest() {
}

void BFSForest::buildForest(int maxDepth) {
	QList<RoadVertexDesc> seeds;
	QList<int> groups;

	QMap<RoadEdgeDesc, bool> visitedEdge;
	QMap<RoadVertexDesc, bool> visitedVertex;

	// For each root node
	for (int i = 0; i < roots.size() / 2; i++) {
		RoadVertexDesc src = roots[i * 2];
		RoadVertexDesc tgt = roots[i * 2 + 1];

		// Get the edge
		RoadEdgeDesc e_desc = GraphUtil::getEdge(*roads, src, tgt);

		// Set the group and the seed flag for the edge
		roads->graph[e_desc]->group = i;
		roads->graph[e_desc]->seed = true;

		// If the src node is already used as a seed
		if (seeds.contains(src)) {
			// copy the src vertex
			RoadVertexPtr v = RoadVertexPtr(new RoadVertex(roads->graph[src]->pt));
			RoadVertexDesc new_src = boost::add_vertex(roads->graph);
			roads->graph[new_src] = v;

			// remove the old edge
			roads->graph[e_desc]->valid = false;

			// add a new edge
			e_desc = GraphUtil::addEdge(*roads, new_src, tgt, roads->graph[e_desc]);

			src = new_src;
		}

		// If the tgt node is already used as a seed
		if (seeds.contains(tgt)) {
			// copy the tgt vertex
			RoadVertexPtr v = RoadVertexPtr(new RoadVertex(roads->graph[tgt]->pt));
			RoadVertexDesc new_tgt = boost::add_vertex(roads->graph);
			roads->graph[new_tgt] = v;

			// remove the old edge
			roads->graph[e_desc]->valid = false;

			// add a new edge
			e_desc = GraphUtil::addEdge(*roads, src, new_tgt, roads->graph[e_desc]);

			tgt = new_tgt;
		}

		// update the roots
		roots[i * 2] = src;
		roots[i * 2 + 1] = tgt;

		// register the seeds
		seeds.push_back(src);
		seeds.push_back(tgt);
		groups.push_back(i);
		groups.push_back(i);

		// mark root edges as visited
		visitedEdge[e_desc] = true;
		visitedVertex[src] = true;
		visitedVertex[tgt] = true;
	}

	// starting from the roots, traverse all the nodes in the BFS manner
	while (!seeds.empty()) {
		RoadVertexDesc parent = seeds.front();
		seeds.pop_front();

		int group = groups.front();
		groups.pop_front();

		// check the depth
		if (maxDepth > 0 && getDepth(parent) > maxDepth) continue;

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
				RoadEdgeDesc orig_e_desc = GraphUtil::getEdge(*roads, parent, child);

				// invalidate the original edge
				roads->graph[orig_e_desc]->valid = false;

				// if the node is already visited, copy it and add it as a child.
				RoadVertexDesc child2 = GraphUtil::addVertex(*roads, roads->graph[child]);
				roads->graph[child2]->virt = false;

				// create a new edge
				RoadEdgeDesc e_desc = GraphUtil::addEdge(*roads, parent, child2, roads->graph[orig_e_desc]);

				roads->graph[e_desc]->group = group;

				children.push_back(child2);
			} else { // if it is not visited
				visitedVertex[child] = true;
				roads->graph[edges[i]]->group = group;

				children.push_back(child);

				seeds.push_back(child);
				groups.push_back(group);
			}
		}

		this->children.insert(parent, children);
	}
}