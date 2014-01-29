#include "GraphUtil.h"
#include "Util.h"
#include "BFSForest.h"
#include <time.h>
#include <QList>
#include <QSet>
#include <QDebug>
#include <boost/geometry.hpp>
#include <boost/geometry/geometries/point_xy.hpp>
#include <boost/geometry/geometries/linestring.hpp>

#ifndef M_PI
#define M_PI	3.141592653
#endif

EdgePair::EdgePair(RoadEdgeDesc edge1, RoadEdgeDesc edge2) {
	this->edge1 = edge1;
	this->edge2 = edge2;
}

EdgePairComparison::EdgePairComparison(RoadGraph* roads1, RoadGraph* roads2) {
	this->roads1 = roads1;
	this->roads2 = roads2;
}

/**
 * Sort the edge pairs according to the dissimilarity.
 * The pair with low dissimilarity comes to the head of the list, i.e. the head of the list contains the most similar edge pair.
 */
bool EdgePairComparison::operator()(const EdgePair& left, const EdgePair& right) const {
	float dissimilarity1 = GraphUtil::computeDissimilarityOfEdges(roads1, left.edge1, roads2, left.edge2);
	float dissimilarity2 = GraphUtil::computeDissimilarityOfEdges(roads1, right.edge1, roads2, right.edge2);

	return dissimilarity1 < dissimilarity2;
}

/**
 * Return the number of vertices.
 *
 */
int GraphUtil::getNumVertices(RoadGraph& roads, bool onlyValidVertex) {
	if (!onlyValidVertex) {
		return boost::num_vertices(roads.graph);
	}

	int count = 0;
	RoadVertexIter vi, vend;
	for (boost::tie(vi, vend) = boost::vertices(roads.graph); vi != vend; ++vi) {
		if (!roads.graph[*vi]->valid) continue;

		count++;
	}

	return count;
}

/**
 * Return the number of vertices which are connected to the specified vertex.
 */
int GraphUtil::getNumConnectedVertices(RoadGraph& roads, RoadVertexDesc start, bool onlyValidVertex) {
	int count = 1;

	QList<RoadVertexDesc> queue;
	queue.push_back(start);

	QList<RoadVertexDesc> visited;
	visited.push_back(start);

	while (!queue.empty()) {
		RoadVertexDesc v = queue.front();
		queue.pop_front();

		RoadOutEdgeIter ei, eend;
		for (boost::tie(ei, eend) = boost::out_edges(v, roads.graph); ei != eend; ++ei) {
			if (onlyValidVertex && !roads.graph[*ei]->valid) continue;

			RoadVertexDesc u = boost::target(*ei, roads.graph);
			if (onlyValidVertex && !roads.graph[u]->valid) continue;

			if (visited.contains(u)) continue;

			visited.push_back(u);
			queue.push_back(u);
			count++;
		}
	}

	return count;
}

/**
 * Return the index-th vertex.
 */
RoadVertexDesc GraphUtil::getVertex(RoadGraph& roads, int index, bool onlyValidVertex) {
	int count = 0;
	RoadVertexIter vi, vend;
	for (boost::tie(vi, vend) = boost::vertices(roads.graph); vi != vend; ++vi) {
		if (onlyValidVertex && !roads.graph[*vi]->valid) continue;

		if (count == index) return *vi;

		count++;
	}

	throw "Index exceeds the number of vertices.";
}

/**
 * Find the closest vertex from the specified point. 
 */
RoadVertexDesc GraphUtil::getVertex(RoadGraph& roads, const QVector2D& pt, bool onlyValidVertex) {
	RoadVertexDesc nearest_desc;
	float min_dist = std::numeric_limits<float>::max();

	RoadVertexIter vi, vend;
	for (boost::tie(vi, vend) = boost::vertices(roads.graph); vi != vend; ++vi) {
		if (onlyValidVertex && !roads.graph[*vi]->valid) continue;

		float dist = (roads.graph[*vi]->getPt() - pt).lengthSquared();
		if (dist < min_dist) {
			nearest_desc = *vi;
			min_dist = dist;
		}
	}

	return nearest_desc;
}

/**
 * 近隣頂点を探す。
 * ただし、方向ベクトルがangle方向からしきい値を超えてる場合、その頂点はスキップする。
 */
RoadVertexDesc GraphUtil::getVertex(RoadGraph& roads, const QVector2D& pt, float angle, float angle_threshold, bool onlyValidVertex) {
	RoadVertexDesc nearest_desc;
	float min_dist = std::numeric_limits<float>::max();

	RoadVertexIter vi, vend;
	for (boost::tie(vi, vend) = boost::vertices(roads.graph); vi != vend; ++vi) {
		if (onlyValidVertex && !roads.graph[*vi]->valid) continue;

		QVector2D vec = roads.graph[*vi]->getPt() - pt;
		float angle2 = atan2f(vec.y(), vec.x());
		if (diffAngle(angle, angle2) > angle_threshold) continue;

		float dist = vec.lengthSquared();
		if (dist < min_dist) {
			nearest_desc = *vi;
			min_dist = dist;
		}
	}

	return nearest_desc;
}

/**
 * Find the closest vertex from the specified point. 
 * If the closet vertex is within the threshold, return true. Otherwise, return false.
 */
bool GraphUtil::getVertex(RoadGraph& roads, const QVector2D& pos, float threshold, RoadVertexDesc& desc, bool onlyValidVertex) {
	float min_dist = std::numeric_limits<float>::max();

	RoadVertexIter vi, vend;
	for (boost::tie(vi, vend) = boost::vertices(roads.graph); vi != vend; ++vi) {
		if (onlyValidVertex && !roads.graph[*vi]->valid) continue;

		float dist = (roads.graph[*vi]->getPt() - pos).lengthSquared();
		if (dist < min_dist) {
			min_dist = dist;
			desc = *vi;
		}
	}

	if (min_dist <= threshold * threshold) return true;
	else return false;
}

/**
 * Find the closest vertex from the specified vertex. 
 * If the closet vertex is within the threshold, return true. Otherwise, return false.
 */
bool GraphUtil::getVertex(RoadGraph& roads, RoadVertexDesc v, float threshold, RoadVertexDesc& desc, bool onlyValidVertex) {
	return getVertex(roads, roads.graph[v]->pt, threshold, v, desc, onlyValidVertex);
}

/**
 * Find the closest vertex from the specified point. 
 * If the closet vertex is within the threshold, return true. Otherwise, return false.
 */
bool GraphUtil::getVertex(RoadGraph& roads, const QVector2D& pos, float threshold, RoadVertexDesc ignore, RoadVertexDesc& desc, bool onlyValidVertex) {
	float min_dist = std::numeric_limits<float>::max();

	RoadVertexIter vi, vend;
	for (boost::tie(vi, vend) = boost::vertices(roads.graph); vi != vend; ++vi) {
		if (onlyValidVertex && !roads.graph[*vi]->valid) continue;
		if (*vi == ignore) continue;

		float dist = (roads.graph[*vi]->getPt() - pos).lengthSquared();
		if (dist < min_dist) {
			min_dist = dist;
			desc = *vi;
		}
	}

	if (min_dist <= threshold * threshold) return true;
	else return false;
}

/**
 * 当該頂点が、何番目の頂点かを返却する。
 */
int GraphUtil::getVertexIndex(RoadGraph* roads, RoadVertexDesc desc, bool onlyValidVertex) {
	int count = 0;
	RoadVertexIter vi, vend;
	for (boost::tie(vi, vend) = boost::vertices(roads->graph); vi != vend; ++vi) {
		if (onlyValidVertex && !roads->graph[*vi]->valid) continue;

		if (*vi == desc) return count;

		count++;
	}

	throw "The specified vertex does not exist.";
}

/**
 * Add a vertex.
 */
RoadVertexDesc GraphUtil::addVertex(RoadGraph& roads, RoadVertexPtr v) {
	RoadVertexPtr new_v = RoadVertexPtr(new RoadVertex(*v));
	RoadVertexDesc new_v_desc = boost::add_vertex(roads.graph);
	roads.graph[new_v_desc] = new_v;

	roads.setModified();

	return new_v_desc;
}

/**
 * Move the vertex to the specified location.
 * The outing edges are also moved accordingly.
 */
void GraphUtil::moveVertex(RoadGraph& roads, RoadVertexDesc v, const QVector2D& pt) {
	// Move the outing edges
	RoadOutEdgeIter ei, eend;
	for (boost::tie(ei, eend) = boost::out_edges(v, roads.graph); ei != eend; ++ei) {
		RoadVertexDesc tgt = boost::target(*ei, roads.graph);

		std::vector<QVector2D> polyLine = roads.graph[*ei]->polyLine;
		if ((polyLine[0] - roads.graph[v]->getPt()).lengthSquared() < (polyLine[0] - roads.graph[tgt]->getPt()).lengthSquared()) {
			std::reverse(polyLine.begin(), polyLine.end());
		}

		int num = polyLine.size();
		QVector2D dir = pt - roads.graph[v]->pt;
		for (int i = 0; i < num - 1; i++) {
			polyLine[i] += dir * (float)i / (float)(num - 1);
		}
		polyLine[num - 1] = pt;

		roads.graph[*ei]->polyLine = polyLine;
	}

	// Move the vertex
	roads.graph[v]->pt = pt;

	roads.setModified();
}

/**
 * Return the degree of the specified vertex.
 */
int GraphUtil::getDegree(RoadGraph& roads, RoadVertexDesc v, bool onlyValidEdge) {
	if (onlyValidEdge) {
		int count = 0;
		RoadOutEdgeIter ei, eend;
		for (boost::tie(ei, eend) = boost::out_edges(v, roads.graph); ei != eend; ++ei) {
			if (roads.graph[*ei]->valid) count++;
		}
		return count;
	} else {
		return boost::degree(v, roads.graph);
	}
}

/**
 * Return the list of vertices.
 */
std::vector<RoadVertexDesc> GraphUtil::getVertices(RoadGraph* roads, bool onlyValidVertex) {
	std::vector<RoadVertexDesc> ret;

	RoadVertexIter vi, vend;
	for (boost::tie(vi, vend) = boost::vertices(roads->graph); vi != vend; ++vi) {
		if (onlyValidVertex && !roads->graph[*vi]->valid) continue;

		ret.push_back(*vi);
	}

	return ret;
}

/**
 * Remove the isolated vertices.
 * Note that this function does not change neither the vertex desc nor the edge desc.
 */
void GraphUtil::removeIsolatedVertices(RoadGraph& roads, bool onlyValidVertex) {
	RoadVertexIter vi, vend;
	for (boost::tie(vi, vend) = boost::vertices(roads.graph); vi != vend; ++vi) {
		if (onlyValidVertex && !roads.graph[*vi]->valid) continue;

		if (getDegree(roads, *vi, onlyValidVertex) == 0) {
			roads.graph[*vi]->valid = false;
		}
	}
}

/**
 * Snap v1 to v2.
 */
void GraphUtil::snapVertex(RoadGraph& roads, RoadVertexDesc v1, RoadVertexDesc v2) {
	if (v1 == v2) return;

	moveVertex(roads, v1, roads.graph[v2]->pt);

	if (hasEdge(roads, v1, v2)) {
		RoadEdgeDesc e = getEdge(roads, v1, v2);

		// if the edge is too short, remove it. (This might be contraversial...)
		if (roads.graph[e]->getLength() < 1.0f) {
			roads.graph[e]->valid = false;
		}
	}

	// Snap all the outing edges from v1
	RoadOutEdgeIter ei, eend;
	for (boost::tie(ei, eend) = boost::out_edges(v1, roads.graph); ei != eend; ++ei) {
		if (!roads.graph[*ei]->valid) continue;

		RoadVertexDesc v1b = boost::target(*ei, roads.graph);

		// invalidate the old edge
		roads.graph[*ei]->valid = false;

		if (v1b == v2) continue;
		if (hasEdge(roads, v2, v1b)) continue;
		//if (hasCloseEdge(roads, v2, v1b)) continue;	// <-- In this case, snap the edge to the other instead of discard it.

		// add a new edge
		addEdge(roads, v2, v1b, roads.graph[*ei]);
	}

	// invalidate v1
	roads.graph[v1]->valid = false;

	roads.setModified();
}

/**
 * Return the central vertex.
 */
RoadVertexDesc GraphUtil::getCentralVertex(RoadGraph& roads) {
	BBox box = getAABoundingBox(roads);
	return getVertex(roads, box.midPt());
}

/**
 * Return the index-th edge.
 */
RoadEdgeDesc GraphUtil::getEdge(RoadGraph& roads, int index, bool onlyValidEdge) {
	int count = 0;
	RoadEdgeIter ei, eend;
	for (boost::tie(ei, eend) = boost::edges(roads.graph); ei != eend; ++ei) {
		if (onlyValidEdge && !roads.graph[*ei]) continue;

		if (index == count) return *ei;
		count++;
	}

	throw "No edge found for the specified index.";
}

/**
 * Return the total lengths of the edges outing from the specified vertex.
 */
float GraphUtil::getTotalEdgeLength(RoadGraph& roads, RoadVertexDesc v) {
	float ret = 0.0f;

	RoadOutEdgeIter ei, eend;
	for (boost::tie(ei, eend) = boost::out_edges(v, roads.graph); ei != eend; ++ei) {
		if (!roads.graph[*ei]->valid) continue;
		ret += roads.graph[*ei]->getLength();
	}

	return ret;
}

/**
 * Remove the edge. 
 * Remove the vertex that has smaller number of degrees.
 */
/*void GraphUtil::collapseEdge(RoadGraph* roads, RoadEdgeDesc e) {
	RoadVertexDesc v1 = boost::source(e, roads->graph);
	RoadVertexDesc v2 = boost::target(e, roads->graph);
	if (v1 == v2) return;

	// invalidate the edge
	roads->graph[e]->valid = false;

	if (getDegree(roads, v1) < getDegree(roads, v2)) {
		snapVertex(roads, v1, v2);
	} else {
		snapVertex(roads, v2, v1);
	}
}*/

/**
 * Return the number of edges.
 */
int GraphUtil::getNumEdges(RoadGraph& roads, bool onlyValidEdge) {
	if (!onlyValidEdge) {
		return boost::num_edges(roads.graph);
	}

	int count = 0;

	RoadEdgeIter ei, eend;
	for (boost::tie(ei, eend) = boost::edges(roads.graph); ei != eend; ++ei) {
		if (roads.graph[*ei]->valid) count++;
	}

	return count;
}

int GraphUtil::getNumEdges(RoadGraph& roads, RoadVertexDesc v, bool onlyValidEdge) {
	int count = 0;
	RoadOutEdgeIter ei, eend;
	for (boost::tie(ei, eend) = out_edges(v, roads.graph); ei != eend; ++ei) {
		if (!roads.graph[*ei]->valid) continue;

		count++;
	}

	return count;
}

/**
 * Add an edge.
 * This function creates a straight line of edge.
 */
RoadEdgeDesc GraphUtil::addEdge(RoadGraph& roads, RoadVertexDesc src, RoadVertexDesc tgt, unsigned int type, unsigned int lanes, bool oneWay) {
	roads.setModified();

	// エッジを新規追加する
	RoadEdgePtr e = RoadEdgePtr(new RoadEdge(type, lanes, oneWay));
	e->addPoint(roads.graph[src]->getPt());
	e->addPoint(roads.graph[tgt]->getPt());

	std::pair<RoadEdgeDesc, bool> edge_pair = boost::add_edge(src, tgt, roads.graph);
	roads.graph[edge_pair.first] = e;

	return edge_pair.first;
}

/**
 * Add an edge.
 * This function creates a edge which is copied from the reference edge.
 */
RoadEdgeDesc GraphUtil::addEdge(RoadGraph& roads, RoadVertexDesc src, RoadVertexDesc tgt, RoadEdgePtr ref_edge) {
	roads.setModified();

	RoadEdgePtr e = RoadEdgePtr(new RoadEdge(*ref_edge));
	e->valid = true;

	std::pair<RoadEdgeDesc, bool> edge_pair = boost::add_edge(src, tgt, roads.graph);
	roads.graph[edge_pair.first] = e;

	return edge_pair.first;
}

/**
 * Check if there is an edge between two vertices.
 */
bool GraphUtil::hasEdge(RoadGraph& roads, RoadVertexDesc desc1, RoadVertexDesc desc2, bool onlyValidEdge) {
	RoadOutEdgeIter ei, eend;
	for (boost::tie(ei, eend) = boost::out_edges(desc1, roads.graph); ei != eend; ++ei) {
		if (onlyValidEdge && !roads.graph[*ei]->valid) continue;

		RoadVertexDesc tgt = boost::target(*ei, roads.graph);
		if (tgt == desc2) return true;
	}

	for (boost::tie(ei, eend) = boost::out_edges(desc2, roads.graph); ei != eend; ++ei) {
		if (onlyValidEdge && !roads.graph[*ei]->valid) continue;

		RoadVertexDesc tgt = boost::target(*ei, roads.graph);
		if (tgt == desc1) return true;
	}

	return false;
}

/**
 * Return the edge between src and tgt.
 */
RoadEdgeDesc GraphUtil::getEdge(RoadGraph& roads, RoadVertexDesc src, RoadVertexDesc tgt, bool onlyValidEdge) {
	RoadOutEdgeIter ei, eend;
	for (boost::tie(ei, eend) = boost::out_edges(src, roads.graph); ei != eend; ++ei) {
		if (onlyValidEdge && !roads.graph[*ei]->valid) continue;

		if (boost::target(*ei, roads.graph) == tgt) return *ei;
	}

	for (boost::tie(ei, eend) = boost::out_edges(tgt, roads.graph); ei != eend; ++ei) {
		if (onlyValidEdge && !roads.graph[*ei]->valid) continue;

		if (boost::target(*ei, roads.graph) == src) return *ei;
	}

	throw "No edge found.";
}

/**
 * Sort the points of the polyline of the edge in such a way that the first point is the location of the src vertex.
 */
/*
std::vector<QVector2D> GraphUtil::getOrderedPolyLine(RoadGraph& roads, RoadEdgeDesc e) {
	std::vector<QVector2D> ret = roads.graph[e]->getPolyLine();

	RoadVertexDesc src = boost::source(e, roads.graph);
	RoadVertexDesc tgt = boost::target(e, roads.graph);
	if ((roads.graph[src]->getPt() - roads.graph[e]->getPolyLine()[0]).lengthSquared() <= (roads.graph[tgt]->getPt() - roads.graph[e]->getPolyLine()[0]).lengthSquared()) {
		return ret;
	} else {
		std::reverse(ret.begin(), ret.end());
		return ret;
	}
}
*/

/**
 * Sort the points of the polyline of the edge in such a way that the first point is the location of the src vertex.
 */
void GraphUtil::getOrderedPolyLine(RoadGraph& roads, RoadEdgeDesc e, std::vector<QVector2D>& polyline) {
	polyline = roads.graph[e]->getPolyLine();

	RoadVertexDesc src = boost::source(e, roads.graph);
	RoadVertexDesc tgt = boost::target(e, roads.graph);
	if ((roads.graph[src]->getPt() - roads.graph[e]->getPolyLine()[0]).lengthSquared() > (roads.graph[tgt]->getPt() - roads.graph[e]->getPolyLine()[0]).lengthSquared()) {
		std::reverse(polyline.begin(), polyline.end());
	}
}

/**
 * Sort the points of the polyline of the edge in such a way that the first point is the location of the src vertex.
 */
void GraphUtil::orderPolyLine(RoadGraph& roads, RoadEdgeDesc e, RoadVertexDesc src) {
	RoadVertexDesc tgt;

	RoadVertexDesc s = boost::source(e, roads.graph);
	RoadVertexDesc t = boost::target(e, roads.graph);

	if (s == src) {
		tgt = t;
	} else {
		tgt = s;
	}

	// If the order is opposite, reverse the order.
	if ((roads.graph[src]->getPt() - roads.graph[e]->getPolyLine()[0]).lengthSquared() > (roads.graph[tgt]->getPt() - roads.graph[e]->getPolyLine()[0]).lengthSquared()) {
		std::reverse(roads.graph[e]->polyLine.begin(), roads.graph[e]->polyLine.end());
	}
}

/**
 * Move the edge to the specified location.
 * src_posは、エッジeのsource頂点の移動先
 * tgt_posは、エッジeのtarget頂点の移動先
 */
void GraphUtil::moveEdge(RoadGraph& roads, RoadEdgeDesc e, QVector2D& src_pos, QVector2D& tgt_pos) {
	RoadVertexDesc src = boost::source(e, roads.graph);
	RoadVertexDesc tgt = boost::target(e, roads.graph);

	int n = roads.graph[e]->polyLine.size();

	if ((roads.graph[e]->polyLine[0] - roads.graph[src]->pt).lengthSquared() < (roads.graph[e]->polyLine[0] - roads.graph[tgt]->pt).lengthSquared()) {
		QVector2D src_diff = src_pos - roads.graph[e]->polyLine[0];
		QVector2D tgt_diff = tgt_pos - roads.graph[e]->polyLine[n - 1];

		for (int i = 1; i < n - 1; i++) {
			roads.graph[e]->polyLine[i] += src_diff + (tgt_diff - src_diff) * (float)i / (float)(n - 1);
		}
		roads.graph[e]->polyLine[0] = src_pos;
		roads.graph[e]->polyLine[n - 1] = tgt_pos;
	} else {
		QVector2D src_diff = src_pos - roads.graph[e]->polyLine[n - 1];
		QVector2D tgt_diff = tgt_pos - roads.graph[e]->polyLine[0];

		for (int i = 1; i < n - 1; i++) {
			roads.graph[e]->polyLine[i] += tgt_diff + (src_diff - tgt_diff) * (float)i / (float)(n - 1);
		}
		roads.graph[e]->polyLine[0] = tgt_pos;
		roads.graph[e]->polyLine[n - 1] = src_pos;
	}

	roads.setModified();
}

/**
 * Remove all the dead-end edges.
 */
bool GraphUtil::removeDeadEnd(RoadGraph& roads) {
	bool removed = false;

	bool removedOne = true;
	while (removedOne) {
		removedOne = false;
		RoadVertexIter vi, vend;
		for (boost::tie(vi, vend) = boost::vertices(roads.graph); vi != vend; ++vi) {
			if (!roads.graph[*vi]->valid) continue;

			if (getDegree(roads, *vi) == 1) {
				// invalidate all the outing edges.
				RoadOutEdgeIter ei, eend;
				for (boost::tie(ei, eend) = boost::out_edges(*vi, roads.graph); ei != eend; ++ei) {
					roads.graph[*ei]->valid = false;
				}

				// invalidate the vertex as well.
				roads.graph[*vi]->valid = false;

				removedOne = true;
				removed = true;
			}
		}
	}

	if (removed) {
		roads.setModified();
	}

	return removed;
}

/**
 * Interpolate two polylines.
 * If the number of nodes are same, just interpolate them one by one.
 * Otherwise, discritize them into 10 points, and interpolate them one by one.
 */
std::vector<QVector2D> GraphUtil::interpolateEdges(RoadGraph* roads1, RoadEdgeDesc e1, RoadVertexDesc src1, RoadGraph* roads2, RoadEdgeDesc e2, RoadVertexDesc src2, float t) {

	orderPolyLine(*roads1, e1, src1);
	orderPolyLine(*roads2, e2, src2);

	std::vector<QVector2D> polyLine1 = roads1->graph[e1]->polyLine;
	std::vector<QVector2D> polyLine2 = roads2->graph[e2]->polyLine;

	std::vector<QVector2D> ret;

	int n1 = polyLine1.size();
	int n2 = polyLine2.size();

	if (n1 == n2) {
		for (int i = 0; i < n1; i++) {
			ret.push_back(polyLine1[i] * t + polyLine2[i] * (1.0f - t));
		}
	} else {
		for (float index = 0.0f; index < 1.0f; index += 0.1f) {
			// compute the location of the index-th point of the 1st polyline.
			int j1 = index * (float)(n1 - 1);

			float s1 = index - (float)j1 / (float)(n1 - 1);
			float t1 = (float)(j1 + 1) / (float)(n1 - 1) - index;

			QVector2D pt1 = polyLine1[j1] * t1 / (s1 + t1) + polyLine1[j1 + 1] * s1 / (s1 + t1);

			// compute the location of the index-th point of the 2nd polyline.
			int j2 = index * (float)(n2 - 1);

			float s2 = index - (float)j2 / (float)(n2 - 1);
			float t2 = (float)(j2 + 1) / (float)(n2 - 1) - index;

			QVector2D pt2 = polyLine2[j2] * t2 / (s2 + t2) + polyLine2[j2 + 1] * s2 / (s2 + t2);

			// interpolate
			ret.push_back(pt1 * t + pt2 * (1.0f - t));
		}

		// interpolate the last points
		ret.push_back(polyLine1[polyLine1.size() - 1] * t + polyLine2[polyLine2.size() - 1] * (1.0f - t));
	}

	return ret;
}

/**
 * Return the dissimilarity of two road graphs.
 *
 * dissimilarity = distance of two vertices + difference in angle of edges + difference in degrees + difference in lanes.
 */
float GraphUtil::computeDissimilarityOfEdges(RoadGraph* roads1, RoadEdgeDesc e1, RoadGraph* roads2, RoadEdgeDesc e2) {
	float w_distance = 0.001f;
	float w_angle = 1.25f;
	float w_degree = 0.3f;
	float w_lanes = 0.3f;

	RoadVertexDesc src1 = boost::source(e1, roads1->graph);
	RoadVertexDesc tgt1 = boost::target(e1, roads1->graph);

	RoadVertexDesc src2 = boost::source(e2, roads2->graph);
	RoadVertexDesc tgt2 = boost::target(e2, roads2->graph);

	// if src1 and tgt2, tgt1 and src2 are close to each other, exchange src2 and tgt2.
	if ((roads1->graph[src1]->pt - roads2->graph[src2]->pt).length() + (roads1->graph[tgt1]->pt - roads2->graph[tgt2]->pt).length() > (roads1->graph[src1]->pt - roads2->graph[tgt2]->pt).length() + (roads1->graph[tgt1]->pt - roads2->graph[src2]->pt).length()) {
		src2 = boost::target(e2, roads2->graph);
		tgt2 = boost::source(e2, roads2->graph);
	}

	// compute each factor
	float dist = (roads1->graph[src1]->pt - roads2->graph[src2]->pt).length() + (roads1->graph[tgt1]->pt - roads2->graph[tgt2]->pt).length();
	float angle = diffAngle(roads1->graph[src1]->pt - roads1->graph[tgt1]->pt, roads2->graph[src2]->pt - roads2->graph[tgt2]->pt);
	float degree = abs(getDegree(*roads1, src1) - getDegree(*roads2, src2)) + abs(getDegree(*roads1, tgt1) - getDegree(*roads2, tgt2));
	float lanes = abs((double)(roads1->graph[e1]->lanes - roads2->graph[e2]->lanes));

	return dist * w_distance + angle * w_angle + degree * w_degree + lanes * w_lanes;
}

/**
 * Remove the isolated edges.
 * Note that this function does not change neither the vertex desc nor the edge desc.
 */
void GraphUtil::removeIsolatedEdges(RoadGraph& roads, bool onlyValidEdge) {
	RoadEdgeIter ei, eend;
	for (boost::tie(ei, eend) = boost::edges(roads.graph); ei != eend; ++ei) {
		if (onlyValidEdge && !roads.graph[*ei]->valid) continue;

		RoadVertexDesc src = boost::source(*ei, roads.graph);
		RoadVertexDesc tgt = boost::target(*ei, roads.graph);

		if (getDegree(roads, src, onlyValidEdge) == 1 && getDegree(roads, tgt, onlyValidEdge) == 1) {
			roads.graph[*ei]->valid = false;
			roads.graph[src]->valid = false;
			roads.graph[tgt]->valid = false;
		}
	}

	roads.setModified();
}

/**
 * Split the edge at the specified point.
 */
RoadVertexDesc GraphUtil::splitEdge(RoadGraph* roads, RoadEdgeDesc edge_desc, const QVector2D& pt) {
	RoadEdgePtr edge = roads->graph[edge_desc];

	// find which point along the polyline is the closest to the specified split point.
	int index;
	QVector2D pos;
	float min_dist = std::numeric_limits<float>::max();
	for (int i = 0; i < roads->graph[edge_desc]->polyLine.size() - 1; i++) {
		QVector2D vec = roads->graph[edge_desc]->polyLine[i + 1] - roads->graph[edge_desc]->polyLine[i];
		float length = vec.length();
		for (int j = 0; j < length; j += 1.0f) {
			QVector2D pt2 = roads->graph[edge_desc]->polyLine[i] + vec * (float)j / length;
			float dist = (pt2 - pt).lengthSquared();
			if (dist < min_dist) {
				min_dist = dist;
				index = i;
				pos = pt2;
			}
		}
	}

	RoadVertexDesc src = boost::source(edge_desc, roads->graph);
	RoadVertexDesc tgt = boost::target(edge_desc, roads->graph);

	// add a new vertex at the specified point on the edge
	RoadVertexPtr v = RoadVertexPtr(new RoadVertex(pos));
	RoadVertexDesc v_desc = boost::add_vertex(roads->graph);
	roads->graph[v_desc] = v;

	// add the first edge
	RoadEdgePtr e1 = RoadEdgePtr(new RoadEdge(edge->type, edge->lanes, edge->oneWay));
	if ((edge->getPolyLine()[0] - roads->graph[src]->pt).lengthSquared() < (edge->getPolyLine()[0] - roads->graph[tgt]->pt).lengthSquared()) {
		for (int i = 0; i <= index; i++) {
			e1->addPoint(edge->getPolyLine()[i]);
		}
		e1->addPoint(pos);
	} else {
		e1->addPoint(pos);
		for (int i = index + 1; i < edge->getPolyLine().size(); i++) {
			e1->addPoint(edge->getPolyLine()[i]);
		}
	}
	std::pair<RoadEdgeDesc, bool> edge_pair1 = boost::add_edge(src, v_desc, roads->graph);
	roads->graph[edge_pair1.first] = e1;

	// add the second edge
	RoadEdgePtr e2 = RoadEdgePtr(new RoadEdge(edge->type, edge->lanes, edge->oneWay));
	if ((edge->getPolyLine()[0] - roads->graph[src]->pt).lengthSquared() < (edge->getPolyLine()[0] - roads->graph[tgt]->pt).lengthSquared()) {
		e2->addPoint(pos);
		for (int i = index + 1; i < edge->getPolyLine().size(); i++) {
			e2->addPoint(edge->getPolyLine()[i]);
		}
	} else {
		for (int i = 0; i <= index; i++) {
			e2->addPoint(edge->getPolyLine()[i]);
		}
		e2->addPoint(pos);
	}
	std::pair<RoadEdgeDesc, bool> edge_pair2 = boost::add_edge(v_desc, tgt, roads->graph);
	roads->graph[edge_pair2.first] = e2;

	// remove the original edge
	roads->graph[edge_desc]->valid = false;

	return v_desc;
}

/**
 * Check if there is an edge outing from v1 that is too close to the line v1 - v2.
 */
bool GraphUtil::hasCloseEdge(RoadGraph* roads, RoadVertexDesc v1, RoadVertexDesc v2, float angle_threshold) {
	RoadOutEdgeIter ei, eend;
	for (boost::tie(ei, eend) = boost::out_edges(v1, roads->graph); ei != eend; ++ei) {
		if (!roads->graph[*ei]->valid) continue;

		RoadVertexDesc tgt = boost::target(*ei, roads->graph);

		float angle = diffAngle(roads->graph[tgt]->pt - roads->graph[v1]->pt, roads->graph[v2]->pt - roads->graph[v1]->pt);
		if (angle < angle_threshold) return true;
	}

	return false;
}

/**
 * Check if the poly line intersects with the existing road segments.
 */
bool GraphUtil::isIntersect(RoadGraph* roads, std::vector<QVector2D>& polyLine) {
	RoadEdgeIter ei, eend;
	for (boost::tie(ei, eend) = boost::edges(roads->graph); ei != eend; ++ei) {
		if (!roads->graph[*ei]->valid) continue;

		if (isIntersect(roads, roads->graph[*ei]->polyLine, polyLine)) return true;
	}

	return false;
}

/**
 * Check if the two poly lines intersect with each other.
 */
bool GraphUtil::isIntersect(RoadGraph* roads, std::vector<QVector2D>& polyLine1, std::vector<QVector2D>& polyLine2) {
	for (int i = 0; i < polyLine1.size() - 1; i++) {
		for (int j = 0; j < polyLine2.size() - 1; j++) {
			float tab, tcd;
			QVector2D intPt;
			if (Util::segmentSegmentIntersectXY(polyLine1[i], polyLine1[i + 1], polyLine2[j], polyLine2[j + 1], &tab, &tcd, true, intPt)) {
				return true;
			}
		}
	}

	return false;
}

/**
 * Simplify a polyline.
 */
std::vector<QVector2D> GraphUtil::simplifyPolyLine(std::vector<QVector2D>& polyLine, float threshold) {
	std::vector<QVector2D> ret;
	
	typedef boost::geometry::model::d2::point_xy<double> xy;
	boost::geometry::model::linestring<xy> line;
	for (int i = 0; i < polyLine.size(); i++) {
		line.push_back(xy(polyLine[i].x(), polyLine[i].y()));
	}

	boost::geometry::model::linestring<xy> simplified;
	boost::geometry::simplify(line, simplified, threshold);

	for (int i = 0; i < simplified.size(); i++) {
		ret.push_back(QVector2D(simplified[i].x(), simplified[i].y()));
	}

	return ret;
}

/**
 * Remove the short edges.
 */
void GraphUtil::removeShortEdges(RoadGraph& roads, float threshold) {
	RoadEdgeIter ei, eend;
	for (boost::tie(ei, eend) = boost::edges(roads.graph); ei != eend; ++ei) {
		if (!roads.graph[*ei]->valid) continue;

		if (roads.graph[*ei]->getLength() <= threshold) {
			roads.graph[*ei]->valid = false;
		}
	}
}

/**
 * Make the road graph realistic.
 * 1) remove isolated vertices.
 * 2) remove isolated edges.
 */
void GraphUtil::realize(RoadGraph& roads) {
	removeIsolatedVertices(roads);
	removeIsolatedEdges(roads);
}

/**
 * Make the edge finer by inserting more points along the polyline.
 */
std::vector<QVector2D> GraphUtil::finerEdge(RoadGraph& roads, RoadEdgeDesc e, float step) {
	std::vector<QVector2D> polyLine;

	for (int i = 0; i < roads.graph[e]->polyLine.size() - 1; i++) {
		QVector2D vec = roads.graph[e]->polyLine[i + 1] - roads.graph[e]->polyLine[i];
		float length = vec.length();
		for (int j = 0; j < length; j += step) {
			polyLine.push_back(roads.graph[e]->polyLine[i] + vec * (float)j / length);
		}
	}
	polyLine.push_back(roads.graph[e]->polyLine[roads.graph[e]->polyLine.size() - 1]);

	return polyLine;
}

/**
 * Load the road from a file.
 */
void GraphUtil::loadRoads(RoadGraph& roads, const QString& filename, int roadType) {
	clock_t start, end;

	roads.clear();

	FILE* fp = fopen(filename.toUtf8().data(), "rb");

	QMap<uint, RoadVertexDesc> idToDesc;

	// Read the number of vertices
	unsigned int nVertices;
	fread(&nVertices, sizeof(unsigned int), 1, fp);

	// Read each vertex's information: desc, x, and y.
	start = clock();
	for (int i = 0; i < nVertices; i++) {
		RoadVertexDesc id;
		float x, y;
		fread(&id, sizeof(RoadVertexDesc), 1, fp);
		fread(&x, sizeof(float), 1, fp);
		fread(&y, sizeof(float), 1, fp);

		RoadVertexPtr vertex = RoadVertexPtr(new RoadVertex(QVector2D(x, y)));

		RoadVertexDesc desc = boost::add_vertex(roads.graph);
		roads.graph[desc] = vertex;

		idToDesc[id] = desc;
	}
	end = clock();
	std::cout << "load vertices: " << (double)(end-start)/CLOCKS_PER_SEC << std::endl;

	// Read the number of edges
	unsigned int nEdges;
	fread(&nEdges, sizeof(unsigned int), 1, fp);

	// Read each edge's information: the descs of two vertices, road type, the number of lanes, the number of points along the polyline, and the coordinate of each point along the polyline.
	start = clock();
	for (int i = 0; i < nEdges; i++) {
		RoadEdgePtr edge = RoadEdgePtr(new RoadEdge(1, 1, false));

		RoadVertexDesc id1, id2;
		fread(&id1, sizeof(RoadVertexDesc), 1, fp);
		fread(&id2, sizeof(RoadVertexDesc), 1, fp);

		RoadVertexDesc src = idToDesc[id1];
		RoadVertexDesc tgt = idToDesc[id2];

		fread(&edge->type, sizeof(unsigned int), 1, fp);
		fread(&edge->lanes, sizeof(unsigned int), 1, fp);
		fread(&edge->oneWay, sizeof(unsigned int), 1, fp);

		unsigned int nPoints;
		fread(&nPoints, sizeof(unsigned int), 1, fp);

		for (int j = 0; j < nPoints; j++) {
			float x, y;
			fread(&x, sizeof(float), 1, fp);
			fread(&y, sizeof(float), 1, fp);

			edge->addPoint(QVector2D(x, y));
		}

		// 指定されたタイプの道路エッジのみを読み込む
		if (((int)powf(2, (edge->type - 1)) & roadType)) {
			std::pair<RoadEdgeDesc, bool> edge_pair = boost::add_edge(src, tgt, roads.graph);
			roads.graph[edge_pair.first] = edge;
		}
	}
	end = clock();
	std::cout << "load edges: " << (double)(end-start)/CLOCKS_PER_SEC << std::endl;

	fclose(fp);

	roads.setModified();
}

/**
 * Save the road to a file.
 */
void GraphUtil::saveRoads(RoadGraph& roads, const QString& filename) {
	RoadGraph temp;
	copyRoads(roads, temp);
	clean(temp);

	FILE* fp = fopen(filename.toUtf8().data(), "wb");
	
	int nVertices = boost::num_vertices(temp.graph);
	fwrite(&nVertices, sizeof(int), 1, fp);

	// 各頂点につき、ID、X座標、Y座標を出力する
	RoadVertexIter vi, vend;
	for (boost::tie(vi, vend) = boost::vertices(temp.graph); vi != vend; ++vi) {
		RoadVertexPtr v = temp.graph[*vi];
	
		RoadVertexDesc desc = *vi;
		float x = v->getPt().x();
		float y = v->getPt().y();
		fwrite(&desc, sizeof(RoadVertexDesc), 1, fp);
		fwrite(&x, sizeof(float), 1, fp);
		fwrite(&y, sizeof(float), 1, fp);
	}

	int nEdges = boost::num_edges(temp.graph);
	fwrite(&nEdges, sizeof(int), 1, fp);

	// 各エッジにつき、２つの頂点の各ID、道路タイプ、レーン数、一方通行か、ポリラインを構成するポイント数、各ポイントのX座標とY座標を出力する
	RoadEdgeIter ei, eend;
	for (boost::tie(ei, eend) = boost::edges(temp.graph); ei != eend; ++ei) {
		RoadEdgePtr edge = temp.graph[*ei];

		RoadVertexDesc src = boost::source(*ei, temp.graph);
		RoadVertexDesc tgt = boost::target(*ei, temp.graph);

		fwrite(&src, sizeof(RoadVertexDesc), 1, fp);
		fwrite(&tgt, sizeof(RoadVertexDesc), 1, fp);
		
		unsigned int type = edge->type;
		fwrite(&type, sizeof(unsigned int), 1, fp);

		unsigned int lanes = edge->lanes;
		fwrite(&lanes, sizeof(unsigned int), 1, fp);

		unsigned int oneWay;
		if (edge->oneWay) {
			oneWay = 1;
		} else {
			oneWay = 0;
		}
		fwrite(&oneWay, sizeof(unsigned int), 1, fp);

		int nPoints = edge->polyLine.size();
		fwrite(&nPoints, sizeof(int), 1, fp);

		for (int i = 0; i < edge->getPolyLine().size(); i++) {
			float x = edge->getPolyLine()[i].x();
			float y = edge->getPolyLine()[i].y();
			fwrite(&x, sizeof(float), 1, fp);
			fwrite(&y, sizeof(float), 1, fp);
		}
	}

	fclose(fp);
}

/**
 * Copy the road graph.
 * Note: This function does not change neither the vertex desc nor the edge desc.
 */
/*
RoadGraph* GraphUtil::copyRoads(RoadGraph& roads, int roadType) {
	RoadGraph* new_roads = new RoadGraph();
	
	QMap<RoadVertexDesc, RoadVertexDesc> conv;
	RoadVertexIter vi, vend;
	for (boost::tie(vi, vend) = boost::vertices(roads->graph); vi != vend; ++vi) {
		// Add a vertex
		RoadVertexPtr new_v = RoadVertexPtr(new RoadVertex(roads->graph[*vi]->getPt()));
		new_v->valid = roads->graph[*vi]->valid;
		RoadVertexDesc new_v_desc = boost::add_vertex(new_roads->graph);
		new_roads->graph[new_v_desc] = new_v;	

		conv[*vi] = new_v_desc;
	}

	RoadEdgeIter ei, eend;
	for (boost::tie(ei, eend) = boost::edges(roads->graph); ei != eend; ++ei) {
		RoadVertexDesc src = boost::source(*ei, roads->graph);
		RoadVertexDesc tgt = boost::target(*ei, roads->graph);

		RoadVertexDesc new_src = conv[src];
		RoadVertexDesc new_tgt = conv[tgt];

		// Add an edge
		RoadEdgePtr new_e = RoadEdgePtr(new RoadEdge(*roads->graph[*ei]));
		std::pair<RoadEdgeDesc, bool> edge_pair = boost::add_edge(new_src, new_tgt, new_roads->graph);
		new_roads->graph[edge_pair.first] = new_e;
	}

	if (roadType != 7) {
		extractRoads(new_roads, roadType);
	}

	new_roads->setModified();

	return new_roads;
}
*/

/**
 * Copy the road graph.
 * Note: This function copies all the vertices and edges including the invalid ones. Thus, their IDs will be preserved.
 */
void GraphUtil::copyRoads(RoadGraph& srcRoads, RoadGraph& dstRoads) {
	dstRoads.clear();

	QMap<RoadVertexDesc, RoadVertexDesc> conv;
	RoadVertexIter vi, vend;
	for (boost::tie(vi, vend) = boost::vertices(srcRoads.graph); vi != vend; ++vi) {
		// Add a vertex
		RoadVertexPtr new_v = RoadVertexPtr(new RoadVertex(srcRoads.graph[*vi]->getPt()));
		new_v->valid = srcRoads.graph[*vi]->valid;
		RoadVertexDesc new_v_desc = boost::add_vertex(dstRoads.graph);
		dstRoads.graph[new_v_desc] = new_v;

		conv[*vi] = new_v_desc;
	}

	RoadEdgeIter ei, eend;
	for (boost::tie(ei, eend) = boost::edges(srcRoads.graph); ei != eend; ++ei) {
		RoadVertexDesc src = boost::source(*ei, srcRoads.graph);
		RoadVertexDesc tgt = boost::target(*ei, srcRoads.graph);

		RoadVertexDesc new_src = conv[src];
		RoadVertexDesc new_tgt = conv[tgt];

		// Add an edge
		RoadEdgePtr new_e = RoadEdgePtr(new RoadEdge(*srcRoads.graph[*ei]));
		std::pair<RoadEdgeDesc, bool> edge_pair = boost::add_edge(new_src, new_tgt, dstRoads.graph);
		dstRoads.graph[edge_pair.first] = new_e;
	}

	dstRoads.setModified();
}

/**
 * Merge the 2nd road to the 1st road. As a result, the roads1 will be updated containing all the vertices and edges of roads2.
 */
void GraphUtil::mergeRoads(RoadGraph& roads1, RoadGraph& roads2) {
	QMap<RoadVertexDesc, RoadVertexDesc> conv;

	// copy vertices from the 2nd road to the 1st road
	RoadVertexIter vi, vend;
	for (boost::tie(vi, vend) = boost::vertices(roads2.graph); vi != vend; ++vi) {
		if (!roads2.graph[*vi]->valid) continue;

		RoadVertexPtr v1 = RoadVertexPtr(new RoadVertex(*roads2.graph[*vi]));
		RoadVertexDesc v1_desc = boost::add_vertex(roads1.graph);
		roads1.graph[v1_desc] = v1;

		conv[*vi] = v1_desc;
	}

	// copy edges from the 2nd road to the 1st road
	RoadEdgeIter ei, eend;
	for (boost::tie(ei, eend) = boost::edges(roads2.graph); ei != eend; ++ei) {
		if (!roads2.graph[*ei]->valid) continue;

		RoadVertexDesc src2 = boost::source(*ei, roads2.graph);
		RoadVertexDesc tgt2 = boost::target(*ei, roads2.graph);

		RoadVertexDesc src1 = conv[src2];
		RoadVertexDesc tgt1 = conv[tgt2];

		addEdge(roads1, src1, tgt1, roads2.graph[*ei]);
	}

	roads1.setModified();
}

/**
 * Connect roads1 to roads2 as much as possible.
 * roads1 will be updated. roads2 will be ramained as it is.
 */
void GraphUtil::connectRoads(RoadGraph& roads1, RoadGraph& roads2, float connect_threshold) {
	QMap<RoadVertexDesc, RoadVertexDesc> conv;

	// copy vertices from the 2nd road to the 1st road
	RoadVertexIter vi, vend;
	for (boost::tie(vi, vend) = boost::vertices(roads2.graph); vi != vend; ++vi) {
		if (!roads2.graph[*vi]->valid) continue;

		RoadVertexPtr v1 = RoadVertexPtr(new RoadVertex(*roads2.graph[*vi]));
		RoadVertexDesc v1_desc = boost::add_vertex(roads1.graph);
		roads1.graph[v1_desc] = v1;

		conv[*vi] = v1_desc;
	}

	// copy edges from the 2nd road to the 1st road
	RoadEdgeIter ei, eend;
	for (boost::tie(ei, eend) = boost::edges(roads2.graph); ei != eend; ++ei) {
		if (!roads2.graph[*ei]->valid) continue;

		RoadVertexDesc src2 = boost::source(*ei, roads2.graph);
		RoadVertexDesc tgt2 = boost::target(*ei, roads2.graph);

		if (isIntersect(&roads1, roads2.graph[*ei]->polyLine)) continue;

		RoadVertexDesc src1 = conv[src2];
		RoadVertexDesc tgt1 = conv[tgt2];

		addEdge(roads1, src1, tgt1, roads2.graph[*ei]);
	}

	// for each roads2 vertex, try to find the close vertex of roads1 to connect
	for (boost::tie(vi, vend) = boost::vertices(roads2.graph); vi != vend; ++vi) {
		if (!roads2.graph[*vi]->valid) continue;
		if (getDegree(roads1, conv[*vi]) > 1) continue;

		RoadVertexDesc v1_desc;
		if (getVertex(roads1, roads1.graph[conv[*vi]]->pt, connect_threshold, conv[*vi], v1_desc)) {
			if (!conv.contains(v1_desc)) {
				addEdge(roads1, v1_desc, conv[*vi], 1, 1, false);	// to be updated!!!
				continue;
			}
		}

		RoadEdgeDesc e1_desc;
		if (getEdge(&roads1, *vi, connect_threshold, e1_desc)) {
			RoadVertexDesc src = boost::source(e1_desc, roads1.graph);
			RoadVertexDesc tgt = boost::target(e1_desc, roads1.graph);

			if (!conv.contains(src) || !conv.contains(tgt)) {
				v1_desc = splitEdge(&roads1, e1_desc, roads2.graph[*vi]->pt);
				addEdge(roads1, v1_desc, conv[*vi], 1, 1, false);	// to be updated!!!
			}
		}
	}

	roads1.setModified();
}

/**
 * Return the axix aligned bounding box of the road graph.
 */
BBox GraphUtil::getAABoundingBox(RoadGraph& roads) {
	BBox box;

	RoadVertexIter vi, vend;
	for (boost::tie(vi, vend) = boost::vertices(roads.graph); vi != vend; ++vi) {
		if (!roads.graph[*vi]->valid) continue;

		box.addPoint(roads.graph[*vi]->getPt());
	}

	return box;
}

/**
 * Return the bounding box of the road graph.
 * 
 * The bounding box is not necessarily aligned to X/Y-axis.
 * Algorithm: Rotate the road graph from -90 degree to 90 degree by 5 degree per step, compute the axix aligned bounding box, and find the minimum one in terms of its area.
 * The raod graph is updated to be rotated based on the bounding box in the end.
 */
BBox GraphUtil::getBoudingBox(RoadGraph& roads, float theta1, float theta2, float theta_step) {
	float min_area = std::numeric_limits<float>::max();
	float min_theta;
	BBox min_box;

	for (float theta = theta1; theta <= theta2; theta += theta_step) {
		RoadGraph rotated_roads;
		copyRoads(roads, rotated_roads);
		rotate(rotated_roads, theta);
		BBox box = getAABoundingBox(rotated_roads);
		if (box.dx() * box.dy() < min_area) {
			min_area = box.dx() * box.dy();
			min_theta = theta;
			min_box = box;
		}
	}

	rotate(roads, min_theta);
	return min_box;
}

/**
 * Extract the specified type of road segments.
 * Note that this function does not change neither the vertex desc nor the edge desc.
 */
void GraphUtil::extractRoads(RoadGraph& roads, int roadType) {
	if (roadType == 7) return;

	RoadEdgeIter ei, eend;
	for (boost::tie(ei, eend) = boost::edges(roads.graph); ei != eend; ++ei) {
		if (!roads.graph[*ei]->valid) continue;

		if (!((int)powf(2.0f, roads.graph[*ei]->type - 1) & roadType)) {
			roads.graph[*ei]->valid = false;
		}
	}

	removeIsolatedVertices(roads);

	roads.setModified();
}

/**
 * Extract roads that reside in the specified area.
 * If "strict" is true, only the edges that are completely within the area will be extracted.
 * Note that this function does not change neither the vertex desc nor the edge desc.
 */
void GraphUtil::extractRoads(RoadGraph& roads, const AbstractArea& area, bool strict, int roadType) {
	RoadEdgeIter ei, eend;
	for (boost::tie(ei, eend) = boost::edges(roads.graph); ei != eend; ++ei) {
		if (!roads.graph[*ei]->valid) continue;

		RoadVertexDesc src = boost::source(*ei, roads.graph);
		RoadVertexDesc tgt = boost::target(*ei, roads.graph);

		if ((int)powf(2.0f, roads.graph[*ei]->type - 1) & roadType) {
			if (strict) {
				// if either vertice is out of the range, invalidate this edge.
				if (!area.contains(roads.graph[src]->pt) || !area.contains(roads.graph[tgt]->pt)) {
					roads.graph[*ei]->valid = false;
				}
			} else {
				// if both the vertices is out of the range, invalidate this edge.
				if (!area.contains(roads.graph[src]->pt) && !area.contains(roads.graph[tgt]->pt)) {
					roads.graph[*ei]->valid = false;
				}
			}
		} else {
			roads.graph[*ei]->valid = false;
		}
	}

	removeIsolatedVertices(roads);

	roads.setModified();
}

/**
 * Extract roads that reside in the specified area.
 * If a edge is across the border of the area, add a vertex on the border and split the edge at the vertex.
 */
void GraphUtil::extractRoads2(RoadGraph& roads, const AbstractArea& area, int roadType) {
	QList<RoadEdgeDesc> edges;

	RoadEdgeIter ei, eend;
	for (boost::tie(ei, eend) = boost::edges(roads.graph); ei != eend; ++ei) {
		if (!roads.graph[*ei]->valid) continue;

		RoadVertexDesc src = boost::source(*ei, roads.graph);
		RoadVertexDesc tgt = boost::target(*ei, roads.graph);

		if ((int)powf(2.0f, roads.graph[*ei]->type - 1) & roadType) {
			if (!area.contains(roads.graph[src]->pt) && !area.contains(roads.graph[tgt]->pt)) {
				roads.graph[*ei]->valid = false;
			} else if (!area.contains(roads.graph[src]->pt) || !area.contains(roads.graph[tgt]->pt)) {
				edges.push_back(*ei);
			}
		} else {
			roads.graph[*ei]->valid = false;
		}
	}

	for (int e_id = 0; e_id < edges.size(); e_id++) {
		RoadVertexDesc src = boost::source(edges[e_id], roads.graph);
		RoadVertexDesc tgt = boost::target(edges[e_id], roads.graph);

		// if either vertice is out of the range, add a vertex on the border
		std::vector<QVector2D> polyLine = finerEdge(roads, edges[e_id]);
		QVector2D intPt;
		if (area.contains(polyLine[0])) {
			for (int i = 1; i < polyLine.size(); i++) {
				if (!area.contains(polyLine[i])) {
					intPt = polyLine[i];
					break;
				}
			}
		} else {
			for (int i = polyLine.size() - 1; i >= 0; i--) {
				if (!area.contains(polyLine[i])) {
					intPt = polyLine[i];
					break;
				}
			}
		}

		RoadVertexDesc v = splitEdge(&roads, edges[e_id], intPt);
		if (area.contains(roads.graph[src]->pt)) {
			RoadEdgeDesc e = getEdge(roads, v, tgt);
			roads.graph[e]->valid = false;
		} else {
			RoadEdgeDesc e = getEdge(roads, v, src);
			roads.graph[e]->valid = false;
		}
	}

	removeIsolatedVertices(roads);
	reduce(roads);

	roads.setModified();
}

/**
 * Subtract an area from the road graph.
 * If "strict" is true, only the edges that are completely within the area will be subtracted.
 * Note that this function does not change neighter the vertex desc nor the edge desc.
 */
void GraphUtil::subtractRoads(RoadGraph& roads, const AbstractArea& area, bool strict) {
	RoadEdgeIter ei, eend;
	for (boost::tie(ei, eend) = boost::edges(roads.graph); ei != eend; ++ei) {
		if (!roads.graph[*ei]->valid) continue;

		RoadVertexDesc src = boost::source(*ei, roads.graph);
		RoadVertexDesc tgt = boost::target(*ei, roads.graph);

		if (strict) {
			// if both the vertices is within the range, invalidate this edge.
			if (area.contains(roads.graph[src]->pt) && area.contains(roads.graph[tgt]->pt)) {
				roads.graph[*ei]->valid = false;
			}
		} else {
			// if either vertice is within the range, invalidate this edge.
			if (area.contains(roads.graph[src]->pt) || area.contains(roads.graph[tgt]->pt)) {
				roads.graph[*ei]->valid = false;
			}
		}
	}

	removeIsolatedVertices(roads);

	roads.setModified();
}

/**
 * Subtract an area from the road graph.
 */
void GraphUtil::subtractRoads2(RoadGraph& roads, const AbstractArea& area) {
	QList<RoadEdgeDesc> edges;

	// list up all the edges that are across the border of the area
	RoadEdgeIter ei, eend;
	for (boost::tie(ei, eend) = boost::edges(roads.graph); ei != eend; ++ei) {
		if (!roads.graph[*ei]->valid) continue;

		RoadVertexDesc src = boost::source(*ei, roads.graph);
		RoadVertexDesc tgt = boost::target(*ei, roads.graph);
		
		if (area.contains(roads.graph[src]->pt) && area.contains(roads.graph[tgt]->pt)) {
			roads.graph[*ei]->valid = false;
		} else if (area.contains(roads.graph[src]->pt) || area.contains(roads.graph[tgt]->pt)) {
			edges.push_back(*ei);
		}
	}

	for (int e_id = 0; e_id < edges.size(); e_id++) {
		RoadVertexDesc src = boost::source(edges[e_id], roads.graph);
		RoadVertexDesc tgt = boost::target(edges[e_id], roads.graph);

		// if either vertice is out of the range, add a vertex on the border
		std::vector<QVector2D> polyLine = finerEdge(roads, edges[e_id], 3.0f);
		QVector2D intPt;
		if (area.contains(polyLine[0])) {
			for (int i = 1; i < polyLine.size(); i++) {
				if (!area.contains(polyLine[i])) {
					intPt = polyLine[i];
					break;
				}
			}
		} else {
			for (int i = polyLine.size() - 1; i >= 0; i--) {
				if (!area.contains(polyLine[i])) {
					intPt = polyLine[i];
					break;
				}
			}
		}

		RoadVertexDesc v = splitEdge(&roads, edges[e_id], intPt);
		if (area.contains(roads.graph[src]->pt)) {
			RoadEdgeDesc e = getEdge(roads, v, src);
			roads.graph[e]->valid = false;
		} else {
			RoadEdgeDesc e = getEdge(roads, v, tgt);
			roads.graph[e]->valid = false;
		}
	}

	removeIsolatedVertices(roads);
	reduce(roads);

	roads.setModified();
}

/**
 * Subtract a circle from the road graph.
 * Note that this function does not change neighter the vertex desc nor the edge desc.
 */
/*void GraphUtil::subtractRoadsByCircle(RoadGraph* roads, const QVector2D& center, float radius) {
	RoadEdgeIter ei, eend;
	for (boost::tie(ei, eend) = boost::edges(roads->graph); ei != eend; ++ei) {
		if (!roads->graph[*ei]->valid) continue;

		RoadVertexDesc src = boost::source(*ei, roads->graph);
		RoadVertexDesc tgt = boost::target(*ei, roads->graph);

		// if both the vertices is within the range, invalidate this edge.
		if ((roads->graph[src]->pt - center).length() < radius && (roads->graph[tgt]->pt - center).length() < radius) {
			roads->graph[*ei]->valid = false;
		}
	}

	removeIsolatedVertices(roads);

	roads->setModified();
}
*/

/**
 * Return the neighbors of the specified vertex.
 */
std::vector<RoadVertexDesc> GraphUtil::getNeighbors(RoadGraph& roads, RoadVertexDesc v, bool onlyValidVertex) {
	std::vector<RoadVertexDesc> neighbors;

	RoadOutEdgeIter ei, eend;
	for (boost::tie(ei, eend) = boost::out_edges(v, roads.graph); ei != eend; ++ei) {
		if (onlyValidVertex && !roads.graph[*ei]->valid) continue;

		neighbors.push_back(boost::target(*ei, roads.graph));
	}

	return neighbors;
}

/**
 * v1とv2が隣接した頂点かどうかチェックする。
 */
bool GraphUtil::isNeighbor(RoadGraph& roads, RoadVertexDesc v1, RoadVertexDesc v2) {
	RoadOutEdgeIter ei, eend;
	for (boost::tie(ei, eend) = boost::out_edges(v1, roads.graph); ei != eend; ++ei) {
		if (!roads.graph[*ei]->valid) continue;
		if (boost::target(*ei, roads.graph) == v2) return true;
	}
	for (boost::tie(ei, eend) = boost::out_edges(v2, roads.graph); ei != eend; ++ei) {
		if (!roads.graph[*ei]->valid) continue;
		if (boost::target(*ei, roads.graph) == v1) return true;
	}

	return false;
}

/**
 * Check if desc2 is reachable from desc1.
 */
bool GraphUtil::isConnected(RoadGraph& roads, RoadVertexDesc desc1, RoadVertexDesc desc2, bool onlyValidEdge) {
	QList<RoadVertexDesc> seeds;
	QSet<RoadVertexDesc> visited;

	seeds.push_back(desc1);
	visited.insert(desc1);

	while (!seeds.empty()) {
		RoadVertexDesc v = seeds.front();
		seeds.pop_front();

		visited.insert(v);

		RoadOutEdgeIter ei, eend;
		for (boost::tie(ei, eend) = boost::out_edges(v, roads.graph); ei != eend; ++ei) {
			if (onlyValidEdge && !roads.graph[*ei]->valid) continue;

			RoadVertexDesc u = boost::target(*ei, roads.graph);
			if (onlyValidEdge && !roads.graph[u]->valid) continue;

			if (u == desc2) return true;

			if (!visited.contains(u)) seeds.push_back(u);			
		}
	}

	return false;
}

/**
 * Find the closest vertex from the specified point.
 */
/*RoadVertexDesc GraphUtil::findNearestVertex(RoadGraph* roads, const QVector2D &pt) {
	RoadVertexDesc nearest_desc;
	float min_dist = std::numeric_limits<float>::max();

	RoadVertexIter vi, vend;
	for (boost::tie(vi, vend) = boost::vertices(roads->graph); vi != vend; ++vi) {
		if (!roads->graph[*vi]->valid) continue;

		float dist = (roads->graph[*vi]->getPt() - pt).length();
		if (dist < min_dist) {
			nearest_desc = *vi;
			min_dist = dist;
		}
	}

	return nearest_desc;
}*/

/**
 * Find the closest vertex from the specified point.
 * The vertex "ignore" is ignored.
 */
/*RoadVertexDesc GraphUtil::findNearestVertex(RoadGraph* roads, const QVector2D &pt, RoadVertexDesc ignore) {
	RoadVertexDesc nearest_desc;
	float min_dist = std::numeric_limits<float>::max();

	RoadVertexIter vi, vend;
	for (boost::tie(vi, vend) = boost::vertices(roads->graph); vi != vend; ++vi) {
		if (*vi == ignore) continue;
		if (!roads->graph[*vi]->valid) continue;

		float dist = (roads->graph[*vi]->getPt() - pt).length();
		if (dist < min_dist) {
			nearest_desc = *vi;
			min_dist = dist;
		}
	}

	return nearest_desc;
}*/

/**
 * 指定したノードvと接続されたノードの中で、指定した座標に最も近いノードを返却する。
 */
RoadVertexDesc GraphUtil::findConnectedNearestNeighbor(RoadGraph* roads, const QVector2D &pt, RoadVertexDesc v) {
	QMap<RoadVertexDesc, bool> visited;
	std::list<RoadVertexDesc> seeds;
	seeds.push_back(v);

	float min_dist = std::numeric_limits<float>::max();
	RoadVertexDesc min_desc;

	while (!seeds.empty()) {
		RoadVertexDesc seed = seeds.front();
		seeds.pop_front();

		RoadOutEdgeIter ei, eend;
		for (boost::tie(ei, eend) = boost::out_edges(seed, roads->graph); ei != eend; ++ei) {
			if (!roads->graph[*ei]->valid) continue;

			RoadVertexDesc v2 = boost::target(*ei, roads->graph);
			if (visited.contains(v2)) continue;

			// 指定したノードvは除く（除かない方が良いのか？検討中。。。。）
			//if (v2 == v) continue;

			visited[v2] = true;

			// 指定した座標との距離をチェック
			float dist = (roads->graph[v2]->getPt() - pt).lengthSquared();
			if (dist < min_dist) {
				min_dist = dist;
				min_desc = v2;
			}

			seeds.push_back(v2);
		}
	}

	return min_desc;
}

/**
 * Find the edge which is the closest to the specified point.
 * If the distance is within the threshold, return true. Otherwise, return false.
 */
bool GraphUtil::getEdge(RoadGraph* roads, const QVector2D &pt, float threshold, RoadEdgeDesc& e, bool onlyValidEdge) {
	float min_dist = std::numeric_limits<float>::max();
	RoadEdgeDesc min_e;

	RoadEdgeIter ei, eend;
	for (boost::tie(ei, eend) = boost::edges(roads->graph); ei != eend; ++ei) {
		if (onlyValidEdge && !roads->graph[*ei]->valid) continue;

		RoadVertexPtr src = roads->graph[boost::source(*ei, roads->graph)];
		RoadVertexPtr tgt = roads->graph[boost::target(*ei, roads->graph)];

		if (onlyValidEdge && !src->valid) continue;
		if (onlyValidEdge && !tgt->valid) continue;

		QVector2D pt2;
		for (int i = 0; i < roads->graph[*ei]->polyLine.size() - 1; i++) {
			float dist = Util::pointSegmentDistanceXY(roads->graph[*ei]->polyLine[i], roads->graph[*ei]->polyLine[i + 1], pt, pt2);
			if (dist < min_dist) {
				min_dist = dist;
				e = *ei;
			}
		}
	}

	if (min_dist < threshold) return true;
	else return false;
}

/**
 * Find the edge which is the closest to the specified vertex.
 * If the distance is within the threshold, return true. Otherwise, return false.
 */
bool GraphUtil::getEdge(RoadGraph* roads, RoadVertexDesc v, float threshold, RoadEdgeDesc& e, bool onlyValidEdge) {
	float min_dist = std::numeric_limits<float>::max();
	RoadEdgeDesc min_e;

	RoadEdgeIter ei, eend;
	for (boost::tie(ei, eend) = boost::edges(roads->graph); ei != eend; ++ei) {
		if (onlyValidEdge && !roads->graph[*ei]->valid) continue;

		RoadVertexDesc src = boost::source(*ei, roads->graph);
		RoadVertexDesc tgt = boost::target(*ei, roads->graph);

		if (src == v || tgt == v) continue;

		if (onlyValidEdge && !roads->graph[src]->valid) continue;
		if (onlyValidEdge && !roads->graph[tgt]->valid) continue;

		QVector2D pt2;
		for (int i = 0; i < roads->graph[*ei]->polyLine.size() - 1; i++) {
			float dist = Util::pointSegmentDistanceXY(roads->graph[*ei]->polyLine[i], roads->graph[*ei]->polyLine[i + 1], roads->graph[v]->pt, pt2);
			if (dist < min_dist) {
				min_dist = dist;
				e = *ei;
			}
		}
	}

	if (min_dist < threshold) return true;
	else return false;
}

/**
 * 指定された頂点に最も近いエッジを返却する。
 * ただし、指定された頂点に隣接するエッジは、対象外とする。
 */
RoadEdgeDesc GraphUtil::findNearestEdge(RoadGraph* roads, RoadVertexDesc v, float& dist, QVector2D &closestPt, bool onlyValidEdge) {
	dist = std::numeric_limits<float>::max();
	RoadEdgeDesc min_e;

	RoadEdgeIter ei, eend;
	for (boost::tie(ei, eend) = boost::edges(roads->graph); ei != eend; ++ei) {
		if (onlyValidEdge && !roads->graph[*ei]->valid) continue;

		RoadVertexDesc src = boost::source(*ei, roads->graph);
		RoadVertexDesc tgt = boost::target(*ei, roads->graph);
		if (v == src || v == tgt) continue;

		if (onlyValidEdge && !roads->graph[src]->valid) continue;
		if (onlyValidEdge && !roads->graph[tgt]->valid) continue;

		QVector2D pt2;
		float d = Util::pointSegmentDistanceXY(roads->graph[src]->getPt(), roads->graph[tgt]->getPt(), roads->graph[v]->getPt(), pt2);
		if (d < dist) {
			dist = d;
			min_e = *ei;
			closestPt = pt2;
		}
	}

	return min_e;
}

/**
 * Clean the road graph by removing all the invalid vertices and edges.
 */
void GraphUtil::clean(RoadGraph& roads) {
	RoadGraph temp;
	GraphUtil::copyRoads(roads, temp);

	roads.clear();

	removeIsolatedVertices(temp);

	QMap<RoadVertexDesc, RoadVertexDesc> conv;
	RoadVertexIter vi, vend;
	for (boost::tie(vi, vend) = boost::vertices(temp.graph); vi != vend; ++vi) {
		if (!temp.graph[*vi]->valid) continue;

		// Add a vertex
		RoadVertexPtr new_v = RoadVertexPtr(new RoadVertex(temp.graph[*vi]->getPt()));
		RoadVertexDesc new_v_desc = boost::add_vertex(roads.graph);
		roads.graph[new_v_desc] = new_v;	

		conv[*vi] = new_v_desc;
	}

	RoadEdgeIter ei, eend;
	for (boost::tie(ei, eend) = boost::edges(temp.graph); ei != eend; ++ei) {
		if (!temp.graph[*ei]->valid) continue;

		RoadVertexDesc src = boost::source(*ei, temp.graph);
		RoadVertexDesc tgt = boost::target(*ei, temp.graph);

		RoadVertexDesc new_src = conv[src];
		RoadVertexDesc new_tgt = conv[tgt];

		// Add an edge
		RoadEdgePtr new_e = RoadEdgePtr(new RoadEdge(*temp.graph[*ei]));
		std::pair<RoadEdgeDesc, bool> edge_pair = boost::add_edge(new_src, new_tgt, roads.graph);
		roads.graph[edge_pair.first] = new_e;
	}

	roads.setModified();
}

/**
 * Remove the vertices of degree of 2, and make it as a part of an edge.
 */
void GraphUtil::reduce(RoadGraph& roads) {
	bool actuallReduced = false;

	RoadVertexIter vi, vend;
	bool deleted = false;
	do {
		deleted = false;

		for (boost::tie(vi, vend) = boost::vertices(roads.graph); vi != vend; ++vi) {
			if (!roads.graph[*vi]->valid) continue;

			RoadVertexPtr v = roads.graph[*vi];

			if (getDegree(roads, *vi) == 2) {
				if (reduce(roads, *vi)) {
					deleted = true;
					actuallReduced = true;
					break;
				}
			}
		}
	} while (deleted);

	if (actuallReduced) {
		roads.setModified();
	}
}

/**
 * Remove the vertex of degree 2, and make it as a part of an edge.
 */
bool GraphUtil::reduce(RoadGraph& roads, RoadVertexDesc desc) {
	int count = 0;
	RoadVertexDesc vd[2];
	RoadEdgeDesc ed[2];
	RoadEdgePtr edges[2];

	RoadOutEdgeIter ei, ei_end;
	for (boost::tie(ei, ei_end) = out_edges(desc, roads.graph); ei != ei_end; ++ei) {
		if (!roads.graph[*ei]->valid) continue;

		vd[count] = boost::target(*ei, roads.graph);
		ed[count] = *ei;
		edges[count] = roads.graph[*ei];
		count++;
	}

	if (edges[0]->type != edges[1]->type) return false;
	//if (edges[0]->lanes != edges[1]->lanes) return false;

	// If the vertices form a triangle, don't remove it.
	//if (hasEdge(roads, vd[0], vd[1])) return false;

	RoadEdgePtr new_edge = RoadEdgePtr(new RoadEdge(edges[0]->type, edges[0]->lanes, edges[0]->oneWay));
	orderPolyLine(roads, ed[0], vd[0]);
	orderPolyLine(roads, ed[1], desc);
	
	for (int i = 0; i < edges[0]->getPolyLine().size(); i++) {
		new_edge->addPoint(edges[0]->getPolyLine()[i]);
	}
	for (int i = 1; i < edges[1]->getPolyLine().size(); i++) {
		new_edge->addPoint(edges[1]->getPolyLine()[i]);
	}
	std::pair<RoadEdgeDesc, bool> edge_pair = boost::add_edge(vd[0], vd[1], roads.graph);
	roads.graph[edge_pair.first] = new_edge;

	// invalidate the old edge
	roads.graph[ed[0]]->valid = false;
	roads.graph[ed[1]]->valid = false;

	// invalidate the vertex
	roads.graph[desc]->valid = false;

	return true;
}

/**
 * ノード間の距離が指定した距離よりも近い場合は、１つにしてしまう。
 * ノードとエッジ間の距離が、閾値よりも小さい場合も、エッジ上にノードを移してしまう。
 */
void GraphUtil::simplify(RoadGraph& roads, float dist_threshold) {
	RoadVertexIter vi, vend;
	for (boost::tie(vi, vend) = boost::vertices(roads.graph); vi != vend; ++vi) {
		if (!roads.graph[*vi]->valid) continue;

		while (true) {
			//RoadVertexDesc v2 = findNearestVertex(roads, roads->graph[*vi]->getPt(), *vi);
			RoadVertexDesc v2;
			if (!getVertex(roads, roads.graph[*vi]->pt, dist_threshold, *vi, v2)) break;
			//if ((roads->graph[v2]->getPt() - roads->graph[*vi]->getPt()).length() > dist_threshold) break;

			// define the new position
			QVector2D pt;
			int degree1 = getDegree(roads, *vi);
			int degree2 = getDegree(roads, v2);
			if (degree1 > 2 && degree2 <= 2) {
				pt = roads.graph[*vi]->pt;
			} else if (degree1 <= 2 && degree2 > 2) {
				pt = roads.graph[v2]->pt;
			} else {
				pt = (roads.graph[*vi]->pt + roads.graph[v2]->pt) / 2.0f;
			}

			moveVertex(roads, *vi, pt);
			snapVertex(roads, v2, *vi);
		}

		// find the closest vertex
		QVector2D closestPt;
		float dist;
		RoadEdgeDesc e = GraphUtil::findNearestEdge(&roads, *vi, dist, closestPt);
		if (dist < dist_threshold) {
			// move the vertex to the closest point on the edge
			GraphUtil::moveVertex(roads, *vi, closestPt);

			// retrieve src and tgt of the edge
			RoadVertexDesc src = boost::source(e, roads.graph);
			RoadVertexDesc tgt = boost::target(e, roads.graph);

			// invalidate the edge
			roads.graph[e]->valid = false;

			// update the edge
			if (!GraphUtil::hasEdge(roads, src, *vi)) {
				addEdge(roads, src, *vi, roads.graph[e]->type, roads.graph[e]->lanes, roads.graph[e]->oneWay);
			}
			if (!GraphUtil::hasEdge(roads, tgt, *vi)) {
				addEdge(roads, tgt, *vi, roads.graph[e]->type, roads.graph[e]->lanes, roads.graph[e]->oneWay);
			}
		}
	}

	roads.setModified();
}

/**
 * ノード間の距離が指定した距離よりも近い場合は、１つにしてしまう。
 * ノードとエッジ間の距離が、閾値よりも小さい場合も、エッジ上にノードを移してしまう。
 */
void GraphUtil::simplify2(RoadGraph& roads, float dist_threshold) {
	float threshold2 = dist_threshold * dist_threshold;

	RoadGraph temp;
	copyRoads(roads, temp);

	roads.clear();

	// 全ての頂点同士で、近いものをグループ化する
	int group_id = 0;
	std::vector<QVector2D> group_centers;
	std::vector<int> group_nums;
	QHash<RoadVertexDesc, int> groups;
	RoadVertexIter vi, vend;
	for (boost::tie(vi, vend) = boost::vertices(temp.graph); vi != vend; ++vi) {
		if (!temp.graph[*vi]->valid) continue;

		float min_dist = std::numeric_limits<float>::max();
		int min_group_id = -1;
		for (int i = 0; i < group_centers.size(); i++) {
			float dist = (group_centers[i] - temp.graph[*vi]->pt).lengthSquared();
			if (dist < min_dist) {
				min_dist = dist;
				min_group_id = i;
			}
		}

		if (min_group_id >= 0 && min_dist < threshold2) {
			group_centers[min_group_id] = group_centers[min_group_id] * group_nums[min_group_id] + temp.graph[*vi]->pt;
			group_nums[min_group_id]++;
			group_centers[min_group_id] /= group_nums[min_group_id];
		} else {
			min_group_id = group_centers.size();
			group_centers.push_back(temp.graph[*vi]->pt);
			group_nums.push_back(1);
		}
		
		groups[*vi] = min_group_id;
	}

	// エッジを登録する
	QHash<int, RoadVertexDesc> conv;	// group center ⇒ 実際の頂点desc
	RoadGraph temp2;
	for (int i = 0; i < group_centers.size(); i++) {
		RoadVertexDesc v = boost::add_vertex(temp2.graph);
		temp2.graph[v] = RoadVertexPtr(new RoadVertex(group_centers[i]));
		conv[i] = v;
	}

	RoadEdgeIter ei, eend;
	for (boost::tie(ei, eend) = boost::edges(temp.graph); ei != eend; ++ei) {
		if (!temp.graph[*ei]->valid) continue;

		RoadVertexDesc src = boost::source(*ei, temp.graph);
		RoadVertexDesc tgt = boost::target(*ei, temp.graph);

		RoadVertexDesc new_src = conv[groups[src]];
		RoadVertexDesc new_tgt = conv[groups[tgt]];

		// エッジが点に縮退する場合は、スキップ
		if (new_src == new_tgt) continue;

		// 既にエッジがあれば、スキップ
		if (hasEdge(temp2, new_src, new_tgt)) continue;

		// エッジを追加
		RoadEdgeDesc e = addEdge(temp2, new_src, new_tgt, temp.graph[*ei]);
		moveEdge(temp2, e, temp2.graph[new_src]->pt, temp2.graph[new_tgt]->pt);
	}

	copyRoads(temp2, roads);

	roads.setModified();
}

/**
 * エッジのポリゴンが3つ以上で構成されている場合、中間点を全てノードとして登録する。
 */
void GraphUtil::normalize(RoadGraph& roads) {
	RoadEdgeIter ei, eend;
	for (boost::tie(ei, eend) = boost::edges(roads.graph); ei != eend; ++ei) {
		if (!roads.graph[*ei]->valid) continue;
		if (roads.graph[*ei]->getPolyLine().size() <= 2) continue;

		// invalidate the edge
		roads.graph[*ei]->valid = false;

		RoadVertexDesc src = boost::source(*ei, roads.graph);
		RoadVertexDesc tgt = boost::target(*ei, roads.graph);

		RoadVertexDesc prev_desc;
		RoadVertexDesc last_desc;
		if ((roads.graph[src]->getPt() - roads.graph[*ei]->getPolyLine()[0]).lengthSquared() < (roads.graph[tgt]->getPt() - roads.graph[*ei]->getPolyLine()[0]).lengthSquared()) {
			prev_desc = src;
			last_desc = tgt;
		} else {
			prev_desc = tgt;
			last_desc = src;
		}

		for (int i = 1; i < roads.graph[*ei]->getPolyLine().size() - 1; i++) {
			// add all the points along the poly line as vertices
			RoadVertexPtr new_v = RoadVertexPtr(new RoadVertex(roads.graph[*ei]->getPolyLine()[i]));
			RoadVertexDesc new_v_desc = boost::add_vertex(roads.graph);
			roads.graph[new_v_desc] = new_v;

			// Add an edge
			addEdge(roads, prev_desc, new_v_desc, roads.graph[*ei]->type, roads.graph[*ei]->lanes, roads.graph[*ei]->oneWay);

			prev_desc = new_v_desc;
		}

		// Add the last edge
		addEdge(roads, prev_desc, last_desc, roads.graph[*ei]->type, roads.graph[*ei]->lanes, roads.graph[*ei]->oneWay);
	}
}

/**
 * エッジを指定されたstep_sizeで分割し、それぞれの点を全て頂点として登録する。
 */
void GraphUtil::normalize(RoadGraph& roads, float step_size) {
	RoadGraph temp;
	copyRoads(roads, temp);

	RoadEdgeIter ei, eend;
	for (boost::tie(ei, eend) = boost::edges(roads.graph); ei != eend; ++ei) {
		roads.graph[*ei]->valid = false;
	}
	
	for (boost::tie(ei, eend) = boost::edges(temp.graph); ei != eend; ++ei) {
		if (!temp.graph[*ei]->valid) continue;

		RoadVertexDesc src = boost::source(*ei, temp.graph);
		RoadVertexDesc tgt = boost::target(*ei, temp.graph);

		std::vector<QVector2D> line = finerEdge(temp, *ei, step_size);

		RoadVertexDesc prev_desc;
		RoadVertexDesc last_desc;
		if ((temp.graph[src]->pt - line[0]).lengthSquared() < (temp.graph[tgt]->pt - line[0]).lengthSquared()) {
			prev_desc = src;
			last_desc = tgt;
		} else {
			prev_desc = tgt;
			last_desc = src;
		}

		for (int i = 1; i < line.size() - 1; i++) {
			// add all the points along the poly line as vertices
			RoadVertexDesc new_v_desc = boost::add_vertex(roads.graph);
			roads.graph[new_v_desc] = RoadVertexPtr(new RoadVertex(line[i]));

			// Add an edge
			addEdge(roads, prev_desc, new_v_desc, temp.graph[*ei]->type, temp.graph[*ei]->lanes, temp.graph[*ei]->oneWay);

			prev_desc = new_v_desc;
		}

		// Add the last edge
		addEdge(roads, prev_desc, last_desc, temp.graph[*ei]->type, temp.graph[*ei]->lanes, temp.graph[*ei]->oneWay);
	}
}

/**
 * start頂点と接続されているノードのみ有効とし、それ以外のノード、およびエッジを、全て無効にする。
 * 本実装では、事前の有効・無効フラグを考慮していない。要検討。。。
 */
void GraphUtil::singlify(RoadGraph& roads) {
	int max_size = 0;
	RoadVertexDesc start;

	// 最も大きいかたまり（接続されている）の道路網を探し出す
	RoadVertexIter vi, vend;
	for (boost::tie(vi, vend) = boost::vertices(roads.graph); vi != vend; ++vi) {
		if (!roads.graph[*vi]->valid) continue;

		int size = getNumConnectedVertices(roads, *vi);
		if (size > max_size) {
			max_size = size;
			start = *vi;
		}
	}

	RoadGraph new_roads;

	std::list<RoadVertexDesc> queue;
	queue.push_back(start);

	QMap<RoadVertexDesc, RoadVertexDesc> conv;

	// Add the starting vertex
	RoadVertexPtr new_v = RoadVertexPtr(new RoadVertex(roads.graph[start]->getPt()));
	RoadVertexDesc new_v_desc = boost::add_vertex(new_roads.graph);
	new_roads.graph[new_v_desc] = new_v;

	conv[start] = new_v_desc;

	std::list<RoadVertexDesc> queue2;
	queue2.push_back(new_v_desc);

	while (!queue.empty()) {
		RoadVertexDesc v_desc = queue.front();
		queue.pop_front();

		RoadVertexDesc new_v_desc = queue2.front();
		queue2.pop_front();


		RoadOutEdgeIter oei, oeend;
		for (boost::tie(oei, oeend) = boost::out_edges(v_desc, roads.graph); oei != oeend; ++oei) {
			if (!roads.graph[*oei]->valid) continue;

			RoadVertexDesc u_desc = boost::target(*oei, roads.graph);
			if (!roads.graph[u_desc]->valid) continue;

			RoadVertexDesc new_u_desc;

			if (conv.contains(u_desc)) {
				new_u_desc = conv[u_desc];
			} else {
				// Add a vertex
				RoadVertexPtr new_u = RoadVertexPtr(new RoadVertex(roads.graph[u_desc]->getPt()));
				new_u_desc = boost::add_vertex(new_roads.graph);
				new_roads.graph[new_u_desc] = new_u;
			}

			// Add an edge
			if (!hasEdge(new_roads, new_v_desc, new_u_desc)) {
				addEdge(new_roads, new_v_desc, new_u_desc, roads.graph[*oei]);
			}

			if (!conv.contains(u_desc)) {
				conv[u_desc] = new_u_desc;
				queue.push_back(u_desc);
				queue2.push_back(new_u_desc);
			}
		}
	}

	// copy new_roads to roads
	copyRoads(new_roads, roads);
}

/**
 * Convert the road graph to a planar graph.
 */
void GraphUtil::planarify(RoadGraph& roads) {
	clock_t start, end;
	start = clock();

	bool split = true;

	while (split) {
		split = planarifyOne(roads);
	}

	end = clock();
	std::cout << "planarify: " << (double)(end-start)/CLOCKS_PER_SEC << std::endl;
}

/**
 * Convert one intersected road segments to a planar one by adding the intersection, and return true.
 * If the road segments do not intersect, return false.
 */
bool GraphUtil::planarifyOne(RoadGraph& roads) {
	RoadEdgeIter ei, eend;
	for (boost::tie(ei, eend) = boost::edges(roads.graph); ei != eend; ++ei) {
		RoadEdgePtr e = roads.graph[*ei];
		if (!e->valid) continue;

		RoadVertexDesc src = boost::source(*ei, roads.graph);
		RoadVertexDesc tgt = boost::target(*ei, roads.graph);

		RoadEdgeIter ei2, eend2;
		for (boost::tie(ei2, eend2) = boost::edges(roads.graph); ei2 != eend2; ++ei2) {
			RoadEdgePtr e2 = roads.graph[*ei2];
			if (!e2->valid) continue;

			RoadVertexDesc src2 = boost::source(*ei2, roads.graph);
			RoadVertexDesc tgt2 = boost::target(*ei2, roads.graph);

			//if ((src == src2 && tgt == tgt2) || (src == tgt2 && tgt == src2)) continue;
			if (src == src2 || src == tgt2 || tgt == src2 || tgt == tgt2) continue;

			for (int i = 0; i < e->polyLine.size() - 1; i++) {
				for (int j = 0; j < e2->polyLine.size() - 1; j++) {
					float tab, tcd;
					QVector2D intPt;
					if (Util::segmentSegmentIntersectXY(e->polyLine[i], e->polyLine[i+1], e2->polyLine[j], e2->polyLine[j+1], &tab, &tcd, true, intPt)) {
						// エッジの端、ぎりぎりで、交差する場合は、交差させない
						if ((roads.graph[src]->pt - intPt).length() < 10 || (roads.graph[tgt]->pt - intPt).length() < 10 || (roads.graph[src2]->pt - intPt).length() < 10 || (roads.graph[tgt2]->pt - intPt).length() < 10) continue;

						// 交点をノードとして登録
						RoadVertexPtr new_v = RoadVertexPtr(new RoadVertex(intPt));
						RoadVertexDesc new_v_desc = boost::add_vertex(roads.graph);
						roads.graph[new_v_desc] = new_v;

						// もともとのエッジを無効にする
						roads.graph[*ei]->valid = false;
						roads.graph[*ei2]->valid = false;

						// 新たなエッジを追加する
						addEdge(roads, src, new_v_desc, roads.graph[*ei]->type, roads.graph[*ei]->lanes, roads.graph[*ei]->oneWay);
						addEdge(roads, new_v_desc, tgt, roads.graph[*ei]->type, roads.graph[*ei]->lanes, roads.graph[*ei]->oneWay);

						addEdge(roads, src2, new_v_desc, roads.graph[*ei2]->type, roads.graph[*ei2]->lanes, roads.graph[*ei2]->oneWay);
						addEdge(roads, new_v_desc, tgt2, roads.graph[*ei2]->type, roads.graph[*ei2]->lanes, roads.graph[*ei2]->oneWay);

						return true;
					}
				}
			}
		}
	}

	return false;
}

/**
 * 道路網をスケルトン化する。
 * 具体的には、オリジナル道路網で、degreeが1の頂点と、その隣接エッジを無効にする。
 * 注意：頂点を削除した結果、新たにdegreeが1となる頂点は、その対象ではない。
 */
void GraphUtil::skeltonize(RoadGraph* roads) {
	QList<RoadVertexDesc> list;

	// 削除対象となる頂点リストを取得
	RoadVertexIter vi, vend;
	for (boost::tie(vi, vend) = boost::vertices(roads->graph); vi != vend; ++vi) {
		if (!roads->graph[*vi]->valid) continue;

		if (getDegree(*roads, *vi) == 1) {
			list.push_back(*vi);
		}
	}

	for (int i = 0; i < list.size(); i++) {
		// 隣接エッジを無効にする
		RoadOutEdgeIter ei, eend;
		for (boost::tie(ei, eend) = boost::out_edges(list[i], roads->graph); ei != eend; ++ei) {
			if (!roads->graph[*ei]->valid) continue;

			roads->graph[*ei]->valid = false;
		}

		// 頂点を無効にする
		roads->graph[list[i]]->valid = false;
	}
}

/**
 * Rotate the road graph by theta [rad].
 */
void GraphUtil::rotate(RoadGraph& roads, float theta, const QVector2D& rotationCenter) {
	// Rotate vertices
	RoadVertexIter vi, vend;
	for (boost::tie(vi, vend) = boost::vertices(roads.graph); vi != vend; ++vi) {
		QVector2D pos = roads.graph[*vi]->pt;

		roads.graph[*vi]->pt.setX(cosf(theta) * (pos.x() - rotationCenter.x()) - sinf(theta) * (pos.y() - rotationCenter.y()) + rotationCenter.x());
		roads.graph[*vi]->pt.setY(sinf(theta) * (pos.x() - rotationCenter.x()) + cosf(theta) * (pos.y() - rotationCenter.y()) + rotationCenter.y());
	}

	// Rotate edges
	RoadEdgeIter ei, eend;
	for (boost::tie(ei, eend) = boost::edges(roads.graph); ei != eend; ++ei) {
		for (int i = 0; i < roads.graph[*ei]->polyLine.size(); i++) {
			QVector2D pos = roads.graph[*ei]->polyLine[i];
			roads.graph[*ei]->polyLine[i].setX(cosf(theta) * (pos.x() - rotationCenter.x()) - sinf(theta) * (pos.y() - rotationCenter.y()) + rotationCenter.x());
			roads.graph[*ei]->polyLine[i].setY(sinf(theta) * (pos.x() - rotationCenter.x()) + cosf(theta) * (pos.y() - rotationCenter.y()) + rotationCenter.y());
		}
	}

	roads.setModified();
}

/**
 * Translate the road graph.
 */
void GraphUtil::translate(RoadGraph& roads, const QVector2D& offset) {
	// Translate vertices
	RoadVertexIter vi, vend;
	for (boost::tie(vi, vend) = boost::vertices(roads.graph); vi != vend; ++vi) {
		roads.graph[*vi]->pt += offset;
	}

	// Translate edges
	RoadEdgeIter ei, eend;
	for (boost::tie(ei, eend) = boost::edges(roads.graph); ei != eend; ++ei) {
		for (int i = 0; i < roads.graph[*ei]->polyLine.size(); i++) {
			roads.graph[*ei]->polyLine[i] += offset;
		}
	}

	roads.setModified();
}

/**
 * Scale the road graph
 */
void GraphUtil::scale(RoadGraph& roads, const BBox& bbox1, const BBox& bbox2) {
	float scaleX = bbox2.dx() / bbox1.dx();
	float scaleY = bbox2.dy() / bbox1.dy();

	// Translate vertices
	RoadVertexIter vi, vend;
	for (boost::tie(vi, vend) = boost::vertices(roads.graph); vi != vend; ++vi) {
		QVector2D vec = roads.graph[*vi]->pt - bbox1.minPt;
		float x = vec.x() * scaleX + bbox2.minPt.x();
		float y = vec.y() * scaleY + bbox2.minPt.y();
		roads.graph[*vi]->pt.setX(x);
		roads.graph[*vi]->pt.setY(y);
	}

	// Translate edges
	RoadEdgeIter ei, eend;
	for (boost::tie(ei, eend) = boost::edges(roads.graph); ei != eend; ++ei) {
		for (int i = 0; i < roads.graph[*ei]->polyLine.size(); i++) {
			QVector2D vec = roads.graph[*ei]->polyLine[i] - bbox1.minPt;
			float x = vec.x() * scaleX + bbox2.minPt.x();
			float y = vec.y() * scaleY + bbox2.minPt.y();

			roads.graph[*ei]->polyLine[i].setX(x);
			roads.graph[*ei]->polyLine[i].setY(y);
		}
	}

	roads.setModified();
}

/**
 * 道路網をグリッド型に無理やり変換する。
 * 頂点startから開始し、エッジの方向に基づいて、上下左右方向に、ノードを広げていくイメージ。
 */
RoadGraph* GraphUtil::convertToGridNetwork(RoadGraph* roads, RoadVertexDesc start) {
 	RoadGraph* new_roads = new RoadGraph();

	QList<RoadVertexDesc> queue;
	queue.push_back(start);

	QList<RoadVertexDesc> visited;
	visited.push_back(start);

	// スタート頂点を追加
	RoadVertexPtr v = RoadVertexPtr(new RoadVertex(QVector2D(0, 0)));
	RoadVertexDesc v_desc = boost::add_vertex(new_roads->graph);
	new_roads->graph[v_desc] = v;
	
	QList<RoadVertexDesc> new_queue;
	new_queue.push_back(v_desc);

	while (!queue.empty()) {
		RoadVertexDesc v_desc = queue.front();
		queue.pop_front();
		RoadVertexDesc new_v_desc = new_queue.front();
		new_queue.pop_front();

		RoadOutEdgeIter ei, eend;
		for (boost::tie(ei, eend) = boost::out_edges(v_desc, roads->graph); ei != eend; ++ei) {
			if (!roads->graph[*ei]->valid) continue;

			RoadVertexDesc u_desc = boost::target(*ei, roads->graph);
			if (!roads->graph[u_desc]->valid) continue;

			// オリジナルの道路網で、エッジの方向を取得
			QVector2D dir = roads->graph[u_desc]->getPt() - roads->graph[v_desc]->getPt();

			QVector2D pos;
			if (diffAngle(dir, QVector2D(1, 0)) < M_PI * 0.25f) { // X軸正方向
				pos = new_roads->graph[new_v_desc]->getPt() + QVector2D(100.0f, 0.0f);
			} else if (diffAngle(dir, QVector2D(0, 1)) < M_PI * 0.25f) { // Y軸正方向
				pos = new_roads->graph[new_v_desc]->getPt() + QVector2D(0.0f, 100.0f);
			} else if (diffAngle(dir, QVector2D(-1, 0)) < M_PI * 0.25f) { // X軸負方向
				pos = new_roads->graph[new_v_desc]->getPt() + QVector2D(-100.0f, 0.0f);
			} else if (diffAngle(dir, QVector2D(0, -1)) < M_PI * 0.25f) { // Y軸負方向
				pos = new_roads->graph[new_v_desc]->getPt() + QVector2D(0.0f, -100.0f);
			} 
			
			RoadVertexDesc new_u_desc;
			if (!getVertex(*new_roads, pos, 0.0f, new_u_desc)) {
				// 頂点を追加
				RoadVertexPtr new_u = RoadVertexPtr(new RoadVertex(pos));
				new_u_desc = boost::add_vertex(new_roads->graph);
				new_roads->graph[new_u_desc] = new_u;
			}

			if (!hasEdge(*new_roads, new_v_desc, new_u_desc)) {
				// エッジを追加
				addEdge(*new_roads, new_v_desc, new_u_desc, roads->graph[*ei]->type, roads->graph[*ei]->lanes, roads->graph[*ei]->oneWay);
			}

			if (!visited.contains(u_desc)) {
				visited.push_back(u_desc);

				queue.push_back(u_desc);
				new_queue.push_back(new_u_desc);
			}
		}
	}

	return new_roads;
}

/**
 * 道路網が指定されたareaにおさまるようにする。
 */
void GraphUtil::scaleToBBox(RoadGraph& roads, BBox& area) {
	BBox curArea = getAABoundingBox(roads);
	QVector2D scale(area.dx() / curArea.dx(), area.dy() / curArea.dy());

	// 全ての頂点を、指定したareaに入るよう移動する
	RoadVertexIter vi, vend;
	for (boost::tie(vi, vend) = boost::vertices(roads.graph); vi != vend; ++vi) {
		if (!roads.graph[*vi]->valid) continue;

		QVector2D pos = roads.graph[*vi]->pt - curArea.midPt();
		pos.setX(pos.x() * scale.x());
		pos.setY(pos.y() * scale.y());
		pos += area.midPt();

		RoadOutEdgeIter ei, eend;
		for (boost::tie(ei, eend) = boost::out_edges(*vi, roads.graph); ei != eend; ++ei) {
			if (!roads.graph[*ei]->valid) continue;

			RoadVertexDesc tgt = boost::target(*ei, roads.graph);
			if (!roads.graph[tgt]->valid) continue;

			moveEdge(roads, *ei, pos, roads.graph[tgt]->pt);
		}

		roads.graph[*vi]->pt = pos;
	}
}

/**
 * バネの原理を使って、道路網のエッジの長さを均等にする。
 *
 * まず、全エッジの平均長を計算し、これをエッジの本来の長さと仮定する。
 * 次に、各頂点について、各隣接エッジの長さの、本来長からの変形量を使って、各頂点にかかる仮想的な力を計算する。
 * 最後に、この仮想的な力に、適当なdTをかけた値を使って、各頂点を移動させる。
 * これを、一定数、繰り返す。（終了条件について、要検討）
 */
void GraphUtil::normalizeBySpring(RoadGraph* roads, BBox& area) {
	// バネの原理を使って、各エッジの長さを均等にする
	float step = 0.03f;

	//for (int i = 0; i < 1000; i++) {
	for (int i = 0; i < 1; i++) {
		float avg_edge_length = computeAvgEdgeLength(*roads);

		RoadVertexIter vi, vend;
		for (boost::tie(vi, vend) = boost::vertices(roads->graph); vi != vend; ++vi) {
			if (!roads->graph[*vi]->valid) continue;

			// 頂点にかかる力を計算する
			QVector2D force;

			// 隣接エッジからは、引っ張られる
			RoadOutEdgeIter ei, eend;
			for (boost::tie(ei, eend) = boost::out_edges(*vi, roads->graph); ei != eend; ++ei) {
				if (!roads->graph[*ei]->valid) continue;

				RoadVertexDesc src = *vi;
				RoadVertexDesc tgt = boost::target(*ei, roads->graph);

				QVector2D dir = roads->graph[tgt]->pt - roads->graph[src]->pt;
				float x = (roads->graph[tgt]->pt - roads->graph[src]->pt).length() - avg_edge_length * 1.0f;
				
				force += dir.normalized() * x;
			}

			// 接続されていない、近隣の頂点からは、反発力を受ける
			RoadVertexIter vi2, vend2;
			for (boost::tie(vi2, vend2) = boost::vertices(roads->graph); vi2 != vend2; ++vi2) {
				if (!roads->graph[*vi2]->valid) continue;

				if (hasEdge(*roads, *vi, *vi2)) continue;

				/*
				QVector2D dir = roads->graph[*vi]->pt - roads->graph[*vi2]->pt;
				float x = avg_edge_length * 1.44f - (roads->graph[*vi2]->pt - roads->graph[*vi]->pt).length();

				if (x > 0.0f) {
					force += dir.normalized() * x;
				}*/

				RoadVertexDesc src = *vi;
				RoadVertexDesc tgt = *vi2;

				QVector2D dir = roads->graph[tgt]->pt - roads->graph[src]->pt;
				float x = (roads->graph[tgt]->pt - roads->graph[src]->pt).length() - avg_edge_length * 1.0f;
				
				force += dir.normalized() * x * 0.02f;	// ←　この係数は、微調整が必要。。。
			}

			// 移動後の位置が、指定された範囲内か、チェック
			QVector2D pos = roads->graph[*vi]->pt + force * step;
			if (area.contains(pos)) {
				// 頂点をする
				roads->graph[*vi]->pt = pos;

				// エッジも移動する
				for (boost::tie(ei, eend) = boost::out_edges(*vi, roads->graph); ei != eend; ++ei) {
					if (!roads->graph[*ei]->valid) continue;

					RoadVertexDesc tgt = boost::target(*ei, roads->graph);

					moveEdge(*roads, *ei, pos, roads->graph[tgt]->pt);
				}
			}
		}
	}
}

/**
 * Remove duplicated edges if there are more than one edges between two vertices.
 */
bool GraphUtil::removeDuplicateEdges(RoadGraph* roads) {
	bool removed = false;

	RoadVertexIter vi, vend;
	for (boost::tie(vi, vend) = boost::vertices(roads->graph); vi != vend; ++vi) {
		if (!roads->graph[*vi]->valid) continue;

		QList<RoadVertexDesc> targets;

		RoadOutEdgeIter ei, eend;
		for (boost::tie(ei, eend) = boost::out_edges(*vi, roads->graph); ei != eend; ++ei) {
			if (!roads->graph[*ei]->valid) continue;

			RoadVertexDesc tgt = boost::target(*ei, roads->graph);

			if (targets.contains(tgt)) {
				roads->graph[*ei]->valid = false;
				removed = true;
			} else {
				targets.push_back(tgt);
			}
		}
	}
	
	return removed;
}

/**
 * snap the dead-end edges to the near vertices.
 * First, for vertices of degree more than 1, find the closest vertex.
 * If no such vertex exists, for vertices of degree 1, find the cloest vertex.
 */
void GraphUtil::snapDeadendEdges(RoadGraph& roads, float threshold) {
	float min_angle_threshold = 0.34f;

	RoadVertexIter vi, vend;
	for (boost::tie(vi, vend) = boost::vertices(roads.graph); vi != vend; ++vi) {
		if (!roads.graph[*vi]->valid) continue;

		// only for the vertices of degree more than 1
		if (GraphUtil::getDegree(roads, *vi) != 1) continue;

		// retrieve the tgt vertex
		RoadVertexDesc tgt;
		RoadEdgeDesc e_desc;
		RoadOutEdgeIter ei, eend;
		for (boost::tie(ei, eend) = boost::out_edges(*vi, roads.graph); ei != eend; ++ei) {
			if (!roads.graph[*ei]->valid) continue;

			tgt = boost::target(*ei, roads.graph);
			e_desc = *ei;
			break;
		}

		// find the closest vertex
		RoadVertexDesc nearest_desc;
		float min_dist = std::numeric_limits<float>::max();

		RoadVertexIter vi2, vend2;
		for (boost::tie(vi2, vend2) = boost::vertices(roads.graph); vi2 != vend2; ++vi2) {
			if (!roads.graph[*vi2]->valid) continue;
			if (*vi2 == *vi) continue;
			if (*vi2 == tgt) continue;
			if (GraphUtil::getDegree(roads, *vi2) == 1) continue;

			float dist = (roads.graph[*vi2]->pt - roads.graph[*vi]->pt).length();

			// 近接頂点が、*viよりもtgtの方に近い場合は、当該近接頂点は対象からはずす
			float dist2 = (roads.graph[*vi2]->pt - roads.graph[tgt]->pt).length();
			if (dist > dist2) continue;

			if (dist < min_dist) {
				nearest_desc = *vi2;
				min_dist = dist;
			}

			// *vi2から出るエッジとのなす角度の最小値が小さすぎる場合は、対象からはずす
			float min_angle = std::numeric_limits<float>::max();
			RoadOutEdgeIter ei, eend;
			for (boost::tie(ei, eend) = boost::out_edges(*vi2, roads.graph); ei != eend; ++ei) {
				if (!roads.graph[*ei]->valid) continue;

				RoadVertexDesc tgt2 = boost::target(*ei, roads.graph);
				float angle = GraphUtil::diffAngle(roads.graph[*vi]->pt - roads.graph[tgt]->pt, roads.graph[*vi2]->pt - roads.graph[tgt2]->pt);
				if (angle < min_angle) {
					min_angle = angle;
				}
			}
			if (min_angle < min_angle_threshold) continue;
		}

		// If no such vertex exists, find the closest vertex of degree 1.
		if (min_dist > threshold) {
			for (boost::tie(vi2, vend2) = boost::vertices(roads.graph); vi2 != vend2; ++vi2) {
				if (!roads.graph[*vi2]->valid) continue;
				if (*vi2 == *vi) continue;
				if (*vi2 == tgt) continue;
				if (GraphUtil::getDegree(roads, *vi2) != 1) continue;

				// Find the edge of the vertex
				RoadEdgeDesc e_desc2;
				RoadOutEdgeIter ei, eend;
				for (boost::tie(ei, eend) = boost::out_edges(*vi2, roads.graph); ei != eend; ++ei) {
					if (!roads.graph[*ei]->valid) continue;

					e_desc2 = *ei;
					break;
				}

				// If th edge is too short, skip it.
				//if (roads->graph[e_desc2]->getLength() < deadend_removal_threshold) continue;

				float dist = (roads.graph[*vi2]->pt - roads.graph[*vi]->pt).length();

				// 近接頂点が、*viよりもtgtの方に近い場合は、当該近接頂点は対象からはずす
				float dist2 = (roads.graph[*vi2]->pt - roads.graph[tgt]->pt).length();
				if (dist > dist2) continue;

				if (dist < min_dist) {
					nearest_desc = *vi2;
					min_dist = dist;
				}

				// *vi2から出るエッジとのなす角度の最小値が小さすぎる場合は、対象からはずす
				float min_angle = std::numeric_limits<float>::max();
				for (boost::tie(ei, eend) = boost::out_edges(*vi2, roads.graph); ei != eend; ++ei) {
					if (!roads.graph[*ei]->valid) continue;

					RoadVertexDesc tgt2 = boost::target(*ei, roads.graph);
					float angle = GraphUtil::diffAngle(roads.graph[*vi]->pt - roads.graph[tgt]->pt, roads.graph[*vi2]->pt - roads.graph[tgt2]->pt);
					if (angle < min_angle) {
						min_angle = angle;
					}
				}
				if (min_angle < min_angle_threshold) continue;
			}
		}

		// 当該頂点と近接頂点との距離が、snap_deadend_threshold未満か？
		if (min_dist <= threshold) {
			// 一旦、古いエッジを、近接頂点にスナップするよう移動する
			GraphUtil::moveEdge(roads, e_desc, roads.graph[nearest_desc]->pt, roads.graph[tgt]->pt);

			if (GraphUtil::hasEdge(roads, nearest_desc, tgt, false)) {
				// もともとエッジがあるが無効となっている場合、それを有効にし、エッジのポリラインを更新する
				RoadEdgeDesc new_e_desc = GraphUtil::getEdge(roads, nearest_desc, tgt, false);
				roads.graph[new_e_desc]->valid = true;
				roads.graph[new_e_desc]->polyLine = roads.graph[e_desc]->polyLine;
			} else {
				// 該当頂点間にエッジがない場合は、新しいエッジを追加する
				GraphUtil::addEdge(roads, nearest_desc, tgt, roads.graph[e_desc]);
			}

			// 古いエッジを無効にする
			roads.graph[e_desc]->valid = false;

			// 当該頂点を無効にする
			roads.graph[*vi]->valid = false;
		}
	}
}

/**
 * 指定されたdegreeの頂点について、近くに頂点がある場合は、Snapさせる。
 * ただし、Snap対象となるエッジとのなす角度がmin_angle_threshold以下の場合は、対象外。
 */
void GraphUtil::snapDeadendEdges2(RoadGraph& roads, int degree, float threshold) {
	float angle_threshold = 0.34f;

	RoadVertexIter vi, vend;
	for (boost::tie(vi, vend) = boost::vertices(roads.graph); vi != vend; ++vi) {
		if (!roads.graph[*vi]->valid) continue;

		// 指定されたdegree以外の頂点は、対象外
		if (GraphUtil::getDegree(roads, *vi) != degree) continue;

		// 当該頂点と接続されている唯一の頂点を取得
		RoadVertexDesc tgt;
		RoadEdgeDesc e_desc;
		RoadOutEdgeIter ei, eend;
		for (boost::tie(ei, eend) = boost::out_edges(*vi, roads.graph); ei != eend; ++ei) {
			if (!roads.graph[*ei]->valid) continue;

			tgt = boost::target(*ei, roads.graph);
			e_desc = *ei;
			break;
		}

		// 近接頂点を探す
		RoadVertexDesc nearest_desc;
		float min_dist = std::numeric_limits<float>::max();

		RoadVertexIter vi2, vend2;
		for (boost::tie(vi2, vend2) = boost::vertices(roads.graph); vi2 != vend2; ++vi2) {
			if (!roads.graph[*vi2]->valid) continue;
			if (*vi2 == *vi) continue;
			if (*vi2 == tgt) continue;

			float dist = (roads.graph[*vi2]->pt - roads.graph[*vi]->pt).length();

			// 近接頂点が、*viよりもtgtの方に近い場合は、当該近接頂点は対象からはずす
			//float dist2 = (roads->graph[*vi2]->pt - roads->graph[tgt]->pt).length();
			//if (dist > dist2) continue;

			if (dist < min_dist) {
				nearest_desc = *vi2;
				min_dist = dist;
			}
		}
		
		// 近接頂点が、*viよりもtgtの方に近い場合は、スナップしない
		if ((roads.graph[nearest_desc]->pt - roads.graph[tgt]->pt).length() < (roads.graph[*vi]->pt - roads.graph[tgt]->pt).length()) continue;

		// スナップによるエッジの角度変化が大きすぎる場合は、対象からはずす
		float diff_angle = diffAngle(roads.graph[*vi]->pt - roads.graph[tgt]->pt, roads.graph[nearest_desc]->pt - roads.graph[tgt]->pt);
		if (diff_angle > angle_threshold) continue;

		// tgtとスナップ先との間に既にエッジがある場合は、スナップしない
		if (hasEdge(roads, tgt, nearest_desc)) continue;

		// 当該頂点と近接頂点との距離が、threshold以下なら、スナップする
		if (min_dist <= threshold) {
			snapVertex(roads, *vi, nearest_desc);
		}
	}
}

/**
 * Remove too short dead-end edges unless it has a pair.
 */
void GraphUtil::removeShortDeadend(RoadGraph& roads, float threshold) {
	bool actuallyDeleted = false;

	bool deleted = true;
	while (deleted) {
		deleted = false;

		RoadVertexIter vi, vend;
		for (boost::tie(vi, vend) = boost::vertices(roads.graph); vi != vend; ++vi) {
			if (!roads.graph[*vi]->valid) continue;

			if (getDegree(roads, *vi) > 1) continue;

			RoadOutEdgeIter ei, eend;
			for (boost::tie(ei, eend) = boost::out_edges(*vi, roads.graph); ei != eend; ++ei) {
				if (!roads.graph[*ei]->valid) continue;

				// If the edge has a pair, don't remove it.
				if (roads.graph[*ei]->fullyPaired) continue;

				RoadVertexDesc tgt = boost::target(*ei, roads.graph);

				// invalidate the too short edge, and invalidate the dead-end vertex.
				if (roads.graph[*ei]->getLength() < threshold) {
					roads.graph[*vi]->valid = false;
					roads.graph[*ei]->valid = false;
					deleted = true;
					actuallyDeleted = true;
				}
			}
		}
	}

	if (actuallyDeleted) roads.setModified();
}

/**
 * ２つのデータリストの差の最小値を返却する。
 * 各データは、１回のみ比較に使用できる。
 */
float GraphUtil::computeMinDiffAngle(std::vector<float> *data1, std::vector<float> *data2) {
	float ret = 0.0f;

	if (data1->size() <= data2->size()) {
		std::vector<bool> paired2;
		for (int i = 0; i < data2->size(); i++) {
			paired2.push_back(false);
		}

		for (int i = 0; i < data1->size(); i++) {
			float min_diff = std::numeric_limits<float>::max();


			int min_id = -1;
			for (int j = 0; j < data2->size(); j++) {
				if (paired2[j]) continue;

				float diff = diffAngle(data1->at(i), data2->at(j));
				if (diff < min_diff) {
					min_diff = diff;
					min_id = j;
				}
			}

			paired2[min_id] = true;
			ret += min_diff;
		}
	} else {
		std::vector<bool> paired1;
		for (int i = 0; i < data1->size(); i++) {
			paired1.push_back(false);
		}

		for (int i = 0; i < data2->size(); i++) {
			float min_diff = std::numeric_limits<float>::max();


			int min_id = -1;
			for (int j = 0; j < data1->size(); j++) {
				if (paired1[j]) continue;

				float diff = fabs(diffAngle(data2->at(i), data1->at(j)));
				if (diff < min_diff) {
					min_diff = diff;
					min_id = j;
				}
			}

			paired1[min_id] = true;
			ret += min_diff;
		}
	}

	return ret;
}

/**
 * 角度を正規化し、[-PI , PI]の範囲にする。
 */
float GraphUtil::normalizeAngle(float angle) {
	// まずは、正の値に変換する
	if (angle < 0.0f) {
		angle += ((int)(fabs(angle) / M_PI / 2.0f) + 1) * M_PI * 2;
	}

	// 次に、[0, PI * 2]の範囲にする
	angle -= (int)(angle / M_PI / 2.0f) * M_PI * 2;

	// 最後に、[-PI, PI]の範囲にする
	//if (angle > M_PI) angle = M_PI * 2.0f - angle;
	if (angle > M_PI) angle = angle - M_PI * 2.0f;		// fix this bug on 12/17

	return angle;
}

/**
 * Compute the difference in angle that is normalized in the range of [0, PI].
 */
float GraphUtil::diffAngle(const QVector2D& dir1, const QVector2D& dir2, bool absolute) {
	float ang1 = atan2f(dir1.y(), dir1.x());
	float ang2 = atan2f(dir2.y(), dir2.x());

	if (absolute) {
		return fabs(normalizeAngle(ang1 - ang2));
	} else {
		return normalizeAngle(ang1 - ang2);
	}
}

/**
 * Compute the difference in angle that is normalized in the range of [0, PI].
 */
float GraphUtil::diffAngle(float angle1, float angle2, bool absolute) {
	if (absolute) {
		return fabs(normalizeAngle(angle1 - angle2));
	} else {
		return normalizeAngle(angle1 - angle2);
	}
}

/**
 * 対応する頂点が与えられている時に、２つの道路網のトポロジーの違いを数値化して返却する。
 * トポロジーの違いなので、座標は一切関係ない。隣接ノードとの接続性のみを考慮する。
 *
 * @param w_connectivity		対応するエッジがない場合のペナルティ
 * @param w_split				対応が重複している場合のペナルティ
 * @param w_angle				エッジの角度のペナルティ
 * @param w_distance			対応する頂点の距離に対するペナルティ
 */
/*float GraphUtil::computeDissimilarity(RoadGraph* roads1, QMap<RoadVertexDesc, RoadVertexDesc>& map1, RoadGraph* roads2, QMap<RoadVertexDesc, RoadVertexDesc>& map2, float w_connectivity, float w_split, float w_angle, float w_distance) {
	float penalty = 0.0f;

	//////////////////////////////////////////////////////////////////////////////////////////////////
	// コネクティビティに基づいたペナルティの計上
	RoadVertexIter vi, vend;
	for (boost::tie(vi, vend) = boost::vertices(roads1->graph); vi != vend; ++vi) {
		if (!roads1->graph[*vi]->valid) continue;

		if (map1.contains(*vi)) {
			RoadVertexDesc v2 = map1[*vi];

			RoadOutEdgeIter ei, eend;
			for (boost::tie(ei, eend) = boost::out_edges(*vi, roads1->graph); ei != eend; ++ei) {
				if (!roads1->graph[*ei]->valid) continue;

				RoadVertexDesc v1b = boost::target(*ei, roads1->graph);
				RoadVertexDesc v2b = map1[v1b];

				if (v2 == v2b || !isConnected(roads2, v2, v2b)) { // 対応ノード間が接続されてない場合
				//if (v2 == v2b || !roads2->isConnected(v2, v2b)) { // キャッシュによる高速化（ただし、事前にconnectivityを計算する必要有り
					penalty += roads1->graph[*ei]->getLength() * roads1->graph[*ei]->weight * w_connectivity;
				} else {
					QVector2D dir1 = roads1->graph[v1b]->getPt() - roads1->graph[*vi]->getPt();
					QVector2D dir2 = roads2->graph[v2b]->getPt() - roads2->graph[v2]->getPt();
					if (dir1.lengthSquared() > 0.0f && dir2.lengthSquared() > 0.0f) {
						penalty += diffAngle(dir1, dir2) * w_angle;
					} else {
						// どちらかのエッジの長さ＝０、つまり、エッジがないので、コネクティビティのペナルティを課す
						// 道路網１の方のエッジの長さが０の場合、ペナルティは０となるが、
						// 道路網２の計算の際に、ペナルティが加算されるので、良いだろう。
						penalty += roads1->graph[*ei]->getLength() * roads1->graph[*ei]->weight * w_connectivity;
					}
				}
			}
		} else { // 当該ノードに対応するノードがない場合は、全てのエッジをペナルティとして計上する
			RoadOutEdgeIter ei, eend;
			for (boost::tie(ei, eend) = boost::out_edges(*vi, roads1->graph); ei != eend; ++ei) {
				if (!roads1->graph[*ei]->valid) continue;

				RoadVertexDesc v1b = boost::target(*ei, roads1->graph);

				penalty += roads1->graph[*ei]->getLength() * roads1->graph[*ei]->weight * w_connectivity;
			}
		}
	}

	for (boost::tie(vi, vend) = boost::vertices(roads2->graph); vi != vend; ++vi) {
		if (!roads2->graph[*vi]->valid) continue;

		if (map2.contains(*vi)) {
			RoadVertexDesc v1 = map2[*vi];

			RoadOutEdgeIter ei, eend;
			for (boost::tie(ei, eend) = boost::out_edges(*vi, roads2->graph); ei != eend; ++ei) {
				if (!roads2->graph[*ei]->valid) continue;

				RoadVertexDesc v2b = boost::target(*ei, roads2->graph);
				RoadVertexDesc v1b = map2[v2b];

				if (v1 == v1b || !isConnected(roads1, v1, v1b)) { // 対応ノード間が接続されてない場合
				//if (v1 == v1b || !roads1->isConnected(v1, v1b)) { // キャッシュによる高速化（ただし、事前にconnectivityを計算する必要有り
					penalty += roads2->graph[*ei]->getLength() * roads2->graph[*ei]->weight * w_connectivity;
				} else {
					QVector2D dir1 = roads1->graph[v1b]->getPt() - roads1->graph[v1]->getPt();
					QVector2D dir2 = roads2->graph[v2b]->getPt() - roads2->graph[*vi]->getPt();
					if (dir1.lengthSquared() > 0.0f && dir2.lengthSquared() > 0.0f) {
						penalty += diffAngle(dir1, dir2) * w_angle;
					} else {
						// どちらかのエッジの長さ＝０、つまり、エッジがないので、コネクティビティのペナルティを課す
						// 道路網２の方のエッジの長さが０の場合、ペナルティは０となるが、
						// 道路網１の計算の際に、ペナルティが加算されるので、良いだろう。
						penalty += roads2->graph[*ei]->getLength() * roads2->graph[*ei]->weight * w_connectivity;
					}
				}
			}
		} else { // 当該ノードに対応するノードがない場合は、全てのエッジをペナルティとして計上する
			RoadOutEdgeIter ei, eend;
			for (boost::tie(ei, eend) = boost::out_edges(*vi, roads2->graph); ei != eend; ++ei) {
				if (!roads2->graph[*ei]->valid) continue;

				RoadVertexDesc v2b = boost::target(*ei, roads2->graph);

				penalty += roads2->graph[*ei]->getLength() * roads2->graph[*ei]->weight * w_connectivity;
			}
		}
	}

	//////////////////////////////////////////////////////////////////////////////////////////////////
	// 重複マッチング（モーフィングの際に、スプリットが発生）によるペナルティの計上
	QSet<RoadVertexDesc> used;
	for (QMap<RoadVertexDesc, RoadVertexDesc>::iterator it = map1.begin(); it != map1.end(); ++it) {
		if (used.contains(it.value())) {
			penalty += w_split;
		} else {
			used.insert(it.value());
		}
	}

	//////////////////////////////////////////////////////////////////////////////////////////////////
	// 頂点に距離に関するペナルティの計上
	for (boost::tie(vi, vend) = boost::vertices(roads1->graph); vi != vend; ++vi) {
		if (!roads1->graph[*vi]->valid) continue;

		if (map1.contains(*vi)) {
			RoadVertexDesc v2 = map1[*vi];

			penalty+= (roads1->graph[*vi]->pt - roads2->graph[v2]->pt).length() * w_distance;
		} else {
			// 対応する頂点がない場合、ペナルティはなし？
		}
	}

	return penalty;
}
*/

/**
 * 対応する頂点が与えられている時に、２つの道路網のトポロジーの違いを数値化して返却する。
 * トポロジーの違いなので、座標は一切関係ない。隣接ノードとの接続性のみを考慮する。
 *
 * @param w_matching			対応するエッジがない場合のペナルティ
 */
float GraphUtil::computeDissimilarity2(RoadGraph* roads1, QMap<RoadVertexDesc, RoadVertexDesc>& map1, RoadGraph* roads2, QMap<RoadVertexDesc, RoadVertexDesc>& map2, float w_matching, float w_split, float w_angle, float w_distance) {
	float penalty = 0.0f;

	//////////////////////////////////////////////////////////////////////////////////////////////////
	// 道路網１の各エッジについて、対応エッジがない場合のペナルティ
	RoadEdgeIter ei, eend;
	for (boost::tie(ei, eend) = boost::edges(roads1->graph); ei != eend; ++ei) {
		if (!roads1->graph[*ei]->valid) continue;

		RoadVertexDesc src = boost::source(*ei, roads1->graph);
		RoadVertexDesc tgt = boost::target(*ei, roads1->graph);
		
		if (!map1.contains(src) || !map1.contains(tgt)) {
			// 対応エッジがないので、当該エッジのImportanceに基づいて、ペナルティを追加
			penalty += w_matching;
		}
	}

	//////////////////////////////////////////////////////////////////////////////////////////////////
	// 道路網２の各エッジについて、対応エッジがない場合のペナルティ
	for (boost::tie(ei, eend) = boost::edges(roads2->graph); ei != eend; ++ei) {
		if (!roads2->graph[*ei]->valid) continue;

		RoadVertexDesc src = boost::source(*ei, roads2->graph);
		RoadVertexDesc tgt = boost::target(*ei, roads2->graph);
		
		if (!map2.contains(src) || !map2.contains(tgt)) {
			// 対応エッジがないので、当該エッジのImportanceに基づいて、ペナルティを追加
			penalty += w_matching;
		}
	}

	return penalty;
}

/**
 * Return the similarity of two road graphs.
 */
float GraphUtil::computeSimilarity(RoadGraph* roads1, QMap<RoadVertexDesc, RoadVertexDesc>& map1, RoadGraph* roads2, QMap<RoadVertexDesc, RoadVertexDesc>& map2, float w_connectivity, float w_angle, float w_length) {
	float score = 0.0f;

	//////////////////////////////////////////////////////////////////////////////////////////////////
	// For each edge of the 1st road graph, if there is a corresponding edge, increase the score.
	QList<RoadEdgeDesc> roads2UsedEdges;
	RoadEdgeIter ei, eend;
	for (boost::tie(ei, eend) = boost::edges(roads1->graph); ei != eend; ++ei) {
		if (!roads1->graph[*ei]->valid) continue;

		RoadVertexDesc src1 = boost::source(*ei, roads1->graph);
		RoadVertexDesc tgt1 = boost::target(*ei, roads1->graph);
		
		if (map1.contains(src1) && map1.contains(tgt1)) {
			RoadVertexDesc src2 = map1[src1];
			RoadVertexDesc tgt2 = map1[tgt1];

			if (!hasEdge(*roads2, src2, tgt2)) continue;

			RoadEdgeDesc e2 = getEdge(*roads2, src2, tgt2);
			if (roads2UsedEdges.contains(e2)) continue;
			roads2UsedEdges.push_back(e2);

			// increase the score
			score += w_connectivity;

			// increase the score according to the difference in the angle of the edges.
			//float angle_diff = diffAngle(roads1->graph[tgt1]->pt - roads1->graph[src1]->pt, roads2->graph[tgt2]->pt - roads2->graph[src2]->pt);
			std::vector<QVector2D> polyLine1;
			getOrderedPolyLine(*roads1, *ei, polyLine1);
			std::vector<QVector2D> polyLine2;
			getOrderedPolyLine(*roads2, e2, polyLine2);
			float angle_diff1 = diffAngle(polyLine1[1] - polyLine1[0], polyLine2[1] - polyLine2[0]);
			score += expf(-angle_diff1) * w_angle * 0.5f;
			float angle_diff2 = diffAngle(polyLine1[polyLine1.size() - 1] - polyLine1[polyLine1.size() - 2], polyLine2[polyLine2.size() - 1] - polyLine2[polyLine2.size() - 2]);
			score += expf(-angle_diff2) * w_angle * 0.5f;

			// increase the score according to the length of the edges
			float len_ratio = roads1->graph[*ei]->getLength() / roads2->graph[e2]->getLength();
			if (len_ratio < 1.0f) len_ratio = 1.0f / len_ratio;
			score += expf(1.0f - len_ratio) * w_length * 0.5f;

			float len1 = (roads1->graph[src1]->pt - roads1->graph[tgt1]->pt).length() + 0.1f;
			float len2 = (roads2->graph[src2]->pt - roads2->graph[tgt2]->pt).length() + 0.1f;
			float len_ratio2 = len1 / len2;
			if (len_ratio2 < 1.0f) len_ratio2 = 1.0f / len_ratio2;
			score += expf(1.0f - len_ratio2) * w_length * 0.5f;
		}
	}

	//////////////////////////////////////////////////////////////////////////////////////////////////
	// For each edge of the 2nd road graph, if there is a corresponding edge, increase the score.
	QList<RoadEdgeDesc> roads1UsedEdges;
	for (boost::tie(ei, eend) = boost::edges(roads2->graph); ei != eend; ++ei) {
		if (!roads2->graph[*ei]->valid) continue;

		RoadVertexDesc src2 = boost::source(*ei, roads2->graph);
		RoadVertexDesc tgt2 = boost::target(*ei, roads2->graph);
		
		if (map2.contains(src2) && map2.contains(tgt2)) {
			RoadVertexDesc src1 = map2[src2];
			RoadVertexDesc tgt1 = map2[tgt2];

			if (!hasEdge(*roads1, src1, tgt1)) continue;

			RoadEdgeDesc e1 = getEdge(*roads1, src1, tgt1);
			if (roads1UsedEdges.contains(e1)) continue;
			roads1UsedEdges.push_back(e1);

			// increase the score
			score += w_connectivity;

			// increase the score according to the difference in the angle of the edges.
			//float angle_diff = diffAngle(roads1->graph[tgt1]->pt - roads1->graph[src1]->pt, roads2->graph[tgt2]->pt - roads2->graph[src2]->pt);
			std::vector<QVector2D> polyLine1;
			getOrderedPolyLine(*roads1, e1, polyLine1);
			std::vector<QVector2D> polyLine2;
			getOrderedPolyLine(*roads2, *ei, polyLine2);
			float angle_diff1 = diffAngle(polyLine1[1] - polyLine1[0], polyLine2[1] - polyLine2[0]);
			score += expf(-angle_diff1) * w_angle * 0.5f;
			float angle_diff2 = diffAngle(polyLine1[polyLine1.size() - 1] - polyLine1[polyLine1.size() - 2], polyLine2[polyLine2.size() - 1] - polyLine2[polyLine2.size() - 2]);
			score += expf(-angle_diff2) * w_angle * 0.5f;

			// increase the score according to the length of the edges
			float len_ratio = roads1->graph[e1]->getLength() / roads2->graph[*ei]->getLength();
			if (len_ratio < 1.0f) len_ratio = 1.0f / len_ratio;
			score += expf(1.0f - len_ratio) * w_length * 0.5f;

			float len1 = (roads1->graph[src1]->pt - roads1->graph[tgt1]->pt).length() + 0.1f;
			float len2 = (roads2->graph[src2]->pt - roads2->graph[tgt2]->pt).length() + 0.1f;
			float len_ratio2 = len1 / len2;
			if (len_ratio2 < 1.0f) len_ratio2 = 1.0f / len_ratio2;
			score += expf(1.0f - len_ratio2) * w_length * 0.5f;
		}
	}

	return score;
}

/**
 * NearestNeighborに基づいて、２つの道路網のマッチングを行う。
 */
void GraphUtil::findCorrespondenceByNearestNeighbor(RoadGraph* roads1, RoadGraph* roads2, QMap<RoadVertexDesc, RoadVertexDesc>& map1, QMap<RoadVertexDesc, RoadVertexDesc>& map2) {
	if (getNumVertices(*roads1) < getNumVertices(*roads2)) {
		// 道路網１の各頂点に対して、対応する道路網２の頂点を探す
		RoadVertexIter vi, vend;
		for (boost::tie(vi, vend) = boost::vertices(roads1->graph); vi != vend; ++vi) {
			if (!roads1->graph[*vi]->valid) continue;

			RoadVertexDesc v2 = getVertex(*roads2, roads1->graph[*vi]->pt);
			map1[*vi] = v2;
			map2[v2] = *vi;
		}

		// 道路網２の各頂点に対して、まだペアがない場合は、対応する道路網１の頂点を探す
		for (boost::tie(vi, vend) = boost::vertices(roads2->graph); vi != vend; ++vi) {
			if (!roads2->graph[*vi]->valid) continue;

			if (map2.contains(*vi)) continue;

			RoadVertexDesc v1 = getVertex(*roads1, roads2->graph[*vi]->pt);
			map2[*vi] = v1;
		}
	} else {
		// 道路網２の各頂点に対して、対応する道路網１の頂点を探す
		RoadVertexIter vi, vend;
		for (boost::tie(vi, vend) = boost::vertices(roads2->graph); vi != vend; ++vi) {
			if (!roads2->graph[*vi]->valid) continue;

			RoadVertexDesc v1 = getVertex(*roads1, roads2->graph[*vi]->pt);
			map2[*vi] = v1;
			map1[v1] = *vi;
		}

		// 道路網１の各頂点に対して、まだペアがない場合は、対応する道路網２の頂点を探す
		for (boost::tie(vi, vend) = boost::vertices(roads1->graph); vi != vend; ++vi) {
			if (!roads1->graph[*vi]->valid) continue;

			if (map1.contains(*vi)) continue;

			RoadVertexDesc v2 = getVertex(*roads2, roads1->graph[*vi]->pt);
			map1[*vi] = v2;
		}
	}
}

/**
 * For two corresponding nodes, find the matching of outing edges.
 * Algorithm: minimize the maximum of the difference in angle of two corresponding edges.
 * If the degree is more than 6, the computation time will explode. Therefore, we should use an alternative approximation instead.
 */
QMap<RoadVertexDesc, RoadVertexDesc> GraphUtil::findCorrespondentEdges(RoadGraph* roads1, RoadVertexDesc parent1, std::vector<RoadVertexDesc> children1, RoadGraph* roads2, RoadVertexDesc parent2, std::vector<RoadVertexDesc> children2) {
	if (children1.size() > 6 || children2.size() > 6) {
		return findApproximateCorrespondentEdges(roads1, parent1, children1, roads2, parent2, children2);
	}

	QMap<RoadVertexDesc, RoadVertexDesc> map;

	std::vector<int> permutation;
	float min_diff = std::numeric_limits<float>::max();

	if (children1.size() <= children2.size()) {
		for (int i = 0; i < children2.size(); i++) {
			permutation.push_back(i);
		}

		for (int count = 0; count < 50000; count++) {
			// find the maximum of the difference in angle
			float diff = 0.0f;
			for (int i = 0; i < children1.size(); i++) {
				RoadEdgeDesc e1 = getEdge(*roads1, parent1, children1[i]);
				RoadEdgeDesc e2 = getEdge(*roads2, parent2, children2[permutation[i]]);

				QVector2D dir1;
				if ((roads1->graph[parent1]->pt - roads1->graph[e1]->polyLine[0]).length() < (roads1->graph[parent1]->pt - roads1->graph[e1]->polyLine[roads1->graph[e1]->polyLine.size() - 1]).length()) {
					dir1 = roads1->graph[e1]->polyLine[1] - roads1->graph[e1]->polyLine[0];
				} else {
					dir1 = roads1->graph[e1]->polyLine[roads1->graph[e1]->polyLine.size() - 2] - roads1->graph[e1]->polyLine[roads1->graph[e1]->polyLine.size() - 1];
				}

				QVector2D dir2;
				if ((roads2->graph[parent2]->pt - roads2->graph[e2]->polyLine[0]).length() < (roads2->graph[parent2]->pt - roads1->graph[e2]->polyLine[roads2->graph[e2]->polyLine.size() - 1]).length()) {
					dir2 = roads2->graph[e2]->polyLine[1] - roads2->graph[e2]->polyLine[0];
				} else {
					dir2 = roads2->graph[e2]->polyLine[roads2->graph[e2]->polyLine.size() - 2] - roads2->graph[e2]->polyLine[roads2->graph[e2]->polyLine.size() - 1];
				}

				/*
				QVector2D dir1 = roads1->graph[children1[i]]->pt - roads1->graph[parent1]->pt;
				QVector2D dir2 = roads2->graph[children2[permutation[i]]]->pt - roads2->graph[parent2]->pt;
				*/

				diff = std::max(diff, diffAngle(dir1, dir2));
			}

			if (diff < min_diff) {
				min_diff = diff;
				map.clear();
				for (int i = 0; i < children1.size(); i++) {
					map[children1[i]] = children2[permutation[i]];
				}
			}

			if (!std::next_permutation(permutation.begin(), permutation.end())) break;
		}
	} else {
		for (int i = 0; i < children1.size(); i++) {
			permutation.push_back(i);
		}

		for (int count = 0; count < 50000; count++) {
			// find the maximum of difference in angle
			float diff = 0.0f;
			for (int i = 0; i < children2.size(); i++) {
				RoadEdgeDesc e1 = getEdge(*roads1, parent1, children1[permutation[i]]);
				RoadEdgeDesc e2 = getEdge(*roads2, parent2, children2[i]);

				QVector2D dir1;
				if ((roads1->graph[parent1]->pt - roads1->graph[e1]->polyLine[0]).length() < (roads1->graph[parent1]->pt - roads1->graph[e1]->polyLine[roads1->graph[e1]->polyLine.size() - 1]).length()) {
					dir1 = roads1->graph[e1]->polyLine[1] - roads1->graph[e1]->polyLine[0];
				} else {
					dir1 = roads1->graph[e1]->polyLine[roads1->graph[e1]->polyLine.size() - 2] - roads1->graph[e1]->polyLine[roads1->graph[e1]->polyLine.size() - 1];
				}

				QVector2D dir2;
				if ((roads2->graph[parent2]->pt - roads2->graph[e2]->polyLine[0]).length() < (roads2->graph[parent2]->pt - roads1->graph[e2]->polyLine[roads2->graph[e2]->polyLine.size() - 1]).length()) {
					dir2 = roads2->graph[e2]->polyLine[1] - roads2->graph[e2]->polyLine[0];
				} else {
					dir2 = roads2->graph[e2]->polyLine[roads2->graph[e2]->polyLine.size() - 2] - roads2->graph[e2]->polyLine[roads2->graph[e2]->polyLine.size() - 1];
				}

				//QVector2D dir1 = roads1->graph[children1[permutation[i]]]->pt - roads1->graph[parent1]->pt;
				//QVector2D dir2 = roads2->graph[children2[i]]->pt - roads2->graph[parent2]->pt;

				diff = std::max(diff, diffAngle(dir1, dir2));
			}

			if (diff < min_diff) {
				min_diff = diff;
				map.clear();
				for (int i = 0; i < children2.size(); i++) {
					map[children1[permutation[i]]] = children2[i];
				}
			}

			if (!std::next_permutation(permutation.begin(), permutation.end())) break;
		}
	}

	return map;
}

/**
 * For two corresponding nodes, find the matching of outing edges.
 * Algorithm: This is an approximation algorithm. Find the most similar pair of edges in terms of their angles, and make the a pair. Keep this process until there is no edge in one of the children lists.
 */
QMap<RoadVertexDesc, RoadVertexDesc> GraphUtil::findApproximateCorrespondentEdges(RoadGraph* roads1, RoadVertexDesc parent1, std::vector<RoadVertexDesc> children1, RoadGraph* roads2, RoadVertexDesc parent2, std::vector<RoadVertexDesc> children2) {
	QMap<RoadVertexDesc, RoadVertexDesc> map;
	QList<int> used1;
	QList<int> used2;

	while (true) {
		float min_diff = std::numeric_limits<float>::max();
		int min_i = -1;
		int min_j = -1;

		for (int i = 0; i < children1.size(); i++) {
			if (used1.contains(i)) continue;

			for (int j = 0; j < children2.size(); j++) {
				if (used2.contains(j)) continue;

				float diff = diffAngle(roads1->graph[children1[i]]->pt - roads1->graph[parent1]->pt, roads2->graph[children2[j]]->pt - roads2->graph[parent2]->pt);
				if (diff < min_diff) {
					min_diff = diff;
					min_i = i;
					min_j = j;
				}
			}
		}

		if (min_i == -1) break;

		map[children1[min_i]] = children2[min_j];
		used1.push_back(min_i);
		used2.push_back(min_j);
	}

	return map;
}

/**
 * Find the correspondence in two road graphs.
 */
void GraphUtil::findCorrespondence(RoadGraph* roads1, AbstractForest* forest1, RoadGraph* roads2, AbstractForest* forest2, bool findAllMatching, float threshold_angle, QMap<RoadVertexDesc, RoadVertexDesc>& map1, QMap<RoadVertexDesc, RoadVertexDesc>& map2) {
	std::list<RoadVertexDesc> seeds1;
	std::list<RoadVertexDesc> seeds2;

	// For each root edge
	for (int i = 0; i < forest1->getRoots().size(); i++) {
		RoadVertexDesc v1 = forest1->getRoots()[i];
		RoadVertexDesc v2 = forest2->getRoots()[i];

		// Match the root vertices
		map1[v1] = v2;
		map2[v2] = v1;

		// register the root vertices as seeds.
		seeds1.push_back(v1);
		seeds2.push_back(v2);
	}

	while (!seeds1.empty()) {
		RoadVertexDesc parent1 = seeds1.front();
		seeds1.pop_front();
		RoadVertexDesc parent2 = seeds2.front();
		seeds2.pop_front();

		// If there is no child, skip it.
		if (forest1->getChildren(parent1).size() == 0 && forest2->getChildren(parent2).size() == 0) continue;

		// retrieve the children list
		std::vector<RoadVertexDesc> children1 = forest1->getChildren(parent1);
		std::vector<RoadVertexDesc> children2 = forest2->getChildren(parent2);

		// retrieve the matching for the children lists.
		QMap<RoadVertexDesc, RoadVertexDesc> children_map = findCorrespondentEdges(roads1, parent1, children1, roads2, parent2, children2);
		for (QMap<RoadVertexDesc, RoadVertexDesc>::iterator it = children_map.begin(); it != children_map.end(); ++it) {
			RoadVertexDesc child1 = it.key();
			RoadVertexDesc child2 = it.value();

			// if the difference in angle is too large, skip this pair.
			if (diffAngle(roads1->graph[child1]->pt - roads1->graph[parent1]->pt, roads2->graph[child2]->pt - roads2->graph[parent2]->pt) > threshold_angle) continue;

			// set fullyPaired flags
			roads1->graph[getEdge(*roads1, parent1, child1)]->fullyPaired = true;
			roads2->graph[getEdge(*roads2, parent2, child2)]->fullyPaired = true;

			// update the matching
			if (map1.contains(child1) || map2.contains(child2)) continue;
			map1[child1] = child2;
			map2[child2] = child1;

			seeds1.push_back(child1);
			seeds2.push_back(child2);
		}

		if (!findAllMatching) continue;

		// find the matching for the remained children
		while (true) {
			RoadVertexDesc child1, child2;
			if (!forceMatching(roads1, parent1, forest1, map1, roads2, parent2, forest2, map2, child1, child2)) break;

			// update the matching
			map1[child1] = child2;
			map2[child2] = child1;

			seeds1.push_back(child1);
			seeds2.push_back(child2);
		}
	}
}

/**
 * 相手のいない子ノードの中の１つに対して、対応する道路網の親ノードに無理やり対応させ、そのペアを返却する。
 * 相手のいない子ノードが１つもない場合は、falseを返却する。
 */
bool GraphUtil::forceMatching(RoadGraph* roads1, RoadVertexDesc parent1, AbstractForest* forest1, QMap<RoadVertexDesc, RoadVertexDesc>& map1, RoadGraph* roads2, RoadVertexDesc parent2, AbstractForest* forest2, QMap<RoadVertexDesc, RoadVertexDesc>& map2, RoadVertexDesc& child1, RoadVertexDesc& child2) {
	float min_angle = std::numeric_limits<float>::max();

	// retrieve the children list
	std::vector<RoadVertexDesc> children1 = forest1->getChildren(parent1);
	std::vector<RoadVertexDesc> children2 = forest2->getChildren(parent2);

	// ベストペアが見つからない、つまり、一方のリストが、全てペアになっている場合
	for (int i = 0; i < children1.size(); i++) {
		if (map1.contains(children1[i])) continue;
		if (!roads1->graph[children1[i]]->valid) continue;

		// 相手の親ノードをコピーしてマッチさせる
		RoadVertexPtr v = RoadVertexPtr(new RoadVertex(roads2->graph[parent2]->getPt()));
		RoadVertexDesc v_desc = boost::add_vertex(roads2->graph);
		roads2->graph[v_desc] = v;

		RoadEdgeDesc e1_desc = GraphUtil::getEdge(*roads1, parent1, children1[i]);

		// 相手の親ノードと子ノードの間にエッジを作成する
		//RoadEdgeDesc e2_desc = GraphUtil::addEdge(roads2, parent2, v_desc, roads1->graph[e1_desc]->lanes, roads1->graph[e1_desc]->type, roads1->graph[e1_desc]->oneWay);
		RoadEdgeDesc e2_desc = GraphUtil::addEdge(*roads2, parent2, v_desc, roads1->graph[e1_desc]);
		roads2->graph[e2_desc]->polyLine.clear();
		roads2->graph[e2_desc]->addPoint(roads2->graph[parent2]->pt);
		roads2->graph[e2_desc]->addPoint(roads2->graph[v_desc]->pt);

		forest2->addChild(parent2, v_desc);

		child1 = children1[i];
		child2 = v_desc;

		return true;
	}

	for (int i = 0; i < children2.size(); i++) {
		if (map2.contains(children2[i])) continue;
		if (!roads2->graph[children2[i]]->valid) continue;

		// 相手の親ノードをコピーしてマッチさせる
		RoadVertexPtr v = RoadVertexPtr(new RoadVertex(roads1->graph[parent1]->getPt()));
		RoadVertexDesc v_desc = boost::add_vertex(roads1->graph);
		roads1->graph[v_desc] = v;

		RoadEdgeDesc e2_desc = GraphUtil::getEdge(*roads2, parent2, children2[i]);

		// 相手の親ノードと子ノードの間にエッジを作成する
		//GraphUtil::addEdge(roads1, parent1, v_desc, roads2->graph[e2_desc]->lanes, roads2->graph[e2_desc]->type, roads2->graph[e2_desc]->oneWay);
		RoadEdgeDesc e1_desc = GraphUtil::addEdge(*roads1, parent1, v_desc, roads2->graph[e2_desc]);
		roads1->graph[e1_desc]->polyLine.clear();
		roads1->graph[e1_desc]->addPoint(roads1->graph[parent1]->pt);
		roads1->graph[e1_desc]->addPoint(roads1->graph[v_desc]->pt);

		forest1->addChild(parent1, v_desc);

		child1 = v_desc;
		child2 = children2[i];

		return true;
	}

	// No pair is found, i.e. all the children should have pairs.
	return false;
}

/**
 * Interpolate two road graphs. (Type1 interpolateion)
 * The roads1 shows its appearance with the ratio of t, whereas the roads2 shows it appeareance with the ratio of (1-t).
 */
RoadGraph* GraphUtil::interpolate(RoadGraph* roads1, RoadGraph* roads2, QMap<RoadVertexDesc, RoadVertexDesc>& map, float t) {
	RoadGraph* new_roads = new RoadGraph();

	if (t == 0.0f) {
		GraphUtil::copyRoads(*roads2, *new_roads);
		return new_roads;
	} else if (t == 1.0f) {
		GraphUtil::copyRoads(*roads1, *new_roads);
		return new_roads;
	}

	QMap<RoadVertexDesc, RoadVertexDesc> conv;

	// Add vertices to the interpolated roads
	RoadVertexIter vi, vend;
	for (boost::tie(vi, vend) = boost::vertices(roads1->graph); vi != vend; ++vi) {
		if (!roads1->graph[*vi]->valid) continue;

		RoadVertexDesc v2 = map[*vi];

		// Add a vertex
		RoadVertexPtr new_v = RoadVertexPtr(new RoadVertex(roads1->graph[*vi]->pt * t + roads2->graph[v2]->pt * (1 - t)));
		RoadVertexDesc new_v_desc = boost::add_vertex(new_roads->graph);
		new_roads->graph[new_v_desc] = new_v;

		conv[*vi] = new_v_desc;
	}

	// Add edges to the interpolated roads
	RoadEdgeIter ei, eend;
	for (boost::tie(ei, eend) = boost::edges(roads1->graph); ei != eend; ++ei) {
		if (!roads1->graph[*ei]->valid) continue;

		RoadVertexDesc v1 = boost::source(*ei, roads1->graph);
		RoadVertexDesc u1 = boost::target(*ei, roads1->graph);

		RoadVertexDesc v2 = map[v1];
		RoadVertexDesc u2 = map[u1];

		// Is there a corresponding edge?
		if (GraphUtil::hasEdge(*roads2, v2, u2)) {
			RoadEdgeDesc e2 = GraphUtil::getEdge(*roads2, v2, u2);

			RoadEdgeDesc e_desc = GraphUtil::addEdge(*new_roads, conv[v1], conv[u1], roads1->graph[*ei]->type, roads1->graph[*ei]->lanes, roads1->graph[*ei]->oneWay);
			new_roads->graph[e_desc]->polyLine = GraphUtil::interpolateEdges(roads1, *ei, v1, roads2, e2, v2, t);
		} else {
			// since there is no corresponding edge on roads2, just add the edge of roads1.
			RoadEdgeDesc e_desc = GraphUtil::addEdge(*new_roads, conv[v1], conv[u1], roads1->graph[*ei]);
			GraphUtil::moveEdge(*new_roads, e_desc, new_roads->graph[conv[v1]]->pt, new_roads->graph[conv[u1]]->pt);
		}
	}
	
	return new_roads;
}

/**
 * Interpolate two road graphs. (Type2 interpolateion)
 * At the center1, the resulting graph shows roads1's graph, whereas at the boundary by radius from the center, the resulting graph shows roads2's graph.
 * This interpolation is usefrul when you put a roads1 on top of the roads2 and keep the smooth transition between them.
 */
RoadGraph* GraphUtil::interpolate(RoadGraph* roads1, const QVector2D& center, float radius, RoadGraph* roads2, QMap<RoadVertexDesc, RoadVertexDesc>& map) {
	RoadGraph* new_roads = new RoadGraph();

	QMap<RoadVertexDesc, RoadVertexDesc> conv;

	// Add vertices to the interpolated roads
	RoadVertexIter vi, vend;
	for (boost::tie(vi, vend) = boost::vertices(roads1->graph); vi != vend; ++vi) {
		if (!roads1->graph[*vi]->valid) continue;

		RoadVertexDesc v2 = map[*vi];

		// compute the interpolation factor
		float s = radius - (roads1->graph[*vi]->pt - center).length();
		float t = (roads2->graph[v2]->pt - center).length();

		// Add a vertex
		RoadVertexPtr new_v = RoadVertexPtr(new RoadVertex(roads1->graph[*vi]->getPt() * s / (s + t) + roads2->graph[v2]->getPt() * t / (s + t)));
		RoadVertexDesc new_v_desc = boost::add_vertex(new_roads->graph);
		new_roads->graph[new_v_desc] = new_v;

		conv[*vi] = new_v_desc;
	}

	// Add edges to the interpolated roads
	RoadEdgeIter ei, eend;
	for (boost::tie(ei, eend) = boost::edges(roads1->graph); ei != eend; ++ei) {
		if (!roads1->graph[*ei]->valid) continue;

		RoadVertexDesc v1 = boost::source(*ei, roads1->graph);
		RoadVertexDesc u1 = boost::target(*ei, roads1->graph);

		RoadVertexDesc v2 = map[v1];
		RoadVertexDesc u2 = map[u1];

		// Is there a corresponding edge?
		if (GraphUtil::hasEdge(*roads2, v2, u2)) {
			RoadEdgeDesc e2 = GraphUtil::getEdge(*roads2, v2, u2);

			RoadEdgeDesc e_desc = GraphUtil::addEdge(*new_roads, conv[v1], conv[u1], roads1->graph[*ei]->type, roads1->graph[*ei]->lanes, roads1->graph[*ei]->oneWay);
			//new_roads->graph[e_desc]->polyLine.clear();

			// interpolate two polylines


		} else {
			// since there is no corresponding edge on roads2, just add the edge of roads1.
			RoadEdgeDesc e_desc = GraphUtil::addEdge(*new_roads, conv[v1], conv[u1], roads1->graph[*ei]);
			GraphUtil::moveEdge(*new_roads, e_desc, new_roads->graph[conv[v1]]->pt, new_roads->graph[conv[u1]]->pt);
		}
	}
	
	return new_roads;
}

/**
 * 与えられた数列の、先頭の値を１インクリメントする。
 * N進法なので、Nになったら、桁が繰り上がる。つまり、次の要素の値を１インクリメントする。
 * ex. {1, 2, 3} => {2, 2, 3}
 * ex. {N-1, 3, 3} => {0, 4, 3}
 * ex. {N-1, N-1, 3} => {0, 0, 4}
 */
bool GraphUtil::nextSequence(std::vector<int>& seq, int N) {
	int index = 0;
	while (true) {
		if (seq[index] < N - 1) break;

		seq[index] = 0;

		if (++index >= seq.size()) break;
	}

	if (index < seq.size()) {
		seq[index]++;
		return true;
	} else {
		return false;
	}
}

/**
 * Return a histogram of the edge lengths.
 * 以下の式に基づいて、ビンを決定する。
 * e * log(L) - 4
 * 
 * ただし、Lはエッジ長。
 */
cv::MatND GraphUtil::computeEdgeLengthHistogram(RoadGraph& roads, bool normalize) {
	int histSize = 15;

	// initialize the histogram
	cv::MatND hist = cv::MatND::zeros(1, histSize, CV_32F);

	// check the length of each edge
	int count = 0;
	RoadEdgeIter ei, eend;
	for (boost::tie(ei, eend) = boost::edges(roads.graph); ei != eend; ++ei) {
		if (!roads.graph[*ei]->valid) continue;

		float length = roads.graph[*ei]->getLength();
		int bin = logf(length) * expf(1) - 4;
		if (bin < 0) bin = 0;
		if (bin >= histSize) bin = histSize - 1;
		hist.at<float>(0, bin)++;

		count++;
	}

	// normalize the histogram
	if (normalize) {
		if (count > 0) hist /= count;
	}

	return hist;
}

/**
 * Return a histogram of the edge curvature.
 */
cv::MatND GraphUtil::computeEdgeCurvatureHistogram(RoadGraph& roads, int size) {
	cv::Mat curvatureMat = cv::Mat::zeros(1, getNumEdges(roads), CV_32FC1);
	
	// build a matrix that contains the length of each edge
	int count = 0;
	RoadEdgeIter ei, eend;
	for (boost::tie(ei, eend) = boost::edges(roads.graph); ei != eend; ++ei) {
		if (!roads.graph[*ei]->valid) continue;

		QVector2D baseDir = roads.graph[*ei]->polyLine[1] - roads.graph[*ei]->polyLine[0];
		for (int i = 2; i < roads.graph[*ei]->polyLine.size(); i++) {
			QVector2D dir = roads.graph[*ei]->polyLine[i] - roads.graph[*ei]->polyLine[i - 1];
			curvatureMat.at<float>(0, count) += diffAngle(baseDir, dir);
		}
		if (curvatureMat.at<float>(0, count) > M_PI) {
			curvatureMat.at<float>(0, count) = M_PI;
		}

		count++;
	}

	// create a histogram
	float range[] = { 0, M_PI };
	const float* ranges = { range };
	cv::MatND hist;
	cv::calcHist(&curvatureMat, 1, 0, cv::Mat(), hist, 1, &size, &ranges, true, false);

	// normalize the histogram
	cv::normalize(hist, hist, 0, 1, cv::NORM_MINMAX, -1, cv::Mat());

	return hist;
}

/**
 * Return the average edge length.
 */
float GraphUtil::computeAvgEdgeLength(RoadGraph& roads) {
	float length = 0.0f;
	int count = 0;

	RoadEdgeIter ei, eend;
	for (boost::tie(ei, eend) = boost::edges(roads.graph); ei != eend; ++ei) {
		if (!roads.graph[*ei]->valid) continue;

		length += roads.graph[*ei]->getLength();
		count++;
	}

	if (count == 0) return 0;
	else return length / (float)count;
}

/**
 * 道路網から、交差点のアームの方向、長さの２次元ヒストグラムを生成する。
 */
void GraphUtil::computeHistogram(RoadGraph& roads, cv::Mat& hist) {
	// histogram size
	int histDirSize = 18;
	int histLengthSize = 15;

	// initialize the histogram
	hist = cv::Mat::zeros(histDirSize, histLengthSize, CV_32FC1);

	int count = 0;
	RoadVertexIter vi, vend;
	for (boost::tie(vi, vend) = boost::vertices(roads.graph); vi != vend; ++vi) {
		if (!roads.graph[*vi]->valid) continue;

		RoadOutEdgeIter ei, eend;
		for (boost::tie(ei, eend) = boost::out_edges(*vi, roads.graph); ei != eend; ++ei) {
			if (!roads.graph[*ei]->valid) continue;

			RoadVertexDesc tgt = boost::target(*ei, roads.graph);

			// get the direction
			std::vector<QVector2D> polyline;
			getOrderedPolyLine(roads, *ei, polyline);
			QVector2D baseDir = polyline[1] - polyline[0];
			float theta = atan2f(baseDir.y(), baseDir.x());
			if (theta < 0) theta += M_PI * 2;
			int binDir = theta * (float)histDirSize / M_PI / 2.0f;
			if (binDir >= histDirSize) binDir = histDirSize - 1;

			// get the length
			float length = roads.graph[*ei]->getLength();
			int binLength = logf(length) * expf(1) - 4;
			if (binLength < 0) binLength = 0;
			if (binLength >= histLengthSize) binLength = histLengthSize - 1;

			// get the curvature
			/*
			float totalT = 0.0f;
			float totalS = 0.0f;
			baseDir.normalize();
			int numSegments = 0;
			for (int i = 2; i < polyline.size(); i++) {
				QVector2D dir = roads.graph[*ei]->polyLine[i] - roads.graph[*ei]->polyLine[i - 1];
				float s = dir.length();
				dir.normalize();

				totalT += (dir - baseDir).length();
				totalS += s;
			}
			float curvature = 0.0f;
			if (totalT > 0) {
				curvature = totalS / totalT;
			}
			int binCurvature = logf(curvature + 0.00001f) + 5;
			if (binCurvature < 0) binCurvature = 0;
			if (binCurvature >= histCurvatureSize) binCurvature = histCurvatureSize - 1;
			*/

			hist.at<float>(binDir, binLength)++;
		}
	}
}

/**
 * Apply the global rigid ICP in order to fit the 1st road graph to the 2nd road graph in the least square manner.
 * As a result, roads1 will be updated to best fit roads2.
 */
void GraphUtil::rigidICP(RoadGraph* roads1, RoadGraph* roads2, QList<EdgePair>& pairs) {
	cv::Mat src(pairs.size() * 2, 2, CV_32FC2);
	cv::Mat dst(pairs.size() * 2, 2, CV_32FC2);

	for (int i = 0; i < pairs.size(); i++) {
		RoadEdgeDesc e1 = pairs[i].edge1;
		RoadEdgeDesc e2 = pairs[i].edge2;

		// エッジの両端頂点を取得
		RoadVertexDesc src1 = boost::source(e1, roads1->graph);
		RoadVertexDesc tgt1 = boost::target(e1, roads1->graph);
		RoadVertexDesc src2 = boost::source(e2, roads2->graph);
		RoadVertexDesc tgt2 = boost::target(e2, roads2->graph);

		// もしsrc1-tgt2、tgt1-src2の方が近かったら、src2とtgt2を入れ替える
		if ((roads1->graph[src1]->pt - roads2->graph[src2]->pt).length() + (roads1->graph[tgt1]->pt - roads2->graph[tgt2]->pt).length() > (roads1->graph[src1]->pt - roads2->graph[tgt2]->pt).length() + (roads1->graph[tgt1]->pt - roads2->graph[src2]->pt).length()) {
			src2 = boost::target(e2, roads2->graph);
			tgt2 = boost::source(e2, roads2->graph);
		}

		// 頂点の座標を行列に格納
		src.at<float>(i * 2, 0) = roads1->graph[src1]->pt.x();
		src.at<float>(i * 2, 1) = roads1->graph[src1]->pt.y();
		src.at<float>(i * 2 + 1, 0) = roads1->graph[tgt1]->pt.x();
		src.at<float>(i * 2 + 1, 1) = roads1->graph[tgt1]->pt.y();
		dst.at<float>(i * 2, 0) = roads2->graph[src2]->pt.x();
		dst.at<float>(i * 2, 1) = roads2->graph[src2]->pt.y();
		dst.at<float>(i * 2 + 1, 0) = roads2->graph[tgt2]->pt.x();
		dst.at<float>(i * 2 + 1, 1) = roads2->graph[tgt2]->pt.y();
	}

	// Rigid ICP 変換行列を計算
	cv::Mat transformMat = cv::estimateRigidTransform(src, dst, false);

	// 道路網１の頂点座標を、変換行列を使って更新
	src = convertVerticesToCVMatrix(roads1, false);
	cv::Mat src2;
	cv::transform(src, src2, transformMat);

	// 道路網１の頂点座標を実際に更新する
	RoadVertexIter vi, vend;
	int count = 0;
	for (boost::tie(vi, vend) = boost::vertices(roads1->graph); vi != vend; ++vi) {
		roads1->graph[*vi]->pt.setX(src2.at<float>(count, 0));
		roads1->graph[*vi]->pt.setY(src2.at<float>(count, 1));
		count++;
	}

	// 道路網１のエッジの座標も更新する
	src = convertEdgesToCVMatrix(roads1, false);
	cv::transform(src, src2, transformMat);

	// 道路網１のエッジ座標を実際に更新する
	RoadEdgeIter ei, eend;
	count = 0;
	for (boost::tie(ei, eend) = boost::edges(roads1->graph); ei != eend; ++ei) {
		for (int i = 0; i < roads1->graph[*ei]->polyLine.size(); i++) {
			roads1->graph[*ei]->polyLine[i].setX(src2.at<float>(count, 0));
			roads1->graph[*ei]->polyLine[i].setY(src2.at<float>(count, 1));
			count++;
		}
	}
}

/**
 * Apply the global rigid ICP in order to fit the 1st road graph to the 2nd road graph in the least square manner.
 * As a result, roads1 will be updated to best fit roads2.
 * Originally, I used cv::estimateRigidTransform, but this function does not work well in some cases.
 * Therefore, I decided to implement simple function in my own way.
 */
/*cv::Mat GraphUtil::rigidICP(RoadGraph* roads1, RoadGraph* roads2, QMap<RoadVertexDesc, RoadVertexDesc>& map) {
	cv::Mat src(map.size(), 2, CV_32FC1);
	cv::Mat dst(map.size(), 2, CV_32FC1);

	int count = 0;
	for (QMap<RoadVertexDesc, RoadVertexDesc>::iterator it = map.begin(); it != map.end(); ++it, ++count) {
		RoadVertexDesc v1 = it.key();
		RoadVertexDesc v2 = it.value();

		src.at<float>(count, 0) = roads1->graph[v1]->pt.x();
		src.at<float>(count, 1) = roads1->graph[v1]->pt.y();
		dst.at<float>(count, 0) = roads2->graph[v2]->pt.x();
		dst.at<float>(count, 1) = roads2->graph[v2]->pt.y();
	}

	// Rigid ICP 変換行列を計算
	cv::Mat transformMat = cv::estimateRigidTransform(src.reshape(2), dst.reshape(2), false);

	return transformMat;
}
*/
cv::Mat GraphUtil::rigidICP(RoadGraph* roads1, RoadGraph* roads2, QMap<RoadVertexDesc, RoadVertexDesc>& map) {
	QVector2D center1, center2;

	int count = 0;
	RoadVertexIter vi, vend;
	for (boost::tie(vi, vend) = boost::vertices(roads1->graph); vi != vend; ++vi) {
		if (!map.contains(*vi)) continue;

		RoadVertexDesc v2 = map[*vi];
		center1 += roads1->graph[*vi]->pt;
		center2 += roads2->graph[v2]->pt;
		count++;
	}
	center1 /= (float)count;
	center2 /= (float)count;

	float angle = 0.0f;
	count = 0;
	RoadEdgeIter ei, eend;
	for (boost::tie(ei, eend) = boost::edges(roads1->graph); ei != eend; ++ei) {
		if (!roads1->graph[*ei]->valid) continue;

		RoadVertexDesc src1 = boost::source(*ei, roads1->graph);
		RoadVertexDesc tgt1 = boost::target(*ei, roads1->graph);

		if (!map.contains(src1) || !map.contains(tgt1)) continue;

		RoadVertexDesc src2 = map[src1];
		RoadVertexDesc tgt2 = map[tgt1];

		angle += GraphUtil::diffAngle(roads2->graph[src2]->pt - roads2->graph[tgt2]->pt, roads1->graph[src1]->pt - roads1->graph[tgt1]->pt, false);
		count++;
	}

	angle /= (float)count;

	// Rigid ICP 変換行列を計算
	cv::Mat transformMat1 = cv::Mat::eye(3, 3, CV_64FC1);
	transformMat1.at<double>(0, 2) = center2.x();
	transformMat1.at<double>(1, 2) = center2.y();
	cv::Mat transformMat2 = cv::Mat::eye(3, 3, CV_64FC1);
	transformMat2.at<double>(0, 0) = cos(angle);
	transformMat2.at<double>(0, 1) = -sin(angle);
	transformMat2.at<double>(1, 0) = sin(angle);
	transformMat2.at<double>(1, 1) = cos(angle);
	cv::Mat transformMat3 = cv::Mat::eye(3, 3, CV_64FC1);
	transformMat3.at<double>(0, 2) = -center1.x();
	transformMat3.at<double>(1, 2) = -center1.y();

	transformMat1 = transformMat1 * transformMat2 * transformMat3;

	cv::Mat transformMat(3, 3, CV_64FC1);
	for (int i = 0; i < 2; i++) {
		for (int j = 0; j < 3; j++) {
			transformMat.at<double>(i, j) = transformMat1.at<double>(i, j);
		}
	}

	std::cout << transformMat << endl;

	return transformMat;
}

void GraphUtil::transform(RoadGraph* roads, const cv::Mat& transformMat) {
	// 道路網１の頂点座標を、変換行列を使って更新
	cv::Mat src = convertVerticesToCVMatrix(roads, false);
	cv::Mat src2;
	cv::transform(src, src2, transformMat);

	// 道路網１の頂点座標を実際に更新する
	RoadVertexIter vi, vend;
	int count = 0;
	for (boost::tie(vi, vend) = boost::vertices(roads->graph); vi != vend; ++vi) {
		roads->graph[*vi]->pt.setX(src2.at<float>(count, 0));
		roads->graph[*vi]->pt.setY(src2.at<float>(count, 1));
		count++;
	}

	// 道路網１のエッジの座標も更新する
	src = convertEdgesToCVMatrix(roads, false);
	cv::transform(src, src2, transformMat);

	// 道路網１のエッジ座標を実際に更新する
	RoadEdgeIter ei, eend;
	count = 0;
	for (boost::tie(ei, eend) = boost::edges(roads->graph); ei != eend; ++ei) {
		for (int i = 0; i < roads->graph[*ei]->polyLine.size(); i++) {
			roads->graph[*ei]->polyLine[i].setX(src2.at<float>(count, 0));
			roads->graph[*ei]->polyLine[i].setY(src2.at<float>(count, 1));
			count++;
		}
	}
}

/**
 * 道路網の頂点座標を、Nx2の行列に変換する
 */
cv::Mat GraphUtil::convertVerticesToCVMatrix(RoadGraph* roads, bool onlyValidVertex) {
	cv::Mat ret(getNumVertices(*roads, onlyValidVertex), 2, CV_32FC2);

	RoadVertexIter vi, vend;
	int count = 0;
	for (boost::tie(vi, vend) = boost::vertices(roads->graph); vi != vend; ++vi) {
		if (onlyValidVertex && !roads->graph[*vi]->valid) continue;

		ret.at<float>(count, 0) = roads->graph[*vi]->pt.x();
		ret.at<float>(count, 1) = roads->graph[*vi]->pt.y();
		count++;
	}

	return ret;
}

/**
 * 道路網のエッジ座標を、Nx2の行列に変換する
 */
cv::Mat GraphUtil::convertEdgesToCVMatrix(RoadGraph* roads, bool onlyValidVertex) {
	std::vector<QVector2D> data;
	
	RoadEdgeIter ei, eend;
	for (boost::tie(ei, eend) = boost::edges(roads->graph); ei != eend; ++ei) {
		if (onlyValidVertex && !roads->graph[*ei]->valid) continue;

		for (int i = 0; i < roads->graph[*ei]->polyLine.size(); i++) {
			data.push_back(roads->graph[*ei]->polyLine[i]);
		}
	}
	
	cv::Mat ret(data.size(), 2, CV_32FC2);

	for (int i = 0; i < data.size(); i++) {
		ret.at<float>(i, 0) = data[i].x();
		ret.at<float>(i, 1) = data[i].y();
	}

	return ret;
}

/**
 * グリッドスタイルの道路網を作成する。
 *
 * @param size		一辺の長さ [m]
 * @param num		一辺のノード数
 */
RoadGraph* GraphUtil::createGridNetwork(float size, int num) {
	RoadGraph* roads = new RoadGraph();

	// 各エッジの長さ
	float length = size / (float)(num - 1);

	// 原点座標
	QVector2D orig(-size / 2.0f, -size / 2.0f);

	// ノードを作成
	for (int i = 0; i < num - 2; i++) {
		for (int j = 0; j < num; j++) {
			RoadVertexPtr v = RoadVertexPtr(new RoadVertex(orig + QVector2D(j * length, i * length + length)));
			RoadVertexDesc desc = boost::add_vertex(roads->graph);
			roads->graph[desc] = v;
		}
	}
	for (int i = 0; i < num - 2; i++) {
		RoadVertexPtr v = RoadVertexPtr(new RoadVertex(orig + QVector2D(i * length + length, 0)));
		RoadVertexDesc desc = boost::add_vertex(roads->graph);
		roads->graph[desc] = v;
	}
	for (int i = 0; i < num - 2; i++) {
		RoadVertexPtr v = RoadVertexPtr(new RoadVertex(orig + QVector2D(i * length + length, size)));
		RoadVertexDesc desc = boost::add_vertex(roads->graph);
		roads->graph[desc] = v;
	}

	// エッジを作成
	for (int i = 0; i < num - 2; i++) {
		for (int j = 0; j < num - 1; j++) {
			addEdge(*roads, i * num + j, i * num + j + 1, 2, 2);
		}
	}
	for (int i = 0; i < num - 3; i++) {
		for (int j = 0; j < num - 2; j++) {
			addEdge(*roads, i * num + 1 + j, i * num + 1 + j + num, 2, 2);
		}
	}
	for (int i = 0; i < num - 2; i++) {
		addEdge(*roads, num * (num - 2) + i, i + 1, 2, 2);
		addEdge(*roads, num * (num - 2) + (num - 2) + i, num * (num - 3) + i + 1, 2, 2);
	}

	return roads;
}

/**
 * 曲がったスタイルの道路網を作成する。
 *
 * @param size		一辺の長さ [m]
 * @param num		一辺のノード数
 * @param angle		傾ける角度 [rad]
 */
RoadGraph* GraphUtil::createCurvyNetwork(float size, int num, float angle) {
	RoadGraph* roads = new RoadGraph();

	// 各エッジの長さ
	float length = size / (float)(num - 1);

	// 原点座標
	QVector2D orig(-size / 2.0f, -size / 2.0f);

	// ノードを作成
	for (int i = 0; i < num - 2; i++) {
		for (int j = 0; j < num; j++) {
			QVector2D pos = orig + QVector2D(j * length, i * length + length);
			QVector2D pos2;
			pos2.setX(pos.x() * cosf(angle) - pos.y() * sinf(angle));
			pos2.setY(pos.x() * sinf(angle) + pos.y() * cosf(angle));
			RoadVertexPtr v = RoadVertexPtr(new RoadVertex(pos2));
			RoadVertexDesc desc = boost::add_vertex(roads->graph);
			roads->graph[desc] = v;
		}
	}
	for (int i = 0; i < num - 2; i++) {
		QVector2D pos = orig + QVector2D(i * length + length, 0);
		QVector2D pos2;
		pos2.setX(pos.x() * cosf(angle) - pos.y() * sinf(angle));
		pos2.setY(pos.x() * sinf(angle) + pos.y() * cosf(angle));
		RoadVertexPtr v = RoadVertexPtr(new RoadVertex(pos2));
		RoadVertexDesc desc = boost::add_vertex(roads->graph);
		roads->graph[desc] = v;
	}
	for (int i = 0; i < num - 2; i++) {
		QVector2D pos = orig + QVector2D(i * length + length, size);
		QVector2D pos2;
		pos2.setX(pos.x() * cosf(angle) - pos.y() * sinf(angle));
		pos2.setY(pos.x() * sinf(angle) + pos.y() * cosf(angle));
		RoadVertexPtr v = RoadVertexPtr(new RoadVertex(pos2));
		RoadVertexDesc desc = boost::add_vertex(roads->graph);
		roads->graph[desc] = v;
	}

	// エッジを作成
	for (int i = 0; i < num - 2; i++) {
		for (int j = 0; j < num - 1; j++) {
			RoadEdgePtr e = RoadEdgePtr(new RoadEdge(2, 2, false));
			QVector2D pos = orig + QVector2D(j * length, i * length + length);
			for (int k = 0; k <= 10; k++) {
				QVector2D pos2 = pos + QVector2D((float)k * 0.1f * length, length * 0.1f * sinf((float)k * M_PI * 2 * 0.1f));
				QVector2D pos3;
				pos3.setX(pos2.x() * cosf(angle) - pos2.y() * sinf(angle));
				pos3.setY(pos2.x() * sinf(angle) + pos2.y() * cosf(angle));
				e->addPoint(pos3);
			}
			
			std::pair<RoadEdgeDesc, bool> edge_pair = boost::add_edge(i * num + j, i * num + j + 1, roads->graph);
			roads->graph[edge_pair.first] = e;
		}
	}
	for (int i = 0; i < num - 3; i++) {
		for (int j = 0; j < num - 2; j++) {
			RoadEdgePtr e = RoadEdgePtr(new RoadEdge(2, 2, false));
			QVector2D pos = orig + QVector2D(j * length + length, i * length + length);
			for (int k = 0; k <= 10; k++) {
				QVector2D pos2 = pos + QVector2D(length * 0.1f * sinf((float)k * M_PI * 2 * 0.1f), (float)k * 0.1f * length);
				QVector2D pos3;
				pos3.setX(pos2.x() * cosf(angle) - pos2.y() * sinf(angle));
				pos3.setY(pos2.x() * sinf(angle) + pos2.y() * cosf(angle));
				e->addPoint(pos3);
			}
			
			std::pair<RoadEdgeDesc, bool> edge_pair = boost::add_edge(i * num + 1 + j, i * num + 1 + j + num, roads->graph);
			roads->graph[edge_pair.first] = e;

			//addEdge(roads, i * num + 1 + j, i * num + 1 + j + num, 2, 2);
		}
	}
	for (int i = 0; i < num - 2; i++) {
		RoadEdgePtr e = RoadEdgePtr(new RoadEdge(2, 2, false));
		QVector2D pos = orig + QVector2D(i * length + length, 0);
		for (int k = 0; k <= 10; k++) {
			QVector2D pos2 = pos + QVector2D(length * 0.1f * sinf((float)k * M_PI * 2 * 0.1f), (float)k * 0.1f * length);
			QVector2D pos3;
			pos3.setX(pos2.x() * cosf(angle) - pos2.y() * sinf(angle));
			pos3.setY(pos2.x() * sinf(angle) + pos2.y() * cosf(angle));
			e->addPoint(pos3);
		}
			
		std::pair<RoadEdgeDesc, bool> edge_pair = boost::add_edge(num * (num - 2) + i, i + 1, roads->graph);
		roads->graph[edge_pair.first] = e;

		//addEdge(roads, num * (num - 2) + i, i + 1, 2, 2);
	}
	for (int i = 0; i < num - 2; i++) {
		RoadEdgePtr e = RoadEdgePtr(new RoadEdge(2, 2, false));
		QVector2D pos = orig + QVector2D(i * length + length, size - length);
		for (int k = 0; k <= 10; k++) {
			QVector2D pos2 = pos + QVector2D(length * 0.1f * sinf((float)k * M_PI * 2 * 0.1f), (float)k * 0.1f * length);
			QVector2D pos3;
			pos3.setX(pos2.x() * cosf(angle) - pos2.y() * sinf(angle));
			pos3.setY(pos2.x() * sinf(angle) + pos2.y() * cosf(angle));
			e->addPoint(pos3);
		}
			
		std::pair<RoadEdgeDesc, bool> edge_pair = boost::add_edge(num * (num - 2) + (num - 2) + i, num * (num - 3) + i + 1, roads->graph);
		roads->graph[edge_pair.first] = e;
		
		//addEdge(roads, num * (num - 2) + (num - 2) + i, num * (num - 3) + i + 1, 2, 2);
	}

	return roads;
}

/**
 * 放射線状の道路網を作成する。
 *
 * @param size		一辺の長さ [m]
 * @param num		円を何個作るか？
 * @param degree	中心から出るエッジの数
 */
RoadGraph* GraphUtil::createRadialNetwork(float size, int num, int degree) {
	RoadGraph* roads = new RoadGraph();

	float length = size / (float)(num + 1) / 2.0f;

	// 頂点を追加
	RoadVertexPtr v = RoadVertexPtr(new RoadVertex(QVector2D(0, 0)));
	RoadVertexDesc desc = boost::add_vertex(roads->graph);
	roads->graph[desc] = v;

	for (int i = 0; i < num + 1; i++) {
		for (int j = 0; j < degree; j++) {
			float theta = (float)j / (double)degree * M_PI * 2.0f;
			RoadVertexPtr v = RoadVertexPtr(new RoadVertex(length * QVector2D((float)(i + 1) * cosf(theta), (float)(i + 1) * sinf(theta))));
			RoadVertexDesc desc = boost::add_vertex(roads->graph);
			roads->graph[desc] = v;
		}
	}

	// エッジを追加
	for (int i = 0; i < degree; i++) {
		addEdge(*roads, 0, i + 1, 2, 2);
	}
	for (int i = 0; i < num; i++) {
		for (int j = 0; j < degree; j++) {
			addEdge(*roads, 1 + i * degree + j, 1 + (i + 1) * degree + j, 2, 2);
		}
	}
	for (int i = 0; i < num; i++) {
		for (int j = 0; j < degree; j++) {
			float theta = (float)j / (double)degree * M_PI * 2.0f;
			float dt = 1.0f / (double)degree * M_PI * 2.0f;
			RoadEdgePtr e = RoadEdgePtr(new RoadEdge(2, 2, false));
			for (int k = 0; k <= 4; k++) {
				QVector2D pos = length * QVector2D((float)(i + 1) * cosf(theta + dt * (float)k / 4.0f), (float)(i + 1) * sinf(theta + dt * (float)k / 4.0f));
				e->addPoint(pos);
			}
			
			std::pair<RoadEdgeDesc, bool> edge_pair = boost::add_edge(1 + i * degree + j, 1 + i * degree + (j + 1) % degree, roads->graph);
			roads->graph[edge_pair.first] = e;
			
			//addEdge(roads, 1 + i * 12 + j, 1 + i * 12 + (j + 1) % 12, 2, 2);
		}
	}

	return roads;
}

/**
 * Print the statistics of the road graph.
 * Print the histogram of degrees and the historgram of lanes.
 */
void GraphUtil::printStatistics(RoadGraph* roads) {
	int degreesHistogram[10];
	int lanesHistogram[10];

	for (int i = 0; i < 10; i++) {
		degreesHistogram[i] = 0;
		lanesHistogram[i] = 0;
	}

	// degreeのヒストグラムを作成
	RoadVertexIter vi, vend;
	for (boost::tie(vi, vend) = boost::vertices(roads->graph); vi != vend; ++vi) {
		if (!roads->graph[*vi]->valid) continue;

		int degree = getDegree(*roads, *vi);
		if (degree < 10) {
			degreesHistogram[degree]++;
		}
	}

	qDebug() << "Degrees:";
	for (int i = 0; i < 10; i++) {
		qDebug() << i << ": " << degreesHistogram[i];
	}

	// レーン数のヒストグラムを作成
	RoadEdgeIter ei, eend;
	for (boost::tie(ei, eend) = boost::edges(roads->graph); ei != eend; ++ei) {
		if (!roads->graph[*ei]->valid) continue;

		int lanes = roads->graph[*ei]->lanes;
		if (lanes < 10) {
			lanesHistogram[lanes]++;
		}
	}

	qDebug() << "Lanes:";
	for (int i = 0; i < 10; i++) {
		qDebug() << i << ": " << lanesHistogram[i];
	}
}

/**
 * 道路網をcv::Mat行列に置き換える
 */
void GraphUtil::convertToMat(RoadGraph& roads, cv::Mat_<uchar>& mat, const cv::Size& size, int width, bool flip) {
	mat = cv::Mat_<uchar>(size, 0);

	RoadEdgeIter ei, eend;
	for (boost::tie(ei, eend) = boost::edges(roads.graph); ei != eend; ++ei) {
		if (!roads.graph[*ei]->valid) continue;

		drawRoadSegmentOnMat(roads, *ei, mat, width);
	}

	// 上下を反転
	if (flip) cv::flip(mat, mat, 0);
}

/**
 * 道路のエッジを、cv::Mat行列上に描画する
 * brightnessは、0から255で指定。（デフォルト値は255）
 */
void GraphUtil::drawRoadSegmentOnMat(RoadGraph& roads, RoadEdgeDesc e, cv::Mat& mat, int width, int brightness) {
	for (int i = 0; i < roads.graph[e]->polyLine.size() - 1; i++) {
		QVector2D p0 = roads.graph[e]->polyLine[i];
		QVector2D p1 = roads.graph[e]->polyLine[i + 1];
		cv::line(mat, cv::Point(p0.x(), p0.y()), cv::Point(p1.x(), p1.y()), cv::Scalar(brightness), width, CV_AA);
	}
}
