#include "AbstractForest.h"

AbstractForest::AbstractForest(RoadGraph* roads) {
	this->roads = roads;
}

AbstractForest::~AbstractForest() {
}

/**
 * Return the children nodes.
 */
std::vector<RoadVertexDesc>& AbstractForest::getChildren(RoadVertexDesc node) {
	if (!children.contains(node)) {
		std::vector<RoadVertexDesc> c;
		children[node] = c;
	}

	return children[node];
}

/**
 * Add a child node.
 */
void AbstractForest::addChild(RoadVertexDesc parent, RoadVertexDesc child) {
	std::vector<RoadVertexDesc> list = getChildren(parent);
	list.push_back(child);
	children[parent] = list;
}

/**
 * Return the parents nodes.
 */
QList<RoadVertexDesc> AbstractForest::getParent(RoadVertexDesc node) {
	QList<RoadVertexDesc> ret;

	for (QMap<RoadVertexDesc, std::vector<RoadVertexDesc> >::iterator it = children.begin(); it != children.end(); ++it) {
		RoadVertexDesc parent = it.key();
		for (int i = 0; i < children[parent].size(); i++) {
			if (children[parent][i] == node) {
				ret.push_back(parent);
			}
		}
	}

	return ret;
}

/**
 * Return the root nodes.
 */
QList<RoadVertexDesc> AbstractForest::getRoots() {
	return roots;
}

/**
 * Return the depth of the node.
 * For instance, the root node has depth 0.
 */
int AbstractForest::getDepth(RoadVertexDesc node) {
	if (roots.contains(node)) return 0;

	return getDepth(getParent(node)[0]) + 1;
}
