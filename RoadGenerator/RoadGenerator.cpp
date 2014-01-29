#include "RoadGenerator.h"
#include "Util.h"
#include "GraphUtil.h"

RoadGenerator::RoadGenerator() {
}

RoadGenerator::~RoadGenerator() {
}

/**
 * 道路網を生成する
 */
void RoadGenerator::generateRoadNetwork(RoadGraph& roads, const BBox& area, const GridFeature& gf) {
	// Avenue用シード
	std::list<RoadVertexDesc> initSeeds;

	// Avenue用のシードを生成
	generateHorizontalAvenues(roads, area, gf, initSeeds);

	// Avenue生成
	std::list<RoadVertexDesc> seeds = initSeeds;
	while (!seeds.empty()) {
		seeds = expandHorizontalAvenues(roads, area, gf, seeds, 1, gf.generateLength(1, Util::uniform_rand()));
	}
	seeds = initSeeds;
	while (!seeds.empty()) {
		seeds = expandHorizontalAvenues(roads, area, gf, seeds, 3, gf.generateLength(3, Util::uniform_rand()));
	}
}

/**
 * エリアの中心を通る横方向のライン上に、Avenueを生成し、シードとする。
 */
void RoadGenerator::generateHorizontalAvenues(RoadGraph& roads, const BBox& area, const GridFeature& gf, std::list<RoadVertexDesc>& seeds) {
	seeds.clear();

	// エリアの中心に頂点を作成する
	RoadVertexPtr center_v = RoadVertexPtr(new RoadVertex(area.midPt()));
	center_v->angles = gf.getAngles();
	center_v->lengths = gf.getLengths();

	RoadVertexDesc center_desc = GraphUtil::addVertex(roads, center_v);
	seeds.push_back(center_desc);

	// 中心点から、左方向に頂点を追加していく
	QVector2D pt = area.midPt();
	float angle2 = gf.getAngles()[2];
	QVector2D dir2 = QVector2D(cosf(angle2), sinf(angle2));
	RoadVertexDesc prev_desc = center_desc;
	while (true) {
		pt = pt + dir2 * gf.generateLength(2, Util::uniform_rand());

		// エリアの外に出たらストップ
		if (!area.contains(pt)) break;

		// 頂点を追加
		RoadVertexPtr v = RoadVertexPtr(new RoadVertex(pt));
		RoadVertexDesc desc = GraphUtil::addVertex(roads, v);

		// エッジを追加
		GraphUtil::addEdge(roads, prev_desc, desc, 2, 1);
		prev_desc = desc;

		// シードに追加
		seeds.push_front(desc);
	}

	// 中心点から、右方向に頂点を追加していく
	pt = area.midPt();
	float angle1 = gf.getAngles()[0];
	QVector2D dir1 = QVector2D(cosf(angle1), sinf(angle1));
	prev_desc = center_desc;
	while (true) {
		pt = pt + dir1 * gf.generateLength(0, Util::uniform_rand());

		// エリアの外に出たらストップ
		if (!area.contains(pt)) break;

		// 頂点を追加
		RoadVertexPtr v = RoadVertexPtr(new RoadVertex(pt));
		RoadVertexDesc desc = GraphUtil::addVertex(roads, v);

		// エッジを追加
		GraphUtil::addEdge(roads, prev_desc, desc, 2, 1);
		prev_desc = desc;

		// シードに追加
		seeds.push_back(desc);
	}

}

/**
 * 横方向のシードAvenueを上下に延長し、生成された頂点を返却する。
 */
std::list<RoadVertexDesc> RoadGenerator::expandHorizontalAvenues(RoadGraph& roads, const BBox& area, const GridFeature& gf, std::list<RoadVertexDesc>& seeds, int dir, float length) {
	std::list<RoadVertexDesc> newSeeds;

	std::vector<float> angles = gf.getAngles();

	QVector2D offset;
	switch (dir) {
	case 1:
		offset = QVector2D(cosf(angles[1]), sinf(angles[1])) * length;
		break;
	case 3:
		offset = QVector2D(cosf(angles[3]), sinf(angles[3])) * length;
		break;
	}

	// 最初のシード頂点の情報を保管しておく
	RoadVertexDesc start_seed_desc = seeds.front();
	RoadVertexDesc start_desc;

	// 各シード頂点からオフセットの位置に新たな頂点を追加していく
	RoadVertexDesc prev_desc;
	bool prev_valid = false;
	while (!seeds.empty()) {
		RoadVertexDesc seed_desc = seeds.front();
		seeds.pop_front();

		QVector2D pt = roads.graph[seed_desc]->pt + offset;
		
		// エリアの外なら、スキップ
		if (!area.contains(pt)) continue;

		// 頂点を追加
		RoadVertexPtr v = RoadVertexPtr(new RoadVertex(pt));
		RoadVertexDesc desc = GraphUtil::addVertex(roads, v);

		// 縦方向のエッジを追加
		GraphUtil::addEdge(roads, seed_desc, desc, 2, 1);

		// 横方向のエッジを追加
		if (prev_valid) {
			GraphUtil::addEdge(roads, prev_desc, desc, 2, 1);
		} else {
			start_desc = desc;
		}

		// シードに追加
		newSeeds.push_back(desc);

		prev_desc = desc;
		prev_valid = true;
	}

	// 最初のシード頂点からオフセットの位置を基点とし、左方向に伸ばしてみる
	if (prev_valid) {
		QVector2D pt = roads.graph[start_desc]->pt + offset;
		float angle2 = gf.getAngles()[2];
		QVector2D dir2 = QVector2D(cosf(angle2), sinf(angle2));
		prev_desc = start_desc;
		while (true) {
			pt = pt + dir2 * gf.generateLength(2, Util::uniform_rand());

			// エリアの外に出たらストップ
			if (!area.contains(pt)) break;

			// 頂点を追加
			RoadVertexPtr v = RoadVertexPtr(new RoadVertex(pt));
			RoadVertexDesc desc = GraphUtil::addVertex(roads, v);

			// 横方向のエッジを追加
			GraphUtil::addEdge(roads, prev_desc, desc, 2, 1);
			prev_desc = desc;

			// シードに追加
			newSeeds.push_front(desc);
		}
	}

	return newSeeds;
}
