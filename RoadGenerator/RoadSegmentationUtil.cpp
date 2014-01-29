#include "RoadSegmentationUtil.h"
#include "GraphUtil.h"
#include "Util.h"
#include "BBox.h"
#include <math.h>

#ifndef Q_MOC_RUN
#include <boost/graph/planar_face_traversal.hpp>
#include <boost/graph/boyer_myrvold_planar_test.hpp>
#endif

#ifndef SQR(x)
#define SQR(x)	((x) * (x))
#endif

RoadGraph* roadGraphPtr;
std::vector<RoadEdgeDesc> plaza;
int numOutingEdges;
float plazaLength;
std::vector<std::vector<RoadEdgeDesc> > plaza_list;

//Vertex visitor
struct faceVisitorForPlazaDetection : public boost::planar_face_traversal_visitor {
	void begin_face() {
		plaza.clear();
		numOutingEdges = 0;
		plazaLength = 0.0f;
	}

	void end_face() {
		if (plaza.size() > 4 && numOutingEdges >= 4 && plazaLength < 1000.0f && plazaLength > 200.0f) {
			plaza_list.push_back(plaza);
		}
	}

	template <typename Vertex> 
	void next_vertex(Vertex v) {
		//plaza.push_back(v);
		numOutingEdges += GraphUtil::getNumEdges(*roadGraphPtr, v) - 2;
	}

	template <typename Edge> 
	void next_edge(Edge e) {
		plaza.push_back(e);
		plazaLength += roadGraphPtr->graph[e]->getLength();
	}
};

/**
 * グリッドを検知する
 * まず、最初に最も大きいグリッド領域を抽出する。次に、それ以外の領域について、次に大きいグリッド領域を抽出する。以降、最大6回、一定サイズ以上のグリッド領域が見つからなくなるまで繰り返す。
 *
 * @param numBins				ヒストグラムのビン数
 * @param minTotalLength		最大頻度となるビンに入ったエッジの総延長距離が、この値より小さい場合、顕著な特徴ではないと考え、グリッド検知せずにfalseを返却する
 * @param minMaxBinRatio		最大頻度となるビンの割合が、この値より小さい場合は、顕著な特徴ではないと考え、グリッド検知せずにfalseを返却する
 * @param votingRatioThreshold	各エッジについて、構成するラインが所定のグリッド方向に従っているかの投票率を計算し、この閾値未満なら、グリッドに従っていないと見なす
 */
void RoadSegmentationUtil::detectGrid(RoadGraph& roads, AbstractArea& area, int roadType, int maxIteration, float numBins, float minTotalLength, float minMaxBinRatio, float angleThreshold, float votingRatioThreshold, float extendingDistanceThreshold) {
	for (int i = 0; i < maxIteration; i++) {
		GridFeature gf(i);
		if (!detectOneGrid(roads, area, roadType, gf, numBins, minTotalLength, minMaxBinRatio, angleThreshold, votingRatioThreshold, extendingDistanceThreshold)) break;
	}
}

/**
 * １つのグリッドを検知する。
 * 既にグループに属しているエッジはスキップする。
 *
 * @param numBins				ヒストグラムのビン数
 * @param minTotalLength		最大頻度となるビンに入ったエッジの総延長距離が、この値より小さい場合、顕著な特徴ではないと考え、グリッド検知せずにfalseを返却する
 * @param minMaxBinRatio		最大頻度となるビンの割合が、この値より小さい場合は、顕著な特徴ではないと考え、グリッド検知せずにfalseを返却する
 */
bool RoadSegmentationUtil::detectOneGrid(RoadGraph& roads, AbstractArea& area, int roadType, GridFeature& gf, int numBins, float minTotalLength, float minMaxBinRatio, float angleThreshold, float votingRatioThreshold, float extendingDistanceThreshold) {
	// ヒストグラムの初期化
	cv::Mat dirMat = cv::Mat::zeros(numBins, 1, CV_32F);

	int count = 0;
	RoadEdgeIter ei, eend;
	for (boost::tie(ei, eend) = boost::edges(roads.graph); ei != eend; ++ei) {
		if (!roads.graph[*ei]->valid) continue;

		// 指定されたタイプ以外は、スキップする。
		if (!(roads.graph[*ei]->type & roadType)) continue;

		// 既にshapeTypeが確定しているエッジは、スキップする
		if (roads.graph[*ei]->shapeType > 0) continue;

		// エリアの外のエッジは、スキップする
		RoadVertexDesc src = boost::source(*ei, roads.graph);
		if (!area.contains(roads.graph[src]->pt)) continue;
		RoadVertexDesc tgt = boost::target(*ei, roads.graph);
		if (!area.contains(roads.graph[tgt]->pt)) continue;		

		for (int i = 0; i < roads.graph[*ei]->polyLine.size() - 1; i++) {
			QVector2D dir = roads.graph[*ei]->polyLine[i + 1] - roads.graph[*ei]->polyLine[i];
			float theta = atan2f(dir.y(), dir.x());
			if (theta < 0) theta += M_PI;
			if (theta > M_PI * 0.5f) theta -= M_PI * 0.5f;

			// どのビンか決定
			int bin_id = theta * (float)numBins / M_PI * 2.0f;
			if (bin_id >= numBins) bin_id = numBins - 1;

			// 投票する
			dirMat.at<float>(bin_id, 0) += dir.length();
			count++;
		}
	}

	// ヒストグラムの中で、最大頻度となるビンを探す
	float max_hist_value = 0.0f;
	int max_bin_id;
	for (int i = 0; i < dirMat.rows; i++) {
		if (dirMat.at<float>(i, 0) > max_hist_value) {
			max_hist_value = dirMat.at<float>(i, 0);
			max_bin_id = i;
		}
	}

	// 最頻値（モード）のビンの割合がminMaxBinRatio%未満なら、グリッドとは見なさない
	if (max_hist_value < minTotalLength || max_hist_value < (float)count * minMaxBinRatio) return false;

	// グリッド方向の近似値をセットする
	gf.setAngle(((float)max_bin_id + 0.5f) / (float)numBins * M_PI * 0.5f);

	// 最頻値（モード）のビンに入るエッジを使って、高精度のグリッド方向を計算する
	for (boost::tie(ei, eend) = boost::edges(roads.graph); ei != eend; ++ei) {
		if (!roads.graph[*ei]->valid) continue;

		// 指定されたタイプ以外は、スキップする。
		if (!(roads.graph[*ei]->type & roadType)) continue;
		
		// 既にshapeTypeが確定しているエッジは、スキップする
		if (roads.graph[*ei]->shapeType > 0) continue;

		// エリアの外のエッジは、スキップする
		RoadVertexDesc src = boost::source(*ei, roads.graph);
		if (!area.contains(roads.graph[src]->pt)) continue;
		RoadVertexDesc tgt = boost::target(*ei, roads.graph);
		if (!area.contains(roads.graph[tgt]->pt)) continue;	

		float length = 0.0f;
		for (int i = 0; i < roads.graph[*ei]->polyLine.size() - 1; i++) {
			gf.addEdge(roads.graph[*ei]->polyLine[i + 1] - roads.graph[*ei]->polyLine[i], M_PI * 0.5f / (float)numBins);
		}
	}
	gf.computeFeature();

	// 正確なグリッド方向を使って、グリッド方向に近いエッジをグループに登録する
	QMap<RoadEdgeDesc, float> edges;
	for (boost::tie(ei, eend) = boost::edges(roads.graph); ei != eend; ++ei) {
		if (!roads.graph[*ei]->valid) continue;

		// 指定されたタイプ以外は、スキップする。
		if (!(roads.graph[*ei]->type & roadType)) continue;
		
		// 既にshapeTypeが確定しているエッジは、スキップする
		if (roads.graph[*ei]->shapeType > 0) continue;

		// エリアの外のエッジは、スキップする
		RoadVertexDesc src = boost::source(*ei, roads.graph);
		if (!area.contains(roads.graph[src]->pt)) continue;
		RoadVertexDesc tgt = boost::target(*ei, roads.graph);
		if (!area.contains(roads.graph[tgt]->pt)) continue;

		float length = 0.0f;
		for (int i = 0; i < roads.graph[*ei]->polyLine.size() - 1; i++) {
			QVector2D dir = roads.graph[*ei]->polyLine[i + 1] - roads.graph[*ei]->polyLine[i];

			// エッジの方向が、グリッド方向に近い場合、投票する
			if (gf.isClose(dir, angleThreshold)) {
				length += dir.length();
			}
		}

		// votingRatioThreshold%以上、グリッドの方向と同じ方向のエッジなら、そのエッジを当該グループに入れる
		if (length >= roads.graph[*ei]->getLength() * votingRatioThreshold) {
			edges.insert(*ei, length / roads.graph[*ei]->getLength());
		}
	}

	// 最大連結成分に属さないエッジは、グループから外す
	reduceGridGroup(roads, gf, edges);

	//
	extendGridGroup(roads, area, roadType, gf, edges, angleThreshold, votingRatioThreshold, extendingDistanceThreshold);

	// 最後に、このグループに属するエッジを、RoadGraphオブジェクトに反映させる
	for (QMap<RoadEdgeDesc, float>::iterator it = edges.begin(); it != edges.end(); ++it) {
		RoadEdgeDesc e = it.key();
		roads.graph[e]->shapeType = RoadEdge::SHAPE_GRID;
		roads.graph[e]->group = gf.group_id;
		roads.graph[e]->gridness = it.value();
	}
	
	roads.setModified();

	return true;
}

/**
 * 指定されたエッジから、同じグループに属するエッジを辿っていく。
 */
int RoadSegmentationUtil::traverseConnectedEdges(RoadGraph& roads, RoadEdgeDesc e, QMap<RoadEdgeDesc, int>& edges, int segment_id) {
	int count = 0;

	QList<RoadVertexDesc> queue;
	RoadVertexDesc src = boost::source(e, roads.graph);
	queue.push_back(src);

	QList<RoadVertexDesc> visited;
	visited.push_back(src);

	while (!queue.empty()) {
		RoadVertexDesc v = queue.front();
		queue.pop_front();

		RoadOutEdgeIter ei, eend;
		for (boost::tie(ei, eend) = boost::out_edges(v, roads.graph); ei != eend; ++ei) {
			if (!roads.graph[*ei]->valid) continue;

			// 隣接エッジが、同じグループに属するなら、再帰的に辿っていく
			if (edges.contains(*ei)) {
				edges[*ei] = segment_id;

				RoadVertexDesc u = boost::target(*ei, roads.graph);
				if (!roads.graph[u]->valid) continue;
				if (visited.contains(u)) continue;

				visited.push_back(u);
				queue.push_back(u);
				count++;
			}
		}
	}

	return count;
}

/**
 * 最大連結成分に属さないエッジは、group_idを-1に戻す
 */
void RoadSegmentationUtil::reduceGridGroup(RoadGraph& roads, GridFeature& gf, QMap<RoadEdgeDesc, float>& edges) {
	// 各エッジが、どのグループに属するか
	QMap<RoadEdgeDesc, int> groups;

	// 指定されたグループに属するエッジのセットを生成する
	for (QMap<RoadEdgeDesc, float>::iterator it = edges.begin(); it != edges.end(); ++it) {
		groups.insert(it.key(), -1);
	}

	// 各エッジと接続されているエッジの数をカウントする
	int numSegments = 0;
	std::vector<int> hist;
	for (QMap<RoadEdgeDesc, int>::iterator it = groups.begin(); it != groups.end(); ++it) {
		if (groups[it.key()] >= 0) continue;

		int num = traverseConnectedEdges(roads, it.key(), groups, numSegments);
		hist.push_back(num);

		numSegments++;
	}

	// 最大グループを取得
	int max_num = 0;
	int max_segment;
	for (int i = 0; i < hist.size(); i++) {
		if (hist[i] > max_num) {
			max_num = hist[i];
			max_segment = i;
		}
	}

	// 当該グループに属するエッジについて、最大グループ以外は、グループから外す
	for (QMap<RoadEdgeDesc, float>::iterator it = edges.begin(); it != edges.end(); ) {
		if (groups[it.key()] != max_segment) {
			it = edges.erase(it);
		} else {
			++it;
		}
	}
}

/**
 * グリッドのグループの属さないエッジについて、近くに属するエッジがあれば、そのグループの仲間に入れちゃう。
 * これをしてあげないと、例えばラウンドアバウトに挟まれたエッジなどが、グリッドの仲間に入れない。
 */
void RoadSegmentationUtil::extendGridGroup(RoadGraph& roads, AbstractArea& area, int roadType, GridFeature& gf, QMap<RoadEdgeDesc, float>& edges, float angleThreshold, float votingRatioThreshold, float distanceThreshold) {
	float distanceThreshold2 = distanceThreshold * distanceThreshold;

	RoadEdgeIter ei, eend;
	for (boost::tie(ei, eend) = boost::edges(roads.graph); ei != eend; ++ei) {
		if (!roads.graph[*ei]->valid) continue;

		// 指定されたタイプ以外は、スキップする。
		if (!(roads.graph[*ei]->type & roadType)) continue;
		
		// 既にshapeTypeが確定しているエッジは、スキップする
		if (roads.graph[*ei]->shapeType > 0) continue;

		// エリアの外のエッジは、スキップする
		RoadVertexDesc src = boost::source(*ei, roads.graph);
		if (!area.contains(roads.graph[src]->pt)) continue;
		RoadVertexDesc tgt = boost::target(*ei, roads.graph);
		if (!area.contains(roads.graph[tgt]->pt)) continue;

		float length = 0.0f;
		for (int i = 0; i < roads.graph[*ei]->polyLine.size() - 1; i++) {
			QVector2D dir = roads.graph[*ei]->polyLine[i + 1] - roads.graph[*ei]->polyLine[i];

			// エッジの方向が、グリッド方向に近い場合、投票する
			if (gf.isClose(dir, angleThreshold)) {
				length += dir.length();
			}
		}

		// votingRatioThreshold%以上、グリッドの方向と同じ方向のエッジなら、そのエッジを当該グループに入れる
		if (length < roads.graph[*ei]->getLength() * votingRatioThreshold) continue;

		for (QMap<RoadEdgeDesc, float>::iterator it = edges.begin(); it != edges.end(); ++it) {
			RoadVertexDesc src2 = boost::source(it.key(), roads.graph);
			RoadVertexDesc tgt2 = boost::target(it.key(), roads.graph);

			// 当該グループのエッジと距離が近い場合、当該グループに入れる
			if ((roads.graph[src]->pt - roads.graph[src2]->pt).lengthSquared() < distanceThreshold2 || 
				(roads.graph[src]->pt - roads.graph[tgt2]->pt).lengthSquared() < distanceThreshold2 || 
				(roads.graph[tgt]->pt - roads.graph[src2]->pt).lengthSquared() < distanceThreshold2 || 
				(roads.graph[tgt]->pt - roads.graph[tgt2]->pt).lengthSquared() < distanceThreshold2) {
				roads.graph[*ei]->shapeType = RoadEdge::SHAPE_GRID;
				roads.graph[*ei]->group = gf.group_id;
				roads.graph[*ei]->gridness = length / roads.graph[*ei]->getLength();
				break;
			}
		}
	}
}

/**
 * Plazaを検知する
 * グラフのfaceについて、そのサイズ、faceから出るスポークの数などから、プラザかどうか判断する。
 * ぜんぜん検知精度が良くない。
 */
void RoadSegmentationUtil::detectPlaza(RoadGraph& roads, AbstractArea& area) {
	plaza_list.clear();
	roadGraphPtr = &roads;

	//Make sure graph is planar
	typedef std::vector<RoadEdgeDesc > tEdgeDescriptorVector;
	std::vector<tEdgeDescriptorVector> embedding(boost::num_vertices(roads.graph));
	if (!boost::boyer_myrvold_planarity_test(boost::boyer_myrvold_params::graph = roads.graph, boost::boyer_myrvold_params::embedding = &embedding[0])) {
		std::cout << "The road graph is not a planar graph." << std::endl;
	}

	//Create edge index property map
	typedef std::map<RoadEdgeDesc, size_t> EdgeIndexMap;
	EdgeIndexMap mapEdgeIdx;
	boost::associative_property_map<EdgeIndexMap> pmEdgeIndex(mapEdgeIdx);		
	RoadEdgeIter ei, eend;	
	int edge_count = 0;
	for (boost::tie(ei, eend) = boost::edges(roads.graph); ei != eend; ++ei) {
		mapEdgeIdx.insert(std::make_pair(*ei, edge_count++));
	}

	//Extract blocks from road graph using boost graph planar_face_traversal
	faceVisitorForPlazaDetection vis;	
	boost::planar_face_traversal(roads.graph, &embedding[0], vis, pmEdgeIndex);

	for (int i = 0; i < plaza_list.size(); i++) {
		for (int j = 0; j < plaza_list[i].size(); j++) {
			RoadEdgeDesc e = plaza_list[i][j];

			RoadVertexDesc src = boost::source(e, roads.graph);
			if (!area.contains(roads.graph[src]->pt)) continue;
			RoadVertexDesc tgt = boost::target(e, roads.graph);
			if (!area.contains(roads.graph[tgt]->pt)) continue;

			roads.graph[e]->shapeType = RoadEdge::SHAPE_PLAZA;
			roads.graph[e]->group = 8;
		}
	}

	roads.setModified();
}

/**
 * Plazaを検知する
 * Hough transformにより、円を検知する。
 */
void RoadSegmentationUtil::detectRadial(RoadGraph& roads, AbstractArea& area, int roadType, int maxIteration, float scale1, float scale2, float centerErrorTol2, float angleThreshold2, float scale3, float centerErrorTol3, float angleThreshold3, float votingRatioThreshold, float seedDistance, float extendingAngleThreshold) {
	for (int i = 0; i < maxIteration; i++) {
		if (!detectOneRadial(roads, area, roadType, i, scale1, scale2, centerErrorTol2, angleThreshold2, scale3, centerErrorTol3, angleThreshold3, votingRatioThreshold, seedDistance, extendingAngleThreshold)) break;
	}
}

/**
 * Plazaを検知する
 * Hough transformにより、円を検知する。
 */
bool RoadSegmentationUtil::detectOneRadial(RoadGraph& roads, AbstractArea& area, int roadType, int group_id, float scale1, float scale2, float centerErrorTol2, float angleThreshold2, float scale3, float centerErrorTol3, float angleThreshold3, float votingRatioThreshold, float seedDistance, float extendingAngleThreshold) {
	// 0.01スケールで、円の中心を求める
	QVector2D center = detectRadialCenterInScaled(roads, area, roadType, scale1);

	// 0.1スケールで、より正確な円の中心を求める
	center = refineRadialCenterInScaled(roads, area, roadType, scale2, center, centerErrorTol2, angleThreshold2);

	// 0.2スケールで、より正確な円の中心を求める
	center = refineRadialCenterInScaled(roads, area, roadType, scale3, center, centerErrorTol3, angleThreshold3);

	// 各エッジについて、radialの中心点に合うものを、グループに登録する
	RoadEdgeIter ei, eend;
	for (boost::tie(ei, eend) = boost::edges(roads.graph); ei != eend; ++ei) {
		if (!roads.graph[*ei]->valid) continue;

		// 指定されたタイプ以外は、スキップする。
		if (!(roads.graph[*ei]->type & roadType)) continue;

		// 既にshapeTypeが確定しているエッジは、スキップする
		if (roads.graph[*ei]->shapeType > 0) continue;

		// 範囲外のエッジはスキップ
		RoadVertexDesc src = boost::source(*ei, roads.graph);
		if (!area.contains(roads.graph[src]->pt)) continue;
		RoadVertexDesc tgt = boost::target(*ei, roads.graph);
		if (!area.contains(roads.graph[tgt]->pt)) continue;

		float length = 0.0f;
		for (int i = 0; i < roads.graph[*ei]->polyLine.size() - 1; i++) {
			QVector2D dir1 = roads.graph[*ei]->polyLine[i + 1] - roads.graph[*ei]->polyLine[i];
			QVector2D dir2 = roads.graph[*ei]->polyLine[i] - center;

			if (GraphUtil::diffAngle(dir1, dir2) < angleThreshold3 || GraphUtil::diffAngle(dir1, -dir2) < angleThreshold3) {
				length += dir1.length();
			}
		}

		// votingRatioThreshold%以上、このradialと同じ方向なら、そのエッジを当該グループに入れる
		if (length >= roads.graph[*ei]->getLength() * votingRatioThreshold) {
			roads.graph[*ei]->shapeType = RoadEdge::SHAPE_RADIAL;
			roads.graph[*ei]->group = group_id;
			roads.graph[*ei]->gridness = 0.0f;
		}
	}

	// 円の中心から一定距離以内のエッジのみを残す
	if (!reduceRadialGroup(roads, center, group_id, seedDistance, 5)) return false;

	// 残したエッジから周辺のエッジを辿り、方向がほぼ同じなら、グループに再度登録していく
	extendRadialGroup(roads, area, roadType, center, group_id, extendingAngleThreshold, votingRatioThreshold);

	roads.setModified();

	return true;
}

/**
 * Plazaを１つ検知し、円の中心を返却する
 * Hough transformにより、円を検知する。
 */
QVector2D RoadSegmentationUtil::detectRadialCenterInScaled(RoadGraph& roads, AbstractArea& area, int roadType, float scale) {
	BBox bbox = GraphUtil::getAABoundingBox(roads);
	cv::Mat houghTransform = cv::Mat::zeros(bbox.dy() * scale, bbox.dx() * scale, CV_32F);

	float sigma = bbox.dx() * scale * 0.05f;

	RoadEdgeIter ei, eend;
	for (boost::tie(ei, eend) = boost::edges(roads.graph); ei != eend; ++ei) {
		if (!roads.graph[*ei]->valid) continue;
		
		// 指定されたタイプ以外は、スキップする。
		if (!(roads.graph[*ei]->type & roadType)) continue;

		// 既にshapeTypeが確定しているエッジは、スキップする
		if (roads.graph[*ei]->shapeType > 0) continue;

		// 範囲の外のエッジはスキップする
		RoadVertexDesc src = boost::source(*ei, roads.graph);
		if (!area.contains(roads.graph[src]->pt)) continue;
		RoadVertexDesc tgt = boost::target(*ei, roads.graph);
		if (!area.contains(roads.graph[tgt]->pt)) continue;

		for (int i = 0; i < roads.graph[*ei]->polyLine.size() - 1; i++) {
			QVector2D v1 = (roads.graph[*ei]->polyLine[i] - bbox.minPt) * scale;
			QVector2D v2 = (roads.graph[*ei]->polyLine[i + 1] - bbox.minPt) * scale;
			QVector2D v12 = (v1 + v2) * 0.5f;

			QVector2D dir = v2 - v1;
			float len = dir.length();

			if (dir.x() > dir.y()) {
				for (int x = 0; x < houghTransform.cols; x++) {
					int y = dir.y() * ((float)x - v1.x()) / dir.x() + v1.y() + 0.5f;
					if (y < 0 || y >= houghTransform.rows) continue;

					float h = 0;
					if (x >= std::min(v1.x(), v2.x()) && x <= std::max(v1.x(), v2.x())) {
						h = len;
					} else if (x < std::min(v1.x(), v2.x())) {
						h = len * expf(-SQR(x - std::min(v1.x(), v2.x())) / 2.0f / SQR(sigma));
					} else {
						h = len * expf(-SQR(x - std::max(v1.x(), v2.x())) / 2.0f / SQR(sigma));
					}

					houghTransform.at<float>(y, x) += h;
				}
			} else {
				for (int y = 0; y < houghTransform.rows; y++) {
					int x = dir.x() * ((float)y - v1.y()) / dir.y() + v1.x() + 0.5f;
					if (x < 0 || x >= houghTransform.cols) continue;

					float h = 0;
					if (y >= std::min(v1.y(), v2.y()) && y <= std::max(v1.y(), v2.y())) {
						h = len;
					} else if (y < std::min(v1.y(), v2.y())) {
						h = len * expf(-SQR(y - std::min(v1.y(), v2.y())) / 2.0f / SQR(sigma));
					} else {
						h = len * expf(-SQR(y - std::max(v1.y(), v2.y())) / 2.0f / SQR(sigma));
					}

					houghTransform.at<float>(y, x) += h;
				}
			}
		}
	}

	// 最大値を取得する
	QVector2D center;
	float max_value = 0.0f;
	for (int v = 0; v < houghTransform.rows; v++) {
		for (int u = 0; u < houghTransform.cols; u++) {
			if (houghTransform.at<float>(v, u) > max_value) {
				max_value = houghTransform.at<float>(v, u);
				center.setX(u + 0.5f);
				center.setY(v + 0.5f);
			}
		}
	}

	// 投票結果を画像として保存する
	cv::Mat m;
	cv::flip(houghTransform, m, 0);
	m /= (max_value / 255.0f);
	m.convertTo(m, CV_8U);
	cv::imwrite(QString("result%1.jpg").arg(scale).toUtf8().data(), m);

	// radial の中心点を求める
	center /= scale;
	center += bbox.minPt;

	return center;
}

/**
 * 円のだいたいの中心点を使って、より正確な円の中心を返却する
 * Hough transformにより、円を検知する。
 */
QVector2D RoadSegmentationUtil::refineRadialCenterInScaled(RoadGraph& roads, AbstractArea& area, int roadType, float scale, QVector2D& centerApprox, float distanceThreshold, float angleThreshold) {
	BBox bbox = GraphUtil::getAABoundingBox(roads);
	cv::Mat houghTransform = cv::Mat::zeros(bbox.dy() * scale, bbox.dx() * scale, CV_32F);

	float sigma = bbox.dx() * scale * 0.05f;

	QVector2D c = (centerApprox - bbox.minPt) * scale;

	RoadEdgeIter ei, eend;
	for (boost::tie(ei, eend) = boost::edges(roads.graph); ei != eend; ++ei) {
		if (!roads.graph[*ei]->valid) continue;

		// 指定されたタイプ以外は、スキップする。
		if (!(roads.graph[*ei]->type & roadType)) continue;

		// 既にshapeTypeが確定しているエッジは、スキップする
		if (roads.graph[*ei]->shapeType > 0) continue;

		// 範囲の外のエッジはスキップする
		RoadVertexDesc src = boost::source(*ei, roads.graph);
		if (!area.contains(roads.graph[src]->pt)) continue;
		RoadVertexDesc tgt = boost::target(*ei, roads.graph);
		if (!area.contains(roads.graph[tgt]->pt)) continue;

		for (int i = 0; i < roads.graph[*ei]->polyLine.size() - 1; i++) {
			QVector2D v1 = (roads.graph[*ei]->polyLine[i] - bbox.minPt) * scale;
			QVector2D v2 = (roads.graph[*ei]->polyLine[i + 1] - bbox.minPt) * scale;

			// だいたいの中心点から離れすぎたエッジは、スキップする
			if (Util::pointSegmentDistanceXY(v1, v2, c, false) > distanceThreshold) continue;

			QVector2D dir = v2 - v1;

			// だいたいの中心点への方向が大きくずれている場合は、スキップする
			if (GraphUtil::diffAngle(v1 - c, dir) > angleThreshold && GraphUtil::diffAngle(v1 - c, -dir) > angleThreshold) continue;
			
			float len = dir.length();

			if (dir.x() > dir.y()) {
				for (int x = 0; x < houghTransform.cols; x++) {
					int y = dir.y() * ((float)x - v1.x()) / dir.x() + v1.y() + 0.5f;
					if (y < 0 || y >= houghTransform.rows) continue;

					float h = 0;
					if (x >= std::min(v1.x(), v2.x()) && x <= std::max(v1.x(), v2.x())) {
						h = len;
					} else if (x < std::min(v1.x(), v2.x())) {
						h = len * expf(-SQR(x - std::min(v1.x(), v2.x())) / 2.0f / SQR(sigma));
					} else {
						h = len * expf(-SQR(x - std::max(v1.x(), v2.x())) / 2.0f / SQR(sigma));
					}

					houghTransform.at<float>(y, x) += h;
				}
			} else {
				for (int y = 0; y < houghTransform.rows; y++) {
					int x = dir.x() * ((float)y - v1.y()) / dir.y() + v1.x() + 0.5f;
					if (x < 0 || x >= houghTransform.cols) continue;

					float h = 0;
					if (y >= std::min(v1.y(), v2.y()) && y <= std::max(v1.y(), v2.y())) {
						h = len;
					} else if (y < std::min(v1.y(), v2.y())) {
						h = len * expf(-SQR(y - std::min(v1.y(), v2.y())) / 2.0f / SQR(sigma));
					} else {
						h = len * expf(-SQR(y - std::max(v1.y(), v2.y())) / 2.0f / SQR(sigma));
					}

					houghTransform.at<float>(y, x) += h;
				}
			}
		}
	}

	// 最大値を取得する
	QVector2D center;
	float max_value = 0.0f;
	for (int v = 0; v < houghTransform.rows; v++) {
		for (int u = 0; u < houghTransform.cols; u++) {
			if (houghTransform.at<float>(v, u) > max_value) {
				max_value = houghTransform.at<float>(v, u);
				center.setX(u + 0.5f);
				center.setY(v + 0.5f);
			}
		}
	}

	// 投票結果を画像として保存する
	cv::Mat m;
	cv::flip(houghTransform, m, 0);
	m /= (max_value / 255.0f);
	m.convertTo(m, CV_8U);
	cv::imwrite(QString("result%1.jpg").arg(scale).toUtf8().data(), m);

	// radial の中心点を求める
	center /= scale;
	center += bbox.minPt;

	return center;
}

/**
 * 円の中心からdistanceThreshold以内のRadialグループのエッジのみを残し、それ以外はグループから外す。
 *
 * @param distanceThreshold		円の中心から、この距離よりも遠いエッジは、グループから外す
 * @param minNumSeeds			残ったエッジの数がこの値未満の場合は、全てのエッジをグループから外し、falseを返す
 */
bool RoadSegmentationUtil::reduceRadialGroup(RoadGraph& roads, QVector2D& center, int group_id, float distanceThreshold, int minNumSeeds) {
	float distanceThreshold2 = distanceThreshold * distanceThreshold;

	int count = 0;
	RoadEdgeIter ei, eend;
	for (boost::tie(ei, eend) = boost::edges(roads.graph); ei != eend; ++ei) {
		if (!roads.graph[*ei]->valid) continue;

		// 異なるshapeTypeのエッジは、スキップする
		if (roads.graph[*ei]->shapeType != RoadEdge::SHAPE_RADIAL) continue;

		// 異なるグループのエッジは、スキップする
		if (roads.graph[*ei]->group != group_id) continue;

		RoadVertexDesc src = boost::source(*ei, roads.graph);
		RoadVertexDesc tgt = boost::target(*ei, roads.graph);

		// 円の中心から離れているエッジは、グループから外す
		if ((roads.graph[src]->pt - center).lengthSquared() > distanceThreshold2 && (roads.graph[tgt]->pt - center).lengthSquared() > distanceThreshold2) {
			roads.graph[*ei]->shapeType = RoadEdge::SHAPE_DEFAULT;
			roads.graph[*ei]->group = -1;
		} else {
			count++;
		}
	}

	if (count >= minNumSeeds) return true;

	// 残ったエッジの数が少なすぎるので、全てグループから外す
	for (boost::tie(ei, eend) = boost::edges(roads.graph); ei != eend; ++ei) {
		if (!roads.graph[*ei]->valid) continue;

		// 異なるshapeTypeのエッジは、スキップする
		if (roads.graph[*ei]->shapeType != RoadEdge::SHAPE_RADIAL) continue;

		// 異なるグループのエッジは、スキップする
		if (roads.graph[*ei]->group != group_id) continue;

		roads.graph[*ei]->shapeType = RoadEdge::SHAPE_DEFAULT;
		roads.graph[*ei]->group = -1;
	}

	return false;
}

/**
 * 指定したグループに属するエッジについて、円の中心から離れる方向に周辺のエッジを辿り、グループに登録していく。
 */
void RoadSegmentationUtil::extendRadialGroup(RoadGraph& roads, AbstractArea& area, int roadType, QVector2D& center, int group_id, float angleThreshold, float dirCheckRatio) {
	QList<RoadVertexDesc> queue;

	QList<RoadVertexDesc> visited;

	QMap<RoadEdgeDesc, bool> edges;

	// シードの構築
	RoadEdgeIter ei, eend;
	for (boost::tie(ei, eend) = boost::edges(roads.graph); ei != eend; ++ei) {
		if (!roads.graph[*ei]->valid) continue;

		// 指定されたタイプ以外は、スキップする。
		if (!(roads.graph[*ei]->type & roadType)) continue;

		// radialタイプではないエッジは、スキップする
		if (roads.graph[*ei]->shapeType != RoadEdge::SHAPE_RADIAL) continue;

		// 異なるグループのエッジは、スキップする
		if (roads.graph[*ei]->group != group_id) continue;

		// 範囲の外のエッジは除外する
		RoadVertexDesc src = boost::source(*ei, roads.graph);
		if (!area.contains(roads.graph[src]->pt)) continue;
		RoadVertexDesc tgt = boost::target(*ei, roads.graph);
		if (!area.contains(roads.graph[tgt]->pt)) continue;

		queue.push_back(src);
		queue.push_back(tgt);
		visited.push_back(src);
		visited.push_back(tgt);

		edges.insert(*ei, true);
	}

	while (!queue.empty()) {
		RoadVertexDesc v = queue.front();
		queue.pop_front();

		// 範囲の外の頂点は除外する
		if (!area.contains(roads.graph[v]->pt)) continue;

		RoadOutEdgeIter ei, eend;
		for (boost::tie(ei, eend) = boost::out_edges(v, roads.graph); ei != eend; ++ei) {
			if (!roads.graph[*ei]->valid) continue;

			float length = 0.0f;
			for (int i = 0; i < roads.graph[*ei]->polyLine.size() - 1; i++) {
				QVector2D dir1 = roads.graph[*ei]->polyLine[i + 1] - roads.graph[*ei]->polyLine[i];
				QVector2D dir2 = roads.graph[*ei]->polyLine[i] - center;

				if (GraphUtil::diffAngle(dir1, dir2) < angleThreshold || GraphUtil::diffAngle(dir1, -dir2) < angleThreshold) {
					length += dir1.length();
				}
			}

			// 方向がずれてたら、そのエッジは当該グループに入れない
			if (length < roads.graph[*ei]->getLength() * dirCheckRatio) continue;

			// グループに追加する
			roads.graph[*ei]->shapeType = RoadEdge::SHAPE_RADIAL;
			roads.graph[*ei]->group = group_id;
			roads.graph[*ei]->gridness = 0.0f;

			RoadVertexDesc u = boost::target(*ei, roads.graph);
			if (!roads.graph[u]->valid) continue;
			if (visited.contains(u)) continue;

			// シードとして、追加する
			visited.push_back(u);
			queue.push_back(u);
		}
	}
}

/**
 * ラウンドアバウトを検知する。
 */
void RoadSegmentationUtil::detectRoundabout(RoadGraph& roads, AbstractArea& area) {
}
