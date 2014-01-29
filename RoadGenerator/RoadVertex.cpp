#include "RoadVertex.h"

RoadVertex::RoadVertex() {
	this->pt = QVector2D(0.0f, 0.0f);
	this->virt = false;
	this->valid = true;
}

RoadVertex::RoadVertex(const QVector2D &pt) {
	this->pt = pt;
	this->virt = false;
	this->valid = true;
}

const QVector2D& RoadVertex::getPt() const {
	return pt;
}