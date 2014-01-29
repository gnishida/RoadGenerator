#include "Renderable.h"

Renderable::Renderable(GLenum type) {
	this->type = type;
	this->size = 1.0f;
}

Renderable::Renderable(GLenum type, float size) {
	this->type = type;
	this->size = size;
}

Renderable::~Renderable() {
}

void Renderable::addQuad(QVector3D p0, QVector3D p1, QVector3D p2, QVector3D p3, QVector3D n, QColor color) {
	Vertex v;

	// set the color
	v.color[0] = color.redF();
	v.color[1] = color.greenF();
	v.color[2] = color.blueF();

	// set the normal
	v.normal[0] = n.x();
	v.normal[1] = n.y();
	v.normal[2] = n.z();

	// set the coordinate of 3 vertices of the 1st triangle
	v.location[0] = p0.x();
	v.location[1] = p0.y();
	v.location[2] = p0.z();
	vertices.push_back(v);

	v.location[0] = p1.x();
	v.location[1] = p1.y();
	v.location[2] = p1.z();
	vertices.push_back(v);

	v.location[0] = p2.x();
	v.location[1] = p2.y();
	v.location[2] = p2.z();
	vertices.push_back(v);

	// set the coordinate of 3 vertices of the 2nd triangle
	v.location[0] = p0.x();
	v.location[1] = p0.y();
	v.location[2] = p0.z();
	vertices.push_back(v);

	v.location[0] = p2.x();
	v.location[1] = p2.y();
	v.location[2] = p2.z();
	vertices.push_back(v);

	v.location[0] = p3.x();
	v.location[1] = p3.y();
	v.location[2] = p3.z();
	vertices.push_back(v);
}
