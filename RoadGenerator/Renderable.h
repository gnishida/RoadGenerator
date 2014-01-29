#pragma once

#include <QtOpenGL>
#include <boost/shared_ptr.hpp>

typedef struct {
	float location[3];
	float tex[2];
	float normal[3];
	float color[4];
	char padding[16];
} Vertex;

class Renderable {
public:
	GLenum type;
	float size;
	std::vector<Vertex> vertices;

public:
	Renderable(GLenum type);
	Renderable(GLenum type, float size);
	~Renderable();

	void addQuad(QVector3D p0, QVector3D p1, QVector3D p2, QVector3D p3, QVector3D n, QColor color);
};

typedef boost::shared_ptr<Renderable> RenderablePtr;