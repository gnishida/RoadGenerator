#include "RoadGraphRenderer.h"
#include <QtOpenGL>

RoadGraphRenderer::RoadGraphRenderer() {
}

void RoadGraphRenderer::render(std::vector<RenderablePtr>& renderables) {
	for (int i = 0; i < renderables.size(); i++) {
		renderOne(renderables[i]);
	}
}

void RoadGraphRenderer::renderOne(RenderablePtr renderable) {
	if (renderable->type == GL_LINE_STIPPLE) {
		glEnable(GL_LINE_STIPPLE);
		glLineStipple(1 , 0xF0F0);
		glLineWidth(renderable->size);
	} else {
		glDisable(GL_LINE_STIPPLE);
	}

	if (renderable->type == GL_LINES || renderable->type == GL_LINE_STRIP) {
		glLineWidth(renderable->size);
	}

	if (renderable->type == GL_POINTS) {
		glPointSize(renderable->size);
	}

	if (renderable->type != GL_LINE_STIPPLE) {
		glBegin(renderable->type);
	} else {
		glBegin(GL_LINE_STRIP);
	}
	for (int j = 0; j < renderable->vertices.size(); ++j) {
		glColor3f(renderable->vertices[j].color[0], renderable->vertices[j].color[1], renderable->vertices[j].color[2]);
		glNormal3f(renderable->vertices[j].normal[0], renderable->vertices[j].normal[1], renderable->vertices[j].normal[2]);
		glVertex3f(renderable->vertices[j].location[0], renderable->vertices[j].location[1], renderable->vertices[j].location[2]);
	}
	glEnd();
}

void RoadGraphRenderer::renderArea(const AbstractArea& area, float height) {
	std::vector<RenderablePtr> renderables;
	renderables.push_back(RenderablePtr(new Renderable(GL_LINE_STIPPLE, 3.0f)));
	renderables.push_back(RenderablePtr(new Renderable(GL_POINTS, 10.0f)));

	Vertex v;

	v.color[0] = 0.0f;
	v.color[1] = 0.0f;
	v.color[2] = 1.0f;
	v.normal[0] = 0.0f;
	v.normal[1] = 0.0f;
	v.normal[2] = 1.0f;

	for (int i = 0; i < area.polyline().size(); i++) {
		v.location[0] = area.polyline()[i].x();
		v.location[1] = area.polyline()[i].y();
		v.location[2] = height;
		renderables[0]->vertices.push_back(v);
		renderables[1]->vertices.push_back(v);
	}

	v.location[0] = area.polyline()[0].x();
	v.location[1] = area.polyline()[0].y();
	v.location[2] = height;
	renderables[0]->vertices.push_back(v);

	render(renderables);
}

void RoadGraphRenderer::renderDenseArea(const AbstractArea& area, float height) {
	RenderablePtr renderable = RenderablePtr(new Renderable(GL_POINTS, 10.0f));

	Vertex v;
	v.color[0] = 1.0f;
	v.color[1] = 0.0f;
	v.color[2] = 0.0f;
	v.normal[0] = 0.0f;
	v.normal[1] = 0.0f;
	v.normal[2] = 1.0f;

	for (int y = -10000; y <= 10000; y+=10) {
		for (int x = -10000; x<= 10000; x+=10) {
			if (area.contains(QVector2D(x, y))) {
				v.location[0] = x;
				v.location[1] = y;
				v.location[2] = height;
				renderable->vertices.push_back(v);
			}
		}
	}

	renderOne(renderable);
}

void RoadGraphRenderer::renderPoint(const QVector2D& pt, float height) {
	RenderablePtr renderable = RenderablePtr(new Renderable(GL_POINTS, 10.0f));

	Vertex v;

	v.location[0] = pt.x();
	v.location[1] = pt.y();
	v.location[2] = height;
	v.color[0] = 0.0f;
	v.color[1] = 0.0f;
	v.color[2] = 1.0f;
	v.normal[0] = 0.0f;
	v.normal[1] = 0.0f;
	v.normal[2] = 1.0f;

	renderable->vertices.push_back(v);

	renderOne(renderable);
}

void RoadGraphRenderer::renderPolyline(std::vector<QVector2D>& polyline, float height) {
	std::vector<RenderablePtr> renderables;
	renderables.push_back(RenderablePtr(new Renderable(GL_LINE_STRIP, 3.0f)));
	renderables.push_back(RenderablePtr(new Renderable(GL_POINTS, 10.0f)));
	
	Vertex v;
	v.color[0] = 0.0f;
	v.color[1] = 0.0f;
	v.color[2] = 1.0f;
	v.normal[0] = 0.0f;
	v.normal[1] = 0.0f;
	v.normal[2] = 1.0f;

	// add lines
	for (int i = 0; i < polyline.size(); i++) {
		v.location[0] = polyline[i].x();
		v.location[1] = polyline[i].y();
		v.location[2] = height;

		renderables[0]->vertices.push_back(v);
		renderables[1]->vertices.push_back(v);
	}

	render(renderables);
}

