#pragma once

#include <QtOpenGL>

class Camera {
public:
	float xrot;
	float yrot;
	float zrot;

	float dx;
	float dy;
	float dz;

	float lookAtX;
	float lookAtY;
	float lookAtZ;

	float fovy;

	Camera() {
		xrot = 0.0f;//-75.0;
		yrot = 0.0;
		zrot = 0.0f;//-45.0;
		dx = 0.0;
		dy = 0.0;
		dz = 100.0;
		lookAtX = 0.0f;
		lookAtY = 0.0f;
		lookAtZ = 0.0f;
		fovy = 60.0f;
	}

	QVector4D getCamPos() {
		GLfloat m[16];
		glGetFloatv(GL_MODELVIEW_MATRIX, m);
		QMatrix4x4 mvMat(m[0], m[1], m[2], m[3],
		m[4], m[5], m[6], m[7],
		m[8], m[9], m[10], m[11],
		m[12], m[13], m[14], m[15]);

		QVector4D eye(0.0f, 0.0f, 0.0f, 1.0f);
		return ((mvMat.transposed()).inverted())*eye;
	}

	void applyCamTransform() {
		glLoadIdentity();
		glTranslatef(-dx, -dy, -dz);
		glRotatef(xrot, 1.0, 0.0, 0.0);		
		glRotatef(yrot, 0.0, 1.0, 0.0);
		glRotatef(zrot, 0.0, 0.0, 1.0);
		glTranslatef(-lookAtX, -lookAtY, -lookAtZ);
	}

	static void qNormalizeAngle(float &angle) {
		while (angle < 0)
			angle += 360.0;
		while (angle > 360.0)
			angle -= 360.0;
	}

	float getCamElevation() {	
		return getCamPos().z();
	}

	void setRotation(float x, float y, float z) {
		setXRotation(x);
		setYRotation(y);
		setZRotation(z);		
	}

	void setXRotation(float angle) {
		qNormalizeAngle(angle);
		xrot = angle;			
	}

	void setYRotation(float angle) {
		qNormalizeAngle(angle);
		yrot = angle;			
	}

	void setZRotation(float angle) {
		qNormalizeAngle(angle);
		zrot = angle;			
	}

	void changeXRotation(float angle) {
		setXRotation(xrot+angle);
	}

	void changeYRotation(float angle) {
		setYRotation(yrot+angle);
	}

	void changeZRotation(float angle) {
		setZRotation(zrot+angle);
	}

	void setTranslation(float x, float y, float z) {
		dx = x;
		dy = y;
		dz = z;
	}

	void changeXYZTranslation(float x, float y, float z) {
		dx += x;
		dy += y;
		dz += z;
		//printf("dx: %f,dy: %f, dz: %f\n",dx,dy,dz);
	}

	void setLookAt(float x, float y, float z) {
		lookAtX = x;
		lookAtY = y;
		lookAtZ = z;
	}
};

