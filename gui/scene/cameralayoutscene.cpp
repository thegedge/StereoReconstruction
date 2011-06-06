//---------------------------------------------------------------------
//
// Copyright © 2011, Jason Gedge <gedge -at- ualberta -dot- ca>
//
// This file is part of StereoReconstruction.
//
// StereoReconstruction is free software: you can redistribute it and/or
// modify it under the terms of the GNU General Public License as published
// by the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// StereoReconstruction is distributed in the hope that it will be
// useful, but WITHOUT ANY WARRANTY; without even the implied warranty
// of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License along
// with StereoReconstruction. If not, see <http:www.gnu.org/licenses/>.
//
//---------------------------------------------------------------------
#define _USE_MATH_DEFINES
#include "cameralayoutscene.hpp"

#include <cmath>
#include <limits>
#include <algorithm>

#include <Eigen/Geometry>

#include <QGLWidget>
#include <QDebug>

#include "project/camera.hpp"

//---------------------------------------------------------------------

#ifndef M_PI
#   define M_PI      3.14159265358979323846
#endif

#ifndef M_PI_2
#   define M_PI_2    1.57079632679489661923
#endif

//---------------------------------------------------------------------

using std::min;
using std::fabs;

//---------------------------------------------------------------------

CameraLayoutScene::CameraLayoutScene()
	: radius(5.0), theta(M_PI_2), phi(M_PI_2)
	, lastx(0), lasty(0)
	, width(1), height(1)
{
	font.setBold(true);
	font.setPointSize(18);
}

CameraLayoutScene::~CameraLayoutScene()
{ }

//---------------------------------------------------------------------

void CameraLayoutScene::onShow() {
	glLineWidth(2);
	glDepthFunc(GL_LESS);
	glDisable(GL_CULL_FACE);
	glClearColor(1, 1, 1, 1);
	//glHint(GL_LINE_SMOOTH_HINT, GL_NICEST);
	//glHint(GL_PERSPECTIVE_CORRECTION_HINT, GL_NICEST);
	updateCameras();
}

void CameraLayoutScene::onHide() {
	glLineWidth(1);
}

//---------------------------------------------------------------------

void CameraLayoutScene::setSelectedCamera(const CameraPtr cam) {
	selectedCamera = cam;
	updateViewMatrix();
	updateCameras();
}

//---------------------------------------------------------------------

void CameraLayoutScene::setCameras(const std::vector<CameraPtr> &cams) {
	cameras = cams;
	selectedCamera.reset();
	updateCameras();
}

void CameraLayoutScene::updateCameras() {
	cx = cy = cz = 0.0;
	foreach(CameraPtr cam, cameras) if(cam) {
		const Eigen::Vector3d &C = cam->C();
		cx += C[0];
		cy += C[1];
		cz += C[2];
	}

	cx /= cameras.size();
	cy /= cameras.size();
	cz /= cameras.size();

	fx = fy = fz = std::numeric_limits<double>::infinity();
	foreach(CameraPtr cam, cameras) if(cam) {
		const Eigen::Vector3d &C = cam->C();
		fx = min(fx, 1.0 / fabs(C[0] - cx));
		fy = min(fy, 1.0 / fabs(C[1] - cy));
		fz = min(fz, 1.0 / fabs(C[2] - cz));
	}

	if(std::isinf(fx)) fx = 1;
	if(std::isinf(fy)) fy = 1;
	if(std::isinf(fz)) fz = 1;

	fx = fy = fz = 5*std::min(std::min(fx, fy), fz);

	updateViewMatrix();
}

//---------------------------------------------------------------------

void CameraLayoutScene::onResize(int w, int h) {
	GLdouble ratio = static_cast<GLdouble>(w) / h;
	width = w;
	height = h;

	// Reset projection matrix
	glViewport(0, 0, w, h);

	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	gluPerspective(45.0, ratio, 0.5, 128.0);

	// Reset modelview matrix
	updateViewMatrix();
}

//---------------------------------------------------------------------

void CameraLayoutScene::onPaint(QGLWidget *parent) {
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	if(cameras.size() > 0) {
		//
		// Camera local coordinate systems
		//
		glBegin(GL_LINES);
		foreach(CameraPtr cam, cameras) {
			Eigen::Matrix3d R = 0.5*cam->R();
			const Eigen::Vector3d &C = cam->C();

			double x = (C[0] - cx)*fx;
			double y = (C[1] - cy)*fy;
			double z = (C[2] - cz)*fz;

			if(cam == selectedCamera) glColor3f(1, 1, 1);

			//
			if(cam != selectedCamera) glColor3f(1, 0, 0);
			glVertex3f(x, y, z);
			glVertex3f(x + R(0,0), y + R(0,1), z + R(0,2));
			if(cam != selectedCamera) glColor3f(0, 1, 0);
			glVertex3f(x, y, z);
			glVertex3f(x + R(1,0), y + R(1,1), z + R(1,2));
			if(cam != selectedCamera) glColor3f(0, 0, 1);
			glVertex3f(x, y, z);
			glVertex3f(x + R(2,0), y + R(2,1), z + R(2,2));
		}
		glEnd();

		// Refractive interfaces
		glEnable(GL_BLEND);
		glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
		foreach(CameraPtr cam, cameras) {
			if(cam->isRefractive()) {
				// Find verticies of a quad that represents the plane
				Plane3d plane = cam->fromLocalToGlobal(cam->plane());

				const Eigen::Matrix3d &R = cam->R();
				double x = (plane.x0().x() - cx)*fx;
				double y = (plane.x0().y() - cy)*fy;
				double z = (plane.x0().z() - cz)*fz;

				Eigen::Vector3d p(x, y, z);
				Eigen::Vector3d vz = plane.normal();
				Eigen::Vector3d vx = vz.cross(R.row(1)).normalized();
				Eigen::Vector3d vy = vx.cross(vz);

				//vx *= 0.5;
				//vy *= 0.5;

				//glColor4f(1, 1, 1, 0.5);
				glColor4f(0, 0, 0, 0.5);
				glBegin(GL_QUADS);
					p += vx + vy;
					glVertex3f(p.x(), p.y(), p.z());
					p -= 2*vy;
					glVertex3f(p.x(), p.y(), p.z());
					p -= 2*vx;
					glVertex3f(p.x(), p.y(), p.z());
					p += 2*vy;
					glVertex3f(p.x(), p.y(), p.z());
				glEnd();

				p = Eigen::Vector3d(x, y, z);

				//glColor4f(1, 1, 1, 1);
				glColor4f(0, 0, 0, 1);
				glBegin(GL_LINE_LOOP);
					p += vx + vy;
					glVertex3f(p.x(), p.y(), p.z());
					p -= 2*vy;
					glVertex3f(p.x(), p.y(), p.z());
					p -= 2*vx;
					glVertex3f(p.x(), p.y(), p.z());
					p += 2*vy;
					glVertex3f(p.x(), p.y(), p.z());
				glEnd();
			}
		}

		// Camera labels
		glDisable(GL_DEPTH_TEST);
		glColor3f(1, 1, 1);
		foreach(CameraPtr cam, cameras) {
			const Eigen::Vector3d &C = cam->C();
			parent->renderText(
					(C[0] - cx)*fx,
					(C[1] - cy)*fy,
					(C[2] - cz)*fz,
					cam->name(),
					font);
		}
		glEnable(GL_DEPTH_TEST);
	}
}

//---------------------------------------------------------------------

void CameraLayoutScene::updateViewMatrix() {
	double rx = radius*cos(theta)*sin(phi);
	double ry = radius*           cos(phi);
	double rz = radius*sin(theta)*sin(phi);

# if 0
	if(selectedCamera) {
		const Eigen::Matrix3d &R = selectedCamera->R();
		const Eigen::Vector3d &C = selectedCamera->C();

		double x = C[0]*fx;
		double y = C[1]*fy;
		double z = C[2]*fz;

		double lx = x + rx*R[0] + ry*R[1] + rz*R[2];
		double ly = y + rx*R[3] + ry*R[4] + rz*R[5];
		double lz = z + rx*R[6] + ry*R[7] + rz*R[8];

		double ux = R[3];
		double uy = R[4];
		double uz = R[5];

		GLdouble ratio = static_cast<GLdouble>(width) / height;
		glMatrixMode(GL_PROJECTION);
		glLoadIdentity();
		gluPerspective(45.0 + 25.0*((radius - 50)*0.02), ratio, 0.5, 128.0);

		glMatrixMode(GL_MODELVIEW);
		glLoadIdentity();
		gluLookAt(x, y, z, lx, ly, lz, ux, uy, uz);
#else
	if(selectedCamera) {
		const Eigen::Matrix3d &R = selectedCamera->R();
		const Eigen::Vector3d &C = selectedCamera->C();

		double x = (C[0] - cx)*fx;
		double y = (C[1] - cy)*fy;
		double z = (C[2] - cz)*fz;

		glMatrixMode(GL_MODELVIEW);
		glLoadIdentity();
		gluLookAt(x + rx, y + ry, z + rz, x, y, z, -R(1, 0), -R(1, 1), -R(1, 2));
#endif
	} else {
		GLdouble ratio = static_cast<GLdouble>(width) / height;
		glMatrixMode(GL_PROJECTION);
		glLoadIdentity();
		glOrtho(-radius*ratio, radius*ratio, -radius, radius, 0.125, 128.0);

		glMatrixMode(GL_MODELVIEW);
		glLoadIdentity();
		//gluLookAt(cx*fx + rx, cy*fy + ry, cz*fz + rz, cx*fx, cy*fy, cz*fz, 0, -1, 0);
		gluLookAt(rx, ry, rz, 0, 0, 0, 0, -1, 0);
	}
}

//---------------------------------------------------------------------

bool CameraLayoutScene::onMousePress(QMouseEvent *evt) {
	lastx = evt->x();
	lasty = evt->y();
	return false;
}

//---------------------------------------------------------------------

bool CameraLayoutScene::onMouseMove(QMouseEvent *evt) {
	if(evt->buttons() == Qt::NoButton)
		return false;

	int deltax = evt->x() - lastx;
	int deltay = evt->y() - lasty;
	if(deltax != 0 || deltay != 0) {
		theta -= (M_PI*deltax) / width;
		//phi -= (M_PI_2*deltay) / height;
		phi += (M_PI_2*deltay) / height;

		if(theta < 0)      theta += 2*M_PI;
		if(theta > 2*M_PI) theta -= 2*M_PI;

		if(phi < 0)    phi += M_PI;
		if(phi > M_PI) phi -= M_PI;

		updateViewMatrix();

		lastx = evt->x();
		lasty = evt->y();
		return true;
	}

	return false;
}

//---------------------------------------------------------------------

bool CameraLayoutScene::onMouseWheel(QWheelEvent *evt) {
	double scale = -120.0;
	if(evt->modifiers() & Qt::ControlModifier)
		scale = -24.0;

	radius += evt->delta() / scale;

	if(radius < 1)        radius = 1;
	else if(radius > 100) radius = 100;

	updateViewMatrix();

	return true;
}

//---------------------------------------------------------------------
