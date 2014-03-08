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
#ifndef CAMERALAYOUTSCENE_H
#define CAMERALAYOUTSCENE_H

#include <QFont>
#include <QGLWidget>
#include <QMap>
#include <QString>


FORWARD_DECLARE(Project);
FORWARD_DECLARE(Camera);

//
//
//
class QGLWidget;

//! An OpenGL scene that shows the layout of a project's cameras
class CameraLayoutScene : public QGLWidget {
	Q_OBJECT

public:
    CameraLayoutScene();
	virtual ~CameraLayoutScene();

public:
	void initializeGL();
	void paintGL();
	void resizeGL(int w, int h);

	void show();
	void hide();

	void mouseMoveEvent(QMouseEvent *evt);
	void mousePressEvent(QMouseEvent *evt);
	void mouseReleaseEvent(QMouseEvent *evt);
	void wheelEvent(QWheelEvent *evt);

public slots:
	void updateCameras();
	void setSelectedCamera(CameraPtr cam);
	void setProject(ProjectPtr project);

private:
	void updateViewMatrix();

private:
	ProjectPtr project;
	CameraWeakPtr selectedCamera;
	QFont font;

	double cx, cy, cz;          // centroid of the camera cloud
	double fx, fy, fz;          // multiplicative factors to bring cams to unit cube
	double radius, theta, phi;  // spherical coordinates
	int lastx, lasty;           // mouse movement
	int width, height;
};

#endif // CAMERALAYOUTSCENE_H
