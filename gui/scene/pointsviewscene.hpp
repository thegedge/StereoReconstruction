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
#ifndef POINTSVIEWSCENE_H
#define POINTSVIEWSCENE_H

#include "sceneviewer.hpp"

//
//
//
class QGLWidget;

//! A scene for viewing a set of points.
class PointsViewScene : public Scene {
public:
    PointsViewScene();
	~PointsViewScene();

public:
	//! Set the points to be shown.
	void setPoints(std::vector<GLfloat> &verticies, std::vector<GLuint> &indicies);

public:
	//
	// Scene implementation / overrides
	//
	bool isAnimated() const { return animated; }

	void initialize();
	void onPaint(QGLWidget *parent);
	void onResize(int w, int h);
	void onShow();
	void onHide();

	bool onMousePress(QMouseEvent *evt);
	bool onMouseMove(QMouseEvent *evt);
	bool onMouseWheel(QWheelEvent *evt);

private:
	void drawBounds();

private:
	static const int NUM_PASSES = 3;

	std::vector<GLfloat> verticies; // interleaved (position, normal, color) values
	std::vector<GLuint> indicies; // (v1, v2, v3) tri indicies
	GLuint vbo;

	GLuint vertexShaders[NUM_PASSES], fragmentShaders[NUM_PASSES], programs[NUM_PASSES];
	GLint uniformWindowSize[2], uniformSplatRadius[2], uniformViewDir[2];

	GLuint frameBuffer;
	GLuint depthBuffer, colorTexture;

	double transx, transy, transz, scale;
	double point_size;       // size of rendered points
	double zoom;             // zoom level
	double rotx, roty, rotz; // rotations for viewing models
	double lastx, lasty;     // mouse movement
	int width, height;

	bool animated;
};

#endif // POINTSVIEWSCENE_H
