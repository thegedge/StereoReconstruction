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
#include "sceneviewer.hpp"

#include <QDebug>

//---------------------------------------------------------------------

SceneViewer::SceneViewer(QWidget *parent)
	: QGLWidget(parent)
{
	setMouseTracking(true);

	animationTimer.setInterval(1029 / 30);
	animationTimer.setSingleShot(false);
	connect(&animationTimer, SIGNAL(timeout()), SLOT(animate()));
	animationTimer.start();
}

//---------------------------------------------------------------------

void SceneViewer::initializeGL() {
#ifdef PLATFORM_WIN
	if(glewInit() != GLEW_OK)
		qDebug() << "Uh oh!";
#endif

	//
	// Enable/disable various OpenGL stuff
	//
	glClearColor(0, 0, 0, 0);
	glPixelStorei(GL_UNPACK_ALIGNMENT, 1);

	glEnable(GL_DEPTH_TEST);
	glEnable(GL_TEXTURE_2D);
	glEnable(GL_LINE_STIPPLE);
	glEnable(GL_POINT_SPRITE);
	glEnable(GL_VERTEX_PROGRAM_POINT_SIZE);
	glEnable(GL_CULL_FACE);
	glEnable(GL_COLOR_MATERIAL);
	glDisable(GL_NORMALIZE);
	glDisable(GL_DITHER);

#ifndef PLATFORM_WIN
	glPointParameteri(GL_POINT_SPRITE_COORD_ORIGIN, GL_LOWER_LEFT);
#endif
	glTexEnvi(GL_POINT_SPRITE, GL_COORD_REPLACE, GL_TRUE);
	glTexEnvi(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_REPLACE);
	glBlendFunc(GL_ONE, GL_ONE);
	glDepthFunc(GL_LEQUAL);
}

//---------------------------------------------------------------------

void SceneViewer::resizeGL(int w, int h) {
	if(scene && isValid()) scene->onResize(w, h);
}

//---------------------------------------------------------------------

void SceneViewer::paintGL() {
	if(isValid()) {
		if(scene) scene->onPaint(this);
		else      glClear(GL_COLOR_BUFFER_BIT);
	}
}

//---------------------------------------------------------------------

void SceneViewer::animate() {
	if(scene && scene->isAnimated())
		updateGL();
}

//---------------------------------------------------------------------

void SceneViewer::setScene(std::shared_ptr<Scene> scene) {
	if(isValid()) {
		makeCurrent();
		if(scene != this->scene) {
			if(this->scene)
				this->scene->onHide();

			this->scene = scene;
			if(scene) {
				if(!scene->isInitialized())
					scene->initialize();

				scene->onResize(width(), height());
				scene->onShow();
			}
		}
		updateGL();
	}
}

//---------------------------------------------------------------------
// MOUSE EVENTS
//
void SceneViewer::mousePressEvent(QMouseEvent *evt) {
	setCursor(QCursor(Qt::ClosedHandCursor));
	if(scene && scene->onMousePress(evt))
		updateGL();
}

void SceneViewer::mouseReleaseEvent(QMouseEvent *evt) {
	setCursor(QCursor(Qt::OpenHandCursor));
	if(scene && scene->onMouseRelease(evt))
		updateGL();
}

void SceneViewer::mouseMoveEvent(QMouseEvent *evt) {
	if(scene && scene->onMouseMove(evt))
		updateGL();
}

void SceneViewer::wheelEvent(QWheelEvent *evt) {
	if(scene && scene->onMouseWheel(evt))
		updateGL();
}


//---------------------------------------------------------------------
// KEY EVENTS
//
void SceneViewer::keyPressEvent(QKeyEvent *evt) {
	if(scene && scene->onKeyPress(evt))
		updateGL();
}

void SceneViewer::keyReleaseEvent(QKeyEvent *evt) {
	if(scene && scene->onKeyRelease(evt))
		updateGL();
}

//---------------------------------------------------------------------
