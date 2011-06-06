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
#ifndef SCENEVIEWER_H
#define SCENEVIEWER_H

#ifdef PLATFORM_WIN
#   include <GL/glew.h>
#   include <GL/wglew.h>
#endif

#include <QGLWidget>
#include <QKeyEvent>
#include <QMouseEvent>
#include <QTimer>
#include <QWheelEvent>

#include "util/c++0x.hpp"

/*!
 * A scene that can be displayed in a SceneViewer widget. Basically, any
 * concrete implementation of this class acts as a delegate for the following
 * QGLWidget functions:
 *
 *    onPaint        -> paintGL
 *    onResize       -> resizeGL
 *    onMousePress   -> mousePressEvent
 *    onMouseRelease -> mouseReleaseEvent
 *    onMouseMove    -> mouseMoveEvent
 *    onMouseWheel   -> wheelEvent
 *    onKeyPress     -> keyPressEvent
 *    onKeyRelease   -> keyReleaseEvent
 *
 * The special onShow event is for when the scene is initially shown in the
 * QGLWidget. All key/mouse events return can return a boolean value indicating
 * whether they need a repaint (true) or not (false). The base abstract class
 * also provides helper functions, such as shader loading routines.
 */
class Scene {
public:
	Scene() : initialized_(false) { }
	virtual ~Scene() { }

	bool isInitialized() const { return initialized_; }
	virtual void initialize() { }

	virtual bool isAnimated() const { return false; }

	virtual void onPaint(QGLWidget *parent) = 0;
	virtual void onResize(int w, int h) = 0;
	virtual void onShow() = 0; // set up OpenGL state
	virtual void onHide() = 0; // restore OpenGL state

	virtual bool onMousePress(QMouseEvent *)  { return false; }
	virtual bool onMouseRelease(QMouseEvent *) { return false; }
	virtual bool onMouseMove(QMouseEvent *) { return false; }
	virtual bool onMouseWheel(QWheelEvent *) { return false; }
	virtual bool onKeyPress(QKeyEvent *) { return false; }
	virtual bool onKeyRelease(QKeyEvent *) { return false; }

protected:
	static GLuint loadShader(GLenum type, const char *file);
	static void loadProgram(const char *vsFile, const char *fsFile, GLuint &vs, GLuint &fs, GLuint &program);

private:
	bool initialized_;
};

//! A widget for showing an OpenGL \ref Scene
class SceneViewer : public QGLWidget {
	Q_OBJECT;

public:
    SceneViewer(QWidget *parent = 0);

public:
	//! Set the scene to be viewed. Use NULL to clear the scene.
	void setScene(std::shared_ptr<Scene> scene);

protected:
	void mouseMoveEvent(QMouseEvent *evt);
	void mousePressEvent(QMouseEvent *evt);
	void mouseReleaseEvent(QMouseEvent *evt);
	void wheelEvent(QWheelEvent *evt);

	void keyPressEvent(QKeyEvent *evt);
	void keyReleaseEvent(QKeyEvent *evt);

	void resizeGL(int w, int h);
	void initializeGL();
	void paintGL();

private slots:
	void animate();

private:
	QTimer animationTimer;

	std::shared_ptr<Scene> scene;
};

#endif // SCENEVIEWER_H
