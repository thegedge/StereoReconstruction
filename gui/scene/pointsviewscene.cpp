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
#include "pointsviewscene.hpp"

#define _USE_MATH_DEFINES

#include <algorithm>
#include <limits>
#include <math.h>

#include <QDebug>
#include <QGLWidget>

//---------------------------------------------------------------------

#ifndef M_PI
#   define M_PI      3.14159265358979323846
#endif

#ifndef M_PI_2
#   define M_PI_2    1.57079632679489661923
#endif

#ifndef M_1_PI
#   define M_1_PI    0.31830988618379067154
#endif

//---------------------------------------------------------------------

PointsViewScene::PointsViewScene()
	: frameBuffer(0)
	, depthBuffer(0), colorTexture(0)
    , transx(-1), transy(-1), transz(-1), scale(-1)
	, point_size(1)
	, zoom(5)
	, rotx(0.3), roty(M_PI), rotz(M_PI)
	, lastx(0), lasty(0)
	, width(1), height(1)
	, animated(false)
{ }

//---------------------------------------------------------------------

PointsViewScene::~PointsViewScene() {
#ifdef USE_SPLATS
	if(isInitialized()) {
		for(int pass = 0; pass < NUM_PASSES; ++pass) {
			glDeleteShader(vertexShaders[pass]);
			glDeleteShader(fragmentShaders[pass]);
			glDeleteProgram(programs[pass]);
		}

		glDeleteRenderbuffersEXT(1, &depthBuffer);
		glDeleteTextures(1, &colorTexture);
		glDeleteFramebuffersEXT(1, &frameBuffer);
	}
#endif
}

//---------------------------------------------------------------------

void PointsViewScene::initialize() {
#ifdef USE_SPLATS
	//
	// Create FBO and textures
	//
	glGenTextures(1, &colorTexture);
	glGenRenderbuffersEXT(1, &depthBuffer);

	glGenFramebuffersEXT(1, &frameBuffer);
	glBindFramebufferEXT(GL_FRAMEBUFFER_EXT, frameBuffer);
		glClearColor(0, 0, 0, 0);
	glBindFramebufferEXT(GL_FRAMEBUFFER_EXT, 0);

	//
	// Load shaders
	//
	loadProgram("shaders/splats_pass1.vs", "shaders/splats_pass1.fs", vertexShaders[0], fragmentShaders[0], programs[0]);
	loadProgram("shaders/splats_pass2.vs", "shaders/splats_pass2.fs", vertexShaders[1], fragmentShaders[1], programs[1]);
	loadProgram("shaders/splats_pass3.vs", "shaders/splats_pass3.fs", vertexShaders[2], fragmentShaders[2], programs[2]);

	uniformWindowSize[0] = glGetUniformLocation(programs[0], "windowSize");
	uniformSplatRadius[0] = glGetUniformLocation(programs[0], "splatRadius");
	uniformViewDir[0] = glGetUniformLocation(programs[0], "viewDir");

	uniformWindowSize[1] = glGetUniformLocation(programs[1], "windowSize");
	uniformSplatRadius[1] = glGetUniformLocation(programs[1], "splatRadius");
	uniformViewDir[1] = glGetUniformLocation(programs[1], "viewDir");

	glUseProgram(programs[0]);
		glUniform2f(uniformWindowSize[0], 1, 1);
		glUniform1f(uniformSplatRadius[0], zoom);
	glUseProgram(programs[1]);
		glUniform2f(uniformWindowSize[1], 1, 1);
		glUniform1f(uniformSplatRadius[1], zoom);
	glUseProgram(0);
#endif
}

//---------------------------------------------------------------------

void PointsViewScene::onShow() {
	//glClearColor(0.1f, 0.1f, 0.1f, 0.0f);
	glClearColor(1.0f, 1.0f, 1.0f, 0.0f);
}

void PointsViewScene::onHide() {
	glClearColor(0.0f, 0.0f, 0.0f, 0.0f);
}

//---------------------------------------------------------------------

void PointsViewScene::onResize(int w, int h) {
	GLdouble ratio = static_cast<GLdouble>(w) / h;

	//
	// Reset projection matrix
	//
	glViewport(0, 0, w, h);

	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	gluPerspective(35.0, ratio, 0.1, 10.0);
	//glOrtho(-ratio, ratio, -1.2, 1.2, 0.1, 10.0);

	width = w;
	height = h;
#ifdef USE_SPLATS
	//
	// Update FBO textures
	//
	glBindTexture(GL_TEXTURE_2D, colorTexture);
		glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA32F_ARB, w, h, 0, GL_RGBA, GL_FLOAT, NULL);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
	glBindTexture(GL_TEXTURE_2D, 0);

	glBindRenderbufferEXT(GL_RENDERBUFFER_EXT, depthBuffer);
		glRenderbufferStorageEXT(GL_RENDERBUFFER_EXT, GL_DEPTH_COMPONENT32, w, h);
	glBindRenderbufferEXT(GL_RENDERBUFFER_EXT, 0);

	glBindFramebufferEXT(GL_FRAMEBUFFER_EXT, frameBuffer);
		glFramebufferTexture2DEXT(GL_FRAMEBUFFER_EXT, GL_COLOR_ATTACHMENT0_EXT, GL_TEXTURE_2D, colorTexture, 0);
		glFramebufferRenderbufferEXT(GL_FRAMEBUFFER_EXT, GL_DEPTH_ATTACHMENT_EXT, GL_RENDERBUFFER_EXT, depthBuffer);
	glBindFramebufferEXT(GL_FRAMEBUFFER_EXT, 0);

	glUseProgram(programs[0]);
		glUniform2f(uniformWindowSize[0], w, h);
	glUseProgram(programs[1]);
		glUniform2f(uniformWindowSize[1], w, h);
	glUseProgram(0);
#endif
}

//---------------------------------------------------------------------

void PointsViewScene::drawBounds() {
	glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
	glCullFace(GL_FRONT);
	glBegin(GL_QUADS);
		glColor3d(0.5, 0.5, 0.5);

		// front/back
		glVertex3d( 1.0f, -1.0f, 1.0f);
		glVertex3d( 1.0f,  1.0f, 1.0f);
		glVertex3d(-1.0f,  1.0f, 1.0f);
		glVertex3d(-1.0f, -1.0f, 1.0f);

		glVertex3d( 1.0f, -1.0f, -1.0f);
		glVertex3d(-1.0f, -1.0f, -1.0f);
		glVertex3d(-1.0f,  1.0f, -1.0f);
		glVertex3d( 1.0f,  1.0f, -1.0f);

		// left/right
		glVertex3d(-1.0f,  1.0f, -1.0f);
		glVertex3d(-1.0f, -1.0f, -1.0f);
		glVertex3d(-1.0f, -1.0f,  1.0f);
		glVertex3d(-1.0f,  1.0f,  1.0f);

		glVertex3d( 1.0f,  1.0f, -1.0f);
		glVertex3d( 1.0f,  1.0f,  1.0f);
		glVertex3d( 1.0f, -1.0f,  1.0f);
		glVertex3d( 1.0f, -1.0f, -1.0f);

		// top/bottom
		glVertex3d( 1.0f,  1.0f, -1.0f);
		glVertex3d(-1.0f,  1.0f, -1.0f);
		glVertex3d(-1.0f,  1.0f,  1.0f);
		glVertex3d( 1.0f,  1.0f,  1.0f);

		glVertex3d( 1.0f, -1.0f, -1.0f);
		glVertex3d( 1.0f, -1.0f,  1.0f);
		glVertex3d(-1.0f, -1.0f,  1.0f);
		glVertex3d(-1.0f, -1.0f, -1.0f);
	glEnd();
	glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
	glCullFace(GL_BACK);
}

//---------------------------------------------------------------------

void PointsViewScene::onPaint(QGLWidget *) {
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	if(verticies.size() > 0) {
		//
		glEnableClientState(GL_VERTEX_ARRAY);
		glEnableClientState(GL_NORMAL_ARRAY);
		glEnableClientState(GL_COLOR_ARRAY);
		glVertexPointer(3, GL_FLOAT, sizeof(float)*9, &(verticies[0]));
		glNormalPointer(   GL_FLOAT, sizeof(float)*9, &(verticies[3]));
		glColorPointer (3, GL_FLOAT, sizeof(float)*9, &(verticies[6]));

		//
		glMatrixMode(GL_MODELVIEW);
		glLoadIdentity();
		gluLookAt(0.0, 0.0, zoom, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0);

		if(animated)
			roty += 0.01;

		glRotated(180*rotx*M_1_PI, 1.0, 0.0, 0.0);
		glRotated(180*roty*M_1_PI, 0.0, 1.0, 0.0);
		glRotated(180*rotz*M_1_PI, 0.0, 0.0, 1.0);

		//`
		if(indicies.empty()) {
#ifdef USE_SPLATS
			glBindFramebufferEXT(GL_FRAMEBUFFER_EXT, frameBuffer);
			glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

			//
			// First pass, calculate depth information
			//
			glUseProgram(programs[0]);
				glUniform3f(uniformViewDir[0], 0.0, 0.0, 1.0);
				glDrawArrays(GL_POINTS, 0, verticies.size() / 9);
			glUseProgram(0);

			//
			// Second pass, calculate Gaussian weights
			//
			glEnable(GL_BLEND);
			glDepthMask(GL_FALSE);

			glUseProgram(programs[1]);
				glUniform3f(uniformViewDir[1], 0.0, 0.0, 1.0);
				glDrawArrays(GL_POINTS, 0, verticies.size() / 9);
			glUseProgram(0);

			glDepthMask(GL_TRUE);
			glDisable(GL_BLEND);

			//
			// Final pass, normalize pixels
			//
			glBindFramebufferEXT(GL_FRAMEBUFFER_EXT, 0);
			drawBounds();

			glMatrixMode(GL_PROJECTION);
			glPushMatrix();
			glLoadIdentity();
			glOrtho(-1, 1, -1, 1, 0.1, 100.0);

			glMatrixMode(GL_MODELVIEW);
			glLoadIdentity();

			static float fsQuadVerts[] = {
				 -1, -1, -1, 0, 0,
				  1, -1, -1, 1, 0,
				  1,  1, -1, 1, 1,
				 -1,  1, -1, 0, 1 };

			glEnableClientState(GL_TEXTURE_COORD_ARRAY);
			glVertexPointer  (3, GL_FLOAT, sizeof(float)*5, &(fsQuadVerts[0]));
			glTexCoordPointer(2, GL_FLOAT, sizeof(float)*5, &(fsQuadVerts[3]));

			glBindTexture(GL_TEXTURE_2D, colorTexture);
			glUseProgram(programs[2]);
				glDrawArrays(GL_QUADS, 0, 4);
			glUseProgram(0);
			glBindTexture(GL_TEXTURE_2D, 0);

			//
			// Reset state
			//
			glEnable(GL_CULL_FACE);

			glMatrixMode(GL_PROJECTION);
			glPopMatrix();
#else
			drawBounds();
			glPointSize(point_size);
			glDrawArrays(GL_POINTS, 0, verticies.size() / 9);
#endif
		} else { // indicies, so draw triangles
			drawBounds();
			glDrawElements(GL_TRIANGLES, indicies.size(), GL_UNSIGNED_INT, &(indicies[0]));
		}

		glDisableClientState(GL_COLOR_ARRAY);
		glDisableClientState(GL_NORMAL_ARRAY);
		glDisableClientState(GL_VERTEX_ARRAY);
	}
}

//---------------------------------------------------------------------

bool PointsViewScene::onMousePress(QMouseEvent *evt) {
	if(evt->modifiers().testFlag(Qt::ControlModifier))
		animated = !animated;

	lastx = evt->x();
	lasty = evt->y();
	return false;
}

//---------------------------------------------------------------------

bool PointsViewScene::onMouseMove(QMouseEvent *evt) {
	if(evt->buttons() == Qt::NoButton)
		return false;

	int deltax = evt->x() - lastx;
	int deltay = evt->y() - lasty;
	if(deltax != 0 || deltay != 0) {
		if(evt->modifiers() & Qt::ShiftModifier) {
			rotz -= (M_PI*deltax) / width;
			if(rotz < -M_PI) rotz += 2*M_PI;
			if(rotz >  M_PI) rotz -= 2*M_PI;
		} else {
			roty -= (M_PI*deltax) / width;
			rotx += (M_PI*deltay) / height;

			if(roty < -M_PI) roty += 2*M_PI;
			if(roty >  M_PI) roty -= 2*M_PI;

			if(rotx < -M_PI) rotx += 2*M_PI;
			if(rotx >  M_PI) rotx -= 2*M_PI;
		}
	}

	lastx = evt->x();
	lasty = evt->y();
	return true;
}

//---------------------------------------------------------------------

bool PointsViewScene::onMouseWheel(QWheelEvent *evt) {
	if(evt->modifiers().testFlag(Qt::ControlModifier)) {
		point_size += evt->delta() / 600.0;

		if(point_size < 0.1)     point_size = 0.1;
		else if(point_size > 10) point_size = 10;
#ifdef USE_SPLATS
		glUseProgram(programs[0]);
			glUniform1f(uniformSplatRadius[0], point_size);
		glUseProgram(programs[1]);
			glUniform1f(uniformSplatRadius[1], point_size);
		glUseProgram(0);
#endif
	} else {
		zoom -= evt->delta() / 600.0;
		if(zoom < 0.1)     zoom = 0.1;
		else if(zoom > 50) zoom = 50;
	}
	return true;
}

//---------------------------------------------------------------------

void PointsViewScene::setPoints(std::vector<GLfloat> &verticies, std::vector<GLuint> &indicies) {
	//
	// Scale all points so that most are within a "unit box" centered at the origin
	//
	if(scale < 0) {
		GLfloat minx, maxx, miny, maxy, minz, maxz;
		minx = miny = minz = std::numeric_limits<GLfloat>::infinity();
		maxx = maxy = maxz = -std::numeric_limits<GLfloat>::infinity();

		for(size_t i = 0; i < verticies.size(); i += 9) {
			minx = std::min(minx, verticies[i]);
			maxx = std::max(maxx, verticies[i]);

			miny = std::min(miny, verticies[i+1]);
			maxy = std::max(maxy, verticies[i+1]);

			minz = std::min(minz, verticies[i+2]);
			maxz = std::max(maxz, verticies[i+2]);
		}

		transx = (minx + maxx) * 0.5;
		transy = (miny + maxy) * 0.5;
		transz = (minz + maxz) * 0.5;

		scale = fabs(std::max(std::max(maxx - minx, maxy - miny), maxz - minz));
	}

	GLfloat invScale = 2.0 / scale;
	for(size_t i = 0; i < verticies.size(); i += 9) {
		verticies[i] = (verticies[i] - transx)*invScale;
		verticies[i+1] = (verticies[i+1] - transy)*invScale;
		verticies[i+2] = (verticies[i+2] - transz)*invScale;
	}

	//
	// Reset parameters
	//
	if(indicies.empty()) {
		glDisable(GL_LIGHTING);
		glEnable(GL_CULL_FACE);
	} else {
		float position[] = {1.0f, 1.0f, 1.0f, 0.0f};
		float diffuse[] = {0.9f, 0.9f, 0.9f, 0.9f, 1.0f};
		float specular[] = {0.2f, 0.2f, 0.2f, 1.0f};

		glDisable(GL_CULL_FACE);
		glEnable(GL_LIGHTING);
		glEnable(GL_LIGHT0);
		glLightfv(GL_LIGHT0, GL_DIFFUSE, diffuse);
		glLightfv(GL_LIGHT0, GL_SPECULAR, specular);
		glLightfv(GL_LIGHT0, GL_POSITION, position);

		glMaterialfv(GL_FRONT_AND_BACK, GL_SPECULAR, specular);
		glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE, diffuse);
	}

	//
	// TODO
	//   - Post processing? Clean up data by removing small clusters? Points
	//     with a color value which is significantly different than its
	//     neighbours?
	//
	//   - Find a dominant normal and place the camera in that direction
	//     initially perhaps?
	//

	//
	// Swap over new data
	//
	this->verticies.swap(verticies);
	this->indicies.swap(indicies);
}

//---------------------------------------------------------------------


