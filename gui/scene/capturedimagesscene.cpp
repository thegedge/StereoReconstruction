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
#include "capturedimagesscene.hpp"

#include <QGLWidget>
#include <QDebug>

//---------------------------------------------------------------------
namespace {
	//const int IMAGE_W = 640, IMAGE_H = 480;
	const int IMAGE_W = 1024, IMAGE_H = 768;
}
//---------------------------------------------------------------------

CapturedImagesScene::CapturedImagesScene()
	: transx(0), transy(0)
	, scale(5)
	, width(1), height(1)
{ }

//---------------------------------------------------------------------

CapturedImagesScene::~CapturedImagesScene() {
	if(textureHandles.size() > 0)
		glDeleteTextures(textureHandles.size(), &(textureHandles[0]));
}

//---------------------------------------------------------------------

void
CapturedImagesScene::onShow() {

}

void
CapturedImagesScene::onHide() {

}

//---------------------------------------------------------------------

void
CapturedImagesScene::onResize(int w, int h) {
	glViewport(0, 0, w, h);

	GLdouble ratio = static_cast<GLdouble>(h) / w;
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	glOrtho(-scale, scale, -ratio*scale, ratio*scale, 1, 10.0);

	width = w;
	height = h;
}

//---------------------------------------------------------------------

void
CapturedImagesScene::onPaint(QGLWidget *) {
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	if(textureHandles.size() > 0) {
		glMatrixMode(GL_MODELVIEW);
		glLoadIdentity();

		// Find the smallest natural number x such that x*x >= images.size()
		size_t IMAGES_PER_ROW = 2;
		while(IMAGES_PER_ROW*IMAGES_PER_ROW < textureHandles.size())
			++IMAGES_PER_ROW;

		//
		const double width = 2.5*IMAGES_PER_ROW;
		const double height = 2.5*((1.0*textureHandles.size()) / IMAGES_PER_ROW);

		glEnable(GL_TEXTURE_2D);
		for(size_t img = 0; img < textureHandles.size(); ++img) {
			glPushMatrix();

			double x = 2.5*(img % IMAGES_PER_ROW);
			double y = 2.5*(img / IMAGES_PER_ROW + 0.5);
			glTranslated(x - 0.5*width + transx, 0.5*height - y + transy, 0);
			glScaled(1, (1.0*IMAGE_H) / IMAGE_W, 1.0);

			glBindTexture(GL_TEXTURE_2D, textureHandles[img]);
			glBegin(GL_QUADS);
				glTexCoord2d(0, 1);
				glVertex3d(0, -1, -2);
				glTexCoord2d(1, 1);
				glVertex3d(2, -1, -2);
				glTexCoord2d(1, 0);
				glVertex3d(2,  1, -2);
				glTexCoord2d(0, 0);
				glVertex3d(0,  1, -2);
			glEnd();

			glPopMatrix();
		}
		glDisable(GL_TEXTURE_2D);
	}
}

//---------------------------------------------------------------------

bool
CapturedImagesScene::onMousePress(QMouseEvent *evt) {
	lastx = evt->x();
	lasty = evt->y();
	return false;
}

//---------------------------------------------------------------------

bool
CapturedImagesScene::onMouseMove(QMouseEvent *evt) {
	if(evt->buttons() == Qt::NoButton)
		return false;

	int deltax = evt->x() - lastx;
	int deltay = evt->y() - lasty;
	if(deltax != 0 || deltay != 0) {
		transx += (10.0*deltax) / width;
		transy -= (10.0*deltay) / height;
	}

	lastx = evt->x();
	lasty = evt->y();
	return true;
}

//---------------------------------------------------------------------

bool
CapturedImagesScene::onMouseWheel(QWheelEvent *evt) {
	scale += evt->delta() / -120.0;

	if(scale < 1)       scale = 1;
	else if(scale > 10) scale = 10;

	GLdouble ratio = static_cast<GLdouble>(height) / width;

	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	glOrtho(-scale, scale, -ratio*scale, ratio*scale, 1, 10.0);

	return true;
}

//---------------------------------------------------------------------

void
CapturedImagesScene::setImages(const std::vector<Image> &images) {
	static unsigned char blank[3] = {255, 255, 255};

	//
	// Delete existing textures and generate new ones
	//
	if(textureHandles.size() > 0)
		glDeleteTextures(textureHandles.size(), &(textureHandles[0]));

	textureHandles.resize(images.size());
	if(textureHandles.size() > 0)
		glGenTextures(images.size(), &(textureHandles[0]));

	//
	// Supply texture data
	//
	glEnable(GL_TEXTURE_2D);
	for(size_t index = 0; index < images.size(); ++index) {
		glBindTexture(GL_TEXTURE_2D, textureHandles[index]);
		glTexEnvi(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_MODULATE);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP);

		Image img = images[index];
		if(img.width > 0 && img.height > 0 && img.data)
			glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, img.width, img.height, 0, GL_RGB, GL_UNSIGNED_BYTE, img.data.get());
		else
			glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, 1, 1, 0, GL_RGB, GL_UNSIGNED_BYTE, blank);
	}
	glDisable(GL_TEXTURE_2D);

	//
	// Reset parameters
	//
	//scale = 5;
	//transx = transy = 0;

	//
	//
	//
	glDisable(GL_LIGHTING);
}

//---------------------------------------------------------------------

