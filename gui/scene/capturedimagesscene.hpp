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
#ifndef CAPTUREDIMAGESSCENE_H
#define CAPTUREDIMAGESSCENE_H

#include "sceneviewer.hpp"
#include "gui/captureimagesthread.hpp"

#include <vector>

//
//
//
class QGLWidget;

//! A scene meant to show a set of [captured] images.
class CapturedImagesScene : public Scene {
public:
	typedef CaptureImagesThread<unsigned char>::ImageType Image;

public:
    CapturedImagesScene();
	~CapturedImagesScene();

public:
	//! Set the images to be shown
	void setImages(const std::vector<Image> &images);

public:
	//
	// Scene implementation / overrides
	//
	void onPaint(QGLWidget *parent);
	void onResize(int w, int h);
	void onShow();
	void onHide();

	bool onMousePress(QMouseEvent *evt);
	bool onMouseMove(QMouseEvent *evt);
	bool onMouseWheel(QWheelEvent *evt);

private:
	std::vector<GLuint> textureHandles;

	double transx, transy;   // translations for image viewing
	double scale;            // image scale factor
	double lastx, lasty;     // mouse movement
	int width, height;
};

#endif // CAPTUREDIMAGESSCENE_H
