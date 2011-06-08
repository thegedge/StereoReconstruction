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

#include <QGraphicsView>
#include "gui/captureimagesthread.hpp"

//
// Forward declarations
//
class QAction;

//! A scene for showing a set of [captured] images.
class CapturedImagesScene : public QGraphicsView {
	Q_OBJECT

public:
	typedef CaptureImagesThread<unsigned char>::ImageType Image;

public:
    CapturedImagesScene(QWidget *parent = 0);

public:
	//! Set the images to be shown
	void setImages(const std::vector<Image> &images);

	//! \copydoc setImages(const std::vector<Image> &images)
	void setImages(const std::vector<QImage> &images);

	//! \copydoc setImages(const std::vector<Image> &images)
	void setImages(const std::vector<QString> &images);

private slots:
	void on_actionZoom_In_triggered();
	void on_actionZoom_Out_triggered();

public:
	void changeEvent(QEvent *);

private:
	QAction *actionZoom_In;
	QAction *actionZoom_Out;
};

#endif // CAPTUREDIMAGESSCENE_H
