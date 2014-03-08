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
#ifndef PROJECTIMAGE_H
#define PROJECTIMAGE_H

#include <QObject>
#include <QFileInfo>

//
// Forward declarations
//
FORWARD_DECLARE(Camera);
FORWARD_DECLARE(ImageSet);

//! An image in the project
class ProjectImage : public QObject {
	Q_OBJECT

public:
	ProjectImage(QString filePath) : file_(filePath), exposure_(-1) { }

public:
	const QString & file() const { return file_; }

	void setFile(QString file) {
		if(file_ != file) {
			file_ = file;
			emit fileChanged(file_);
		}
	}

	double exposure() const { return exposure_; }

	void setExposure(double exposure) {
		if(fabs(exposure_ - exposure) > 1e-10) {
			exposure_ = exposure;
			emit exposureChanged(exposure_);
		}
	}

	CameraPtr camera() const { return camera_; }

	void setCamera(CameraPtr cam) {
		if(camera_ != cam) {
			CameraPtr old = camera_;
			camera_ = cam;
			emit cameraChanged(old, cam);
		}
	}

	ImageSetPtr imageSet() const { return imageSet_.lock(); }

	void setImageSet(ImageSetPtr imageSet) {
		if(imageSet_.lock() != imageSet) {
			imageSet_ = imageSet;
			emit imageSetChanged(imageSet);
		}
	}

signals:
	void fileChanged(QString file);
	void exposureChanged(double exposure);
	void cameraChanged(CameraPtr oldCam, CameraPtr newCam);
	void imageSetChanged(ImageSetPtr imageSet);

private:
	QString file_;
	double exposure_;
	CameraPtr camera_;
	ImageSetWeakPtr imageSet_;
};

//---------------------------------------------------------------------

#endif // PROJECTIMAGE_H
