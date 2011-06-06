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

#include <QFileInfo>
#include "util/c++0x.hpp"

//
// Forward declarations
//
FORWARD_DECLARE(Camera);
FORWARD_DECLARE(ImageSet);

//!
class ProjectImage {
public:
	ProjectImage(QString filePath);

public:
	void setFile(QString path) { file_.setFile(path); }
	const QFileInfo & file() const { return file_; }

	void setMask(QString path) { mask_.setFile(path); }
	const QFileInfo & mask() const { return mask_; }

	void setExposure(double exposure) { exposure_ = exposure; }
	double exposure() const { return exposure_; }

	void setCamera(CameraPtr cam) { camera_ = cam; }
	CameraPtr camera() const { return camera_; }

	void setImageSet(ImageSetPtr imageSet) { imageSet_ = imageSet; }
	ImageSetPtr imageSet() const { return imageSet_.lock(); }

	double & exposure() { return exposure_; }
	CameraPtr & camera() { return camera_; }

public:
	QString fileString() const { return file_.absoluteFilePath(); }
	QString maskString() const { return mask_.absoluteFilePath(); }

public:
	//! Returns \c true if this image has a mask, \c false otherwise
	bool hasMask() const { return mask_.exists(); }

private:
	friend class Project;

	QFileInfo file_;
	QFileInfo mask_;
	double exposure_;
	CameraPtr camera_;
	std::weak_ptr<ImageSet> imageSet_;
};

//---------------------------------------------------------------------

#endif // PROJECTIMAGE_H
