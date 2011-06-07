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
#ifndef IMAGESET_H
#define IMAGESET_H

#include <QString>
#include <QDir>
#include <QMap>

#include <vector>
#include <map>

#include <Eigen/Core>

#include "features/featuredb.hpp"
#include "util/c++0x.hpp"
#include "util/plane.hpp"

//
// Forward declarations
//
FORWARD_DECLARE(Camera);
FORWARD_DECLARE(ProjectImage);

//! A set of images
class ImageSet : public std::enable_shared_from_this<ImageSet> {
public:
	ImageSet(QString id, QString name = QString());

public:
	QString id() const { return id_; }

	QString name() const { return name_; }
	void setName(QString name) { name_ = name; }

	QDir root() const { return root_; }
	void setRoot(QString path) { root_.setPath(path); }

	const std::vector<ProjectImagePtr> & images() const { return images_; }

public:
	//! Remove an image from this set
	void removeImage(ProjectImagePtr image);

	//! Add an image for a specified camera
	void addImageForCamera(CameraPtr cam, ProjectImagePtr image);

	//! Fetches the default image in this image set for the specified camera
	ProjectImagePtr defaultImageForCamera(CameraPtr cam) const;

	//! Removes any image references for the specified camera
	void removeImagesForCamera(CameraPtr cam);

	//! Whether or not this set of images has a reference to \a cam
	bool hasImageForCamera(CameraPtr cam) const { return defaults_.contains(cam); }

private:
	friend class Project;

	QString id_;
	QString name_;
	QDir root_;
	std::vector<ProjectImagePtr> images_;
	QMap<CameraPtr, ProjectImagePtr> defaults_;
};

#endif // IMAGESET_H