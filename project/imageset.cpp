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
#include "imageset.hpp"
#include "projectimage.hpp"

#include <algorithm>

//---------------------------------------------------------------------

ImageSet::ImageSet(QString id)
    : id_(id)
    , name_("<no name>")
{ }

//---------------------------------------------------------------------

void ImageSet::removeImage(ProjectImagePtr image) {
	if(image) {
		// Remove
		images_.erase(std::remove(images_.begin(), images_.end(), image), images_.end());

		// See if any remaining images refer to this camera
		CameraPtr cam = image->camera();
		int count = std::count_if(images_.begin(),
		                          images_.end(),
		                          [cam](const ProjectImagePtr &img) {
		                              return (img->camera() == cam);
		                          });

		// If no such images exist, remove it from defaults
		if(count == 0)
			defaults_.remove(cam);
	}
}

//---------------------------------------------------------------------

void ImageSet::addImageForCamera(CameraPtr cam, ProjectImagePtr image) {
	ImageSetPtr oldImageSet = image->imageSet();
	if(this != oldImageSet.get()) {
		if(oldImageSet) oldImageSet->removeImagesForCamera(image->camera());

		image->setCamera(cam);
		image->setImageSet(shared_from_this());
		images_.push_back(image);

		if(!defaults_.contains(cam))
			defaults_[cam] = image;
	}
}

//---------------------------------------------------------------------

ProjectImagePtr ImageSet::defaultImageForCamera(CameraPtr cam) const {
	if(defaults_.contains(cam))
		return defaults_[cam];
	return ProjectImagePtr();
}

//---------------------------------------------------------------------

void ImageSet::removeImagesForCamera(CameraPtr cam) {
	if(defaults_.contains(cam))
		defaults_.remove(cam);

	images_.erase(std::remove_if(images_.begin(), images_.end(), [&cam](ProjectImagePtr image) {
		return (image->camera() == cam);
	}), images_.end());
}

//---------------------------------------------------------------------
