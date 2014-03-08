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

//---------------------------------------------------------------------

ImageSet::ImageSet(QString id, QString name)
    : id_(id)
    , name_(name.isNull() ? "<no name>" : name)
{ }

//---------------------------------------------------------------------

void ImageSet::removeImage(ProjectImagePtr image) {
	if(image && std::find(images_.begin(), images_.end(), image) != images_.end()) {
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

		emit imageRemoved(image);
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

		if(cam && !defaults_.contains(cam))
			defaults_[cam] = image;

		emit imageAdded(image);
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

	for(int index = images_.size() - 1; index >= 0; --index) {
		if(images_[index]->camera() == cam) {
			emit imageRemoved(images_[index]);
			images_.erase(images_.begin() + index);
		}
	}
}

//---------------------------------------------------------------------
