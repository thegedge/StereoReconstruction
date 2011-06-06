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
#ifndef FINDFEATURESTASK_HPP
#define FINDFEATURESTASK_HPP

#include "gui/task.hpp"
#include "util/c++0x.hpp"
#include <vector>

FORWARD_DECLARE(Project);
FORWARD_DECLARE(Camera);
FORWARD_DECLARE(ImageSet);
FORWARD_DECLARE(FeatureDetector);

//! Detector that finds features in an image using SURF.
class FindFeaturesTask : public Task {
public:
	FindFeaturesTask(ProjectPtr project,
	                 const std::vector<CameraPtr> &cameras,
	                 const std::vector<ImageSetPtr> &imageSets,
	                 FeatureDetectorPtr detector)
		: project(project)
	    , cameras(cameras)
	    , imageSets(imageSets)
	    , detector(detector)
	{ }

public:
	QString title() const { return tr("Find Features"); }
	int numSteps() const;
	void runTask();

private:
	ProjectPtr project;
	const std::vector<CameraPtr> cameras;
	const std::vector<ImageSetPtr> imageSets;
	const FeatureDetectorPtr detector;
};

#endif // FINDFEATURESTASK_HPP
