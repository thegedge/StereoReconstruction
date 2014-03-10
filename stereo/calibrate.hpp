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
#ifndef CALIBRATE_HPP
#define CALIBRATE_HPP

#include "gui/task.hpp"

//---------------------------------------------------------------------

extern const cv::Size board_size;

FORWARD_DECLARE(Project);
FORWARD_DECLARE(Camera);
FORWARD_DECLARE(ImageSet);

//---------------------------------------------------------------------

/*!
 * Performs various forms of camera calibration for intrinsic and
 * extrinsic parameters of the cameras in a project.
 */
class CameraCalibration : public Task {
public:
    CameraCalibration(ProjectPtr project,
					  const std::vector<CameraPtr> &cameras,
					  const std::vector<ImageSetPtr> &imageSets);

public:
    void calibrate();

public:
	QString title() const { return "Camera Calibration"; }
	int numSteps() const;

protected:
	void runTask() {
		calibrate();
	}

private:
	void estimateIntrinsics(const std::vector<int> &imageIndices);
	void estimateExtrinsics(const std::vector<int> &imageIndices);
	void bundleAdjust();
	double computeError();

private:
	ProjectPtr project;
	std::vector<CameraPtr> cameras;
	std::vector<ImageSetPtr> imageSets;

	//!
	bool findExtrinsicParameters;

	//! Fundamental matricies
	std::vector<std::vector<cv::Mat> > F;

	//! Image dimensions for each cameras
	std::vector<cv::Size> image_sizes;

	//! image_points[i][j][k] = k^th feature of the j^th image for the i^th camera
	std::vector<std::vector<std::vector<cv::Point2f> > > image_points;

	//! .
	std::vector<cv::Point3f> object_points;
};

#endif // CALIBRATE_HPP
