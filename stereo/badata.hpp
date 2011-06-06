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
#ifndef BADATA_HPP
#define BADATA_HPP

#include "util/c++0x.hpp"

#include <Eigen/Core>
#include <vector>

FORWARD_DECLARE(Project);
FORWARD_DECLARE(Camera);
FORWARD_DECLARE(ImageSet);

/*!
 * Given a set of cameras and image sets, constructs data structures that
 * can be passed on to the SBA library.
 */
class BundleAdjustmentData {
public:
    BundleAdjustmentData(ProjectPtr project,
						 const std::vector<CameraPtr> &cameras,
						 const std::vector<ImageSetPtr> &imageSets,
						 int num_camera_params);

public:
	//! Cameras of interest
	std::vector<CameraPtr> cameras;

	//! Triangulated points
	std::vector<Eigen::Vector3d> scene_points;

	//! Image feature points, dense vector. dense_points[i][j] = projection of point i in camera j
	std::vector<std::vector<Eigen::Vector2d> > dense_points;

public: // These can be passed directly to optimizing routines in the SBA library
	//! Image feature points
	std::vector<double> points;

	//! Image feature points
	std::vector<char> mask;

	//! Image feature points
	std::vector<double> params;
};

//! Triangulate a 3D point from a set of views and 2D feature correspondences.
Eigen::Vector3d triangulate(const std::vector<CameraPtr> &cameras,
							 const std::vector<Eigen::Vector2d> &pts,
							 const std::vector<bool> &mask);

#endif // BADATA_HPP
