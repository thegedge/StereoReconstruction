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
#include "badata.hpp"
#include "project/project.hpp"
#include "project/camera.hpp"
#include "project/imageset.hpp"
#include "project/projectimage.hpp"
#include "calibrate.hpp" // for board_size

#include <opencv/cv.h>
#include <Eigen/SVD>

//---------------------------------------------------------------------

using namespace Eigen;

using std::sin;
using std::cos;
using std::acos;
using std::fabs;

namespace {
	const double NaN = std::numeric_limits<double>::quiet_NaN();
}

//---------------------------------------------------------------------
// Iterative-LS method presented in Hartley's Triangulation paper
//
Vector3d triangulate(const std::vector<CameraPtr> &cameras,
					  const std::vector<Vector2d> &pts,
					  const std::vector<bool> &mask)
{
	// Find the number of views that have a point
	size_t num_points = 0;
	for(size_t index = 0; index < mask.size(); ++index)
		if(mask[index])
			++num_points;

	if(num_points < 2)
		return Vector3d(NaN, NaN, NaN);

	// Iterative-LS method
	MatrixXd A(2*num_points, 3);
	VectorXd b(2*num_points);

	Vector4d x(0, 0, 0, 1);
	for(int iteration = 0; iteration < 10; ++iteration) {
		for(size_t index = 0, tindex = 0; index < pts.size(); ++index) if(mask[index]) {
			const Vector2d pt = pts[index];
			const ProjMat &P = cameras[index]->P();

			const double weight = (iteration == 0 ? 1.0 : 1.0 / P.row(2).dot(x));
			//const Vector4d t1 = weight*(pt.x() * P.row(2) - P.row(0)).head<4>();
			//const Vector4d t2 = weight*(pt.y() * P.row(2) - P.row(1)).head<4>();
			const Vector4d t1 = weight*(pt.x() * P.row(2) - P.row(0)).start<4>();
			const Vector4d t2 = weight*(pt.y() * P.row(2) - P.row(1)).start<4>();

			//A.row(tindex + 0) = t1.head<3>();
			//A.row(tindex + 1) = t2.head<3>();
			A.row(tindex + 0) = t1.start(3);
			A.row(tindex + 1) = t2.start(3);
			b[tindex + 0] = -t1[3];
			b[tindex + 1] = -t2[3];

			tindex += 2;
		}

		//
		//Vector3d xt = A.jacobiSvd().solve(b);
		Vector3d xt;
		A.svd().solve(b, &xt);

		// Break if new solution isn't much different from previous
		//if(iteration > 0 && (x.head<3>() - xt).squaredNorm() < 1e-10)
		if(iteration > 0 && (x.start<3>() - xt).squaredNorm() < 1e-10)
			break;

		//x.head<3>() = xt;
		x.start<3>() = xt;
		break;
	}

	//return x.head<3>();
	return x.start<3>();
}

//---------------------------------------------------------------------

BundleAdjustmentData::BundleAdjustmentData(ProjectPtr project,
										   const std::vector<CameraPtr> &cameras,
										   const std::vector<ImageSetPtr> &imageSets,
										   int num_camera_params)
	: cameras(cameras)
{
	// Collect info
	for(size_t img_index = 0; img_index < imageSets.size(); ++img_index) {
		for(size_t pt_index = 0; pt_index < board_size.area(); ++pt_index) {
			// First see if we can actually triangulate a point
			std::vector<bool> triangulate_mask(cameras.size(), false);
			std::vector<Vector2d> triangulate_points(cameras.size());

			for(size_t cam_index = 0; cam_index < cameras.size(); ++cam_index) {
				ProjectImagePtr img = imageSets[img_index]->defaultImageForCamera(cameras[cam_index]);

				// If features were found in this image for this camera...
				const Features &features = project->features().features(img);
				if(features.size() > 0) {
					triangulate_mask[cam_index] = true;
					triangulate_points[cam_index][0] = features[pt_index]->x();
					triangulate_points[cam_index][0] = features[pt_index]->y();
				}
			}

			//
			Vector3d pt = triangulate(cameras, triangulate_points, triangulate_mask);
			if(!std::isnan(pt.squaredNorm())) {
				// We have a point, collect data for SBA
				scene_points.push_back(pt);
				dense_points.push_back( std::vector<Vector2d>(cameras.size()) );

				for(size_t cam_index = 0; cam_index < cameras.size(); ++cam_index) {
					ProjectImagePtr img = imageSets[img_index]->defaultImageForCamera(cameras[cam_index]);

					// If features were found in this image for this camera...
					const Features &features = project->features().features(img);
					if(features.size() > 0) {
						mask.push_back(1);
						points.push_back(features[pt_index]->x());
						points.push_back(features[pt_index]->y());
						dense_points.back()[cam_index][0] = features[pt_index]->x();
						dense_points.back()[cam_index][1] = features[pt_index]->y();
					} else {
						mask.push_back(0);
					}
				}
			}
		}
	}

	// Camera params
	params.resize(num_camera_params*cameras.size() + 3*scene_points.size(), 0.0);
	for(size_t pt_index = 0; pt_index < scene_points.size(); ++pt_index) {
		params[num_camera_params*cameras.size() + 3*pt_index + 0] = scene_points[pt_index].x();
		params[num_camera_params*cameras.size() + 3*pt_index + 1] = scene_points[pt_index].y();
		params[num_camera_params*cameras.size() + 3*pt_index + 2] = scene_points[pt_index].z();
	}
}

//---------------------------------------------------------------------
