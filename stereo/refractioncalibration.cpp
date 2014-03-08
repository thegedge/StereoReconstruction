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
//
// If you include this class/code in your own work, the following citations
// should be used:
//
//    Gedge, J. (2011) Underwater Stereo Matching and its Calibration. M.Sc.
//       Department of Computing Science, University of Alberta. Canada.
//
//    Gedge, J., Gong M., and Yang Y-H. (2011) Refractive Epipolar Geometry
//       For Underwater Stereo Matching. Proceedings of the Eighth Canadian
//       Conference on Computer and Robot Vision.
//
//---------------------------------------------------------------------
#include "refractioncalibration.hpp"

#include "badata.hpp" // for triangulate
#include "calibrate.hpp" // for board_size
#include "features/feature.hpp"
#include "project/project.hpp"
#include "project/camera.hpp"
#include "project/imageset.hpp"
#include "stereo/multiviewstereo.hpp"

#include <QDebug>

#ifdef USE_SBA
#   include <sba.h>
#endif

//#define SINGLE_VIEW
//#define BUNDLE_ADJUST
//#define JUST_CAMERA_PARAMS

//---------------------------------------------------------------------
using namespace Eigen;

using std::fabs;
using std::exp;

namespace {
	typedef std::pair<int, int> IntPair;

	typedef LevenbergMarquardt::Point Point;
	typedef LevenbergMarquardt::Model Model;
	typedef LevenbergMarquardt::FixedParams FixedParams;
	typedef LevenbergMarquardt::Points Points;
	typedef LevenbergMarquardt::PointPair PointPair;
}

//---------------------------------------------------------------------
#ifdef BUNDLE_ADJUST
#ifdef JUST_CAMERA_PARAMS
namespace {
	void projection_error(int j, int i, double *aj, double *xij, void *adata) {
		//
		CameraPtr camera = static_cast<BundleAdjustmentData *>(adata)->cameras[j];

		//
		Vector3d normal = camera->Kinv() * Vector3d(aj[0], aj[1], 1);
		Plane3d plane(normal, 0.01*aj[2]);
		camera->setPlane(plane);

		xij[0] = xij[1] = 1000;

		//
		Vector3d point = static_cast<BundleAdjustmentData *>(adata)->scene_points[i];
		if(camera->project(point)) {
			//const Vector2d &mp = static_cast<BundleAdjustmentData *>(adata)->dense_points[i][j];
			//xij[0] = point.x() - mp.x();
			//xij[1] = point.y() - mp.y();
			xij[0] = point.x();
			xij[1] = point.y();
		}
	}

} // end of unnamed namespace
#else
namespace {
	void projection_error(int j, int i, double *aj, double *bi, double *xij, void *adata) {
		//
		CameraPtr camera = static_cast<BundleAdjustmentData *>(adata)->cameras[j];

		//
		Vector3d normal = camera->Kinv() * Vector3d(aj[0], aj[1], 1);
		Plane3d plane(normal, aj[2]);
		camera->setPlane(plane);

		xij[0] = xij[1] = 100000;

		//
		Vector3d point(bi[0], bi[1], bi[2]);
		//Vector3d point = static_cast<BundleAdjustmentData *>(adata)->scene_points[i];
		if(camera->project(point)) {
			//const Vector2d &mp = static_cast<BundleAdjustmentData *>(adata)->dense_points[i][j];
			//xij[0] = point.x() - mp.x();
			//xij[1] = point.y() - mp.y();
			xij[0] = point.x();
			xij[1] = point.y();
		}
	}

} // end of unnamed namespace
#endif
#endif
//---------------------------------------------------------------------

/*!
 * Ray/Ray distance metric used in LM optimization to calibrate parameters
 * of the refractive interface.
 */
class RefractiveCalibrationFunction : public LevenbergMarquardt::Function {
public:
	RefractiveCalibrationFunction(const std::vector<CameraPtr> &views)
		: views(views)
	{ }

	RefractiveCalibrationFunction(const std::vector<CameraPtr> &views,
	                              const std::vector<IntPair> &point2cams)
		: views(views)
	    , point2cams(point2cams)
	{ }

	RefractiveCalibrationFunction(const std::vector<CameraPtr> &views,
	                              const std::vector<IntPair> &point2cams,
	                              const Points &points)
		: views(views)
	    , point2cams(point2cams)
		, points(points)
	{ }

	~RefractiveCalibrationFunction() {
		for(size_t view_index = 0; view_index < views.size(); ++view_index)
			views[view_index]->setPlane(original_P[view_index]);
	}

public:
	void initialize() {
		P.resize(views.size());
		original_P.resize(views.size());
		for(size_t view_index = 0; view_index < views.size(); ++view_index)
			original_P[view_index] = views[view_index]->plane();
	}

public: // LevenbergMarquardt::Function implementation
#ifdef SINGLE_VIEW
	Point diff(const PointPair &pp, int point_index, const Model &model) {
		//
	}
#else
	Point diff(const PointPair &pp, int point_index, const Model &model) {
		const IntPair &p2c = point2cams[point_index];
		return diff(pp, views[p2c.first], views[p2c.second], model);
	}
#endif

	Point diff(const PointPair &pp, CameraPtr view1, CameraPtr view2, const Model &) {
		const Point &pp1 = pp.first;
		const Point &pp2 = pp.second;

		// Cast rays
		Ray3d R1 = view1->unproject(pp1[0], pp1[1]);
		Ray3d R2 = view2->unproject(pp2[0], pp2[1]);
		Point out(1);

		// Distance between two casted rays used as error metric
		Ray3d::Point p1, p2;
		R1.closestPoints(R2, p1, p2);
		out[0] = (p1 - p2).norm();

		// Scale by distance to image plane to approximate image-space distance
		Vector3d mid = (p1 + p2) * 0.5;
		Vector3d mid1 = view1->fromGlobalToLocal(mid);
		Vector3d mid2 = view2->fromGlobalToLocal(mid);

		const double v1 = (0.5 * view1->K()(0, 0) * out[0]) / mid1.z();
		const double v2 = (0.5 * view2->K()(0, 0) * out[0]) / mid2.z();
		out[0] = v1 + v2;

		return out;
	}

	Point gradient(const PointPair &pp, int point_index, const Model &model, int paramIndex) {
		// If parameter not for this camera, gradient is trivially zero
		const IntPair &p2c = point2cams[point_index];
		if((paramIndex / 3 != p2c.first) && (paramIndex / 3 != p2c.second))
			return Point::Zero(1);

		// Finite differences
		Model m1 = model;
		Model m2 = model;

		if(paramIndex == 0) {                  // refractive index
			m1[paramIndex] = model[paramIndex] - 0.01;
			m2[paramIndex] = model[paramIndex] + 0.01;
		} else if((paramIndex - 1) % 3 == 0) { // pixel_x, for interface normal
			m1[paramIndex] = model[paramIndex] - 0.5;
			m2[paramIndex] = model[paramIndex] + 0.5;
		} else if((paramIndex - 1) % 3 == 1) { // pixel_y, for interface normal
			m1[paramIndex] = model[paramIndex] - 0.1;
			m2[paramIndex] = model[paramIndex] + 0.1;
		} else {                               // distance, for interface normal
			m1[paramIndex] = model[paramIndex];
			m2[paramIndex] = model[paramIndex] + 0.0001;
		}

		update(m1);
		Point val1 = diff(pp, point_index, m1);
		update(m2);
		Point val2 = diff(pp, point_index, m2);
		update(model);

		return (val2 - val1) / (m2[paramIndex] - m1[paramIndex]);
	}

	bool update(const Model &model) {
		for(size_t view_index = 0; view_index < views.size(); ++view_index)
			if(model[3*view_index + 2] < 1e-4)
				return false;

		for(size_t view_index = 0; view_index < views.size(); ++view_index) {
			CameraPtr view = views[view_index];
			Vector3d normal = view->Kinv() * Vector3d(model[3*view_index + 1], model[3*view_index + 2], 1);

			normal.normalize();
			P[view_index].setDistance(model[3*view_index + 3]);
			P[view_index].setNormal(normal);
			view->setRefractiveIndex(model[0]);
			view->setPlane(P[view_index]);
		}

		return true;
	}

private:
	std::vector<CameraPtr> views;
	std::vector<IntPair> point2cams;
	std::vector<Plane3d> P;
	std::vector<Plane3d> original_P;
	Points points;
};

//---------------------------------------------------------------------

RefractionCalibration::RefractionCalibration()
	: db(nullptr)
	, model_(Model::Zero(1))
    , fixed(1, false)
{  }

//---------------------------------------------------------------------

void RefractionCalibration::setModel(const Model &model) {
	setModel(model, FixedParams(model.size(), false));
}

void RefractionCalibration::setModel(const Model &model, const FixedParams &fixed) {
	this->model_ = model;
	this->fixed = fixed;
}

//---------------------------------------------------------------------

void RefractionCalibration::setProject(ProjectPtr project) {
	this->project = project;
	this->db = &project->features();
}

//---------------------------------------------------------------------

bool RefractionCalibration::calibrate() {
#ifdef SINGLE_VIEW
	if(!db || imageSets.size() == 0 || model_.size() != 3)
		return false;
#else
	if(!db || views.size() < 2 || imageSets.size() == 0 || 3*views.size() != model_.size())
		return false;
#endif

#ifdef BUNDLE_ADJUST
	//
	BundleAdjustmentData data(project, views, imageSets, 3);

	//
	for(size_t view_index = 0; view_index < views.size(); ++view_index) {
		data.params[3*view_index + 0] = views[view_index]->K()(0, 2);
		data.params[3*view_index + 1] = views[view_index]->K()(1, 2);
		data.params[3*view_index + 2] = 1;
	}

	//
	double opts[SBA_OPTSSZ] = { SBA_INIT_MU,
								 SBA_STOP_THRESH,
							     SBA_STOP_THRESH,
							     SBA_STOP_THRESH,
							     0.0 };

	double info[SBA_INFOSZ];
#ifdef JUST_CAMERA_PARAMS
	int ret = sba_mot_levmar(data.scene_points.size(),
							 views.size(),
							 0,
							 &data.mask[0],
							 &data.params[0],
							 3,
							 &data.points[0],
							 nullptr,
							 2,
							 projection_error,
							 nullptr, //projection_grad,
							 &data,
							 500,
							 0,
							 opts,
							 info);
#else
	int ret = sba_motstr_levmar(data.scene_points.size(),
								0,
								views.size(),
								0,
								&data.mask[0],
								&data.params[0],
								3, // num params per cam
								3, // num params per point
								&data.points[0],
								nullptr,
								2,
								projection_error,
								nullptr, //projection_grad,
								&data,
								500,
								0,
								opts,
								info);
#endif
#else
	// For each image pair, find features if necessary, establish correspondences
	std::vector<IntPair> point2cams;
	Points points;
	foreach(ImageSetPtr imageSet, imageSets) {
		for(size_t view_index1 = 0; view_index1 < views.size(); ++view_index1) {
			for(size_t view_index2 = view_index1 + 1; view_index2 < views.size(); ++view_index2) {
				ProjectImagePtr leftImage = imageSet->defaultImageForCamera(views[view_index1]);
				ProjectImagePtr rightImage = imageSet->defaultImageForCamera(views[view_index2]);

				bool swap = false;
				const Correspondences &corr = db->correspondences(leftImage, rightImage, &swap);
				foreach(const Correspondence &c, corr) {
					Point p1(2), p2(2);
					if(swap) {
						p1 << c.second->x(), c.second->y();
						p2 << c.first->x(), c.first->y();
					} else {
						p1 << c.first->x(), c.first->y();
						p2 << c.second->x(), c.second->y();
					}

					points.push_back( PointPair(p1, p2) );
					point2cams.push_back( IntPair(view_index1, view_index2) );
				}
			}
		}
	}

	//
	RefractiveCalibrationFunction func(views, point2cams, points);
	func.initialize();
	func.update(model_);
	qDebug() << "Initial error:"
			 << LevenbergMarquardt::chiSquared(func, points, model_);

	// Optimize function
	LevenbergMarquardt lm(100, 1);
	lm.optimize(func, points, model_, fixed);

	func.update(model_);
	qDebug() << "Error after optimization:"
	         << LevenbergMarquardt::chiSquared(func, points, model_) << "\n";
#endif
	// NaN values = bad optimization, so return false if they exist
	for(int index = 0; index < model_.size(); ++index)
		if(std::isnan(model_[index]))
			return false;

	return true;
}

//------------------------------------------------------------------

double RefractionCalibration::totalError(double *average) const {
	if(!db || views.size() < 2)
		return std::numeric_limits<double>::quiet_NaN();

	RefractiveCalibrationFunction func(views);
	func.initialize();
	func.update(model_);

	//
	int count = 0;
	double total = 0.0;
	foreach(ImageSetPtr imageSet, imageSets) {
		for(size_t view_index1 = 0; view_index1 < views.size(); ++view_index1) {
			CameraPtr view1 = views[view_index1];
			for(size_t view_index2 = view_index1 + 1; view_index2 < views.size(); ++view_index2) {
				CameraPtr view2 = views[view_index2];

				ProjectImagePtr leftImage = imageSet->defaultImageForCamera(views[view_index1]);
				ProjectImagePtr rightImage = imageSet->defaultImageForCamera(views[view_index2]);

				bool swap = false;
				const Correspondences &corr = db->correspondences(leftImage, rightImage, &swap);
				foreach(const Correspondence &c, corr) {
					Point p1(2), p2(2);
					if(swap) {
						p1 << c.second->x(), c.second->y();
						p2 << c.first->x(), c.first->y();
					} else {
						p1 << c.first->x(), c.first->y();
						p2 << c.second->x(), c.second->y();
					}

					total += func.diff(PointPair(p1, p2), view1, view2, model_).squaredNorm();
					++count;
				}
			}
		}
	}

	if(average)
		*average = (total / count);

	return total;
}

//---------------------------------------------------------------------

double RefractionCalibration::error(const Correspondence &c, CameraPtr view1, CameraPtr view2) const {
	if(!view1 || !view2)
		return std::numeric_limits<double>::quiet_NaN();

	Point p1(2), p2(2);
	p1 << c.first->x(), c.first->y();
	p2 << c.second->x(), c.second->y();

	RefractiveCalibrationFunction func(views);
	func.initialize();
	func.update(model_);
	return func.diff(PointPair(p1, p2), view1, view2, model_).norm();
}

//---------------------------------------------------------------------
