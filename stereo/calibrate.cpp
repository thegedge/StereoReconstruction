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
// TODO
//   - currently bundle adjustment is not making use of lens distortion
//     parameters when projecting
//
//   - options for selecting whether or not to perform intrinsic calibration,
//     extrinsic calibration, and/or bundle adjustment
//
#include "calibrate.hpp"
#include "project/project.hpp"
#include "project/camera.hpp"
#include "project/imageset.hpp"
#include "project/projectimage.hpp"
#include "util/c++0x.hpp"
#include "util/floydwarshall.hpp"

#include "badata.hpp"
#include "multiviewstereo.hpp"

#include <QDateTime>
#include <QDebug>
#include <QVector>

#include <algorithm>
#include <cmath>
#include <fstream>
#include <limits>
#include <string>
#include <vector>

#include <boost/bind.hpp>
#include <opencv/cv.h>
#include <opencv/highgui.h>

#ifdef USE_SBA
#   include <sba.h>
#   include <opencv2/contrib/contrib.hpp>
#   include "util/lm.hpp"
#endif

//#define MY_BA
//#define OPENCV_BA
//#define INCLUDE_INTRINSIC_IN_BA

//---------------------------------------------------------------------

using namespace Eigen;

using std::sin;
using std::cos;
using std::acos;
using std::fabs;
using std::sqrt;

//---------------------------------------------------------------------

const cv::Size board_size(11, 9); // TODO allow customizing this parameter
//const cv::Size board_size(8, 9); // NEPTUNE data
//const cv::Size board_size(6, 8); // CalibrationTest data

namespace {
	// TODO most of these should be configurable via GUI
	const int NUM_ITERATIONS = 5;

	const double cell_size = 11; // mm, air pattern
	//const double cell_size = 0.011; // m, air pattern

	//const double cell_size = 0.00725; // m, water pattern
	//const double cell_size = 1; // no measurement
	//const double cell_size = 0.05; // for synthetic data

	const double NaN = std::numeric_limits<double>::quiet_NaN();
}

namespace {
#ifdef INCLUDE_INTRINSIC_IN_BA
	const int NUM_CAM_PARAMS = 10;
#else
	const int NUM_CAM_PARAMS = 6;
#endif

	void projection_error(int j, int /*i*/, double *aj, double *bi, double *xij, void *adata) {
		//
		CameraPtr camera = static_cast<BundleAdjustmentData *>(adata)->cameras[j];

		// For rodriques' rotation formula
		Vector3d w(aj[0], aj[1], aj[2]);
		double theta = w.norm();
		if(fabs(theta) > 1e-5)
			w.normalize();

		Matrix3d wx; // "cross product" matrix for w
		wx << 0, -w.z(), w.y(), w.z(), 0, -w.x(), -w.y(), w.x(), 0;

		const Matrix3d R = Matrix3d::Identity() + sin(theta)*wx + (1 - cos(theta))*wx*wx;
#ifdef INCLUDE_INTRINSIC_IN_BA
		Matrix3d K = camera->K();
		K(0, 0) = aj[6];
		K(1, 1) = aj[7];
		K(0, 2) = aj[8];
		K(1, 2) = aj[9];
#else
		const Matrix3d &K = camera->K();
#endif

		//
		Vector3d px(bi[0], bi[1], bi[2]);
		camera->set(K, R, Vector3d(aj[3], aj[4], aj[5]));
		camera->project(px);

		//
		xij[0] = px.x();
		xij[1] = px.y();
	}

	void projection_grad(int /*j*/, int /*i*/, double */*aj*/, double */*Aij*/, void */*adata*/) {

	}
} // end of unnamed namespace

//---------------------------------------------------------------------
// XXX how about templating?
//
// template <class _Scalar, class _Rows, class _Cols>
// void storeCalibration(Eigen::Matrix<_Scalar, _Rows, _Cols> &m1, cv::Mat &m2)
//
void storeCalibration(CameraPtr cam,
                      cv::Mat &K,
                      cv::Mat &lensDistortion,
                      cv::Mat &R,
                      cv::Mat &t)
{
	if(K.rows == 3 && K.cols == 3) {
		K.at<double>(0, 0) = cam->K()(0, 0);
		K.at<double>(0, 1) = cam->K()(0, 1);
		K.at<double>(0, 2) = cam->K()(0, 2);
		K.at<double>(1, 0) = cam->K()(1, 0);
		K.at<double>(1, 1) = cam->K()(1, 1);
		K.at<double>(1, 2) = cam->K()(1, 2);
		K.at<double>(2, 0) = cam->K()(2, 0);
		K.at<double>(2, 1) = cam->K()(2, 1);
		K.at<double>(2, 2) = cam->K()(2, 2);
	}

	if((lensDistortion.rows == 5 && lensDistortion.cols == 1)
	        || (lensDistortion.rows == 1 && lensDistortion.cols == 5))
	{
		const LensDistortions &lensDist = cam->lensDistortion();
		lensDistortion.at<double>(0) = lensDist[0];
		lensDistortion.at<double>(1) = lensDist[1];
		lensDistortion.at<double>(2) = lensDist[2];
		lensDistortion.at<double>(3) = lensDist[3];
		lensDistortion.at<double>(4) = lensDist[4];
	}

	if(R.rows == 3 && R.cols == 3) {
		R.at<double>(0, 0) = cam->R()(0, 0);
		R.at<double>(0, 1) = cam->R()(0, 1);
		R.at<double>(0, 2) = cam->R()(0, 2);
		R.at<double>(1, 0) = cam->R()(1, 0);
		R.at<double>(1, 1) = cam->R()(1, 1);
		R.at<double>(1, 2) = cam->R()(1, 2);
		R.at<double>(2, 0) = cam->R()(2, 0);
		R.at<double>(2, 1) = cam->R()(2, 1);
		R.at<double>(2, 2) = cam->R()(2, 2);
	}

	if((t.rows == 3 && t.cols == 1) || (t.rows == 1 && t.cols == 3)) {
		t.at<double>(0) = cam->t()[0];
		t.at<double>(1) = cam->t()[1];
		t.at<double>(2) = cam->t()[2];
	}
}

void loadCalibration(CameraPtr cam,
                     const cv::Mat &K,
                     const cv::Mat &lensDistortion,
                     const cv::Mat &R,
                     const cv::Mat &t)
{
	if(K.rows == 3 && K.cols == 3) {
		Matrix3d camK;
		camK(0, 0) = K.at<double>(0, 0);
		camK(0, 1) = K.at<double>(0, 1);
		camK(0, 2) = K.at<double>(0, 2);
		camK(1, 0) = K.at<double>(1, 0);
		camK(1, 1) = K.at<double>(1, 1);
		camK(1, 2) = K.at<double>(1, 2);
		camK(2, 0) = K.at<double>(2, 0);
		camK(2, 1) = K.at<double>(2, 1);
		camK(2, 2) = K.at<double>(2, 2);
		cam->setK(camK);
	}

	if((lensDistortion.rows == 5 && lensDistortion.cols == 1)
	        || (lensDistortion.rows == 1 && lensDistortion.cols == 5))
	{
		LensDistortions lensDist;
		lensDist[0] = lensDistortion.at<double>(0);
		lensDist[1] = lensDistortion.at<double>(1);
		lensDist[2] = lensDistortion.at<double>(2);
		lensDist[3] = lensDistortion.at<double>(3);
		lensDist[4] = lensDistortion.at<double>(4);
		cam->setLensDistortion(lensDist);
	}

	if(R.rows == 3 && R.cols == 3) {
		Matrix3d camR;
		camR(0, 0) = R.at<double>(0, 0);
		camR(0, 1) = R.at<double>(0, 1);
		camR(0, 2) = R.at<double>(0, 2);
		camR(1, 0) = R.at<double>(1, 0);
		camR(1, 1) = R.at<double>(1, 1);
		camR(1, 2) = R.at<double>(1, 2);
		camR(2, 0) = R.at<double>(2, 0);
		camR(2, 1) = R.at<double>(2, 1);
		camR(2, 2) = R.at<double>(2, 2);
		cam->setR(camR);
	}

	if((t.rows == 3 && t.cols == 1) || (t.rows == 1 && t.cols == 3)) {
		Vector3d camt;
		camt[0] = t.at<double>(0);
		camt[1] = t.at<double>(1);
		camt[2] = t.at<double>(2);
		cam->sett(camt);
	}
}

//---------------------------------------------------------------------

CameraCalibration::CameraCalibration(
		ProjectPtr project,
		const std::vector<CameraPtr> &cameras,
		const std::vector<ImageSetPtr> &imageSets)
	: project(project)
	, cameras(cameras)
	, imageSets(imageSets)
    , findExtrinsicParameters(true)
	, image_sizes(cameras.size(), cv::Size(-1, -1))
	, image_points(cameras.size())
	, object_points(board_size.area())
{
	for(int row = 0, index = 0; row < board_size.height; ++row) {
		for(int col = 0; col < board_size.width; ++col, ++index) {
			object_points[index].x = cell_size*col;
			object_points[index].y = cell_size*row;
			object_points[index].z = 0;
		}
	}
}

//---------------------------------------------------------------------

int CameraCalibration::numSteps() const {
	return NUM_ITERATIONS // intrinsic/extrinsic params
	       + 1;  // bundle adjustment
}

//---------------------------------------------------------------------

void outputMatlabMatrixHeader(QTextStream &stream, QString name, int rows, int cols) {
	stream << "# Created by StereoReconstruction, " << QDateTime::currentDateTime().toString() << '\n';
	stream << "# name: " << name << '\n';
	stream << "# type: matrix\n";
	stream << "# rows: " << rows << '\n';
	stream << "# columns: " << cols << '\n';
}

//---------------------------------------------------------------------

double CameraCalibration::computeError() {
	//
	// TODO undistort points so fundamental matrix can be used
	//
	const size_t numPoints = board_size.area();
	double totalError = 0.0;
	long count = 0;
	for(size_t cam_index = 0; cam_index < cameras.size(); ++cam_index) {
		CameraPtr cam1 = cameras[cam_index];
		for(size_t cam_index2 = cam_index + 1; cam_index2 < cameras.size(); ++cam_index2) {
			CameraPtr cam2 = cameras[cam_index2];
			for(size_t img_index = 0; img_index < image_points[cam_index].size(); ++img_index) {
				if(image_points[cam_index][img_index].size() == numPoints
					&& image_points[cam_index2][img_index].size() == numPoints)
				{
					count += numPoints;

					for(size_t pt_index = 0; pt_index < numPoints; ++pt_index) {
						Ray3d r1 = cam1->unproject(image_points[cam_index][img_index][pt_index].x,
						                           image_points[cam_index][img_index][pt_index].y);
						Ray3d r2 = cam2->unproject(image_points[cam_index2][img_index][pt_index].x,
						                           image_points[cam_index2][img_index][pt_index].y);

						double dist = 0.0;
						Ray3d::Point p1, p2;
						r1.closestPoints(r2, p1, p2);

						p1 = p2 = 0.5*(p1 + p2);
						if(cam1->project(p2) && cam2->project(p1)) {
							p2[0] -= image_points[cam_index][img_index][pt_index].x;
							p2[1] -= image_points[cam_index][img_index][pt_index].y;
							p1[0] -= image_points[cam_index2][img_index][pt_index].x;
							p1[1] -= image_points[cam_index2][img_index][pt_index].y;
							dist += 0.5*sqrt(p1[0]*p1[0] + p1[1]*p1[1]);
							dist += 0.5*sqrt(p2[0]*p2[0] + p2[1]*p2[1]);
						}

						totalError += dist;
					}
				}
			}
		}
	}

	return totalError / count;
}

//---------------------------------------------------------------------

void CameraCalibration::estimateIntrinsics(const std::vector<int> &imageIndices) {
	for(size_t cam_index = 0; cam_index < cameras.size(); ++cam_index) {
		if(isCancelled()) break;

		CameraPtr cam = cameras[cam_index];
		int num_points = 0;

		// Form vectors of image points and object points for all images
		// in which the pattern is completely visible
		std::vector<std::vector<cv::Point3f> > object_points_temp;
		std::vector<std::vector<cv::Point2f> > image_points_temp;
		foreach(int img_index, imageIndices) {
			if(image_points[cam_index][img_index].size() == board_size.area()) {
				num_points += board_size.area();
				object_points_temp.push_back(object_points);
				image_points_temp.push_back(image_points[cam_index][img_index]);
			}
		}

		//
		if(num_points > 0) {
			cv::Mat camMatrix(3, 3, CV_64FC1), distCoeffs(5, 1, CV_64FC1);

			// Try at most n times to calibrate the system (just in case it
			// messes up with the intrinsic matrix in some iteration)
			double error = 1000.0;
			for(int iter = 0; error > 5 && iter < 1; ++iter) {
				Matrix3d K = cam->K();
				LensDistortions lensDist = cam->lensDistortion();

				if(iter > 0) {
					camMatrix.at<double>(0, 0) = K(0, 0);
					camMatrix.at<double>(0, 1) = K(0, 1);
					camMatrix.at<double>(0, 2) = K(0, 2);
					camMatrix.at<double>(1, 0) = K(1, 0);
					camMatrix.at<double>(1, 1) = K(1, 1);
					camMatrix.at<double>(1, 2) = K(1, 2);
					camMatrix.at<double>(2, 0) = K(2, 0);
					camMatrix.at<double>(2, 1) = K(2, 1);
					camMatrix.at<double>(2, 2) = K(2, 2);

					distCoeffs.at<double>(0) = lensDist[0];
					distCoeffs.at<double>(1) = lensDist[1];
					distCoeffs.at<double>(2) = lensDist[2];
					distCoeffs.at<double>(3) = lensDist[3];
					distCoeffs.at<double>(4) = lensDist[4];
				}

				std::vector<cv::Mat> rvecs, tvecs;
				error = cv::calibrateCamera(
								object_points_temp,
								image_points_temp,
								image_sizes[cam_index],
								camMatrix,
								distCoeffs,
								rvecs, tvecs,
								iter == 0 ? 0 : CV_CALIB_USE_INTRINSIC_GUESS);

				K(0, 0) = camMatrix.at<double>(0, 0);
				K(0, 1) = camMatrix.at<double>(0, 1);
				K(0, 2) = camMatrix.at<double>(0, 2);
				K(1, 0) = camMatrix.at<double>(1, 0);
				K(1, 1) = camMatrix.at<double>(1, 1);
				K(1, 2) = camMatrix.at<double>(1, 2);
				K(2, 0) = camMatrix.at<double>(2, 0);
				K(2, 1) = camMatrix.at<double>(2, 1);
				K(2, 2) = camMatrix.at<double>(2, 2);

				lensDist[0] = distCoeffs.at<double>(0);
				lensDist[1] = distCoeffs.at<double>(1);
				lensDist[2] = distCoeffs.at<double>(2);
				lensDist[3] = distCoeffs.at<double>(3);
				lensDist[4] = distCoeffs.at<double>(4);

				cam->setK(K);
				cam->setLensDistortion(lensDist);
			}
		}
	}
}

//---------------------------------------------------------------------

void CameraCalibration::estimateExtrinsics(const std::vector<int> &imageIndices) {
	// R_ij = rotation matrix for going from i to j (i < j)
	std::vector<std::vector<cv::Mat> > R(cameras.size());

	// t_ij = translation vector for going from i to j (i < j)
	std::vector<std::vector<cv::Mat> > t(cameras.size());

	// graph_ij = reprojection error from i to j (i < j)
	std::vector<std::vector<double> > graph;

	// Initialize values
	graph.resize(cameras.size());
	for(size_t i = 0; i < cameras.size(); ++i) {
		graph[i].resize(cameras.size(), std::numeric_limits<double>::infinity());
		R[i].resize(cameras.size());
		t[i].resize(cameras.size());
	}

	F.resize(cameras.size());
	for(size_t cam_index = 0; cam_index < cameras.size(); ++cam_index) {
		F[cam_index].resize(cameras.size());
		for(size_t cam_index2 = 0; cam_index2 < cameras.size(); ++cam_index2)
			F[cam_index][cam_index2] = cv::Mat();
	}

	// Use OpenCV to do pairwise calibration
	for(size_t cam_index = 0; cam_index < cameras.size(); ++cam_index) {
		CameraPtr cam1 = cameras[cam_index];
		for(size_t cam_index2 = cam_index + 1; cam_index2 < cameras.size(); ++cam_index2) {
			CameraPtr cam2 = cameras[cam_index2];

			int numSets = 0;

			// Find the image points in which both images had features
			std::vector<std::vector<cv::Point3f> > object_points_temp;
			std::vector<std::vector<cv::Point2f> > image_points1, image_points2;
			foreach(int img_index, imageIndices) {
				if(image_points[cam_index][img_index].size() == board_size.area()
					&& image_points[cam_index2][img_index].size() == board_size.area())
				{
					++numSets;
					object_points_temp.push_back(object_points);
					image_points1.push_back(image_points[cam_index][img_index]);
					image_points2.push_back(image_points[cam_index2][img_index]);
				}
			}

			// No overlap at all, well no need to go any further!
			if(numSets == 0) continue;

			// Load in existing intrinsic calibration
			const Eigen::Matrix3d &K1 = cam1->K();
			const Eigen::Matrix3d &K2 = cam2->K();

			const LensDistortions &lensDist1 = cam1->lensDistortion();
			const LensDistortions &lensDist2 = cam2->lensDistortion();

			cv::Mat camMatrix1(3, 3, CV_64FC1), distCoeffs1(5, 1, CV_64FC1);
			cv::Mat camMatrix2(3, 3, CV_64FC1), distCoeffs2(5, 1, CV_64FC1);
			cv::Mat E(3, 3, CV_64FC1);

			for(int r = 0; r < 3; ++r) {
				for(int c = 0; c < 3; ++c) {
					camMatrix1.at<double>(r, c) = K1(r, c);
					camMatrix2.at<double>(r, c) = K2(r, c);
				}
			}

			for(int r = 0; r < 5; ++r) {
				distCoeffs1.at<double>(r, 0) = lensDist1[r];
				distCoeffs2.at<double>(r, 0) = lensDist2[r];
			}

			//
			double error = cv::stereoCalibrate(object_points_temp,
											   image_points1,
											   image_points2,
											   camMatrix1, distCoeffs1,
											   camMatrix2, distCoeffs2,
											   image_sizes[cam_index],
											   R[cam_index][cam_index2],
											   t[cam_index][cam_index2],
											   E,
			                                  F[cam_index][cam_index2]);

			R[cam_index2][cam_index] = R[cam_index][cam_index2].t();
			t[cam_index2][cam_index] = -R[cam_index][cam_index2].t() * t[cam_index][cam_index2];
			F[cam_index2][cam_index] = F[cam_index][cam_index2].t();

			graph[cam_index][cam_index2] = error;
			graph[cam_index2][cam_index] = error;
		}
	}

	if(isCancelled()) return;

	// Given the graph of reprojection errors, perform all-pairs shortest
	// path. Iterate through every camera and find which one produces the
	// lowest average reprojection error when used as a reference frame.
	size_t bestFrameOfReference = cameras.size();
	double lowestAverageReprojectionError = std::numeric_limits<double>::infinity();

	FloydWarshall fw(graph);
	for(size_t i = 0; i < graph.size(); ++i) {
		double total = 0;
		for(size_t j = 0; j < graph.size(); ++j) {
			if(i != j)
				total += fw.cost(i, j);
		}

		total /= graph.size() - 1;
		if(total < lowestAverageReprojectionError) {
			lowestAverageReprojectionError = total;
			bestFrameOfReference = i;
		}
	}

	if(isCancelled()) return;

	qDebug() << "result: " << bestFrameOfReference << lowestAverageReprojectionError;
	if(bestFrameOfReference >= cameras.size())
		return; // uh oh

	// Frame of reference initialized as identity
	{
		CameraPtr cam = cameras[bestFrameOfReference];
		cam->set(cam->K(), Matrix3d::Identity(), Vector3d::Zero());
	}

	// Given the best frame of reference, concatenate the appropriate
	// pairwise calibrations together so that everyone's projection
	// matrix works in the same basis
	for(size_t i = 0; i < graph.size(); ++i) if(i != bestFrameOfReference) {
		std::deque<int> path = fw.path(bestFrameOfReference, i);

		cv::Mat	accumR = cv::Mat::eye(3, 3, CV_64FC1);
		cv::Mat accumT = cv::Mat::zeros(3, 1, CV_64FC1);
		for(size_t j = 1; j < path.size(); ++j) {
			const int v1 = path[j - 1];
			const int v2 = path[j];
			accumR = R[v1][v2] * accumR;
			accumT = R[v1][v2] * accumT + t[v1][v2];
		}

		// Update matricies in camera instance
		CameraPtr cam = cameras[i];

		Matrix3d newR;
		newR << accumR.at<double>(0, 0), accumR.at<double>(0, 1), accumR.at<double>(0, 2),
		        accumR.at<double>(1, 0), accumR.at<double>(1, 1), accumR.at<double>(1, 2),
		        accumR.at<double>(2, 0), accumR.at<double>(2, 1), accumR.at<double>(2, 2);

		Vector3d newt( accumT.at<double>(0, 0), accumT.at<double>(1, 0), accumT.at<double>(2, 0) );

		cam->set(cam->K(), newR, newt);
	}
}

//---------------------------------------------------------------------

void CameraCalibration::bundleAdjust() {
#ifdef USE_SBA
	//
	BundleAdjustmentData data(project, cameras, imageSets, NUM_CAM_PARAMS);

	for(size_t cam_index = 0; cam_index < cameras.size(); ++cam_index) {
		const Matrix3d &R = cameras[cam_index]->R();
		const Vector3d &t = cameras[cam_index]->t();

		const double theta = acos((R(0, 0) + R(1, 1) + R(2, 2) - 1)*0.5);
		double e1 = 0.0, e2 = 0.0, e3 = 0.0;
		if(theta > 1e-10) {
			const double two_sin_theta_inv = 0.5 / sin(theta);
			e1 = (R(2, 1) - R(1, 2)) * two_sin_theta_inv;
			e2 = (R(0, 2) - R(2, 0)) * two_sin_theta_inv;
			e3 = (R(1, 0) - R(0, 1)) * two_sin_theta_inv;
		}

		data.params[NUM_CAM_PARAMS*cam_index + 0] = theta*e1;
		data.params[NUM_CAM_PARAMS*cam_index + 1] = theta*e2;
		data.params[NUM_CAM_PARAMS*cam_index + 2] = theta*e3;
		data.params[NUM_CAM_PARAMS*cam_index + 3] = t.x();
		data.params[NUM_CAM_PARAMS*cam_index + 4] = t.y();
		data.params[NUM_CAM_PARAMS*cam_index + 5] = t.z();
#if INCLUDE_INTRINSIC_IN_BA
		//
		const Matrix3d &K = cameras[cam_index]->K();
		data.params[NUM_CAM_PARAMS*cam_index + 6] = K(0, 0);
		data.params[NUM_CAM_PARAMS*cam_index + 7] = K(1, 1);
		data.params[NUM_CAM_PARAMS*cam_index + 8] = K(0, 2);
		data.params[NUM_CAM_PARAMS*cam_index + 9] = K(1, 2);
#endif
	}

	//
	double opts[SBA_OPTSSZ] = { SBA_INIT_MU,
								 SBA_STOP_THRESH,
							     SBA_STOP_THRESH,
							     SBA_STOP_THRESH,
							     0.0 };
	double info[SBA_INFOSZ];

	int ret = sba_motstr_levmar(data.scene_points.size(),
								0,
								cameras.size(),
								0,
								&data.mask[0],
								&data.params[0],
								NUM_CAM_PARAMS,
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

	qDebug() << (ret == SBA_ERROR ? "error" : "no error") << info[6];
	if(ret != SBA_ERROR) {
		qDebug() << info[0] << info[1];

		// Update cameras' rotation matricies and translation vectors
		for(size_t cam_index = 0; cam_index < cameras.size(); ++cam_index) {
			//
			Vector3d w(data.params[NUM_CAM_PARAMS*cam_index + 0],
					   data.params[NUM_CAM_PARAMS*cam_index + 1],
					   data.params[NUM_CAM_PARAMS*cam_index + 2]);

			double theta = w.norm();
			if(fabs(theta) > 1e-10)
				w.normalize();

			Matrix3d wx; // "cross product" matrix for w
			wx << 0, -w.z(), w.y(), w.z(), 0, -w.x(), -w.y(), w.x(), 0;
			Matrix3d R = Matrix3d::Identity() + sin(theta)*wx + wx*wx*(1 - cos(theta));

			//
			Vector3d t(data.params[NUM_CAM_PARAMS*cam_index + 3],
					   data.params[NUM_CAM_PARAMS*cam_index + 4],
					   data.params[NUM_CAM_PARAMS*cam_index + 5]);

#ifdef INCLUDE_INTRINSIC_IN_BA
			//
			Matrix3d K = cameras[cam_index]->K();
			K(0, 0) = data.params[NUM_CAM_PARAMS*cam_index + 6];
			K(1, 1) = data.params[NUM_CAM_PARAMS*cam_index + 7];
			K(0, 2) = data.params[NUM_CAM_PARAMS*cam_index + 8];
			K(1, 2) = data.params[NUM_CAM_PARAMS*cam_index + 9];
#else
			const Matrix3d &K = cameras[cam_index]->K();
#endif
			//
			cameras[cam_index]->set(K, R, t);
		}

		// Translate every camera with respect to the first camera, so that
		// the first camera is at the origin
		Vector3d t_off = cameras.front()->t();
		for(size_t cam_index = 0; cam_index < cameras.size(); ++cam_index)
			cameras[cam_index]->sett(cameras[cam_index]->t() - t_off);
	}
#endif // USE_SBA
}

//---------------------------------------------------------------------

void CameraCalibration::calibrate() {
	int currentStep = 0;

	//
	// Load images and find corners
	//
	emit stageUpdate(tr("Finding corners..."));
	for(size_t cam_index = 0; cam_index < cameras.size(); ++cam_index) {
		if(isCancelled()) return;
		CameraPtr cam = cameras[cam_index];

		image_points[cam_index].resize(imageSets.size());
		for(size_t img_index = 0; img_index < imageSets.size(); ++img_index) {

			//
			ProjectImagePtr image = imageSets[img_index]->defaultImageForCamera(cam);
			if(image) {
				if(image_sizes[cam_index].width < 0) {
					cv::Mat img = cv::imread(image->file().absoluteFilePath().toStdString());
					image_sizes[cam_index] = cv::Size(img.cols, img.rows);
				}

				Features &features = project->features().features(image);
				for(Features::const_iterator iter = features.begin(); iter != features.end(); ++iter) {
					cv::Point2f p((*iter)->x(), (*iter)->y());
					image_points[cam_index][img_index].push_back(p);
				}
			}
		}
	}

	//
	// Calibrate intrinsic/extrinsic parameters
	//
	std::vector<int> indices(imageSets.size());
	for(size_t index = 0; index < imageSets.size(); ++index)
		indices[index] = index;

	const int num = std::min(static_cast<int>(indices.size()),
	                         std::max(30, static_cast<int>((4 * indices.size()) / 6)) );

	// Initial lowest error based on calibration that already exists in cams
	double lowestError = computeError();
	if(!std::isfinite(lowestError))
		lowestError = std::numeric_limits<double>::infinity();

	qDebug() << "Initial:" << lowestError;

	// Identity elements to begin
	std::vector<cv::Mat> K(cameras.size());
	std::vector<cv::Mat> lensDistortion(cameras.size());
	std::vector<cv::Mat> R(cameras.size());
	std::vector<cv::Mat> t(cameras.size());
	for(size_t cam_index = 0; cam_index < cameras.size(); ++cam_index) {
		K[cam_index] = cv::Mat::eye(3, 3, CV_64FC1);
	    lensDistortion[cam_index] = cv::Mat::zeros(5, 1, CV_64FC1);
		R[cam_index] = cv::Mat::eye(3, 3, CV_64FC1);
		t[cam_index] = cv::Mat::zeros(3, 1, CV_64FC1);

		storeCalibration(cameras[cam_index],
						 K[cam_index],
						 lensDistortion[cam_index],
						 R[cam_index],
						 t[cam_index]);
	}

	//
	// RANSAC-style approach to calibration to improve robustness. We
	//   randomly sample half of the image sets, use their features to
	//   calibrate the cameras, and then compute the reprojection error over
	//   all features. This way we have a control group to ensure that the
	//   obtained calibration is good.
	//
	emit stageUpdate(tr("Estimating intrinsic/extrinsic parameters of cameras..."));
	for(int iteration = 0; iteration < NUM_ITERATIONS; ++iteration) {
		// Reset to identity for this iteration
		for(size_t camIndex = 0; camIndex < cameras.size(); ++camIndex) {
			if(findExtrinsicParameters) {
				loadCalibration(cameras[camIndex],
				                cv::Mat::eye(3, 3, CV_64FC1),
				                cv::Mat::zeros(5, 1, CV_64FC1),
				                cv::Mat::eye(3, 3, CV_64FC1),
				                cv::Mat::zeros(3, 1, CV_64FC1));
			} else {
				loadCalibration(cameras[camIndex],
				                cv::Mat::eye(3, 3, CV_64FC1),
				                cv::Mat::zeros(5, 1, CV_64FC1),
				                cv::Mat(),
				                cv::Mat());
			}
		}

		// Randomly select indices
		std::random_shuffle(indices.begin(), indices.end());
		std::vector<int> currentIndices(indices.begin(), indices.begin() + num);

		// Estimate intrinsic/extrinsic paramters on given subset
		if(isCancelled()) return;
		estimateIntrinsics(currentIndices);

		if(isCancelled()) return;
		if(findExtrinsicParameters) estimateExtrinsics(currentIndices);

		emit progressUpdate(++currentStep);

		// Reset if not better
		double currentError = computeError();
		qDebug() << currentError;
		if(currentError > 1e-10 && currentError + 1e-10 < lowestError) {
			for(size_t camIndex = 0; camIndex < cameras.size(); ++camIndex) {
				if(findExtrinsicParameters) {
					storeCalibration(cameras[camIndex],
									 K[camIndex],
									 lensDistortion[camIndex],
									 R[camIndex],
									 t[camIndex]);
				} else {
					cv::Mat temp;
					storeCalibration(cameras[camIndex],
									 K[camIndex],
									 lensDistortion[camIndex],
									 temp,
									 temp);
				}
			}

			lowestError = currentError;
		}

		if(num == indices.size())
			break;
	}

	// Load best result
	for(size_t camIndex = 0; camIndex < cameras.size(); ++camIndex) {
		loadCalibration(cameras[camIndex],
		                K[camIndex],
	                    lensDistortion[camIndex],
	                    R[camIndex],
	                    t[camIndex]);
	}

	//
	// Bundle adjust
	//
#ifdef USE_SBA
	if(isCancelled()) return;
	emit stageUpdate(tr("Bundle adjustment..."));
	bundleAdjust();
#endif

	emit progressUpdate(++currentStep);
}

//---------------------------------------------------------------------
