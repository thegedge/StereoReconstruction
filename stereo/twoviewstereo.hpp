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
#ifndef STEREO_HPP
#define STEREO_HPP

#include "gui/task.hpp"
#include "util/plane.hpp"
#include "util/ray.hpp"
#include "util/vectorimage.hpp"

#include <QApplication>
#include <QImage>

#ifdef USE_MRF
#   include <MRF/mrf.h>
#endif

FORWARD_DECLARE(Camera);

//! Performs two-view stereo, producing depth maps for both views.
class TwoViewStereo : public Task {
public:
	typedef std::vector<double> DepthMap;

public:
	TwoViewStereo(CameraPtr leftView, QImage left, QImage leftMask,
				  CameraPtr rightView, QImage right, QImage rightMask,
				  double minDepth, double maxDepth, int numDepthLevels,
				  double imageScale = 1.0);

public: // Task implementation
	QString title() const { return QApplication::translate("TwoViewStereo", "Two-View Stereo"); }
	int numSteps() const { return 8; }
	void runTask() { computeDepthMaps(); }

public:
	//! .
	void computeDepthMaps();

	//! .
	QImage leftDepthMap() const { return VectorImage::toQImage(resultLeft); }

	//! .
	QImage rightDepthMap() const { return VectorImage::toQImage(resultRight); }

	//! .
	std::vector<Eigen::Vector3d> epipolarCurve(
			const Ray3d &ray,
			const Eigen::Vector3d &cameraOffset,
			const Eigen::Vector3d &depthPlaneNormal,
			const VectorImage &mask,
			CameraPtr view) const;

protected:
	//! .
	void computeCostVolumes(CameraPtr leftView, CameraPtr rightView);

	//! .
	void crossCheck(CameraPtr leftView, CameraPtr rightView);

	//! .
	void filterInvalidPixels();

	//! .
	double depthFromLabel(int label) const;

	//! .
	bool pointFromDepth(const Ray3d &ray,
						const Ray3d::Vector &normal,
						double depth,
						Ray3d::Point &p) const;

	//! .
	double cost_sad(
			const VectorImage &left, const VectorImage &right,
			const VectorImage &leftMask, const VectorImage &rightMask,
			int x1, int y1, int x2, int y2);

	//! .
	double cost_ncc(
			const VectorImage &left, const VectorImage &right,
			const VectorImage &leftMask, const VectorImage &rightMask,
			int x1, int y1, int x2, int y2);

	//! .
	double weightedMedian(const DepthMap &depths, const VectorImage &mask, int x, int y);

	//! .
	RGBA colorFromDepth(double depth) const;

private:
	CameraPtr leftView, rightView;

	VectorImage left, leftMask, resultLeft;
	VectorImage right, rightMask, resultRight;

	DepthMap computedDepthLeft;
	DepthMap computedDepthRight;

	double minDepth, maxDepth;
	int numDepthLevels;
	double imageScale;

#ifdef USE_MRF
	std::vector<MRF::CostVal> costVolume;
	std::vector<int> pixelToMRFIndexMap;
#endif
};

#endif // STEREO_HPP
