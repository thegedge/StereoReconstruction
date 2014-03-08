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
#ifndef MULTIVIEWSTEREO_HPP
#define MULTIVIEWSTEREO_HPP

#include <QImage>

#include "gui/task.hpp"
#include "util/ray.hpp"
#include "util/vectorimage.hpp"

FORWARD_DECLARE(Project);
FORWARD_DECLARE(Camera);
FORWARD_DECLARE(ImageSet);

//---------------------------------------------------------------------
// TODO Move this somewhere else
typedef std::pair<Ray3d::Point, RGBA> PLYPoint;

//! Output a set of points to a PLY file
void outputPLYFile(const std::string &path, const std::vector<PLYPoint> &points);

//---------------------------------------------------------------------

//! Performs multi-view stereo reconstruction.
class MultiViewStereo : public Task {
public:
	void initialize(ProjectPtr project,
					ImageSetPtr imageSet,
					const std::vector<CameraPtr> &views,
					double minDepth, double maxDepth,
					int numDepthLevels,
					double crossCheckThreshold,
					double imageScale = 1.0);

public: // Task implementation
	QString title() const { return tr("Multi-view Stereo"); }
	int numSteps() const;

public:
	//! Return the depth map for the specified view
	QImage depthMap(CameraPtr view) const;

	//! Returns the image set used to compute the last set of depth maps
	ImageSetPtr imageSet() const { return imageSet_; }

protected:
	//! .
	RGBA colorFromDepth(double depth) const;

	//! .
	void computeInitialEstimate(size_t viewIndex);

	//! .
	void crossCheck(size_t view_index);

	//! .
	double depthFromLabel(int label) const;

	//! .
	bool pointFromDepth(const Ray3d &ray,
						const Ray3d::Vector &normal,
						double depth,
						Ray3d::Point &p) const;

	//! .
	std::vector<Eigen::Vector3d> epipolarCurve(
			const Ray3d &ray,
			const Eigen::Vector3d &cameraOffset,
			const Eigen::Vector3d &depthPlaneNormal,
			const VectorImage &mask,
			CameraPtr view) const;

protected: // Task implementation
	void runTask();

private:
	ProjectPtr project;
	ImageSetPtr imageSet_;

	std::vector<CameraPtr> views;
	std::vector<std::vector<size_t> > neighbours;

	std::vector<VectorImage> images;
	std::vector<VectorImage> masks;
	std::vector<VectorImage> results;

	typedef std::vector<double> DepthMap;
	std::vector<DepthMap> computedDepths;

	double minDepth, maxDepth;
	int numDepthLevels;
	double crossCheckThreshold;
	double imageScale;
};

#endif // MULTIVIEWSTEREO_HPP
