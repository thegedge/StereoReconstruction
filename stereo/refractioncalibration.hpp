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
#ifndef REFRACTIONCALIBRATION_HPP
#define REFRACTIONCALIBRATION_HPP

#include "features/correspondence.hpp"

#include "util/lm.hpp"

FORWARD_DECLARE(Project);
FORWARD_DECLARE(Camera);
FORWARD_DECLARE(ImageSet);
FORWARD_DECLARE(FeatureDatabase);

/*!
 * Calibrates the parameters for a planar refractive interface in a scene
 * (normal + distance).
 *
 * TODO
 *   Convert this class to use the Task interface from
 *   <tt>gui/progressdialog.hpp</tt>
 */
class RefractionCalibration {
public:
	//! .
    RefractionCalibration();

public:
	//! .
	bool calibrate();

	//! .
	double error(const Correspondence &corr, CameraPtr view1, CameraPtr view2) const;

	//! .
	double totalError(double *average = nullptr) const;

	//! Set the current/initial model estimate used in calibration
	void setModel(const LevenbergMarquardt::Model &model);

	/*!
	 * Set the current/initial model estimate used in calibration, and any
	 * parameters that will remain fixed (i.e., will not be optimized)
	 */
	void setModel(const LevenbergMarquardt::Model &model,
	              const LevenbergMarquardt::FixedParams &fixed);

	/*!
	 * The resulting model from the previous calibration, or the model
	 * that was set using #setModel
	 */
	LevenbergMarquardt::Model model() const { return model_; }

	//! Set the feature database to use
	void setProject(ProjectPtr project);

	//! Set the views used in calibration
	void setViews(const std::vector<CameraPtr> &views) {
		this->views = views;
	}

	//! Set the image sets to use in the next calibration run
	void setImageSets(const std::vector<ImageSetPtr> &imageSets) {
		this->imageSets = imageSets;
	}

private:
	//! .
	ProjectPtr project;

	//! .
	const FeatureDatabase *db;

	//! .
	std::vector<CameraPtr> views;

	//! .
	std::vector<ImageSetPtr> imageSets;

	//! .
	LevenbergMarquardt::Model model_;

	//! .
	LevenbergMarquardt::FixedParams fixed;
};

#endif // REFRACTIONCALIBRATION_HPP
