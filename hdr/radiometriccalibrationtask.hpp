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
#ifndef RADIOMETRICCALIBRATIONTHREAD_HPP
#define RADIOMETRICCALIBRATIONTHREAD_HPP

#include <QList>
#include <QImage>
#include <QThread>

#include "gui/task.hpp"

//
// Forward declarations
//
FORWARD_DECLARE(Camera);
FORWARD_DECLARE(ImageSet);

//! A Task wrapper for performing radiometric calibration
class RadiometricCalibrationTask : public Task {
	Q_OBJECT

public:
    RadiometricCalibrationTask(CameraPtr camera, ImageSetPtr imageSet);

public:
	//! .
	QString title() const { return tr("Calibrating Camera"); }

	//! .
	int numSteps() const { return 5; }

	//! .
	void calibrate();

protected:
	void runTask() { calibrate(); }

private:
#ifdef USE_MITSUNAGA_NAYAR
	typedef std::pair<int, int> Sample;
	typedef std::vector<Sample> Samples;

	/*!
	 * Obtains the response values for a set of samples using the method
	 * described by Debevec and Malik.
	 */
	std::vector<double> responseCurve(const Samples &samples);
#else
	typedef std::pair<int, int> Sample;
	typedef std::vector<Sample> Samples;

	/*!
	 * Obtains the response values for a set of samples using the method
	 * described by Debevec and Malik.
	 *
	 * Assumes that the samples are a multiple of images.size(), where
	 * N = samples.size() / images.size() are the number of pixel locations
	 * used throughout all exposures. Then, samples should be a partitioning
	 * where the first N samples are from images[0], the next N from images[1]
	 * and so on.
	 */
	std::vector<double> responseCurve(QList<int> images, const Samples &samples);
#endif

private:
	CameraPtr camera;
	ImageSetPtr imageSet;

	QList<QImage> images;
};

#endif // RADIOMETRICCALIBRATIONTHREAD_HPP
