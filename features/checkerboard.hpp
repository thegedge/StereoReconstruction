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
#ifndef CHECKERBOARD_HPP
#define CHECKERBOARD_HPP

#include "detector.hpp"

#include <QDomElement>


//! Detector that finds checkerboard corners in an image.
class CheckerboardDetector : public FeatureDetector {
public:
	static FeaturePtr load(const QDomElement &element);
	static const int FeatureType = 1;

	//! Rotate the corner indicies of a set of features
	static void rotateIndicies(Features &features);

public:
	CheckerboardDetector(int numRows, int numCols)
		: numRows(numRows), numCols(numCols)
	{ }

	Features features(ProjectImagePtr img);

private:
	int numRows, numCols;
};

#endif // CHECKERBOARD_HPP
