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
#ifdef HAS_HDR
#ifndef HDR_HPP
#define HDR_HPP

#include <QImage>
#include <QList>
#include <QPair>
#include "util/c++0x.hpp"

FORWARD_DECLARE(Camera);
FORWARD_DECLARE(ImageSet);
FORWARD_DECLARE(ProjectImage);

//! HDR output format
enum class HDRFormat {
	Invalid,
	EXR,
	RGBE
};

//! A class to combine multiple image exposures into an HDR format
class MultiExposureToHDR {
	typedef QPair<ProjectImagePtr, QImage> ImageDataPair;

public:
	MultiExposureToHDR(CameraPtr camera, ImageSetPtr imageSet);

public:
	bool writeHDR(const std::string &path, HDRFormat format = HDRFormat::Invalid) const;

private:
	//! ;
	void radianceFromPixels(QPoint p, double values[4]) const;

	double weight(int pixelValue) const;

private:
	QList<ImageDataPair> images;
	CameraPtr camera;
};

#endif // #ifndef HDR_HPP
#endif // #ifdef HAS_HDR
