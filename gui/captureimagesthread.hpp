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
#ifndef CAPTUREIMAGESTHREAD_H
#define CAPTUREIMAGESTHREAD_H

#include <QThread>
#include <vector>
#include <boost/shared_array.hpp>

#include "util/c++0x.hpp"

//! Simple wrapper for some raw data that represents a 2D image
template <class ImageDataType>
struct Image {
	typedef boost::shared_array<ImageDataType> ImageData;

	ImageData data;
	int width, height;

	Image() : width(0), height(0) { }

	Image(int width, int height)
		: width(width), height(height)
	{ }

	Image(int width, int height, ImageData data)
		: width(width), height(height), data(data)
	{ }
};

//! A thread for capturing images
template <class ImageDataType>
class CaptureImagesThread : public QThread {
public:
	typedef Image<ImageDataType>                      ImageType;
	typedef typename Image<ImageDataType>::ImageData  DataType;

public:
    CaptureImagesThread();

	virtual void run();

public:
	ImageType image(int imageIndex) { return image_data[imageIndex]; }

	const std::vector<ImageType> & images() const { return image_data; }
	std::vector<ImageType> & images()             { return image_data; }

	void setImages(const std::vector<ImageType> &images) { image_data = images; }
	void setFPS(double fps) { this->fps = fps; }
	void setExposureTime(double ms) { this->exposure = ms; }
	void setGain(double db) { this->gain = db; }
	void setShouldPowerDown(bool powerDown) { this->powerDown = powerDown; }

	void msleep(unsigned long msecs) { QThread::msleep(msecs); }

private:
	std::vector<ImageType> image_data;

	const int width, height;
	double fps, exposure, gain;
	bool powerDown;
};

#endif // CAPTUREIMAGESTHREAD_H
