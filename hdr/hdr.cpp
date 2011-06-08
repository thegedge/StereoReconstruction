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

#include "hdr.hpp"
#include "project/camera.hpp"
#include "project/imageset.hpp"
#include "project/projectimage.hpp"

extern "C" {
	#include "rgbe.h"
}

#include <cmath>

#include <boost/filesystem.hpp>

#include <OpenEXR/ImfRgba.h>
#include <OpenEXR/ImfRgbaFile.h>

//---------------------------------------------------------------------

using std::min;
using std::max;
using std::log;
using std::exp;

//---------------------------------------------------------------------

MultiExposureToHDR::MultiExposureToHDR(CameraPtr camera, ImageSetPtr imageSet)
	: camera(camera)
{
	if(camera && camera->response().size() >= 256) {
		foreach(ProjectImagePtr image, imageSet->images()) {
			if(image->camera()->id() == camera->id() && image->exposure() > 0) {
				QImage imgData(image->file());
				if(!imgData.isNull())
					images << ImageDataPair(image, imgData);
			}
		}

		std::sort(images.begin(),
		          images.end(),
		          [](const ImageDataPair &a, const ImageDataPair &b) {
		              return (a.first->exposure() < b.first->exposure());
		          });
	}
}

//---------------------------------------------------------------------

bool MultiExposureToHDR::writeHDR(const std::string &path, HDRFormat format) const {
	if(images.size() <= 1)
		return false;

	// Guess the proper format from file extension if not specified
	if(format == HDRFormat::Invalid) {
		std::string ext = boost::filesystem::extension(path);
		if(ext == ".hdr")
			format = HDRFormat::RGBE;
		else if(ext == ".exr")
			format = HDRFormat::EXR;
	}

	//
	if(format == HDRFormat::Invalid)
		return false;

	//
	const int w = images[0].second.width();
	const int h = images[0].second.height();

	//
	double vals[4];

	switch(format) {
	case HDRFormat::EXR: {
			std::vector<Imf::Rgba> data(w*h);
			for(int y = 0; y < h; ++y) {
				for(int x = 0; x < w; ++x) {
					radianceFromPixels(QPoint(x, y), vals);
					data[y*w + x] = Imf::Rgba(vals[0], vals[1], vals[2], 0.0);
				}
			}

			Imf::RgbaOutputFile fout(path.c_str(), w, h);
			fout.setFrameBuffer(&data[0], 1, w);
			fout.writePixels(h);
		}
		break;
	case HDRFormat::RGBE: {
			std::vector<float> data(3*w*h);
			for(int y = 0; y < h; ++y) {
				for(int x = 0; x < w; ++x) {
					radianceFromPixels(QPoint(x, y), vals);
					data[3*(y*w + x) + 0] = vals[0];
					data[3*(y*w + x) + 1] = vals[1];
					data[3*(y*w + x) + 2] = vals[2];
					//data[4*(y*w + x) + 3] = 0.0;
				}
			}

			FILE *fout = fopen(path.c_str(), "w");
			RGBE_WriteHeader(fout, w, h, 0);
			RGBE_WritePixels_RLE(fout, &data[0], w, h);
			fclose(fout);
		}
		break;
	default:
		break;
	}

	return true;
}

//---------------------------------------------------------------------

void MultiExposureToHDR::radianceFromPixels(QPoint p, double values[4]) const {
	values[0] = values[1] = values[2] = values[3] = 0.0;

	const Responses &response = camera->response();

	//
	double totalWeight[3] = {0.0, 0.0, 0.0};
	foreach(const ImageDataPair &pair, images) {
		QRgb rgb = pair.second.pixel(p);

		double rWeight = weight(qRed(rgb));
		double gWeight = weight(qGreen(rgb));
		double bWeight = weight(qBlue(rgb));
		double deltaT = log(pair.first->exposure() / 1000.0);

		values[0] += rWeight*(response[qRed(rgb)][0]   - deltaT);
		values[1] += gWeight*(response[qGreen(rgb)][1] - deltaT);
		values[2] += bWeight*(response[qBlue(rgb)][2]  - deltaT);

		totalWeight[0] += rWeight;
		totalWeight[1] += gWeight;
		totalWeight[2] += bWeight;
	}

	// Divide by the total weight. If there is no weight at all, the pixel
	// was always underexposed or overexposed so make it as bright/dark as
	// we possibly can with the information that we have
	const ImageDataPair &pair = images[images.size() / 2];
	QRgb rgb = pair.second.pixel(p);
	int cols[3] = {qRed(rgb), qGreen(rgb), qBlue(rgb)};

	for(int ch = 0; ch < 3; ++ch) {
		if(totalWeight[ch] < 1e-10) {
			if(cols[ch] == 0) {
				double deltaT = log(images.back().first->exposure() / 1000.0);
				values[ch] = response[0][ch] - deltaT;
			} else {
				double deltaT = log(images.front().first->exposure() / 1000.0);
				values[ch] = response[255][ch] - deltaT;
			}
		} else {
			values[ch] /= totalWeight[ch];
		}

		values[ch] = std::exp(values[ch]);
	}
}

//---------------------------------------------------------------------

double MultiExposureToHDR::weight(int pixelValue) const {
#if 0
	// Gaussian(x; 127, 50)
	double x = pixelValue - 127;
	double invSigmaSquared = 1.0 / (25.0 * 25.0);
	return exp(-(x*x) * invSigmaSquared);
#elif 0
	// Hat function [truncated]
	return max(0, (pixelValue < 128 ? pixelValue : 255 - pixelValue) - 10) / 117.0;
#else
	// Hat * Gaussian(x; 127, 50)
	double x = pixelValue - 127;
	double invSigmaSquared = 1.0 / (25.0 * 25.0);
	double gv = exp(-x * x * invSigmaSquared);
	double hw = max(0, (pixelValue < 128 ? pixelValue : 255 - pixelValue) - 10) / 117.0;
	return gv * hw;
#endif
}

//---------------------------------------------------------------------

#endif
