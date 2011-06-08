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
#include "surf.hpp"
#include "feature.hpp"
#include "project/projectimage.hpp"

#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>

//---------------------------------------------------------------------

class SurfFeature : public Feature {
public:
	SurfFeature() : Feature(0, 0) { }

	SurfFeature(double x, double y,
				const std::vector<float> &descriptor,
				float size,
				float angle,
				float response)
		: Feature(x, y)
		, descriptor(descriptor)
		, size(size)
		, angle(angle)
		, response(response)
	{ }

public:
	void save(QDomElement &element) {
		element.setAttribute("type", "surf");
		// TODO implement
	}

	void load(const QDomElement &element) {
		x_ = element.attribute("x", "0").toDouble();
		y_ = element.attribute("y", "0").toDouble();
		// TODO implement
	}

	int type() const {
		return SurfDetector::FeatureType;
	}

	double compare(const Feature *other) const {
		double cost = std::numeric_limits<double>::infinity();
		if(other && other->type() == type()) {
			const SurfFeature *o = static_cast<const SurfFeature *>(other);

			if(descriptor.size() == o->descriptor.size()) {
				cost = 0.0;
				for(size_t i = 0; i < descriptor.size(); ++i)
					cost += fabs(descriptor[i] - o->descriptor[i]);
			}
		}
		return cost;
	}

private:
	std::vector<float> descriptor;
	float size;
	float angle;
	float response;
};

//---------------------------------------------------------------------

FeaturePtr SurfDetector::load(const QDomElement &element) {
	FeaturePtr feature(new SurfFeature);
	feature->load(element);
	return feature;
}

//---------------------------------------------------------------------

Features SurfDetector::features(ProjectImagePtr img) {
	cv::Mat image = cv::imread(img->file().toStdString());
	cv::Mat image_gray(image .rows, image .cols, CV_8UC1);
	cv::cvtColor(image, image_gray, CV_RGB2GRAY);

	// Get SURF features
	std::vector<cv::KeyPoint> kps;
	std::vector<float> descrs;
	Features features;

	cv::SURF surfDetector;
	surfDetector(image_gray, cv::Mat(), kps, descrs);
	features.reserve(kps.size() + 1);

	// Convert to our feature type
	std::vector<cv::KeyPoint>::const_iterator kpIter;
	std::vector<float>::const_iterator descrIter;

	const int DESCR_SIZE = surfDetector.descriptorSize();

	for(kpIter = kps.begin(), descrIter = descrs.begin()
	    ; kpIter != kps.end() && descrIter != descrs.end()
		; ++kpIter, descrIter += DESCR_SIZE)
	{
		std::vector<float> descriptor(descrIter, descrIter + DESCR_SIZE);
		features.push_back( std::make_shared<SurfFeature>(kpIter->pt.x,
														   kpIter->pt.y,
														   descriptor,
														   kpIter->size,
														   kpIter->angle,
														   kpIter->response) );
	}

	return features;
}

//---------------------------------------------------------------------
