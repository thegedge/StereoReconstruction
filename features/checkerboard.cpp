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
#include "checkerboard.hpp"
#include "feature.hpp"
#include "project/imageset.hpp"
#include "project/projectimage.hpp"

#include <limits>
#include <opencv/cv.h>
#include <opencv/highgui.h>

//---------------------------------------------------------------------

class CheckerboardFeature : public Feature {
public:
	CheckerboardFeature() : Feature(0, 0) { }
	CheckerboardFeature(double x, double y, int index, QString imageSetID)
		: Feature(x, y)
		, imageSetID(imageSetID)
		, index(index)
	{ }

public:
	int cornerIndex() const { return index; }
	void setCornerIndex(int index) { this->index = index; }

public:
	void save(QDomElement &element) {
		element.setAttribute("type", "checkerboard");
		element.setAttribute("imageSet", imageSetID);
		element.setAttribute("cindex", index);
	}

	void load(const QDomElement &element)  {
		x_ = element.attribute("x", "0").toDouble();
		y_ = element.attribute("y", "0").toDouble();
		imageSetID = element.attribute("imageSet");
		index = element.attribute("cindex", "0").toInt();
	}

	int type() const {
		return CheckerboardDetector::FeatureType;
	}

	double compare(const Feature *other) const {
		double cost = std::numeric_limits<double>::infinity();
		if(other && other->type() == type()) {
			const CheckerboardFeature *a = static_cast<const CheckerboardFeature *>(other);
			if(imageSetID == a->imageSetID && index == a->index)
				cost = 0.0;
		}
		return cost;
	}

	QString shortDescription() {
		return QString::number(index);
	}

private:
	QString imageSetID;
	int index;
};

//---------------------------------------------------------------------

FeaturePtr CheckerboardDetector::load(const QDomElement &element) {
	FeaturePtr feature(new CheckerboardFeature);
	feature->load(element);
	return feature;
}

//---------------------------------------------------------------------

void CheckerboardDetector::rotateIndicies(Features &features) {
	// First find the max corner index so we know the range of indicies
	int maxIndex = 0;
	foreach(FeaturePtr feature, features) {
		if(feature->type() == CheckerboardDetector::FeatureType) {
			int cindex = std::static_pointer_cast<CheckerboardFeature>(feature)->cornerIndex();
			maxIndex = std::max(maxIndex, cindex);
		}
	}

	// Update the indicies
	foreach(FeaturePtr feature, features) {
		if(feature->type() == CheckerboardDetector::FeatureType) {
			std::shared_ptr<CheckerboardFeature> checkerFeature
				= std::static_pointer_cast<CheckerboardFeature>(feature);
			checkerFeature->setCornerIndex(maxIndex - checkerFeature->cornerIndex());
		}
	}

	std::reverse(features.begin(), features.end());
}

//---------------------------------------------------------------------

Features CheckerboardDetector::features(ProjectImagePtr img) {
	const cv::Size board_size(numCols - 1, numRows - 1);
	const int flags1 = cv::CALIB_CB_ADAPTIVE_THRESH
					   ;//+ cv::CALIB_CB_FAST_CHECK;

	const int flags2 = cv::CALIB_CB_ADAPTIVE_THRESH
					   + cv::CALIB_CB_NORMALIZE_IMAGE
					   ;//+ cv::CALIB_CB_FAST_CHECK;

	Features features;
	try {
		cv::Mat imageMat = cv::imread(img->file().toStdString(), 0);
		std::vector<cv::Point2f> corners;

		if(cv::findChessboardCorners(imageMat, board_size, corners, flags1)
		   || cv::findChessboardCorners(imageMat, board_size, corners, flags2))
		{
			QString imageSetID = img->imageSet()->id();
			for(int index = 0; index < board_size.area(); ++index) {
				const cv::Point2f &pt = corners[index];
				features.push_back(
						std::make_shared<CheckerboardFeature>(pt.x, pt.y, index, imageSetID) );
			}
		}
	} catch(const cv::Exception &e) {
		// XXX Do something?
	}

	return features;
}

//---------------------------------------------------------------------
