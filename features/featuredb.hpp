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
#ifndef FEATUREDB_HPP
#define FEATUREDB_HPP

#include <QDomElement>
#include "feature.hpp"
#include "correspondence.hpp"


FORWARD_DECLARE(ProjectImage);

typedef std::pair<ProjectImagePtr, ProjectImagePtr> ImagePair;
typedef std::map<ProjectImagePtr, Features> FeaturesMap;
typedef std::map<ImagePair, Correspondences> CorrespondencesMap;

//! A database of features and feature correspondences
class FeatureDatabase {
public:
	Features & features(ProjectImagePtr image);
	const Features & features(ProjectImagePtr image) const;

	/*!
	 * Grab the correspondences between the features for two images. Since
	 * correspondence is assumed to be
	 */
	Correspondences & correspondences(ProjectImagePtr img1, ProjectImagePtr img2, bool *swap = nullptr);
	const Correspondences & correspondences(ProjectImagePtr img1, ProjectImagePtr img2, bool *swap = nullptr) const;

	const FeaturesMap & features() const { return features_; }
	const CorrespondencesMap & correspondences() const { return correspondences_; }

private:
	FeaturesMap features_;
	CorrespondencesMap correspondences_;
};

#endif // #ifndef FEATUREDB_HPP
