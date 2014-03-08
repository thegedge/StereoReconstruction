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
#include "featuredb.hpp"


Features & FeatureDatabase::features(ProjectImagePtr img) {
	return features_[img];
}

const Features & FeatureDatabase::features(ProjectImagePtr img) const {
	if(img) {
		FeaturesMap::const_iterator iter = features_.find(img);
		if(iter != features_.end())
			return iter->second;
	}

	static Features empty_features;
	return empty_features;
}

//---------------------------------------------------------------------

Correspondences &
FeatureDatabase::correspondences(ProjectImagePtr img1, ProjectImagePtr img2, bool *swap) {
	CorrespondencesMap::iterator iter = correspondences_.find(std::make_pair(img2, img1));

	if(swap)
		*swap = (iter != correspondences_.end());

	if(iter != correspondences_.end())
		return iter->second;

	return correspondences_[std::make_pair(img1, img2)];
}

const Correspondences &
FeatureDatabase::correspondences(ProjectImagePtr img1, ProjectImagePtr img2, bool *swap) const {
	CorrespondencesMap::const_iterator iter = correspondences_.find(std::make_pair(img2, img1));

	if(swap)
		*swap = (iter != correspondences_.end());

	if(iter != correspondences_.end())
		return iter->second;

	iter = correspondences_.find(std::make_pair(img1, img2));
	if(iter != correspondences_.end())
		return iter->second;

	static Correspondences empty;
	return empty;
}

//---------------------------------------------------------------------
