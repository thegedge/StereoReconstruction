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
#include "correspondence.hpp"
#include "feature.hpp"
#include <limits>

//---------------------------------------------------------------------
namespace {
	const double INF = std::numeric_limits<double>::infinity();
}

Correspondences findCorrespondences(const Features &set1, const Features &set2) {
	Correspondences correspondences;
	for(size_t index1 = 0; index1 < set1.size(); ++index1) {
		FeaturePtr feature1 = set1[index1];

		double bestCost = INF;
		double secondBestCost = bestCost;

		FeaturePtr best;
		for(size_t index2 = 0; index2 < set2.size(); ++index2) {
			FeaturePtr feature2 = set2[index2];
			double cost = feature1->compare(feature2.get());
			if(cost < bestCost) {
				secondBestCost = bestCost;
				bestCost = cost;
				best = feature2;
			}
		}

		// Make sure the best correspondence is a certain ratio better than
		// the second best feature (to avoid repetitive image features)
		if(bestCost < INF && bestCost < 0.5 && bestCost < 0.8*secondBestCost)
			correspondences.push_back(Correspondence(feature1, best));
	}
	return correspondences;
}

//---------------------------------------------------------------------
