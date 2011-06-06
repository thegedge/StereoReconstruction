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
#ifndef CORRESPONDENCE_HPP
#define CORRESPONDENCE_HPP

#include "util/c++0x.hpp"
#include "detector.hpp"

#include <utility>
#include <vector>

//
FORWARD_DECLARE(Feature);

typedef std::pair<FeaturePtr, FeaturePtr> Correspondence;
typedef std::vector<Correspondence>       Correspondences;

//! Establishes feature correspondences.
Correspondences findCorrespondences(const Features &set1, const Features &set2);

#endif // CORRESPONDENCE_HPP
