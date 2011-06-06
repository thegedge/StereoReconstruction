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
#include "adaptiveweight.hpp"
#include <cmath>

//---------------------------------------------------------------------

namespace {
	const double COLOR_SIGMA = 10.0;
}

using std::sqrt;

//---------------------------------------------------------------------

void AdaptiveWeight::initialize(int radius) {
	this->radius = radius;

	distance_weights.resize(radius + 1);
	for(int ind = 0; ind <= radius; ++ind)
		distance_weights[ind] = std::exp(-ind / (1.0*radius));

	weights.resize(2*radius + 1);
	for(int ind = 0; ind < 2*radius + 1; ++ind)
		weights[ind].resize(2*radius + 1, 0.0);
}

//---------------------------------------------------------------------

void AdaptiveWeight::init_weights(const VectorImage &img, int cx, int cy) {
	this->cx = cx;
	this->cy = cy;

	//
	crgb = img.pixel(cx, cy);

	//
	for(int row = -radius; row <= radius; ++row)
		for(int col = -radius; col <= radius; ++col)
			weights[row + radius][col + radius] = weight(img, row, col);
}

//---------------------------------------------------------------------

double AdaptiveWeight::weight(const VectorImage &img, int row, int col) {
	double weight = 0.0;

	RGBA rgb = img.pixel(cx + col, cy + row);
	if(rgb.isValid()) {
		rgb -= crgb;

		const double diff = sqrt(rgb.r*rgb.r + rgb.g*rgb.g + rgb.b*rgb.b);
		const double w1 = distance_weights[abs(row)]*distance_weights[abs(col)];
		const double w2 = std::exp(-diff / COLOR_SIGMA);

		weight = w1*w2;
		if(std::isnan(weight))
			weight = 0.0;
	}

	return weight;
}

//---------------------------------------------------------------------
