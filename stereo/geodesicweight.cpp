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
//
// This is an implmentation of the weighting method described in
// Hosni et al's paper "Local Stereo Matching Using Geodesic Support
// Weights" (2009).
//
//---------------------------------------------------------------------
#include "geodesicweight.hpp"

//---------------------------------------------------------------------

namespace {
	//
	const double GEODESIC_SIGMA = 50.0;

	// Num iterations for sweeping shortest-path method
	const int NUM_ITERS = 3;

	// Kernel for sweeping shortest-path method
	const int KERNEL_SIZE = 8;
	const double K1[] = {-1, -1, 0, -1, 1, -1, -1, 0};
	const double K2[] = {-1,  1, 0,  1, 1,  1,  1, 0};
}

using std::sqrt;
using std::min;

//---------------------------------------------------------------------

void GeodesicWeight::initialize(int radius) {
	this->radius = radius;

	weights.resize(2*radius + 1);
	for(int ind = 0; ind < 2*radius + 1; ++ind)
		weights[ind].resize(2*radius + 1, 0.0);
}

//---------------------------------------------------------------------

void GeodesicWeight::init_weights(const VectorImage &img, int xx, int yy) {
	this->cx = xx;
	this->cy = yy;

	//
	const int WINDOW_SIZE = 2*radius + 1;

	// Initialize weights to large values, and center pixel to 0
	for(int y = 0; y < WINDOW_SIZE; ++y)
		std::fill(weights[y].begin(), weights[y].end(), 1000000.0);

	weights[radius][radius] = 0.0;

	// Approximate geodesic distance
	for(int iter = 0; iter < NUM_ITERS; ++iter) {
		// Forward pass
		for(int y = -radius; y <= radius; ++y) {
			for(int x = -radius; x <= radius; ++x) {
				const RGBA &rgb1 = img.pixel(cx + x, cy + y);
				if(!rgb1.isValid())
					continue;

				double &weight = weights[y + radius][x + radius];
				for(int ind = 0; ind < KERNEL_SIZE; ind += 2) {
					int dx = K1[ind + 0];
					int dy = K1[ind + 1];
					//if(x + dx < -radius || y + dy < -radius)
					if(x + dx > radius || y + dy > radius || x + dx < -radius || y + dy < -radius)
						continue;

					RGBA rgb2 = img.pixel(cx + x + dx, cy + y + dy);
					if(rgb2.isValid()) {
						rgb2 -= rgb1;
						double diff = sqrt(rgb2.r*rgb2.r + rgb2.g*rgb2.g + rgb2.b*rgb2.b);
						double cost = weights[y + dy + radius][x + dx + radius];
						weight = min(weight, cost + diff);
					}
				}
			}
		}

		// Backwards pass
		for(int y = radius; y >= -radius; --y) {
			for(int x = radius; x >= -radius; --x) {
				const RGBA &rgb1 = img.pixel(cx + x, cy + y);
				if(!rgb1.isValid())
					continue;

				double &weight = weights[y + radius][x + radius];
				for(int ind = 0; ind < KERNEL_SIZE; ind += 2) {
					int dx = K2[ind + 0];
					int dy = K2[ind + 1];
					//if(x + dx > radius || y + dy > radius)
					if(x + dx > radius || y + dy > radius || x + dx < -radius || y + dy < -radius)
						continue;

					RGBA rgb2 = img.pixel(cx + x + dx, cy + y + dy);
					if(rgb2.isValid()) {
						rgb2 -= rgb1;
						double diff = sqrt(rgb2.r*rgb2.r + rgb2.g*rgb2.g + rgb2.b*rgb2.b);
						double cost = weights[y + dy + radius][x + dx + radius];
						weight = min(weight, cost + diff);
					}
				}
			}
		}
	}

	// Exponential weighting
	for(int y = 0; y < WINDOW_SIZE; ++y)
		for(int x = 0; x < WINDOW_SIZE; ++x)
			weights[y][x] = std::exp(-weights[y][x] / GEODESIC_SIGMA);
}

//---------------------------------------------------------------------

//---------------------------------------------------------------------
