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
#ifndef GEODESIC_HPP
#define GEODESIC_HPP

#include "util/vectorimage.hpp"


struct GeodesicWeight : public std::binary_function<int, int, double> {
public:
	GeodesicWeight() { }
	GeodesicWeight(int radius) { initialize(radius); }

	void initialize(int radius);
	void init_weights(const VectorImage &img, int x, int y);

	double operator()(int row, int col) const {
		return weights[row + radius][col + radius];
	}

private:
	int cx, cy;
	int radius;
	std::vector<std::vector<double> > weights;
};

#endif // GEODESIC_HPP
