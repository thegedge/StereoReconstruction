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
#include "rawimagereader.hpp"

unsigned char * rawImageToRGB_none(const unsigned char *data, unsigned char *out, int w, int h) {
	for(int y = 0; y < h; ++y) {
		for(int x = 0; x < w; ++x) {
			const int maskShift = (((x + 1) % 2) + 2*((y + 1) % 2)) * 8;
			const int maskVal = (MASK & (0xFF << maskShift)) >> maskShift;
			const unsigned char dataVal = data[x + w*y];

			switch(maskVal) {
				case 'R':
					out[3*(x + w*y) + 0] = dataVal;
					out[3*(x + w*y) + 1] = 0;
					out[3*(x + w*y) + 2] = 0;
					break;
				case 'G':
					out[3*(x + w*y) + 0] = 0;
					out[3*(x + w*y) + 1] = dataVal;
					out[3*(x + w*y) + 2] = 0;
					break;
				case 'B':
					out[3*(x + w*y) + 0] = 0;
					out[3*(x + w*y) + 1] = 0;
					out[3*(x + w*y) + 2] = dataVal;
					break;
				}
		}
	}
	return out;
}

/*!
 * Nearest neighbour interpolation. We consider the nearest neighbour to be
 * before the current pixel, that is, to the left, above, or both. If at the
 * boundary then we look to the right or down.
 */
unsigned char * rawImageToRGB_nn(const unsigned char *data, unsigned char *out, int w, int h) {
	for(int y = 0; y < h; ++y) {
		for(int x = 0; x < w; ++x) {
			const int maskShift = (((x + 1) % 2) + 2*((y + 1) % 2)) * 8;
			const int maskVal = (MASK & (0xFF << maskShift)) >> maskShift;
			const unsigned char dataVal = data[x + w*y];

			switch(maskVal) {
				case 'R':
					out[3*(x + w*y) + 0] = dataVal;
					out[3*(x + w*y) + 1] = data[(x-1) + w*y];
					out[3*(x + w*y) + 2] = (y > 0 ? data[(x-1) + w*(y-1)] : data[(x-1) + w]);
					break;
				case 'G':
					out[3*(x + w*y) + 1] = dataVal;
					if(y % 2 == 0) {
						out[3*(x + w*y) + 0] = (x > 0 ? data[(x-1) + w*y] : data[(x+1) + w*y]);
						out[3*(x + w*y) + 2] = (y > 0 ? data[x + w*(y-1)] : data[x + w*(y+1)]);
					} else {
						out[3*(x + w*y) + 0] = data[x + w*(y-1)];
						out[3*(x + w*y) + 2] = data[(x-1) + w*y];
					}
					break;
				case 'B':
					out[3*(x + w*y) + 0] = (x > 0 ? data[(x-1) + w*(y-1)] : data[(x+1) + w*(y-1)] );
					out[3*(x + w*y) + 1] = data[x + w*(y-1)];
					out[3*(x + w*y) + 2] = dataVal;
					break;
			}
		}
	}
	return out;
}
