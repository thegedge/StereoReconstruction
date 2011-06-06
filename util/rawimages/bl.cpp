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

/*!
 * Bilinear interpolation. Considers a weighted average of surrounding pixels
 * of the same color.
 */
unsigned char * rawImageToRGB_bl(const unsigned char *data, unsigned char *ret, int w, int h) {
	for(int y = 0; y < h; ++y) {
		for(int x = 0; x < w; ++x) {
			const int maskShift = (((x + 1) % 2) + 2*((y + 1) % 2)) * 8;
			const int maskVal = (MASK & (0xFF << maskShift)) >> maskShift;

			//
			// Green pixels
			//
			switch(maskVal) {
				case 'R':
				case 'B':
				{
					int counter = 0;
					int sum = 0;
					GET_PIXEL(data, sum, x, y-1, w, h, counter);
					GET_PIXEL(data, sum, x, y+1, w, h, counter);
					GET_PIXEL(data, sum, x-1, y, w, h, counter);
					GET_PIXEL(data, sum, x+1, y, w, h, counter);
					ret[3*(y*w + x) + 1] = (counter == 0 ? 0 : sum / counter);
					break;
				}
				case 'G':
					ret[3*(y*w + x) + 1] = data[y*w + x];
					break;
			}

			//
			// Red/Blue pixels
			//
			switch(maskVal) {
				case 'G':
				{
					int counter1 = 0, counter2 = 0;
					int sum1 = 0, sum2 = 0;
					GET_PIXEL(data, sum1, x, y-1, w, h, counter1);
					GET_PIXEL(data, sum1, x, y+1, w, h, counter1);
					GET_PIXEL(data, sum2, x-1, y, w, h, counter2);
					GET_PIXEL(data, sum2, x+1, y, w, h, counter2);

					if(y % 2 == 0) {
						ret[3*(y*w + x) + 0] = (counter1 == 0 ? 0 : sum2 / counter2);
						ret[3*(y*w + x) + 2] = (counter2 == 0 ? 0 : sum1 / counter1);
					} else {
						ret[3*(y*w + x) + 0] = (counter2 == 0 ? 0 : sum1 / counter1);
						ret[3*(y*w + x) + 2] = (counter1 == 0 ? 0 : sum2 / counter2);
					}

					break;
				}
				case 'R':
				case 'B':
				{
					int counter = 0;
					int sum = 0;
					GET_PIXEL(data, sum, x-1, y-1, w, h, counter);
					GET_PIXEL(data, sum, x-1, y+1, w, h, counter);
					GET_PIXEL(data, sum, x+1, y-1, w, h, counter);
					GET_PIXEL(data, sum, x+1, y+1, w, h, counter);

					if(y % 2 == 0) {
						ret[3*(y*w + x) + 0] = data[y*w + x];
						ret[3*(y*w + x) + 2] = (counter == 0 ? 0 : sum / counter);
					} else {
						ret[3*(y*w + x) + 0] = (counter == 0 ? 0 : sum / counter);
						ret[3*(y*w + x) + 2] = data[y*w + x];
					}

					break;
				}
			}
		}
	}

	return ret;
}
