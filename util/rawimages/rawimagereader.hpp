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

// Bayer filter mapping (currently fixed)
#define MASK 0x47524247 // 'GRBG'

// Get a pixel in a safe way, incrementing counter if the point is
//   within the image bounds
#define GET_PIXEL(data, result, x, y, w, h, counter) \
	if(x >= 0 && y >= 0 && x < w && y < h) {         \
		result += data[(y)*w + x];                   \
		counter++;                                   \
	}

// Get a green pixel in a safe way, incrementing counter if the
//   point is within the image bounds
#define GET_GREEN_PIXEL(data, result, x, y, w, h, counter) \
	if(x >= 0 && y >= 0 && x < w && y < h) {         \
		result += data[3*((y)*w + x) + 1];           \
		counter++;                                   \
	}


//! Bayer filter data to RGB data (no interpolation)
unsigned char * rawImageToRGB_none(const unsigned char *data, unsigned char *out, int w, int h);

//! Bayer filter data to RGB data (nearest neighbour interpolation)
unsigned char * rawImageToRGB_nn(const unsigned char *data, unsigned char *out, int w, int h);

//! Bayer filter data to RGB data (bilinear interpolation)
unsigned char * rawImageToRGB_bl(const unsigned char *data, unsigned char *out, int w, int h);

//! Bayer filter data to RGB data (smooth hue transition interpolation)
unsigned char * rawImageToRGB_hue(const unsigned char *data, unsigned char *out, int w, int h);

//! Bayer filter data to RGB data (edge sensing interpolation)
unsigned char * rawImageToRGB_es(const unsigned char *data, unsigned char *out, int w, int h);

//! Bayer filter data to RGB data (variable gradients interpolation; not implemented)
unsigned char * rawImageToRGB_vg(const unsigned char *data, unsigned char *out, int w, int h);
