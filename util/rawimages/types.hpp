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
#ifndef __TYPES_H__
#define __TYPES_H__

//
#ifndef NULL
#   define NULL 0
#endif

// Bitmap structures
typedef struct BMFH {
	unsigned short bfType;          // file identifier 'MB'
	unsigned long bfSize;           // sizeof(BITMAPFILEHEADER)
	unsigned short bfReserved1;     // reserved
	unsigned short bfReserved2;     // reserved
	unsigned long bfOffBits;        // sizeof(BITMAPFILEHEADER) + sizeof(BITMAPINFOHEADER)
} BITMAPFILEHEADER;

typedef struct BMIH {
	unsigned long biSize;
	unsigned long biWidth;
	unsigned long biHeight;
	unsigned short biPlanes;
	unsigned short biBitCount;
	unsigned long biCompression;
	unsigned long biSizeImage;
	unsigned long biXPelsPerMeter;
	unsigned long biYPelsPerMeter;
	unsigned long biClrUsed;
	unsigned long biClrImportant;
} BITMAPINFOHEADER;


#endif //#ifndef __TYPES_H__
