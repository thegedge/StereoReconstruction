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
#include "vectorimage.hpp"

#include <limits>

//---------------------------------------------------------------------

const RGBA BLACK(0, 0, 0);
const RGBA WHITE(255, 255, 255);

const RGBA RED(255, 0, 0);
const RGBA GREEN(0, 255, 0);
const RGBA BLUE(0, 0, 255);

const RGBA YELLOW(255, 255, 0);
const RGBA MAGENTA(255, 0, 255);

const RGBA INVALID(
		std::numeric_limits<double>::quiet_NaN(),
		std::numeric_limits<double>::quiet_NaN(),
		std::numeric_limits<double>::quiet_NaN() );

//---------------------------------------------------------------------

VectorImage VectorImage::fromFile(const std::string &file) {
	return fromQImage(QImage(file.c_str()));
}

//---------------------------------------------------------------------

VectorImage VectorImage::fromQImage(QImage img) {
	std::vector<RGBA> data;
	int w = 0, h = 0;

	if(!img.isNull()) {
		w = img.width();
		h = img.height();
		data.resize(w*h);

		// XXX Assumes 32-bit data
		for(int y = 0, p = 0; y < h; ++y) {
			QRgb *scanline = reinterpret_cast<QRgb *>(img.scanLine(y));
			for(int x = 0; x < w; ++x, ++scanline, ++p) {
				QRgb rgb = *scanline;
				data[p] = RGBA(qRed(rgb), qGreen(rgb), qBlue(rgb), qAlpha(rgb));
			}
		}
	}

	return VectorImage(w, h, data);
}

//---------------------------------------------------------------------

QImage VectorImage::toQImage(const VectorImage &img, bool includeAlpha) {
	QImage ret;
	if(img.width() > 0 && img.height() > 0) {
		ret = QImage(img.width(), img.height(), QImage::Format_ARGB32);

		for(int y = 0; y < img.height(); ++y) {
			QRgb *scanline = reinterpret_cast<QRgb *>(ret.scanLine(y));
			for(int x = 0; x < img.width(); ++x, ++scanline) {

				const RGBA &rgb = img.pixel(x, y);
				if(includeAlpha)
					*scanline = qRgba(rgb.r, rgb.g, rgb.b, rgb.a);
				else
					*scanline = qRgba(rgb.r, rgb.g, rgb.b, 255);
			}
		}
	}
	return ret;
}

//---------------------------------------------------------------------

VectorImage::VectorImage() : w(0), h(0) { }

VectorImage::VectorImage(int w, int h, const RGBA &fillVal)
	: w(w), h(h), data(w*h, fillVal)
{ }

VectorImage::VectorImage(int w, int h, std::vector<RGBA>& data)
	: w(w), h(h)
{
	this->data.swap(data);
}

//---------------------------------------------------------------------

VectorImage & VectorImage::fill(const RGBA &rgb) {
	std::fill(data.begin(), data.end(), rgb);
	return (*this);
}

//---------------------------------------------------------------------

const RGBA & VectorImage::pixel(int x, int y) const  {
	if(x < 0 || y < 0 || x >= w || y >= h)
		return INVALID;
	return data[y*w + x];
}

void VectorImage::setPixel(int x, int y, const RGBA &rgb) {
	if(x < 0 || y < 0 || x >= w || y >= h)
		return;
	data[y*w + x] = rgb;
}

//---------------------------------------------------------------------

RGBA VectorImage::sample(double x, double y) const {
	// XXX clamp instead?
	RGBA r = INVALID;
	if(x >= 0 && y >= 0 && x + 1 < w && y + 1 < h) {
		int ix = x, iy = y;
		double dx = x - ix, dy = y - iy;

		r.r = r.g = r.b = 0.0;

		RGBA temp = data[ix + iy*w];
		temp *= (1 - dx)*(1 - dy);
		r += temp;

		temp = data[ix + (iy + 1)*w];
		temp *= (1 - dx)*dy;
		r += temp;

		temp = data[ix + 1 + iy*w];
		temp *= dx*(1 - dy);
		r += temp;

		temp = data[ix + 1 + (iy + 1)*w];
		temp *= dx*dy;
		r += temp;
	}
	return r;
}

//---------------------------------------------------------------------
