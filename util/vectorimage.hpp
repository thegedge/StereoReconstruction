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
#ifndef VECTORIMAGE_HPP
#define VECTORIMAGE_HPP

#include <cmath>
#include <string>
#include <vector>
#include <QImage>
#include "util/c++0x.hpp"

//! RGBA quad
struct RGBA {
	double r, g, b, a;
	RGBA() : r(0), g(0), b(0), a(255) { }
	RGBA(double c) : r(c), g(c), b(c), a(255) { }
	RGBA(double r, double g, double b) : r(r), g(g), b(b), a(255) { }
	RGBA(double r, double g, double b, double a) : r(r), g(g), b(b), a(a) { }

	RGBA& operator+=(const RGBA &o) {
		r += o.r;
		g += o.g;
		b += o.b;
		return *this;
	}

	RGBA& operator-=(const RGBA &o) {
		r -= o.r;
		g -= o.g;
		b -= o.b;
		return *this;
	}

	RGBA& operator*=(double s) {
		r *= s;
		g *= s;
		b *= s;
		return *this;
	}

	bool isValid() const {
		return !(std::isnan(r) || std::isnan(g) || std::isnan(b));
	}

	double toGray() const {
		return (0.11*r + 0.59*g + 0.3*b);
	}

	bool operator==(const RGBA &o) const {
		return (fabs(r - o.r) < 1e-10
				&& fabs(g - o.g) < 1e-10
				&& fabs(b - o.b) < 1e-10
				&& fabs(a - o.a) < 1e-10);
	}

	bool operator!=(const RGBA &o) const {
		return !operator==(o);
	}
};

extern const RGBA RED;
extern const RGBA GREEN;
extern const RGBA BLUE;
extern const RGBA YELLOW;
extern const RGBA MAGENTA;
extern const RGBA BLACK;
extern const RGBA WHITE;
extern const RGBA INVALID;

//! A simple image class that uses a 1D vector to store pixels
class VectorImage {
public:
	//! .
	static VectorImage fromFile(const std::string &file);

	//! .
	static VectorImage fromQImage(QImage img);

	//! .
	static QImage toQImage(const VectorImage &img, bool includeAlpha = true);

public:
	VectorImage();
	VectorImage(int w, int h, const RGBA &fillVal = WHITE);

	bool isNull() const { return (w <= 0 || h <= 0 || data.size() == 0); }
	int width()   const { return w; }
	int height()  const { return h; }

	//! .
	VectorImage & fill(const RGBA &rgb);

	//! .
	void setPixel(int x, int y, const RGBA &rgb);

	//! .
	const RGBA & pixel(int x, int y) const;

	//! .
	RGBA sample(double x, double y) const;

private:
	//! .
	VectorImage(int w, int h, std::vector<RGBA>& data);

private:
	int w, h;
	std::vector<RGBA> data;
};

#endif // VECTORIMAGE_HPP
