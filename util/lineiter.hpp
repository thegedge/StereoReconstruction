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
#ifndef LINEITER_HPP
#define LINEITER_HPP


//! .
bool clipLine(int &x1, int &y1, int &x2, int &y2, int w, int h);

/*!
 * A class to iterate through the points on a line using Bresenham's
 * line drawing algorithm.
 */
class LineIterator {
public:
	LineIterator(int x0, int y0, int x1, int y1)
		: x0(0), y0(0), x1(0), y1(0), x(1), y(1)
		, error(0), ystep(0), deltax(0), deltay(0), steep(false)
	{
		initialize(x0, y0, x1, y1);
		this->x0 = x0;
		this->y0 = y0;
		this->x1 = x1;
		this->y1 = y1;
		reset();
	}

	LineIterator(int x0, int y0, int x1, int y1, int w, int h)
		: x0(0), y0(0), x1(0), y1(0), x(1), y(1)
		, error(0), ystep(0), deltax(0), deltay(0), steep(false)
	{
		if(clipLine(x0, y0, x1, y1, w, h)) {
			initialize(x0, y0, x1, y1);
			this->x0 = x0;
			this->y0 = y0;
			this->x1 = x1;
			this->y1 = y1;
			reset();
		} else {
			x1 = 0;
			x = 1;
		}
	}

	bool hasNext() const {
		return (x <= x1);
	}

	void current(int &x, int &y) const {
		if(steep) {
			x = this->y;
			y = this->x;
		} else {
			x = this->x;
			y = this->y;
		}
	}

	void next() {
		++x;
		error -= deltay;
		if(error < 0) {
			y += ystep;
			error += deltax;
		}
	}

	void reset() {
		error = deltax / 2;
		x = x0;
		y = y0;
	}

	void operator++()    { next(); }
	void operator++(int) { next(); }

private:
	void initialize(int &x0, int &y0, int &x1, int &y1) {
		steep = std::abs(y1 - y0) > std::abs(x1 - x0);
		if(steep) {
			std::swap(x0, y0);
			std::swap(x1, y1);
		}

		if(x0 > x1) {
			std::swap(x0, x1);
			std::swap(y0, y1);
		}

		deltax = x1 - x0;
		deltay = std::abs(y1 - y0);
		ystep = (y0 < y1 ? 1 : -1);
	}

public://private:
	int x0, y0, x1, y1;
	int x, y;
	int error, ystep, deltax, deltay;
	bool steep;
};


#endif // LINEITER_HPP
