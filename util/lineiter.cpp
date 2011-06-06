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
#include "lineiter.hpp"

#if 1
//
// Cohen-Sutherland line clipping
//
// Source: http://en.wikipedia.org/wiki/Cohen-Sutherland
//
const int INSIDE = 0; // 0000
const int LEFT = 1;   // 0001
const int RIGHT = 2;  // 0010
const int BOTTOM = 4; // 0100
const int TOP = 8;    // 1000

int ComputeOutCode(int x, int y, int w, int h) {
	int code = INSIDE;
	if(x < 0) code |= LEFT;
	else if(x > w) code |= RIGHT;
	if(y < 0) code |= BOTTOM;
	else if(y > h) code |= TOP;
	return code;
}

bool clipLine(int &x0, int &y0, int &x1, int &y1, int w, int h) {
	w--;
	h--;

	int outcode0 = ComputeOutCode(x0, y0, w, h);
	int outcode1 = ComputeOutCode(x1, y1, w, h);

	bool accept = false;
	while(true) {
		if(!(outcode0 | outcode1)) {
			accept = true;
			break;
		} else if(outcode0 & outcode1) {
			break;
		} else {
			int x = 0, y = 0;
			int outcodeOut = outcode0 ? outcode0 : outcode1;
			if(outcodeOut & TOP) {
				x = x0 + ((x1 - x0) * (h - y0)) / (y1 - y0);
				y = h;
			} else if(outcodeOut & BOTTOM) {
				x = x0 + ((x1 - x0) * (0 - y0)) / (y1 - y0);
				y = 0;
			} else if(outcodeOut & RIGHT) {
				y = y0 + ((y1 - y0) * (w - x0)) / (x1 - x0);
				x = w;
			} else if(outcodeOut & LEFT) {
				y = y0 + ((y1 - y0) * (0 - x0)) / (x1 - x0);
				x = 0;
			}

			if(outcodeOut == outcode0) {
				x0 = x;
				y0 = y;
				outcode0 = ComputeOutCode(x0, y0, w, h);
			} else {
				x1 = x;
				y1 = y;
				outcode1 = ComputeOutCode(x1, y1, w, h);
			}
		}
	}

	return accept;
}
#else
//---------------------------------------------------------------------
// Liang-Barsky line clipping
//
// Source: http://www.skytopia.com/project/articles/compsci/clipping.html
//
bool clipLine(int &x1, int &y1, int &x2, int &y2, int w, int h) {
	y1 = h - y1;
	y2 = h - y2;

    double xdelta = x2 - x1;
    double ydelta = y2 - y1;
    double p = 0.0, q = 0.0, r = 0.0;

    double t0 = 0.0, t1 = 1.0;
    for(int edge = 0; edge < 4; edge++) {
        if(edge == 0) { p = -xdelta; q = x1; }
        if(edge == 1) { p =  xdelta; q = (w - 1) - x1; }
        if(edge == 2) { p = -ydelta; q = y1; }
        if(edge == 3) { p =  ydelta; q = (h - 1) - y1; }
        r = q / p;

        if(p == 0 && q < 0)
			return false; // Don't draw line at all. (parallel line outside)

        if(p < 0) {
            if(r > t1) return false;  // Don't draw line at all.
            else if(r > t0) t0 = r;   // Line is clipped!
        } else if(p > 0) {
            if(r < t0) return false;  // Don't draw line at all.
            else if(r < t1) t1 = r;   // Line is clipped!
        }
    }

    x1 += t0*xdelta;
    y1 += t0*ydelta;
    x2 += t1*xdelta;
    y2 += t1*ydelta;

	y1 = h - y1;
	y2 = h - y2;

    return true;        // (clipped) line is drawn
}
#endif
//---------------------------------------------------------------------
