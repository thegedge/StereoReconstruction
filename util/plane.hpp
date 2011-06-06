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
#ifndef PLANE_HPP
#define PLANE_HPP

#include <Eigen/Core>

//! Three-dimensional plane, using double precision.
/*!
 *
 */
class Plane3d {
public:
	typedef Eigen::Vector3d Point;
	typedef Eigen::Vector3d Vector;

public:
	Plane3d() : normal_(0, 0, 1), x0_(0, 0, 0) { }
	Plane3d(const Vector &normal, const Point &x0) : normal_(normal.normalized()), x0_(x0) { }
	Plane3d(const Vector &normal, double d) : normal_(normal.normalized()), x0_(d*normal_) { }

public:
	void setNormal(const Vector &normal) { normal_ = normal; normal_.normalize(); }
	void setX0(const Point &x0)          { x0_ = x0; }

	const Vector & normal() const { return normal_; }
	const Point & x0()      const { return x0_; }

private:
	Vector normal_;
	Point x0_;
};

#endif // PLANE_HPP
