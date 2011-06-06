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
#ifndef RAY_HPP
#define RAY_HPP

#include <Eigen/Core>

#include "plane.hpp"

//! Three-dimensional ray, using double precision.
/*!
 *
 */
class Ray3d {
public:
	typedef Eigen::Vector3d Point;
	typedef Eigen::Vector3d Vector;

public:
	Ray3d();
	Ray3d(const Point &source, const Vector &dir);

public:
	void setSource(const Point &p)     { source_ = p; }
	void setDirection(const Vector &v) { dir_ = v; dir_.normalize(); }

	const Point & source()     const { return source_; }
	const Vector & direction() const { return dir_; }

	//! Obtain the point at distance \a dist from the source.
	Point point(double dist)   const { return (source_ + dist*dir_); }

public:
	/*!
	 * Finds the point on this ray which is closest to \a ray and stores
	 * the result in \a p. If the point is not on the ray (i.e. it is on
	 * the extension of the ray to a 3D line) then \a p is set to the source.
	 */
	Point closestPoint(const Ray3d &ray) const;

	/*!
	 * Finds the distance between this ray and another.
	 *
	 * \returns \c false if the distance is based on points that are on the
	 *          extension of the ray and not the ray itself
	 *          (\see closestPoint()), returns \a true otherwise.
	 */
	double distance(const Ray3d &ray) const;

	//! Get the points of "closest approach" on this ray and another.
	void closestPoints(const Ray3d &ray, Point &p1, Point &p2) const;

private:
	Point source_;
	Vector dir_;
};

//---------------------------------------------------------------------

/*!
 * Find the intersection between ray \a R and plane \a P and store the
 * result in \a p.
 *
 * \returns \c true if \a R and \a P intersect, \c false otherwise
 */
bool intersect(const Ray3d &R, const Plane3d &P, Ray3d::Point &p);

/*!
 * Given ray \a R, find the refracted ray obtained after \a R hits plane
 * \a P under the refractive ratio \a n.
 *
 * \note    \a Rout can be \a R without any side effects
 * \returns \c true if a refracted ray exists, \c false otherwise
 *          (e.g. ray is parallel to interface, total internal reflection).
 */
bool refract(const Ray3d &R, const Plane3d &P, double n, Ray3d &Rout);

/*!
 * Given two rays, find the corresponding midpoint (i.e. the point that
 * minimizes the sum of the distances between itself and each ray).
 */
Ray3d::Point midpoint(const Ray3d &R1, const Ray3d &R2);

//---------------------------------------------------------------------

#endif // RAY_HPP
