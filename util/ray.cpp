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
#include "ray.hpp"

//---------------------------------------------------------------------

Ray3d::Ray3d()
 : source_(0, 0, 0)
 , dir_(0, 0, 1)
{ }

Ray3d::Ray3d(const Point &source, const Vector &dir)
  : source_(source)
  , dir_(dir.normalized())
{ }

//---------------------------------------------------------------------

Ray3d::Point Ray3d::closestPoint(const Ray3d &ray) const {
	Point p, scrap;
	closestPoints(ray, p, scrap);
	return p;
}

//---------------------------------------------------------------------

double Ray3d::distance(const Ray3d &ray) const {
	Point a, b;
	closestPoints(ray, a, b);
	return (a - b).norm();
}

//---------------------------------------------------------------------

void Ray3d::closestPoints(const Ray3d &ray, Point &p1, Point &p2) const {
	Point w0 = source() - ray.source();

	//
	double a = direction().dot(direction());
	double b = direction().dot(ray.direction());
	double c = ray.direction().dot(ray.direction());
	double d = direction().dot(w0);
	double e = ray.direction().dot(w0);

	double den = 1.0 / (a*c - b*b);
	double tl = (b*e - c*d) * den;
	double tr = (a*e - b*d) * den;

	// Check to see if the points lie on the ray or its extension
	p1 = source();
	p2 = ray.source();

	//
	if(tl > 0) p1 += tl*direction();
	if(tr > 0) p2 += tr*ray.direction();
}

//---------------------------------------------------------------------

bool intersect(const Ray3d &R, const Plane3d &P, Ray3d::Point &p) {
	double nd = P.normal().dot(R.direction());
	if(fabs(nd) < 1e-10) return false;

	double t = P.normal().dot(P.x0() - R.source()) / nd;
	if(t < 1e-10)
		return false;

	p = R.point(t);
	return true;
}

//---------------------------------------------------------------------

bool refract(const Ray3d &R, const Plane3d &P, double n, Ray3d &Rout) {
	Ray3d::Point p;
	if(intersect(R, P, p)) {
		double cosI = -(P.normal().dot(R.direction()));
		double cosT2 = 1.0 - (1.0 - cosI*cosI) / (n*n);
		if(cosT2 > 0.0) {
			double sign = (cosI > 0.0 ? -1.0 : 1.0);
			Rout.setSource(p);
			Rout.setDirection(R.direction() + (cosI + n*sign*sqrt(cosT2))*P.normal());
			return true;
		}
	}

	return false;
}

//---------------------------------------------------------------------

Ray3d::Point midpoint(const Ray3d &R1, const Ray3d &R2) {
	Ray3d::Point a, b;	
	R1.closestPoints(R2, a, b);
	return (a + b) / 2;
}

//---------------------------------------------------------------------
