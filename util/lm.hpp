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
#ifndef __LEVENBERGMARQUARDT_H__
#define __LEVENBERGMARQUARDT_H__

#include <Eigen/Core>
#include <utility>
#include <vector>


/*!
 * A class to perform Levenberg-Marquardt optimization of a function.
 */
class LevenbergMarquardt {
public:
	//! Type used for points in the function that will be optimized
	typedef Eigen::VectorXd Point;

	//! Type used for points in the function that will be optimized
	typedef std::pair<Point, Point> PointPair;

	//! Type used to contain a list of points
	typedef std::vector<PointPair> Points;

	//! Type used for the model parameters of the function being optimized
	typedef Point Model;

	//! Type used for specifying fixed parameters of the function being optimized
	typedef std::vector<bool> FixedParams;

	/*!
	 * Represents a function to optimize. A concrete implementation of this
	 * interface should store its own version of the model being optimized
	 * along with the data points.
	 */
	class Function {
	public:
		virtual ~Function() { }

		//! Allows the function to initialize itself before optimization
		virtual void initialize() { }

		//! Should return the value of <tt>f(p.first; p.second, model)</tt>
		virtual Point diff(const PointPair &p, int point_index, const Model &model) = 0;

		//! Calculate the gradient of a parameter at the given point.
		virtual Point gradient(const PointPair &p, int point_index, const Model &model, int paramIndex) = 0;

		/*!
		 * Allows the function to update internals when the model changes. If
		 * the new model parameters are invalid (e.g., due to some domain
		 * restriction) then false should be returned to allow the
		 * optimizer to reset the model and try again.
		 */
		virtual bool update(const Model &) { return true; }
	};

public:
	//! .
	LevenbergMarquardt(int maxIterations = 1000, double epsilon = 1e-10);

public:
	//! .
	void optimize(Function &f, const Points &point, Model &model);

	//! .
	void optimize(Function &f, const Points &point, Model &model, const FixedParams &fixed);

	//! .
	static double chiSquared(Function &f, const Points &pts, const Model &model);

private:
	//! Max number of iterations allowed before stopping
	int maxIterations;

	//! Termination epsilon
	double epsilon;
};

#endif
