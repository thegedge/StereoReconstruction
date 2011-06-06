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
//
// TODO
// - Completely remove fixed parameters from matrices to use less memory
//   and have a smaller problem to solve
//
#include "lm.hpp"

#include <cmath>
#include <Eigen/LU>
#include <limits>
#include "util/c++0x.hpp"

#ifdef USE_TBB
#   include <tbb/tbb.h>
#endif

//---------------------------------------------------------------------------

using namespace Eigen;
using std::sqrt;
using std::fabs;

//---------------------------------------------------------------------------

LevenbergMarquardt::LevenbergMarquardt(int maxIterations, double epsilon)
	: maxIterations(maxIterations)
	, epsilon(epsilon)
{ }

//---------------------------------------------------------------------------

double LevenbergMarquardt::chiSquared(Function &f, const Points &pts, const Model &model) {
	double sum = 0.0;
	for(size_t pointIndex = 0; pointIndex < pts.size(); ++pointIndex)
		sum += f.diff(pts[pointIndex], pointIndex, model).squaredNorm();
	return sum;
}

//---------------------------------------------------------------------------

void LevenbergMarquardt::optimize(Function &f, const Points &pts, Model &model) {
	FixedParams fixed(model.size(), false);
	optimize(f, pts, model, fixed);
}

void LevenbergMarquardt::optimize(Function &f, const Points &pts, Model &model, const FixedParams &fixed) {
	int npts = pts.size();
	int nparms = model.size();

	assert(nparms > 0 && npts > 0);

	f.initialize();
	f.update(model);

	double e0 = chiSquared(f, pts, model);
	double lambda = 1;

	MatrixXd H;
	VectorXd g;

	int iter = 0;
	int term = 0;	// termination count test
	do {
		// Hessian (H = J^t * J) and (g = J^t * f) approximation
		g.setZero(nparms);
		H.setZero(nparms, nparms);

		for(int i = 0; i < npts; ++i) {
			Point diff = f.diff(pts[i], i, model);

			for(int r = 0; r < nparms; ++r) if(!fixed[r]) {
				Point gradr = f.gradient(pts[i], i, model, r);

				for(int c = 0; c < nparms; ++c) if(!fixed[c]) {
					Point gradc = f.gradient(pts[i], i, model, c);
					H(r, c) += gradr.dot(gradc);
				}

				g[r] += diff.dot(gradr);
			}
		}

		// boost diagonal towards gradient descent
		for(int p = 0; p < nparms; ++p)
			H(p, p) *= 1.0 + lambda;

		// Solve H d = -g
		//Model new_model = H.lu().solve(-g);
		//if( (H*new_model).isApprox(-g, 1e-10) ) {
		Model new_model;
		if(!H.lu().solve(-g, &new_model)) {
			++term;
			continue;
		}

		//
		bool bad_model = false;
		for(int p = 0; p < nparms; p++) {
			if(std::isnan(model[p])) { // NaN check
				bad_model = true;
				lambda *= 10.0;
				++term;
			}
		}

		if(bad_model) continue;

		// Iterative update of model
		new_model += model;
		if(!f.update(new_model)) {
			f.update(model); // reset to old model
			lambda *= 10.0;
			++term;
			continue;
		}

		// Evaluate error at new location and perform a termination test.
		// If error hasn't changed any more than the epsilon specified upon
		// constructing this instance of LevenbergMarquardt, and this has
		// happened N times in a row, end.
		double e1 = chiSquared(f, pts, new_model);
		if(fabs(e1 - e0) > epsilon) {
			term = 0;
		} else {
			++term;
		}

		// Figure out how to update lambda as per LM algorithm
		bool worse = (e0 - e1 < 0);
		if(worse || std::isnan(e1)) { // new location no better than before
			lambda *= 10.0;
			f.update(model); // reset to old model
		} else { // new location better, accept new parameters
			lambda *= 0.1;
			e0 = e1;
			model = new_model;
		}
	} while(++iter < maxIterations && term < 5);
}

//---------------------------------------------------------------------------
