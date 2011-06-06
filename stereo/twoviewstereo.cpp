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
#include "twoviewstereo.hpp"

#include "geodesicweight.hpp"
#include "adaptiveweight.hpp"
#include "project/project.hpp"
#include "project/camera.hpp"
#include "project/imageset.hpp"
#include "project/projectimage.hpp"
#include "util/lineiter.hpp"

#include <algorithm>
#include <cmath>
#include <fstream>

#include <QColor>
#include <QDebug>
#include <QTime>

#undef USE_MRF
#ifdef USE_MRF
#   include <MRF/GCoptimization.h>
#endif

#ifdef USE_OPENMP
#   include <omp.h>
#else
#   define omp_get_num_procs() 1
#   define omp_get_thread_num() 0
#endif

//---------------------------------------------------------------------

using std::max;
using std::min;
using std::exp;

#define PV(x, y, img)        (static_cast<int>(y)*img.width() + static_cast<int>(x))
#define MAP(x, y, img)       pixelToMRFIndexMap[PV(x, y, img)]
#define CONTAINS(img, x, y)  (x >= 0 && y >= 0 && x < img.width() && y < img.height())

namespace {
	const double NaN = std::numeric_limits<double>::quiet_NaN();
	const double INF = std::numeric_limits<double>::infinity();
}
//---------------------------------------------------------------------
// TODO allow passing these values to constructor
//
namespace {
	const int BAD_RET = 1000;
	const int WINDOW_RADIUS = 5;
	const int WINDOW_SIZE = 2*WINDOW_RADIUS + 1;

	const int    SMOOTHNESS_EXP = 1;
	const double SMOOTHNESS_MAX = 2;
	const double SMOOTHNESS_LAMBDA = 0.25;

	const int    GAP_WIDTH_THRESHOLD = 2;//100*imageScale;
	const double MAX_COLOR_DIFF = 120;
	//const double INCONSISTENCY_THRESH = 0.001;
	//const double SECOND_BEST_FACTOR = 1.0;
	//const double INCONSISTENCY_THRESH = 0.1;
	const double INCONSISTENCY_THRESH = 1;
	const double SECOND_BEST_FACTOR = 0.95;
}
//---------------------------------------------------------------------

const int NUM_WEIGHT_FUNCS = 16;
typedef GeodesicWeight WeightFunc;
static WeightFunc weightFuncs[NUM_WEIGHT_FUNCS];

//---------------------------------------------------------------------

TwoViewStereo::TwoViewStereo(
		CameraPtr leftView, QImage left, QImage leftMask,
		CameraPtr rightView, QImage right, QImage rightMask,
		double minDepth, double maxDepth,
		int numDepthLevels,
		double imageScale)
	: leftView(leftView)
	, rightView(rightView)
	, left( VectorImage::fromQImage(left.scaledToWidth(left.width() * imageScale, Qt::SmoothTransformation)) )
	, right( VectorImage::fromQImage(right.scaledToWidth(right.width() * imageScale, Qt::SmoothTransformation)) )
	, minDepth(minDepth)
	, maxDepth(maxDepth)
	, numDepthLevels(numDepthLevels)
	, imageScale(imageScale)
{
	// Images
	if(!leftMask.isNull())
		this->leftMask = VectorImage::fromQImage(leftMask.scaledToWidth(leftMask.width() * imageScale, Qt::SmoothTransformation));
	else
		this->leftMask = VectorImage(this->left.width(), this->left.height(), WHITE);

	if(!rightMask.isNull())
		this->rightMask = VectorImage::fromQImage(rightMask.scaledToWidth(rightMask.width() * imageScale, Qt::SmoothTransformation));
	else
		this->rightMask = VectorImage(this->right.width(), this->right.height(), WHITE);

	// Results
	resultLeft = VectorImage(this->left.width(), this->left.height());
	resultRight = VectorImage(this->right.width(), this->right.height());
	computedDepthLeft = DepthMap(this->left.width()*this->left.height(), NaN);
	computedDepthRight = DepthMap(this->left.width()*this->left.height(), NaN);

	// Weight funcs
	for(int i = 0; i < NUM_WEIGHT_FUNCS; ++i)
		weightFuncs[i].initialize(WINDOW_RADIUS);
}

//---------------------------------------------------------------------

RGBA TwoViewStereo::colorFromDepth(double depth) const {
	if(std::isnan(depth))
		return BLACK;

	if(std::isinf(depth))
		return BLACK;//RGB(125, 0, 125);

	// For grayscale depth maps
	//const double t = min(1.0, max(0.0, (depth - minDepth) / (maxDepth - minDepth)));
	//return RGB(255*(1 - t));

	// For colorfied depth maps (close = warm colors, far = cool colors)
	const double t = (depth - minDepth) / (maxDepth - minDepth);
	if(t < 1e-5) return BLACK;
	if(t > 1.1) return WHITE;

	QColor c = QColor::fromHsvF(2.0 * t / 3.0, 1.0, 1.0);
	return RGBA(c.red(), c.green(), c.blue());
}

//---------------------------------------------------------------------

void TwoViewStereo::computeDepthMaps() {
	//
	resultLeft.fill(INVALID);
	resultRight.fill(INVALID);

	//
	computeCostVolumes(leftView, rightView);
	if(isCancelled()) return;
	{
		for(int y = 0; y < resultLeft.height(); ++y) {
			if(isCancelled()) return;
			for(int x = 0; x < resultLeft.width(); ++x) {
				double d = computedDepthLeft[ PV(x, y, resultLeft) ];
				resultLeft.setPixel(x, y, colorFromDepth(d));
			}
		}

		for(int y = 0; y < resultRight.height(); ++y) {
			if(isCancelled()) return;
			for(int x = 0; x < resultRight.width(); ++x) {
				double d = computedDepthRight[ PV(x, y, resultRight) ];
				resultRight.setPixel(x, y, colorFromDepth(d));
			}
		}

		// TODO maintain results before cross-checking
	}
#if 1
	crossCheck(leftView, rightView);
	if(isCancelled()) return;
	{
		for(int y = 0; y < resultLeft.height(); ++y) {
			if(isCancelled()) return;
			for(int x = 0; x < resultLeft.width(); ++x) {
				double d = computedDepthLeft[ PV(x, y, resultLeft) ];
				resultLeft.setPixel(x, y, colorFromDepth(d));
			}
		}

		for(int y = 0; y < resultRight.height(); ++y) {
			if(isCancelled()) return;
			for(int x = 0; x < resultRight.width(); ++x) {
				double d = computedDepthRight[ PV(x, y, resultRight) ];
				resultRight.setPixel(x, y, colorFromDepth(d));
			}
		}

		// TODO maintain results after cross-checking
	}
#endif
#if 0
	filterInvalidPixels();
	if(isCancelled()) return;
	{
		//
		for(int y = 0; y < resultLeft.height(); ++y) {
			if(isCancelled()) return;
			for(int x = 0; x < resultLeft.width(); ++x) {
				double d = computedDepthLeft[ PV(x, y, resultLeft) ];
				resultLeft.setPixel(x, y, colorFromDepth(d));
			}
		}

		for(int y = 0; y < resultRight.height(); ++y) {
			if(isCancelled()) return;
			for(int x = 0; x < resultRight.width(); ++x) {
				double d = computedDepthRight[ PV(x, y, resultRight) ];
				resultRight.setPixel(x, y, colorFromDepth(d));
			}
		}

		// TODO maintain results after filtering
	}
#endif

	emit progressUpdate(8);
	emit stageUpdate("Finished!");
}

//---------------------------------------------------------------------
// TODO There's a lot of duplicated code here when all we do is swap
//      left/right. Try to refactor this some more.
//
void TwoViewStereo::computeCostVolumes(CameraPtr leftView, CameraPtr rightView) {
	emit progressUpdate(1);
	emit stageUpdate("Computing cost volume for left image...");
	{
		Ray3d::Vector depthPlaneNormal = leftView->principleRay().direction();

		//
#ifdef USE_MRF
		if(leftMask.isNull()) {
			costVolume.resize(left.width() * left.height() * numDepthLevels);
		} else {
			pixelToMRFIndexMap.resize(left.width() * left.height());

			int count = 0;
			for(int y = 0; y < left.height(); ++y) {
				for(int x = 0; x < left.width(); ++x) {
					MAP(x, y, left) = -1;
					if(leftMask.pixel(x, y) != BLACK)
						MAP(x, y, left) = count++;
				}
			}

			costVolume.resize(count * numDepthLevels);
		}

		std::fill(costVolume.begin(), costVolume.end(), WINDOW_SIZE*BAD_RET);
#endif
		QTime time = QTime::currentTime();

		const Ray3d::Point cameraC = leftView->C();

		#pragma omp parallel for
		for(int y = 0; y < left.height(); ++y) {
			if(isCancelled()) continue;
			for(int x = 0; x < left.width(); ++x) {
				computedDepthLeft[ PV(x, y, resultLeft) ] = NaN;
				if(leftMask.pixel(x, y) != WHITE)
					continue;

				weightFuncs[omp_get_thread_num()].init_weights(left, x, y);

				Ray3d ray = leftView->unproject((x + 0.5) / imageScale, (y + 0.5) / imageScale);

#ifdef USE_MRF
				int p = MAP(x, y, left)*numDepthLevels;
#else
				double secondBestCost = INF;
				double minCost = INF;
#endif
#ifndef USE_MRF

				std::vector<Eigen::Vector3d> curve = epipolarCurve(ray, cameraC, depthPlaneNormal, rightMask, rightView);
				foreach(const Eigen::Vector3d &p, curve) {
					Ray3d ray2 = rightView->unproject((p[0] + 0.5) / imageScale, (p[1] + 0.5) / imageScale);

					Eigen::Vector3d p1, p2;
					ray.closestPoints(ray2, p1, p2);

					const double cost = cost_ncc(left, right, leftMask, rightMask, x, y, p[0], p[1]);
					if(cost + 1e-10 < minCost) {
						p1 += p2;
						p1 *= 0.5;
						p1 = leftView->fromGlobalToLocal(p1);

						secondBestCost = minCost;
						minCost = cost;
						computedDepthLeft[ PV(x, y, resultLeft) ] = p1.z();
					}
				}

				if(minCost > SECOND_BEST_FACTOR*secondBestCost)
					computedDepthLeft[ PV(x, y, resultLeft) ] = INF;

#else
				for(int d = 0; d < numDepthLevels; ++d) {
					Ray3d::Point point = cameraC;

					const double depth = depthFromLabel(d);
					if(pointFromDepth(ray, depthPlaneNormal, depth, point)) {
						if(rightView->project(point)) {
							double x2 = point[0]*imageScale - 0.5;
							double y2 = point[1]*imageScale - 0.5;

#ifdef USE_MRF
							costVolume[p++] = cost_ncc(left, leftMask, right, rightMask, x, y, x2, y2);
#else
							double cost = cost_ncc(left, right, leftMask, rightMask, x, y, x2, y2);
							if(cost + 1e-10 < minCost) {
								secondBestCost = minCost;
								minCost = cost;
								computedDepthLeft[ PV(x, y, resultLeft) ] = depth;
							}
#endif
						}
					}
				}
#endif
			}
		}
		qDebug() << '\t' << time.elapsed() / 1000.0 << "seconds";
	}
#ifdef USE_MRF
	emit progressUpdate(2);
	emit stageUpdate("Optimizing...");
	{
		std::unique_ptr<DataCost> dc(new DataCost(&costVolume[0]));
		std::unique_ptr<SmoothnessCost> sc(new SmoothnessCost(SMOOTHNESS_EXP, SMOOTHNESS_MAX, SMOOTHNESS_LAMBDA));
		std::unique_ptr<EnergyFunction> f(new EnergyFunction(dc.get(), sc.get()));
		std::unique_ptr<MRF> mrf;

		// If there's a mask, use a neighbour system where neighbours consist of
		// any of the eight neighbours of a pixel who has a nonzero mask value
		if(leftMask.isNull()) {
			mrf.reset(new Expansion(left.width(), left.height(), numDepthLevels, f.get()));
		} else {
			// XXX there are less than left.width() * left.height() useful pixels
			mrf.reset(new Expansion(costVolume.size() / numDepthLevels, numDepthLevels, f.get()));
			for(int y = 0; y < left.height(); ++y) {
				if(isCancelled()) return;
				for(int x = 0; x < left.width(); ++x) {
					if(leftMask.pixel(x, y) != WHITE)
						continue;

					int curr = MAP(x, y, left);

					if(leftMask.pixel(x - 1, y - 1) != BLACK) mrf->setNeighbors(curr, MAP(x - 1, y - 1, left), 1.4);
					if(leftMask.pixel(x    , y - 1) != BLACK) mrf->setNeighbors(curr, MAP(x    , y - 1, left), 1.0);
					if(leftMask.pixel(x + 1, y - 1) != BLACK) mrf->setNeighbors(curr, MAP(x + 1, y - 1, left), 1.4);
					if(leftMask.pixel(x - 1, y    ) != BLACK) mrf->setNeighbors(curr, MAP(x - 1, y    , left), 1.0);
					if(leftMask.pixel(x + 1, y    ) != BLACK) mrf->setNeighbors(curr, MAP(x + 1, y    , left), 1.0);
					if(leftMask.pixel(x - 1, y + 1) != BLACK) mrf->setNeighbors(curr, MAP(x - 1, y + 1, left), 1.4);
					if(leftMask.pixel(x    , y + 1) != BLACK) mrf->setNeighbors(curr, MAP(x    , y + 1, left), 1.0);
					if(leftMask.pixel(x + 1, y + 1) != BLACK) mrf->setNeighbors(curr, MAP(x + 1, y + 1, left), 1.4);
				}
			}
		}

		//
		mrf->initialize();
		mrf->clearAnswer();

		qDebug() << "\tTotal Energy =" << mrf->totalEnergy() << '('
				 << mrf->dataEnergy() << '+' << mrf->smoothnessEnergy() << ')';

		float time;
		double energy = mrf->totalEnergy();
		double prevEnergy = 0.0;
		int numIters = 50;
		do {
			if(isCancelled()) return;

			prevEnergy = energy;
			mrf->optimize(1, time);
			energy = mrf->totalEnergy();
			qDebug() << "\tTotal Energy =" << energy << '('
					 << mrf->dataEnergy() << '+' << mrf->smoothnessEnergy() << ')';
		} while(prevEnergy - energy > 5 && numIters-- > 0);

		// Store integer disparities for future use
		for(int y = 0; y < left.height(); ++y) {
			if(isCancelled()) return;
			for(int x = 0; x < left.width(); ++x) {
				if(leftMask.pixel(x, y) != BLACK)
					resultLeft.setPixel(x, y, mrf->getLabel(MAP(x, y, left)));
				else if(leftMask.pixel(x, y) != WHITE)
					resultLeft.setPixel(x, y, numDepthLevels);
			}
		}
	}
#endif

	emit progressUpdate(3);
	emit stageUpdate("Computing cost volume for right image...");
	{
		Ray3d::Vector depthPlaneNormal = rightView->principleRay().direction();

#ifdef USE_MRF
		if(rightMask.isNull()) {
			costVolume.resize(right.width() * right.height() * numDepthLevels);
		} else {
			pixelToMRFIndexMap.resize(right.width() * right.height());

			int count = 0;
			for(int y = 0; y < right.height(); ++y) {
				if(isCancelled()) return;
				for(int x = 0; x < right.width(); ++x) {
					MAP(x, y, right) = -1;
					if(rightMask.pixel(x, y) != BLACK)
						MAP(x, y, right) = count++;
				}
			}

			costVolume.resize(count * numDepthLevels);
		}

		std::fill(costVolume.begin(), costVolume.end(), WINDOW_SIZE*BAD_RET);
#endif
		QTime time = QTime::currentTime();

		const Ray3d::Point &cameraC = rightView->C();

		#pragma omp parallel for
		for(int y = 0; y < right.height(); ++y) {
			if(isCancelled()) continue;
			for(int x = 0; x < right.width(); ++x) {
				computedDepthRight[ PV(x, y, resultRight) ] = NaN;
				if(rightMask.pixel(x, y) != WHITE)
					continue;

				weightFuncs[omp_get_thread_num()].init_weights(right, x, y);

				Ray3d ray = rightView->unproject((x + 0.5) / imageScale, (y + 0.5) / imageScale);
#ifdef USE_MRF
				int p = MAP(x, y, right)*numDepthLevels;
#else
				double secondBestCost = INF;
				double minCost = INF;
#endif
#ifndef USE_MRF

				std::vector<Eigen::Vector3d> curve = epipolarCurve(ray, cameraC, depthPlaneNormal, leftMask, leftView);
				foreach(const Eigen::Vector3d &p, curve) {
					Ray3d ray2 = leftView->unproject((p[0] + 0.5) / imageScale, (p[1] + 0.5) / imageScale);

					Eigen::Vector3d p1, p2;
					ray.closestPoints(ray2, p1, p2);

					const double cost = cost_ncc(right, left, rightMask, leftMask, x, y, p[0], p[1]);
					if(cost + 1e-10 < minCost) {
						p1 += p2;
						p1 *= 0.5;
						p1 = rightView->fromGlobalToLocal(p1);

						secondBestCost = minCost;
						minCost = cost;
						computedDepthRight[ PV(x, y, resultRight) ] = p1.z();
					}
				}

				if(minCost > SECOND_BEST_FACTOR*secondBestCost)
					computedDepthRight[ PV(x, y, resultRight) ] = INF;
#else
				for(int d = 0; d < numDepthLevels; ++d) {
					Ray3d::Point point = cameraC;

					const double depth = depthFromLabel(d);
					if(pointFromDepth(ray, depthPlaneNormal, depth, point)) {
						if(leftView->project(point)) {
							double x2 = point[0]*imageScale - 0.5;
							double y2 = point[1]*imageScale - 0.5;
#ifdef USE_MRF
							costVolume[p++] = cost_ncc(right, left, rightMask, leftMask, x, y, x2, y2);
#else
							double cost = cost_ncc(right, left, rightMask, leftMask, x, y, x2, y2);
							if(cost + 1e-10 < minCost) {
								secondBestCost = minCost;
								minCost = cost;
								computedDepthRight[ PV(x, y, resultRight) ] = depth;
							}
#endif
						}
					}
				}
#endif
			}
		}
		qDebug() << '\t' << time.elapsed() / 1000.0 << "seconds";
	}

#ifdef USE_MRF
	emit progressUpdate(4);
	emit stageUpdate("Optimizing...");
	{
		std::unique_ptr<DataCost> dc(new DataCost(&costVolume[0]));
		std::unique_ptr<SmoothnessCost> sc(new SmoothnessCost(SMOOTHNESS_EXP, SMOOTHNESS_MAX, SMOOTHNESS_LAMBDA));
		std::unique_ptr<EnergyFunction> f(new EnergyFunction(dc.get(), sc.get()));
		std::unique_ptr<MRF> mrf;

		// If there's a mask, use a neighbour system where neighbours consist of
		// any of the eight neighbours of a pixel who has a nonzero mask value
		if(rightMask.isNull()) {
			mrf.reset(new Expansion(left.width(), left.height(), numDepthLevels, f.get()));
		} else {
			mrf.reset(new Expansion(costVolume.size() / numDepthLevels, numDepthLevels, f.get()));

			for(int y = 0; y < left.height(); ++y) {
				if(isCancelled()) return;
				for(int x = 0; x < left.width(); ++x) {
					if(rightMask.pixel(x, y) != WHITE)
						continue;

					int curr = MAP(x, y, right);

					if(rightMask.pixel(x - 1, y - 1) != BLACK) mrf->setNeighbors(curr, MAP(x - 1, y - 1, right), 1.4);
					if(rightMask.pixel(x    , y - 1) != BLACK) mrf->setNeighbors(curr, MAP(x    , y - 1, right), 1.0);
					if(rightMask.pixel(x + 1, y - 1) != BLACK) mrf->setNeighbors(curr, MAP(x + 1, y - 1, right), 1.4);
					if(rightMask.pixel(x - 1, y    ) != BLACK) mrf->setNeighbors(curr, MAP(x - 1, y    , right), 1.0);
					if(rightMask.pixel(x + 1, y    ) != BLACK) mrf->setNeighbors(curr, MAP(x + 1, y    , right), 1.0);
					if(rightMask.pixel(x - 1, y + 1) != BLACK) mrf->setNeighbors(curr, MAP(x - 1, y + 1, right), 1.4);
					if(rightMask.pixel(x    , y + 1) != BLACK) mrf->setNeighbors(curr, MAP(x    , y + 1, right), 1.0);
					if(rightMask.pixel(x + 1, y + 1) != BLACK) mrf->setNeighbors(curr, MAP(x + 1, y + 1, right), 1.4);
				}
			}
		}

		//
		mrf->initialize();
		mrf->clearAnswer();

		qDebug() << "\tTotal Energy =" << mrf->totalEnergy() << '('
				 << mrf->dataEnergy() << '+' << mrf->smoothnessEnergy() << ')';

		float time;
		double energy = mrf->totalEnergy();
		double prevEnergy = 0.0;
		int numIters = 50;
		do {
			if(isCancelled()) return;

			prevEnergy = energy;
			mrf->optimize(1, time);
			energy = mrf->totalEnergy();
			qDebug() << "\tTotal Energy =" << energy << '('
					 << mrf->dataEnergy() << '+' << mrf->smoothnessEnergy() << ')';
		} while(prevEnergy - energy > 5 && numIters-- > 0);

		// Store integer disparities for future use
		for(int y = 0; y < right.height(); ++y) {
			for(int x = 0; x < right.width(); ++x) {
				if(rightMask.pixel(x, y) != BLACK)
					resultRight.setPixel(x, y, mrf->getLabel(MAP(x, y, right)));
				else
					resultRight.setPixel(x, y, numDepthLevels);
			}
		}
	}
#endif
#if 0
	//
	for(int y = 0; y < resultLeft.height(); ++y) {
		if(isCancelled()) return;
		for(int x = 0; x < resultLeft.width(); ++x) {
			double &d = computedDepthLeft[ PV(x, y, resultLeft) ];
			if(d < minDepth || d > maxDepth)
				d = INF;
		}
	}

	for(int y = 0; y < resultRight.height(); ++y) {
		if(isCancelled()) return;
		for(int x = 0; x < resultRight.width(); ++x) {
			double &d = computedDepthRight[ PV(x, y, resultRight) ];
			if(d < minDepth || d > maxDepth)
				d = INF;
		}
	}
#endif
}

//---------------------------------------------------------------------

void TwoViewStereo::crossCheck(CameraPtr leftView, CameraPtr rightView) {
	emit progressUpdate(5);
	emit stageUpdate("Detecting inconsistencies...");
	{
		Ray3d::Vector leftPlaneNormal = leftView->principleRay().direction();
		Ray3d::Vector rightPlaneNormal = rightView->principleRay().direction();

		// Invalidate pixels that fail to cross-check
		for(int y = 0; y < left.height(); ++y) {
			if(isCancelled()) return;
			for(int x = 0; x < left.width(); ++x) {
				//
				double &depth = computedDepthLeft[ PV(x, y, resultLeft) ];
				if(!std::isfinite(depth))
					continue;

				//
				Ray3d ray = leftView->unproject((x + 0.5) / imageScale, (y + 0.5) / imageScale);
				Ray3d::Point p1 = leftView->C();
				if(pointFromDepth(ray, leftPlaneNormal, depth, p1)) {
					double x2, y2;
					if(rightView->project(p1, x2, y2)) {
						x2 = x2*imageScale;
						y2 = y2*imageScale;

						if(CONTAINS(resultRight, x2, y2)) {
							double &odepth = computedDepthRight[ PV(x2, y2, resultRight) ];
							if(std::isfinite(odepth)) {
								Ray3d ray2 = rightView->unproject((x2 + 0.5) / imageScale, (y2 + 0.5) / imageScale);
								Ray3d::Point p2 = rightView->C();
								if(pointFromDepth(ray2, rightPlaneNormal, odepth, p2)) {
									const double norm = (p1 - p2).norm();
									if(!std::isfinite(norm) || norm > INCONSISTENCY_THRESH)
										depth = INF;
								} else depth = INF;
							} else depth = INF;
						} else depth = INF;
					} else depth = INF;
				}
			}
		}

		for(int y = 0; y < right.height(); ++y) {
			if(isCancelled()) return;
			for(int x = 0; x < right.width(); ++x) {
				//
				double &depth = computedDepthRight[ PV(x, y, resultRight) ];
				if(!std::isfinite(depth))
					continue;

				//
				Ray3d ray = rightView->unproject((x + 0.5) / imageScale, (y + 0.5) / imageScale);
				Ray3d::Point p1 = rightView->C();
				if(pointFromDepth(ray, rightPlaneNormal, depth, p1)) {
					double x2, y2;
					if(leftView->project(p1, x2, y2)) {
						x2 = x2*imageScale;
						y2 = y2*imageScale;

						if(CONTAINS(resultLeft, x2, y2)) {
							double &odepth = computedDepthLeft[ PV(x2, y2, resultLeft) ];
							if(std::isfinite(odepth)) {
								Ray3d ray2 = leftView->unproject((x2 + 0.5) / imageScale, (y2 + 0.5) / imageScale);
								Ray3d::Point p2 = leftView->C();
								if(pointFromDepth(ray2, leftPlaneNormal, odepth, p2)) {
									const double norm = (p1 - p2).norm();
									if(!std::isfinite(norm) || norm > INCONSISTENCY_THRESH)
										depth = INF;
								} else depth = INF;
							} else depth = INF;
						} else depth = INF;
					} else depth = INF;
				}
			}
		}
	}
}

//---------------------------------------------------------------------

void TwoViewStereo::filterInvalidPixels() {
	DepthMap cdLeftCopy = computedDepthLeft;
	DepthMap cdRightCopy = computedDepthRight;

	emit progressUpdate(6);
	emit stageUpdate("Filling invalid pixels...");
	{
		for(int y = 0; y < left.height(); ++y) {
			if(isCancelled()) return;
			for(int x = 0; x < left.width();) {
				double ldepth = computedDepthLeft[ PV(x, y, resultLeft) ];
				while(x < left.width() && !std::isinf(computedDepthLeft[ PV(x, y, resultLeft) ])) {
					ldepth = computedDepthLeft[ PV(x, y, resultLeft) ];
					++x;
				}

				if(x >= left.width())
					continue;

				double rdepth = INF;
				int start = x;

				while(x < left.width() && std::isinf(computedDepthLeft[ PV(x, y, resultLeft) ])) {
					rdepth = computedDepthLeft[ PV(x, y, resultLeft) ];
					++x;
				}

				// If we didn't go past the end of the image, get the depth
				// at the point where we found a non-NaN value.
				if(x < left.width())
					rdepth = computedDepthLeft[ PV(x, y, resultLeft) ];
				else
					rdepth = NaN;

				// Fill the gap, if necessary
				int end = x - 1;
				if(end - start < GAP_WIDTH_THRESHOLD) {
					if(!std::isfinite(ldepth)) ldepth = rdepth;
					if(!std::isfinite(rdepth)) rdepth = ldepth;

					while(start <= end) {
						computedDepthLeft[ PV(start, y, resultLeft) ] = ldepth;
						computedDepthLeft[ PV(end, y, resultLeft) ] = rdepth;
						++start;
						--end;
					}
				}
			}
		}

		for(int y = 0; y < right.height(); ++y) {
			if(isCancelled()) return;
			for(int x = 0; x < right.width();) {
				double ldepth = computedDepthRight[ PV(x, y, resultRight) ];
				while(x < right.width() && !std::isinf(computedDepthRight[ PV(x, y, resultRight) ])) {
					ldepth = computedDepthRight[ PV(x, y, resultRight) ];
					++x;
				}

				if(x >= right.width())
					continue;

				double rdepth = INF;
				int start = x;

				while(x < right.width() && std::isinf(computedDepthRight[ PV(x, y, resultRight) ])) {
					rdepth = computedDepthRight[ PV(x, y, resultRight) ];
					++x;
				}

				// If we didn't go past the end of the image, get the depth
				// at the point where we found a non-NaN value.
				if(x < left.width())
					rdepth = computedDepthRight[ PV(x, y, resultRight) ];
				else
					rdepth = NaN;

				// Fill the gap, if necessary
				int end = x - 1;
				if(end - start < GAP_WIDTH_THRESHOLD) {
					if(!std::isfinite(ldepth)) ldepth = rdepth;
					if(!std::isfinite(rdepth)) rdepth = ldepth;

					while(start <= end) {
						computedDepthRight[ PV(start, y, resultRight) ] = ldepth;
						computedDepthRight[ PV(end, y, resultRight) ] = rdepth;
						++start;
						--end;
					}
				}
			}
		}
	}
#if 0
	emit progressUpdate(7);
	emit stageUpdate("Filtering invalid pixels...");
	{
		QTime time = QTime::currentTime();

		#pragma omp parallel for
		for(int y = 0; y < left.height(); ++y) {
			if(isCancelled()) continue;
			for(int x = 0; x < left.width(); ++x) {
				if(leftMask.pixel(x, y) != WHITE) {
					cdLeftCopy[ PV(x, y, resultLeft) ] = NaN;
					continue;
				}

				weightFuncs[omp_get_thread_num()].init_weights(left, x, y);
				if(!std::isfinite(cdLeftCopy[ PV(x, y, resultLeft) ]))
					cdLeftCopy[ PV(x, y, resultLeft) ] = weightedMedian(computedDepthLeft, leftMask, x, y);
			}
		}

		#pragma omp parallel for
		for(int y = 0; y < right.height(); ++y) {
			if(isCancelled()) continue;
			for(int x = 0; x < right.width(); ++x) {
				if(rightMask.pixel(x, y) != WHITE) {
					cdRightCopy[ PV(x, y, resultRight) ] = NaN;
					continue;
				}

				weightFuncs[omp_get_thread_num()].init_weights(right, x, y);
				if(!std::isfinite(cdRightCopy[ PV(x, y, resultRight) ]))
					cdRightCopy[ PV(x, y, resultRight) ] = weightedMedian(computedDepthRight, rightMask, x, y);
			}
		}

		qDebug() << '\t' << time.elapsed() / 1000.0 << "seconds";
	}

	computedDepthLeft = cdLeftCopy;
	computedDepthRight = cdRightCopy;
#endif
}

//---------------------------------------------------------------------

typedef std::pair<double, double> ValuePair;

bool comparePairFirst(const ValuePair &a, const ValuePair &b) {
	return (a.first < b.first);
};

double TwoViewStereo::weightedMedian(const DepthMap &depths, const VectorImage &mask, int x, int y) {
	//
	std::vector<ValuePair> vals;
	vals.reserve(WINDOW_SIZE * WINDOW_SIZE);

	//
	double totalWeights = 0.0;
	for(int row = -WINDOW_RADIUS; row <= WINDOW_RADIUS; ++row) {
		for(int col = -WINDOW_RADIUS; col <= WINDOW_RADIUS; ++col) {
			const int xt = x + col;
			const int yt = y + row;
			const double depth = depths[ PV(xt, yt, mask) ];
			if(std::isnan(depth) || depth < minDepth || depth > maxDepth)
				continue;

			const double weight = (weightFuncs[omp_get_thread_num()])(row, col);
			if(weight > 1e-10) {
				vals.push_back(ValuePair(depth, weight));
				totalWeights += weight;
			}
		}
	}

	//
	double ret = NaN;
	if(vals.size() > 1 && totalWeights > 1e-10) {
		std::make_heap(vals.begin(), vals.end(), comparePairFirst);

		//
		double weight1 = 0.0;
		while(weight1 < totalWeights) {
			std::pop_heap(vals.begin(), vals.end(), comparePairFirst);
			weight1 += vals.back().second;
			totalWeights -= vals.back().second;
			ret = vals.back().first;
			vals.pop_back();
		}
	}
	return ret;
}

//---------------------------------------------------------------------

double TwoViewStereo::cost_sad(
		const VectorImage &left, const VectorImage &right,
		const VectorImage &leftMask, const VectorImage &rightMask,
		int x1, int y1, int x2, int y2)
{
	int numPixels = 0;
	double sum = 0.0, totalWeight = 0.0;
	for(int row = -WINDOW_RADIUS; row <= WINDOW_RADIUS; ++row) {
		for(int col = -WINDOW_RADIUS; col <= WINDOW_RADIUS; ++col) {
#if 1
			if(leftMask.pixel(x1 + col, y1 + row) != WHITE)
				continue;

			if(rightMask.pixel(x2 + col, y2 + row) != WHITE)
				continue;
#endif
			//
			//const RGB &lrgb = left.pixel(x1 + col, y1 + row);
			const RGBA &lrgb = left.sample(x1 + col, y1 + row);
			if(!lrgb.isValid()) continue;

			RGBA rrgb = right.pixel(x2 + col, y2 + row);
			if(!rrgb.isValid()) continue;

			double weight = (weightFuncs[omp_get_thread_num()])(row, col);
			if(weight > 1e-10) {
				//rrgb -= lrgb;
				//double diff = sqrt(rrgb.r*rrgb.r + rrgb.g*rrgb.g + rrgb.b*rrgb.b);
				//double diff = (fabs(rrgb.r) + fabs(rrgb.g) + fabs(rrgb.b)) / 3.0;
				double diff = fabs(lrgb.toGray() - rrgb.toGray());

				sum += weight*min(MAX_COLOR_DIFF, diff);
				totalWeight += weight;
				++numPixels;
			}
		}
	}

	if(numPixels <= 4 || totalWeight <= 1e-10)
		return BAD_RET;
	return (sum / totalWeight);
}

//---------------------------------------------------------------------

double TwoViewStereo::cost_ncc(
		const VectorImage &left, const VectorImage &right,
		const VectorImage &leftMask, const VectorImage &rightMask,
		int x1, int y1, int x2, int y2)
{
	// Find neighboudhood means
	double meanL = 0, meanR = 0;
	double totalWeight = 0.0;
	for(int row = -WINDOW_RADIUS; row <= WINDOW_RADIUS; ++row) {
		for(int col = -WINDOW_RADIUS; col <= WINDOW_RADIUS; ++col) {
#if 1
			if(leftMask.pixel(x1 + col, y1 + row) != WHITE)
				continue;

			if(rightMask.pixel(x2 + col, y2 + row) != WHITE)
				continue;
#endif
			//
			const RGBA &lrgb = left.sample(x1 + col, y1 + row);
			if(!lrgb.isValid()) continue;

			const RGBA &rrgb = right.sample(x2 + col, y2 + row);
			if(!rrgb.isValid()) continue;

			const double weight = (weightFuncs[omp_get_thread_num()])(row, col);
			if(weight > 1e-10) {
				meanL += weight*lrgb.toGray();
				meanR += weight*rrgb.toGray();
				totalWeight += weight;
			}
		}
	}

	if(totalWeight < 1e-10)
		return BAD_RET;

	meanL /= totalWeight;
	meanR /= totalWeight;

	// Cross-correlation
	double sum1 = 0, sum2 = 0, sum3 = 0;
	for(int row = -WINDOW_RADIUS; row <= WINDOW_RADIUS; ++row) {
		for(int col = -WINDOW_RADIUS; col <= WINDOW_RADIUS; ++col) {
			const RGBA &lrgb = left.sample(x1 + col, y1 + row);
			const RGBA &rrgb = right.sample(x2 + col, y2 + row);
#if 1
			if(leftMask.pixel(x1 + col, y1 + row) != WHITE)
				continue;

			if(rightMask.pixel(x2 + col, y2 + row) != WHITE)
				continue;
#endif
			//
			if(!lrgb.isValid()) continue;
			if(!rrgb.isValid()) continue;

			const double weight = (weightFuncs[omp_get_thread_num()])(row, col);
			if(weight > 1e-10) {
				const double pixel_gray_l = weight*lrgb.toGray();
				const double pixel_gray_r = weight*rrgb.toGray();
				sum1 += (pixel_gray_l - meanL)*(pixel_gray_r - meanR);
				sum2 += (pixel_gray_l - meanL)*(pixel_gray_l - meanL);
				sum3 += (pixel_gray_r - meanR)*(pixel_gray_r - meanR);
			}
		}
	}

	return min(MAX_COLOR_DIFF, 255*(1.0 - abs(sum1) / std::sqrt(sum2 * sum3)));
}

//---------------------------------------------------------------------

double TwoViewStereo::depthFromLabel(int label) const {
	double t = label / (numDepthLevels - 1.0);
	t /= (5 - 4*t); // non-uniform sampling
	return minDepth*(1 - t) + maxDepth*t;
}

bool TwoViewStereo::pointFromDepth(
		const Ray3d &ray,
		const Ray3d::Vector &normal,
		double depth,
		Ray3d::Point &p) const
{
	Plane3d plane(normal, p + normal*depth);
	return intersect(ray, plane, p);
}

//---------------------------------------------------------------------

std::vector<Eigen::Vector3d>
TwoViewStereo::epipolarCurve(
		const Ray3d &ray,
		const Eigen::Vector3d &cameraOffset,
		const Eigen::Vector3d &depthPlaneNormal,
		const VectorImage &mask,
		CameraPtr view) const
{
	std::vector<Eigen::Vector3d> curve;

	//
	double x1 = NaN, y1 = NaN;

	//
	for(int d = 0; d < numDepthLevels; ++d) {
		Ray3d::Point point = cameraOffset;

		const double depth = depthFromLabel(d);
		if(pointFromDepth(ray, depthPlaneNormal, depth, point)) {
			if(view->project(point)) {
				const double x2 = point[0]*imageScale;
				const double y2 = point[1]*imageScale;
				if(std::isnan(x1)) {
					x1 = x2;
					y1 = y2;
				} else {
					const double dx = x2 - x1;
					const double dy = y2 - y1;
					if(dx*dx + dy*dy >= 1) {
						LineIterator iter(x1, y1, x2, y2);
						while(iter.hasNext()) {
							int tx, ty;
							iter.current(tx, ty);
							if(mask.isNull() || mask.pixel(tx, ty) == WHITE)
								curve.push_back(Eigen::Vector3d(tx, ty, 1.0));
							++iter;
						}

						x1 = x2;
						y1 = y2;
					}
				}
			}
		}
	}
/*
	curve.erase(
		std::unique(
			curve.begin(),
			curve.end(), [](const Eigen::Vector3d &a, const Eigen::Vector3d &b){
				return ((a - b).squaredNorm() < 1e-10);
			}),
		curve.end());
*/
	return curve;
}

//---------------------------------------------------------------------
