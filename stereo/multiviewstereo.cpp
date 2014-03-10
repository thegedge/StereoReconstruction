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
// This is a partial implementation of the method described in
// Campbell et al.'s paper "Using Multiple Hypotheses to Improve
// Depth-Maps for Multi-view Stereo" (2009). Only depth map computation
// is performed (no volumetric graph cuts).
//
// URL to paper: http://portal.acm.org/citation.cfm?id=1478454
//
//---------------------------------------------------------------------
//
// TODO
//  - Less hardcoded parameters. Either remove them, or allow them to be
//    set externally somehow (i.e., through class construction / method)
//
//  - Perhaps instead of always cross-checking, allow the caller of
//    depthMap(CameraPtr) to pass in the threshold so that we can play
//    with it at the GUI level (e.g., to interactively see its impact)
//
//  - Implement volumetric graph cuts.
//
#include "multiviewstereo.hpp"

#include "project/project.hpp"
#include "project/camera.hpp"
#include "project/imageset.hpp"
#include "project/projectimage.hpp"

#include "geodesicweight.hpp"
#include "adaptiveweight.hpp"
#include "project/project.hpp"
#include "project/camera.hpp"
#include "project/imageset.hpp"
#include "project/projectimage.hpp"
#include "util/lineiter.hpp"

#include <QColor>
#include <QDebug>
#include <QTime>

#ifdef USE_MRF
#   include <MRF/TRW-S.h>
#endif

#ifdef USE_TBB
#   include <tbb/tbb.h>
#elif defined(USE_OPENMP)
#   include <omp.h>
#endif

//---------------------------------------------------------------------

using namespace Eigen;

using std::max;
using std::min;
using std::exp;
using std::make_pair;

#define PV(x, y, img)        (static_cast<int>(y)*img.width() + static_cast<int>(x))
#define CONTAINS(img, x, y)  (x >= 0 && y >= 0 && x < img.width() && y < img.height())

namespace {
	const double NaN = std::numeric_limits<double>::quiet_NaN();
	const double INF = std::numeric_limits<double>::infinity();
}

//---------------------------------------------------------------------
// TODO allow passing these values to constructor
//
namespace {
	const int WINDOW_RADIUS = 2;
	const int WINDOW_SIZE = 2*WINDOW_RADIUS + 1;

	const size_t NUM_NEIGHBOURING_VIEWS = 3;

	// From paper
	const int K = 9;
	const double BETA = 1;
	const double LAMBDA = 1;
	const double PHIU = 0.5;
	const double PSIU = 0.002;
}

//---------------------------------------------------------------------
// TODO better way of dealing with weight functions
//
class AdaptiveWeight;
class GeodesicWeight;
typedef GeodesicWeight WeightFunc;

//---------------------------------------------------------------------

double cost_ncc(
		const VectorImage &img1, const VectorImage &img2,
		const VectorImage &img1Mask, const VectorImage &img2Mask,
		int x1, int y1, int x2, int y2,
		const WeightFunc &weightFunc)
{
	// Find neighboudhood means
	double meanL = 0, meanR = 0;
	double totalWeight = 0.0;
	for(int row = -WINDOW_RADIUS; row <= WINDOW_RADIUS; ++row) {
		for(int col = -WINDOW_RADIUS; col <= WINDOW_RADIUS; ++col) {
#if 0
			if(img1Mask.pixel(x1 + col, y1 + row) != WHITE)
				continue;

			if(img2Mask.pixel(x2 + col, y2 + row) != WHITE)
				continue;
#endif
			//
			//const RGB &lrgb = img1.sample(x1 + col, y1 + row);
			const RGBA &lrgb = img1.pixel(x1 + col, y1 + row);
			if(!lrgb.isValid()) continue;

			//const RGB &rrgb = img2.sample(x2 + col, y2 + row);
			const RGBA &rrgb = img2.pixel(x2 + col, y2 + row);
			if(!rrgb.isValid()) continue;

			const double weight = weightFunc(row, col);
			if(weight > 1e-10) {
				meanL += weight*lrgb.toGray();
				meanR += weight*rrgb.toGray();
				totalWeight += weight;
			}
		}
	}

	if(totalWeight < 1e-10)
		return 0;

	meanL /= totalWeight;
	meanR /= totalWeight;

	// Cross-correlation
	double sum1 = 0, sum2 = 0, sum3 = 0;
	for(int row = -WINDOW_RADIUS; row <= WINDOW_RADIUS; ++row) {
		for(int col = -WINDOW_RADIUS; col <= WINDOW_RADIUS; ++col) {
#if 0
			if(img1Mask.pixel(x1 + col, y1 + row) != WHITE)
				continue;

			if(img2Mask.pixel(x2 + col, y2 + row) != WHITE)
				continue;
#endif
			//
			//const RGB &lrgb = img1.sample(x1 + col, y1 + row);
			const RGBA &lrgb = img1.pixel(x1 + col, y1 + row);
			if(!lrgb.isValid()) continue;

			//const RGB &rrgb = img2.sample(x2 + col, y2 + row);
			const RGBA &rrgb = img2.pixel(x2 + col, y2 + row);
			if(!rrgb.isValid()) continue;

			const double weight = weightFunc(row, col);
			if(weight > 1e-10) {
				const double pixel_gray_l = weight*lrgb.toGray();
				const double pixel_gray_r = weight*rrgb.toGray();
				sum1 += (pixel_gray_l - meanL)*(pixel_gray_r - meanR);
				sum2 += (pixel_gray_l - meanL)*(pixel_gray_l - meanL);
				sum3 += (pixel_gray_r - meanR)*(pixel_gray_r - meanR);
			}
		}
	}

	if(sum2 * sum3 < 1e-10)
		return 0;
	return sum1 / std::sqrt(sum2 * sum3);
}

//---------------------------------------------------------------------

void MultiViewStereo::initialize(ProjectPtr project,
                                 ImageSetPtr imageSet,
                                 const std::vector<CameraPtr> &views,
                                 double minDepth, double maxDepth,
                                 int numDepthLevels,
                                 double crossCheckThreshold,
                                 double imageScale)
{
	this->project = project;
	this->imageSet_ = imageSet;
	this->minDepth = minDepth;
	this->maxDepth = maxDepth;
	this->numDepthLevels = numDepthLevels;
	this->crossCheckThreshold = crossCheckThreshold;
	this->imageScale = imageScale;

	this->views.clear();
	images.clear();
	masks.clear();
	results.clear();
	computedDepths.clear();

	// Load images and masks
	for(size_t index = 0; index < views.size(); ++index) if(views[index]) {
		ProjectImagePtr projectImage = imageSet->defaultImageForCamera(views[index]);

		if(projectImage && QFileInfo(projectImage->file()).exists()) {
			QImage baseImage(projectImage->file());
			QImage image = baseImage.scaledToWidth(baseImage.width() * imageScale, Qt::SmoothTransformation);
			images.push_back( VectorImage::fromQImage(image) );

			// TODO if no mask, check alpha channel of original image
			if(baseImage.hasAlphaChannel()) {
				QImage mask = baseImage.scaledToWidth(image.width(), Qt::FastTransformation);
				masks.push_back( VectorImage(mask.width(), mask.height(), WHITE) );
				for(int y = 0; y < mask.height(); ++y) {
					for(int x = 0; x < mask.width(); ++x) {
						// If not fully opaque, we ignore that pixel
						if(qAlpha(mask.pixel(x, y)) != 255)
							masks.back().setPixel(x, y, BLACK);
					}
				}
			} else {
				masks.push_back( VectorImage(image.width(), image.height(), WHITE) );
			}

			results.push_back( VectorImage(image.width(), image.height()) );
			computedDepths.push_back( DepthMap(image.width()*image.height(), NaN) );

			this->views.push_back(views[index]);
		}
	}

	neighbours.resize(this->views.size());
}

//---------------------------------------------------------------------

namespace {
	RGBA NAN_COLOR(255, 255, 255);
	RGBA INF_COLOR(255, 255, 255);
	RGBA UNKNOWN_COLOR(255, 255, 255);
}

RGBA MultiViewStereo::colorFromDepth(double depth) const {
	if(std::isnan(depth)) return NAN_COLOR;
	if(std::isinf(depth)) return INF_COLOR;
	if(depth + 1e-5 < minDepth) return UNKNOWN_COLOR;

	// Reduce depths to [0, 1] range
	const double t = min(1.0, max(0.0, (depth - minDepth) / (maxDepth - minDepth)));

#if 1
	// Black = close, white = far
	return RGBA(255*t);
#elif 0
	// Black = far, white = close
	return RGB(255*(1 - t));
#else
	// Red = close, green = middle, blue = far (close/warm -> far/cool)
	QColor c = QColor::fromHsvF(2.0 * t / 3.0, 1.0, 1.0);
	return RGB(c.red(), c.green(), c.blue());
#endif
}

//---------------------------------------------------------------------

QImage MultiViewStereo::depthMap(CameraPtr view) const {
	for(size_t viewIndex = 0; viewIndex < views.size(); ++viewIndex) {
		if(views[viewIndex] == view)
			return VectorImage::toQImage(results[viewIndex], false);
	}
	return QImage();
}

//---------------------------------------------------------------------
// TODO put this in its own file
//
void outputPLYFile(const std::string &path, const std::vector<PLYPoint> &points) {
	//
	std::ofstream lout(path.c_str());
	lout << "ply\n"
		 << "format ascii 1.0\n"
		 << "element vertex " << points.size() << "\n"
		 << "property float x\n"
		 << "property float y\n"
		 << "property float z\n"
		 << "property uchar diffuse_red\n"
		 << "property uchar diffuse_green\n"
		 << "property uchar diffuse_blue\n"
		 << "end_header\n";

	foreach(const PLYPoint &pp, points) {
		const Ray3d::Point &p = pp.first;
		const RGBA &rgb = pp.second;

		lout << p[0] << ' ' << p[1] << ' ' << p[2]
				<< ' ' << static_cast<int>(rgb.r)
				<< ' ' << static_cast<int>(rgb.g)
				<< ' ' << static_cast<int>(rgb.b)
				<< '\n';
	}
}

//---------------------------------------------------------------------

int MultiViewStereo::numSteps() const {
	return 2*views.size(); // initial estimate + cross-checking;
}

//---------------------------------------------------------------------

void MultiViewStereo::runTask() {
	if(!project || !imageSet_ || views.size() == 0)
		return; // TODO error

	int current_step = 0;

	//
	//  Compute neighbouring images. Take the NUM_NEIGHBOURING_VIEWS closest
	//  cameras that have a reasonably small angle with the current view.
	//
	for(size_t view_index = 0; view_index < views.size(); ++view_index) {
		std::vector< std::pair<double, size_t> > nearViews;

		CameraPtr cam1 = views[view_index];
		for(size_t viewIndex2 = 0; viewIndex2 < views.size(); ++viewIndex2) {
			if(view_index != viewIndex2) {
				CameraPtr cam2 = views[viewIndex2];

				// If angle between viewing directions is less than some threshold
				if(fabs(cam1->principleRay().direction().dot(cam2->principleRay().direction())) > 0.2) {
					double dist = (cam1->C() - cam2->C()).squaredNorm();
					nearViews.push_back(make_pair(dist, viewIndex2));
				}
			}
		}

		auto end = nearViews.end();
		if(NUM_NEIGHBOURING_VIEWS < nearViews.size()) {
			std::sort(nearViews.begin(), nearViews.end());
			end = nearViews.begin() + NUM_NEIGHBOURING_VIEWS;
		}

		neighbours[view_index].clear();
		for(auto iter = nearViews.begin(); iter != end; ++iter)
			neighbours[view_index].push_back(iter->second);
	}

	//
	// Compute initial stereo estimate
	//
	for(size_t view_index = 0; view_index < views.size(); ++view_index) {
		emit progressUpdate(current_step++);
		if(views[view_index]) {
			//if(views[view_index]->id() != "7310095" && views[view_index]->id() != "7310085")
			//	continue;
			//if(views[view_index]->id() != "11")
			//	continue;

			emit stageUpdate(tr("Computing cost volume for camera %1").arg(views[view_index]->name()));
			computeInitialEstimate(view_index);
		}
	}

	//
	// Construct resulting depth map from computed depths
	//
	emit stageUpdate(tr("Constructing depth maps"));
	for(size_t view_index = 0; view_index < views.size(); ++view_index) {
		if(views[view_index]) {
			for(int y = 0; y < results[view_index].height(); ++y) {
				for(int x = 0; x < results[view_index].width(); ++x) {
					if(masks[view_index].pixel(x, y) == WHITE) {
						const double d = computedDepths[view_index][ PV(x, y, results[view_index]) ];
						results[view_index].setPixel(x, y, colorFromDepth(d));
					} else {
						results[view_index].setPixel(x, y, WHITE);
					}
				}
			}
		}
	}

	//NAN_COLOR = UNKNOWN_COLOR = RGBA(255, 0, 0);

	//
	// Output stuff
	//
	for(size_t view_index = 0; view_index < views.size(); ++view_index) {
		if(!views[view_index] || isCancelled())
			continue;

		// Compute percentage of pixels (in mask) that have a depth hypothesis
		int total = 0, have = 0;
		for(int y = 0; y < results[view_index].height(); ++y) {
			for(int x = 0; x < results[view_index].width(); ++x) {
				if(masks[view_index].pixel(x, y) == WHITE) {
					++total;
					const double d = computedDepths[view_index][ PV(x, y, results[view_index]) ];
					if(std::isfinite(d))
						++have;
				}
			}
		}

		qDebug() << QString("%1:").arg(views[view_index]->name())
				 << ((100.0*have) / total) << "percent of pixels have depth hypotheses (before cross-check)";
	}

	//
	// Cross-checking
	//
	emit stageUpdate(tr("Cross-checking"));
	for(size_t view_index = 0; view_index < views.size(); ++view_index) {
		emit progressUpdate(current_step++);
		if(views[view_index])
			crossCheck(view_index);
	}

	//
	// Construct resulting depth map from computed depths
	//
	emit stageUpdate(tr("Constructing depth maps"));
	for(size_t view_index = 0; view_index < views.size(); ++view_index) {
		if(views[view_index]) {
			for(int y = 0; y < results[view_index].height(); ++y) {
				for(int x = 0; x < results[view_index].width(); ++x) {
					if(masks[view_index].pixel(x, y) == WHITE) {
						const double d = computedDepths[view_index][ PV(x, y, results[view_index]) ];
						results[view_index].setPixel(x, y, colorFromDepth(d));
					} else {
						results[view_index].setPixel(x, y, WHITE);
					}
				}
			}
		}
	}

	//
	// Output stuff
	//
	for(size_t view_index = 0; view_index < views.size(); ++view_index) {
		if(!views[view_index] || isCancelled())
			continue;

		// Compute percentage of pixels (in mask) that have a depth hypothesis
		int total = 0, have = 0;
		for(int y = 0; y < results[view_index].height(); ++y) {
			for(int x = 0; x < results[view_index].width(); ++x) {
				if(masks[view_index].pixel(x, y) == WHITE) {
					++total;
					const double d = computedDepths[view_index][ PV(x, y, results[view_index]) ];
					if(std::isfinite(d))
						++have;
				}
			}
		}

		qDebug() << QString("%1:").arg(views[view_index]->name())
				 << ((100.0*have) / total) << "percent of pixels have depth hypotheses";
	}
}

//---------------------------------------------------------------------
// K determined by index in vector, PeakPair = <NCC cost, depth>
typedef std::pair<double, double> PeakPair;

struct CostFunction {
	static std::vector<std::vector<std::vector<PeakPair> > > peakPairs;
	static int width, height;
#ifdef USE_MRF
	static MRF::CostVal dataCost(int p, MRF::Label label) {
		if(label == K) return PHIU;

		const int x = p % width;
		const int y = p / width;
		if(peakPairs[y][x][label].second < 0)
			return LAMBDA;

		//qDebug() << x << y << label << LAMBDA*exp(-BETA * peakPairs[y][x][label].first);
		const double ret = LAMBDA*exp(-BETA * peakPairs[y][x][label].first);
		//if(std::isnan(ret))
		//	qDebug() << ":(" << peakPairs[y][x][label].first;
		return ret;
	}

	static MRF::CostVal smoothnessCost(int p1, int p2, MRF::Label label1, MRF::Label label2) {
		if(label1 == K && label2 == K) return 0.0;
		if(label1 == K || label2 == K) return PSIU;

		const int x1 = p1 % width;
		const int y1 = p1 / width;
		const int x2 = p2 % width;
		const int y2 = p2 / width;
		const double z1 = peakPairs[y1][x1][label1].second;
		const double z2 = peakPairs[y2][x2][label2].second;
		if(z1 < 0 || z2 < 0)
			return 2*PSIU;

		return 2.0 * fabs(z1 - z2) / (z1 + z2);
	}
#endif
};

std::vector<std::vector<std::vector<PeakPair> > > CostFunction::peakPairs;
int CostFunction::width = 0;
int CostFunction::height = 0;

//---------------------------------------------------------------------

void MultiViewStereo::computeInitialEstimate(size_t viewIndex) {
	const int width = images[viewIndex].width();
	const int height = images[viewIndex].height();
	const Ray3d::Point &cameraC = views[viewIndex]->C();
	const Ray3d::Vector &depthPlaneNormal = views[viewIndex]->principleRay().direction();

	CameraPtr view = views[viewIndex];
	const VectorImage &image = images[viewIndex];
	const VectorImage &mask = masks[viewIndex];

	//
	CostFunction::peakPairs.clear();
	CostFunction::peakPairs.resize(height);
	for(int y = 0; y < height; ++y)
		CostFunction::peakPairs[y].resize(width);

	// First, for each pixel, collect the top K NCC peaks from surrounding
	// views which we will later feed to an MRF optimization routine
	{
#ifdef USE_OPENMP
		#pragma omp parallel for
		for(int y = 0; y < height; ++y) {
			WeightFunc weightFunc(WINDOW_RADIUS);
#elif defined(USE_TBB)
		tbb::parallel_for( tbb::blocked_range<int>(0, height), [&](const tbb::blocked_range<int> &r) {

		WeightFunc weightFunc(WINDOW_RADIUS);
		for(int y = r.begin(); y != r.end(); ++y) {
#else
		WeightFunc weightFunc(WINDOW_RADIUS);
		for(int y = 0; y < height; ++y) {
#endif
			for(int x = 0; x < width; ++x) {
                if(this->isCancelled()) return;

				computedDepths[viewIndex][ PV(x, y, results[viewIndex]) ] = INF;

				std::vector<PeakPair> &peaks = CostFunction::peakPairs[y][x];
				peaks.resize(K, PeakPair(0, -1));

				//
				if(masks[viewIndex].pixel(x, y) != WHITE)
					continue;

				// Allow weighting function to initialize anything window-specific
				weightFunc.init_weights(images[viewIndex], x, y);

				// Multi-view NCC cost collection
				Ray3d ray = view->unproject((x + 0.5) / imageScale, (y + 0.5) / imageScale);

				foreach(size_t viewIndex2, neighbours[viewIndex]) {
					const CameraPtr &oview = views[viewIndex2];
					const VectorImage &oimage = images[viewIndex2];
					const VectorImage &omask = masks[viewIndex2];

					// Sample the ray, project samples into right view to obtain
					// a piece-wise linear estimate of the epipolar curve. Find
					// NCC scores along that curve
					std::vector<Eigen::Vector3d> curve = this->epipolarCurve(ray, cameraC, depthPlaneNormal, omask, oview);
					foreach(const Eigen::Vector3d &p, curve) {
						Ray3d ray2 = oview->unproject((p[0] + 0.5) / imageScale, (p[1] + 0.5) / imageScale);
						Eigen::Vector3d p1, p2;
						ray.closestPoints(ray2, p1, p2);

						const double cost = cost_ncc(image, oimage, mask, omask, x, y, p[0], p[1], weightFunc);
						if(cost > 0.95) {
							p1 += p2;
							p1 *= 0.5;
							p1 = view->fromGlobalToLocal(p1);
							peaks.push_back( PeakPair(cost, p1.z()) );
						}
					}
				}

				// Take the top K peaks. Since a larger NCC score is better
				// than a small one, take the last K peaks after sorting.
				std::sort(peaks.begin(), peaks.end());
				//peaks.erase(peaks.begin(), peaks.end() - K);
				peaks = std::vector<PeakPair>(peaks.end() - K, peaks.end());
			}
		}
#ifdef USE_TBB
		}); // lambda + tbb::parallel_for
#endif
	}

#ifdef USE_MRF
	if(isCancelled()) return;

	// MRF optimization
	CostFunction::width = CostFunction::peakPairs.front().size();
	CostFunction::height = CostFunction::peakPairs.size();

	DataCost dataCost(&CostFunction::dataCost);
	SmoothnessCost smoothnessCost(&CostFunction::smoothnessCost);
	EnergyFunction func(&dataCost, &smoothnessCost);

	TRWS optimizer(width, height, K + 1, &func);
	optimizer.initialize();
	optimizer.clearAnswer();

	//qDebug() << "\tTotal Energy =" << optimizer.totalEnergy() << '('
	//         << optimizer.dataEnergy() << '+' << optimizer.smoothnessEnergy() << ')';

	float time;
	double energy = optimizer.totalEnergy();
	double prevEnergy = 0.0;
	int numIters = 50;
	do {
		if(isCancelled()) return;

		prevEnergy = energy;
		optimizer.optimize(1, time);
		energy = optimizer.totalEnergy();
		//qDebug() << "\tTotal Energy =" << energy << '('
		//         << optimizer.dataEnergy() << '+' << optimizer.smoothnessEnergy() << ')';
	} while(prevEnergy - energy > 5 && numIters-- > 0);

	//
	for(int y = 0, p = 0; y < height; ++y) {
		for(int x = 0; x < width; ++x, ++p) if(mask.pixel(x, y) == WHITE) {
			int label = optimizer.getLabel(p);
			double depth = (label == K ? INF : CostFunction::peakPairs[y][x][label].second);
			if(depth > 0)
				computedDepths[viewIndex][p] = depth;
			else
				computedDepths[viewIndex][p] = INF;
		}
	}
#else
	for(int y = 0, p = 0; y < height; ++y) {
		for(int x = 0; x < width; ++x, ++p) {
            if(this->isCancelled()) return;
			if(mask.pixel(x, y) == WHITE)
				computedDepths[viewIndex][p] = CostFunction::peakPairs[y][x].back().second;
		}
	}
#endif
}

//---------------------------------------------------------------------

void MultiViewStereo::crossCheck(size_t view_index) {
	CameraPtr view = views[view_index];
	Ray3d::Vector viewNormal = view->principleRay().direction();

	// Invalidate pixels that fail to cross-check
#ifdef USE_OPENMP
	#pragma omp parallel for
	for(int y = 0; y < images[view_index].height(); ++y) {
#elif defined(USE_TBB)
	tbb::parallel_for( tbb::blocked_range<int>(0, images[view_index].height()),
					   [&](const tbb::blocked_range<int> &r) {
	for(int y = r.begin(); y != r.end(); ++y) {
#else
	for(int y = 0; y < images[view_index].height(); ++y) {
#endif
        if(this->isCancelled()) break;
		for(int x = 0; x < images[view_index].width(); ++x) {
			//
			double &depth = computedDepths[view_index][ PV(x, y, results[view_index]) ];
			if(!std::isfinite(depth))
				continue;

			//
			Ray3d ray = view->unproject((x + 0.5) / imageScale, (y + 0.5) / imageScale);
			Ray3d::Point p1 = view->C();
			if(pointFromDepth(ray, viewNormal, depth, p1)) {
				double x2, y2;
				bool found = false;
				for(size_t view_index2 = 0; view_index2 < views.size(); ++view_index2) {
					if(view_index2 == view_index)
						continue;

					CameraPtr oview = views[view_index2];
					if(oview->project(p1, x2, y2)) {
						x2 = x2*imageScale;
						y2 = y2*imageScale;

						if(CONTAINS(results[view_index2], x2, y2)) {
							double &odepth = computedDepths[view_index2][ PV(x2, y2, results[view_index2]) ];
							if(std::isfinite(odepth)) {
								Ray3d ray2 = oview->unproject((x2 + 0.5) / imageScale, (y2 + 0.5) / imageScale);
								Ray3d::Point p2 = oview->C();
								Ray3d::Vector oviewNormal = oview->principleRay().direction();
								if(pointFromDepth(ray2, oviewNormal, odepth, p2)) {
									const double norm = (p1 - p2).norm();
									if(std::isfinite(norm) && norm < crossCheckThreshold) {
										found = true;
										break;
									}
								}
							}
						}
					}
				}

				if(!found)
					depth = NaN;
			}
		}
	}
#ifdef USE_TBB
	}); // lambda + tbb::parallel_for
#endif
}

//---------------------------------------------------------------------

double MultiViewStereo::depthFromLabel(int label) const {
	double t = label / (numDepthLevels - 1.0);
	return minDepth*(1 - t) + maxDepth*t;
}

//---------------------------------------------------------------------

bool MultiViewStereo::pointFromDepth(
		const Ray3d &ray,
		const Ray3d::Vector &normal,
		double depth,
		Ray3d::Point &p) const
{
	//p = ray.point(depth / normal.dot(ray.direction()));
	//return true; // XXX I think the above is always valid as long as depth > 0...?
	Plane3d plane(normal, p + normal*depth);
	return intersect(ray, plane, p);
}

//---------------------------------------------------------------------

std::vector<Eigen::Vector3d>
MultiViewStereo::epipolarCurve(
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
						LineIterator iter(x1, y1, x2, y2, mask.width(), mask.height());
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

	// Remove any duplicate points
	curve.erase(
		std::unique(
			curve.begin(),
			curve.end(), [](const Eigen::Vector3d &a, const Eigen::Vector3d &b){
				return ((a - b).squaredNorm() < 1e-5);
			}),
		curve.end());

	return curve;
}

//---------------------------------------------------------------------
//---------------------------------------------------------------------
