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
#include "radiometriccalibrationtask.hpp"
#include "project/camera.hpp"
#include "project/imageset.hpp"
#include "project/projectimage.hpp"

#include <QDir>
#include <QHash>
#include <QPair>
#include <QSet>
#include <QTime>

#include <cmath>
#include <Eigen/Dense>
#include <Eigen/LU>

//---------------------------------------------------------------------

RadiometricCalibrationTask::RadiometricCalibrationTask(
		CameraPtr camera,
		ImageSetPtr imageSet)
	: camera(camera)
	, imageSet(imageSet)
{ }

//---------------------------------------------------------------------

typedef struct _Patch {
	QRect r; // rectangle of patch
	int row; // row from images table
	double brightness;
	double variance;

	_Patch(const QRect &r,
		   int row = -1.0,
		   double brightness = 0.0,
		   double variance = 1000.0)
		: r(r), row(row), brightness(brightness), variance(variance)
	{ }

	bool operator==(const _Patch &o) const {
		return (row == o.row && r == o.r);
	}
} Patch;

uint qHash(const Patch &p) {
	// TODO perhaps a more efficient hashing implementation
	return qHash(QString("%1 %2 %3 %4 %5").arg(
			QString::number(p.row),
			QString::number(p.r.x()),
			QString::number(p.r.y()),
			QString::number(p.r.width()),
			QString::number(p.r.height()) ));
}

uint qHash(const QPoint &p) {
	return ((p.x() * 0x1f1f1f1f) ^ p.y());
}

//---------------------------------------------------------------------
/*!
 * Obtain brightness/variance values for a patch whose top-left corner is
 * located at the specified point. Brightness is determined by the Y
 * component in the CIE XYZ color space.
 *
 * @return <code>true</code> if the patch is completely obtained in the image
 *         and contains no over-/under-exposed pixels, in which case the values
 *         of <code>brightness</code> and <code>variation</code> are set.
 *         Otherwise <code>false</code> is returned and no values are computed.
 */
template <class T, T brightnessFunc(QRgb)>
bool brightnessAndVariance(QImage img, const QRect &r, double &brightness, double &variation) {
	if(!QRect(0, 0, img.width(), img.height()).contains(r))
		return false;

	// Gather samples to obtain mean/brightness
	brightness = 0.0;
	for(int y = r.top(); y <= r.bottom(); ++y) {
		for(int x = r.left(); x <= r.right(); ++x) {
			T val = brightnessFunc(img.pixel(x, y));
			if(val <= 1e-5 || val + 1e-5 >= 255) // if over-/under-exposed
				return false;

			brightness += val;
		}
	}

	double mean = brightness / (r.width() * r.height());
	variation = 0.0;
	for(int y = r.top(); y <= r.bottom(); ++y) {
		for(int x = r.left(); x < r.right(); ++x) {
			T val = brightnessFunc(img.pixel(x, y));
			variation += (val - mean)*(val - mean);
		}
	}

	variation /= r.width() * r.height();
	return true;
}

//---------------------------------------------------------------------

template <class T, T brightnessFunc(QRgb)>
QList<Patch> collectSamples(QList<QImage> images, QList<int> rows) {
	//
	// Find sample points (as per page 142 of book High Dynamic Range
	// Imaging, Reinhard et al)
	//
	QSet<Patch> samples; // complete set of samples
	{
		QSet<Patch> previousSamples; // samples from previous exposure

		const int PATCH_SIZE = 7;
		const int NUM_PATCHES_PER_EXPOSURE = 200;
		const double VARIANCE_THRESHOLD = 15*15;

		double B, V; // brightness and variance
		foreach(int img, rows) {
			// Step 3
			QSet<Patch> currentSamples;
			foreach(const Patch &patch, previousSamples) {
				brightnessAndVariance<T, brightnessFunc>(images[img], patch.r, B, V);

				if(V < VARIANCE_THRESHOLD) {
					bool brighter = false;
					foreach(const Patch &patch2, previousSamples) {
						if(B > patch2.brightness) {
							brighter = true;
							break;
						}
					}

					if(brighter)
						currentSamples << Patch(patch.r, img, B, V);
				}
			}

			// Step 4/5
			const int MAX_ITERATIONS = NUM_PATCHES_PER_EXPOSURE * 100;
			for(int iter = 0;
				iter < MAX_ITERATIONS && currentSamples.size() < NUM_PATCHES_PER_EXPOSURE;
				++iter)
			{
				// Random point selection
				int x = images[img].width() * (qrand() / (1.0*RAND_MAX));
				int y = images[img].height() * (qrand() / (1.0*RAND_MAX));
				Patch p(QRect(x, y, PATCH_SIZE, PATCH_SIZE), img);

				// Ensure no overlap between patches
				bool noOverlap = true;
				foreach(const Patch &patch, currentSamples) {
					if(p.r.intersects(patch.r)) {
						noOverlap = false;
						break;
					}
				}

				// If patch is invalid (see book for definition of patch validity)
				// then reject it, otherwise append it to the list of samples
				if(noOverlap
				   && brightnessAndVariance<T, brightnessFunc>(images[img], p.r, p.brightness, p.variance)
				   && p.variance < VARIANCE_THRESHOLD)
				{
					// Make sure patch is brighter than any previous patch
					bool brighter = false;
					foreach(const Patch &patch, previousSamples) {
						if(p.brightness > patch.brightness) {
							brighter = true;
							break;
						}
					}

					// If brighter, or no patches in previous exposure, append
					if(brighter || previousSamples.size() == 0)
						currentSamples << p;
				}
			}

			samples |= currentSamples;
			previousSamples = currentSamples;
		}
	}
	return samples.toList();
}

//---------------------------------------------------------------------

#ifndef USE_MITSUNAGA_NAYAR
std::vector<double>
RadiometricCalibrationTask::responseCurve(QList<int> rows, const Samples &samples) {
	if(samples.size() == 0 || rows.size() == 0)
		return std::vector<double>(256, -1.0);

	//
	const int Zmin = 0;
	const int Zmax = 255;
	const int n = Zmax - Zmin + 1;
	const int lambda = 25;

	//
	const int numSamplePoints = samples.size() / rows.size();
	int M = samples.size() + n - 1;
	int N = n + numSamplePoints;

	int k = 0;

	// Convert table exposures to logarithmic doubles (in seconds)
	std::vector<double> exposures(images.size(), 0.0);
	foreach(int img, rows)
		exposures[img] = log(imageSet->images()[img]->exposure() / 1000.0);

	// Collect cooefficients and constants for linear system
	Eigen::MatrixXd A = Eigen::MatrixXd(M, N);
	Eigen::VectorXd b = Eigen::VectorXd(M);

	// Data-fitting equations
	for(size_t sampleIndex = 0; sampleIndex < samples.size(); ++sampleIndex, ++k) {
		int v = samples[sampleIndex].first + 1;
		int img = samples[sampleIndex].second;
		int wij = ((2*v <= (Zmin + Zmax)) ? v - Zmin : Zmax - v);

		A(k, v - 1) = wij;
		A(k, n + (sampleIndex % numSamplePoints)) = -wij;
		b[k] = wij*exposures[img];
	}

	// Middle value is set to be zero
	A(k++, (Zmin + Zmax) / 2) = 1;

	// Smoothness equations
	for(int v = 0; v + 2 < n; ++v, ++k) {
		int wi = ((2*(v + 1) <= (Zmin + Zmax)) ? v + 1 - Zmin : Zmax - (v + 1));
		A(k, v) = lambda*wi;
		A(k, v+1) = -2*lambda*wi;
		A(k, v+2) = lambda*wi;
	}

	// Solve linear system
	Eigen::MatrixXd transA = A.transpose();
	Eigen::VectorXd x = transA*b;
	A = transA*A;
	//x = A.lu().solve(x);
	A.lu().solve(x, &b);

	//
	std::vector<double> response(n, 0.0);
	for(int i = 0; i < n; ++ i)
		response[i] = b[i];
	return response;
}
#endif

//---------------------------------------------------------------------

void RadiometricCalibrationTask::calibrate() {
	if(!camera || !imageSet)
		return; // XXX error!!

	qsrand(QTime().elapsed());

	emit stageUpdate("Loading images");
	emit progressUpdate(1);
	if(isCancelled())
		return;

	//
	// Load images
	//
	foreach(ProjectImagePtr image, imageSet->images()) {
		QImage img(image->file());
		if(!img.isNull())
			images << img;
	}

	//
	// Collect indicies, sort from highest to lowest exposure
	//
	QMap<QString, QList<int> > cameras;
	{
		typedef QPair<int, double> Pair;
		QList<Pair> exposureRowPairs;
		for(size_t index = 0; index < imageSet->images().size(); ++index)
			exposureRowPairs << Pair(index, imageSet->images()[index]->exposure());

		qSort(exposureRowPairs);
		foreach(const Pair &pair, exposureRowPairs) {
			QString cameraId = imageSet->images()[pair.first]->camera()->id();
			cameras[cameraId] << pair.first;
		}
	}

	//
	// Collect a representative set of patches for calibration
	//
	emit stageUpdate("Finding candidate samples");
	emit progressUpdate(2);
	if(isCancelled())
		return;

	QList<int> rows = cameras[0]; // TODO iterate over all cameras
	QList<Patch> patches[3] = {
		collectSamples<int, qRed>(images, rows),
		collectSamples<int, qGreen>(images, rows),
		collectSamples<int, qBlue>(images, rows) };

#ifdef USE_MITSUNAGA_NAYAR

#else
	//
	// Eliminate samples that existed in only one or two images
	//
	emit stageUpdate("Cleaning candidate set");
	emit progressUpdate(3);

	if(isCancelled()) return;
	else {
		QHash<QPoint, int> counts[3];
		for(int channel = 0; channel < 3; ++channel) {
			for(int sampleIndex = 0; sampleIndex < patches[channel].size(); ++sampleIndex) {
				QPoint p = patches[channel][sampleIndex].r.center();
				counts[channel][p] = counts[channel].value(p, 0) + 1;
			}
		}

		for(int channel = 0; channel < 3; ++channel) {
			patches[channel].clear();
			foreach(const QPoint &p, counts[channel].keys()) {
				//if(counts[channel][p] == rows.size())
					patches[channel] << Patch(QRect(p, QSize(1, 1)), 0);
			}
		}
	}

	//
	// Calibrate (Debevec and Malik's algorithm)
	//
	emit stageUpdate("Finding response curve");
	emit progressUpdate(4);

	if(isCancelled()) return;
	else {
		Samples redSamples(patches[0].size() * rows.size());
		Samples greenSamples(patches[1].size() * rows.size());
		Samples blueSamples(patches[2].size() * rows.size());

		int rindex = 0, gindex = 0, bindex = 0;
		foreach(int img, rows) {
			for(int sampleIndex = 0; sampleIndex < patches[0].size(); ++sampleIndex, ++rindex) {
				const Patch &patch = patches[0][sampleIndex];
				QRgb rgb = images[img].pixel(patch.r.center());
				redSamples[rindex] = Sample(qRed(rgb), img);
			}

			for(int sampleIndex = 0; sampleIndex < patches[1].size(); ++sampleIndex, ++gindex) {
				const Patch &patch = patches[1][sampleIndex];
				QRgb rgb = images[img].pixel(patch.r.center());
				greenSamples[gindex] = Sample(qGreen(rgb), img);
			}

			for(int sampleIndex = 0; sampleIndex < patches[2].size(); ++sampleIndex, ++bindex) {
				const Patch &patch = patches[2][sampleIndex];
				QRgb rgb = images[img].pixel(patch.r.center());
				blueSamples[bindex] = Sample(qBlue(rgb), img);
			}
		}

		std::vector<double> redResponse = responseCurve(rows, redSamples);
		std::vector<double> greenResponse = responseCurve(rows, greenSamples);
		std::vector<double> blueResponse = responseCurve(rows, blueSamples);

		Responses response(256);
		for(size_t index = 0; index < 256; ++index)
			response[index] = Response(redResponse[index], greenResponse[index], blueResponse[index]);
		camera->setResponse(response);
	}
#endif

	emit progressUpdate(5);
	emit stageUpdate(tr("Finished!"));
}

//---------------------------------------------------------------------
