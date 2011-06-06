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
//  - Implement parallel when TBB enabled
//
#include "findfeaturestask.hpp"
#include "detector.hpp"
#include "project/project.hpp"
#include "project/imageset.hpp"

#ifdef USE_TBB
#   include <tbb/tbb.h>
#   define lock(a)   a.lock()
#   define unlock(a) a.unlock()
#elif defined(USE_OPENMP)
#   include <omp.h>
#   define lock(a)   omp_set_lock(&a);
#   define unlock(a) omp_unset_lock(&a);
#else
#   define lock(a)
#   define unlock(a)
#endif

//---------------------------------------------------------------------

int FindFeaturesTask::numSteps() const {
	return imageSets.size();
}

//---------------------------------------------------------------------

void FindFeaturesTask::runTask() {
#ifdef USE_TBB
	tbb::spin_mutex theLock;
#elif defined(USE_OPENMP)
	omp_lock_t theLock;
	omp_init_lock(&theLock);
#endif

	int currentStep = 0;
	foreach(ImageSetPtr imageSet, imageSets) {
		emit stageUpdate(tr("Finding features for %1").arg(imageSet->name()));

#ifdef USE_TBB
		tbb::parallel_for( tbb::blocked_range<size_t>(0, cameras.size()), [&](const tbb::blocked_range<size_t> &r) {
		for(size_t index = r.begin(); index != r.end(); ++index) {
			CameraPtr cam = cameras[index];
#elif defined(USE_OPENMP)
		#pragma omp parallel for
		for(size_t index = 0; index < cameras.size(); ++index) {
			CameraPtr cam = cameras[index];
			if(isCancelled())
				continue;
#else
		foreach(CameraPtr cam, cameras) {
			if(isCancelled())
				return;
#endif

			if(ProjectImagePtr img = imageSet->defaultImageForCamera(cam)) {
				lock(theLock);
				Features &features = project->features().features(img);
				unlock(theLock);

				if(features.empty())
					features = detector->features(img);
			}
		}
#ifdef USE_TBB
		});
#endif
		emit progressUpdate(++currentStep);
	}

#ifdef USE_OPENMP
	omp_destroy_lock(&theLock);
#endif
}

//---------------------------------------------------------------------
