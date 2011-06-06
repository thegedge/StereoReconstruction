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
#include "loadimagesthread.hpp"

//---------------------------------------------------------------------

LoadImagesThread::LoadImagesThread(QStringList images, QObject *parent)
	: QThread(parent)
	, images(images)
	, shouldStop(false)
{ }

//---------------------------------------------------------------------

void LoadImagesThread::run() {
	// Set row heights/column widths
	for(int index = 0; index < images.size() && !shouldStop; ++index) {
		QImage image(images[index]);
		emit imageLoaded(index, image);
	}
}

//---------------------------------------------------------------------

void LoadImagesThread::stop() {
	shouldStop = true;
}

//---------------------------------------------------------------------
