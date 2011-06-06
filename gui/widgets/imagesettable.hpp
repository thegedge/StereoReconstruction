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
#ifndef IMAGESETTABLE_HPP
#define IMAGESETTABLE_HPP

#include <QTableWidget>
#include <QImage>

#include "util/c++0x.hpp"

//
// Forward declarations
//
class LoadImagesThread;

FORWARD_DECLARE(Project);
FORWARD_DECLARE(ImageSet);

//! A widget to show a set of images in a Project
class ImageSetTable : public QTableWidget {
    Q_OBJECT

public:
    ImageSetTable(QWidget *parent = 0);
	~ImageSetTable();

public slots:
	void setProject(ProjectPtr project);
	void setImageSet(int imageSetIndex, ImageSetPtr imageSet);

private slots:
	void imageLoaded(int index, QImage image);
	void cellChanged(int, int);

protected:
    void changeEvent(QEvent *e);

private:
	LoadImagesThread *loadImages;

	ProjectPtr project;
	ImageSetPtr currentSet;
};

#endif // IMAGESETTABLE_HPP
