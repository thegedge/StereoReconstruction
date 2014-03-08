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
#ifndef PROJECTITEM_HPP
#define PROJECTITEM_HPP

#include <QObject>
#include <QTreeWidgetItem>

FORWARD_DECLARE(Camera);
FORWARD_DECLARE(ImageSet);
FORWARD_DECLARE(ProjectImage);

//! .
class ProjectItem : public QObject, public QTreeWidgetItem {
	Q_OBJECT

public:
	ProjectItem(QObject *obj);

public slots:
	void update();
	void update(CameraPtr, CameraPtr);

private:
	QObject *obj;
};

//---------------------------------------------------------------------

#endif // PROJECTITEM_HPP
