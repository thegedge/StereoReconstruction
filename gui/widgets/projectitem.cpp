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
#include "projectitem.hpp"

#include "project/camera.hpp"
#include "project/imageset.hpp"
#include "project/projectimage.hpp"

#include <QDebug>

//---------------------------------------------------------------------

Q_DECLARE_METATYPE(ProjectImage *);

//---------------------------------------------------------------------

ProjectItem::ProjectItem(QObject *obj)
    : obj(obj)
{
	if(const Camera *cam = qobject_cast<const Camera *>(obj)) {
		connect(cam, SIGNAL(nameChanged(QString)), SLOT(update(QString)));
	} else if(const ImageSet *imageSet = qobject_cast<const ImageSet *>(obj)) {
		connect(imageSet, SIGNAL(nameChanged(QString)), SLOT(update(QString)));
		connect(imageSet, SIGNAL(nameChanged(QString)), SLOT(update(QString)));
		connect(imageSet, SIGNAL(nameChanged(QString)), SLOT(update(QString)));
	} else if(const ProjectImage *image = qobject_cast<const ProjectImage *>(obj)) {
		connect(image, SIGNAL(fileChanged(QString)), SLOT(update(QString)));
		connect(image, SIGNAL(cameraChanged(CameraPtr,CameraPtr)), SLOT(update(CameraPtr,CameraPtr)));
		if(CameraPtr cam = image->camera())
			connect(cam.get(), SIGNAL(nameChanged(QString)), SLOT(update(QString)));
	}

	// XXX Does this remove me from tree widget?
	connect(obj, SIGNAL(destroyed()), SLOT(deleteLater()));

	update();
}

//---------------------------------------------------------------------

void ProjectItem::update() {
	if(Camera *cam = qobject_cast<Camera *>(obj)) {
		setFlags(flags() | Qt::ItemIsEditable);
		setData(0, Qt::UserRole, cam->id());
		setText(0, cam->name());
		setToolTip( 0, QString("Name: %1\nId: %2").arg(cam->name(), cam->id()) );
	} else if(ImageSet *imageSet = qobject_cast<ImageSet *>(obj)) {
		setFlags(flags() | Qt::ItemIsEditable);
		setData(0, Qt::UserRole, imageSet->id());
		setText(0, imageSet->name());
		setToolTip( 0, QString("Name: %1\nRoot: %2").arg(imageSet->name(), imageSet->root().absolutePath()) );
	} else if(ProjectImage *image = qobject_cast<ProjectImage *>(obj)) {
		QString camera = tr("<no camera>");
		if(image->camera())
			camera = image->camera()->name();

		setFlags(flags() & ~Qt::ItemIsEditable);
		setData(0, Qt::UserRole, QVariant::fromValue(image)); // XXX Hopefully this is a sufficient "id" for images
		setText(0, image->imageSet()->root().relativeFilePath(image->file()));
		setToolTip(0, QString("Path: %1\nCamera: %2\nExposure: %3")
		                 .arg(image->file(), camera)
		                 .arg(image->exposure(), 0, 'f', 2));
	}
}

//---------------------------------------------------------------------

void ProjectItem::update(CameraPtr oldCam, CameraPtr newCam) {
	if(oldCam) oldCam->disconnect(this);
	if(newCam) connect(newCam.get(), SIGNAL(nameChanged(QString)), SLOT(update(QString)));
	update();
}

//---------------------------------------------------------------------
