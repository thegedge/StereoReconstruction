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
#ifndef PROJECT_H
#define PROJECT_H

#include <QDir>
#include <QMap>
#include <QObject>
#include <QString>

#include <boost/noncopyable.hpp>

#include "features/featuredb.hpp"
#include "util/c++0x.hpp"

//
// Forward declarations
//
class QDomDocument;

FORWARD_DECLARE(Camera);
FORWARD_DECLARE(ImageSet);

typedef QMap<QString, CameraPtr> CameraMap;
typedef QMap<QString, ImageSetPtr> ImageSetMap;

/*!
 * A project for StereoReconstruction.
 */
class Project : public QObject, public boost::noncopyable {
	Q_OBJECT

public:
	Project(QString projectPath = QString());

public:
	//! Set the path for the project
	void setProjectPath(QString path) { projectPath_.setPath(path); }

	//! Get the path for the project
	QDir projectPath() { return projectPath_; }

public: // Cameras
	//! Get a camera from its id
	CameraPtr camera(QString id) { return cameras_.value(id, CameraPtr()); }

	//! \copydoc camera(QString)
	const CameraPtr camera(QString id) const { return cameras_.value(id, CameraPtr()); }

	//! Get a reference to the camera map
	const CameraMap& cameras() const { return cameras_; }

	//! Get a camera from its name
	CameraPtr cameraFromName(QString name);

	//! Add a new camera to the project
	void addCamera(CameraPtr cam);

	/*!
	 * Remove a camera from the project. If \a removeImages is \c true then
	 * any images in this project that reference \a cam will be removed.
	 * Otherwise, just the camera reference will be removed.
	 */
	void removeCamera(CameraPtr cam, bool removeImages);

public: // Image sets
	//! Get a reference to the image sets map
	const ImageSetMap& imageSets() const { return imageSets_; }

	//! Get an image set from its id
	ImageSetPtr imageSet(QString id) { return imageSets_.value(id, ImageSetPtr()); }

	//! \copydoc imageSet(QString)
	const ImageSetPtr imageSet(QString id) const { return imageSets_.value(id, ImageSetPtr()); }

	//! Add an image set to the project
	void addImageSet(ImageSetPtr imageSet);

	//! Remove an image set from the project
	void removeImageSet(ImageSetPtr imageSet);

public:
	//! \todo find a way to remove the non-const version of the below
	FeatureDatabase& features() { return featuresDB_; }
	const FeatureDatabase& features() const { return featuresDB_; }

	//! Get an XML-ized version of the project
	QDomDocument *toXML();

signals:
	void cameraAdded(CameraPtr camera);
	void cameraRemoved(CameraPtr camera);
	void imageSetAdded(ImageSetPtr imageSet);
	void imageSetRemoved(ImageSetPtr imageSet);

private:
	QDir projectPath_;             //!< Path to the project
	CameraMap cameras_;            //!< Map of cameras
	ImageSetMap imageSets_;        //!< Map of image sets
	FeatureDatabase featuresDB_;   //!< Feature database
};

#endif // PROJECT_H
