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
#ifndef PROJECTEXPLORER_H
#define PROJECTEXPLORER_H

#include <QWidget>


//
//
//
namespace Ui { class ProjectExplorer; }

class QTreeWidgetItem;

FORWARD_DECLARE(Project);
FORWARD_DECLARE(Camera);
FORWARD_DECLARE(ImageSet);
FORWARD_DECLARE(ProjectImage);

//! A widget to show the cameras and images in a Project
class ProjectExplorer : public QWidget {
    Q_OBJECT

public:
    ProjectExplorer(QWidget *parent = 0);
    ~ProjectExplorer();

	CameraPtr selectedCamera() const;
	ImageSetPtr selectedImageSet() const;
	ProjectImagePtr selectedImage() const;

	void editCamera(CameraPtr cam);
	void editImageSet(ImageSetPtr imageSet);
	void editImage(ProjectImagePtr image);

signals:
	void cameraSelected(CameraPtr cam);
	void imageSetSelected(ImageSetPtr imageSet);
	void imageSelected(ProjectImagePtr imageSet);

public slots:
	void setProject(ProjectPtr project);
	void addCamera(CameraPtr cam);
	void removeCamera(CameraPtr cam);
	void addImageSet(ImageSetPtr imageSet);
	void removeImageSet(ImageSetPtr imageSet);
	void addImage(ProjectImagePtr image);
	void removeImage(ProjectImagePtr image);

private slots:
	void forwardPopupMenu(const QPoint &);
	void projectTreeSelectionChanged();
	void on_treeWidget_itemChanged(QTreeWidgetItem *item, int column);

private:
	QTreeWidgetItem * itemForCamera(CameraPtr cam);
	QTreeWidgetItem * itemForImageSet(ImageSetPtr imageSet);
	QTreeWidgetItem * itemForImage(ProjectImagePtr image);

private:
	Ui::ProjectExplorer *ui;

	QTreeWidgetItem *camerasItem;
	QTreeWidgetItem *imageSetsItem;

	ProjectPtr project;
};


#endif // PROJECTEXPLORER_H
