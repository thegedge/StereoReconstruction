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
#include "projectexplorer.hpp"
#include "ui_projectexplorer.h"

#include "gui/scene/capturedimagesscene.hpp"
#include "project/project.hpp"
#include "project/camera.hpp"
#include "project/imageset.hpp"
#include "project/projectimage.hpp"

#include <QDebug>

//---------------------------------------------------------------------
namespace {
	//! Check if \a item is a \em strict descendant of \a ancestor
	bool isAncestorOf(QTreeWidgetItem *item, QTreeWidgetItem *ancestor) {
		if(item == ancestor) // has to be strictly an ancestor
			return false;

		while(item && item != ancestor)
			item = item->parent();

		return (item == ancestor);
	}

	//! Remove all of the children from a tree widget item
	void removeAllChildren(QTreeWidgetItem *item) {
		while(item->childCount() > 0)
			item->removeChild(item->child(0));
	}
}
//---------------------------------------------------------------------
//
ProjectExplorer::ProjectExplorer(QWidget *parent)
#if defined(PLATFORM_MAC) && QT_VERSION < 0x040700 // XXX crashes in 4.7
    : QWidget(parent, Qt::Drawer)
#else
    : QWidget(parent, Qt::Tool)
#endif
    , ui(new Ui::ProjectExplorer)
	, camerasItem(new QTreeWidgetItem)
	, imageSetsItem(new QTreeWidgetItem)
{
    ui->setupUi(this);

	//
	QFont boldFont = ui->treeWidget->font();
	boldFont.setBold(true);

	//
	camerasItem->setText(0, tr("Cameras"));
	camerasItem->setFont(0, boldFont);

	imageSetsItem->setText(0, tr("Image Sets"));
	imageSetsItem->setFont(0, boldFont);

	//
	ui->treeWidget->addTopLevelItem(camerasItem);
	ui->treeWidget->addTopLevelItem(imageSetsItem);

	//
	connect(ui->treeWidget, SIGNAL(itemSelectionChanged()), SLOT(projectTreeSelectionChanged()) );
	connect(ui->treeWidget, SIGNAL(customContextMenuRequested(QPoint)), SLOT(forwardPopupMenu(QPoint)));
}

ProjectExplorer::~ProjectExplorer() {
    delete ui;
}

//---------------------------------------------------------------------

void ProjectExplorer::forwardPopupMenu(const QPoint &p) {
	emit customContextMenuRequested(ui->treeWidget->mapToParent(p));
}

//---------------------------------------------------------------------

void ProjectExplorer::setProject(ProjectPtr project) {
	//
	removeAllChildren(camerasItem);
	removeAllChildren(imageSetsItem);

	// Add cameras
	foreach(CameraPtr cam, project->cameras()) {
		QTreeWidgetItem *cameraItem = new QTreeWidgetItem;
		QString tooltip = QString("Name: %1\nId: %2").arg(cam->name(), cam->id());
		cameraItem->setText(0, cam->name());
		cameraItem->setFlags(cameraItem->flags() | Qt::ItemIsEditable);
		cameraItem->setData(0, Qt::UserRole, cam->id());
		cameraItem->setToolTip(0, tooltip);
		camerasItem->addChild(cameraItem);
	}

	// Add image sets
	foreach(ImageSetPtr imageSet, project->imageSets()) {
		QTreeWidgetItem *imageSetItem = new QTreeWidgetItem;
		imageSetItem->setText(0, imageSet->name());
		imageSetItem->setFlags(imageSetItem->flags() | Qt::ItemIsEditable);
		imageSetItem->setData(0, Qt::UserRole, imageSet->id());
		imageSetItem->setToolTip(0,
			 QString("Name: %1\nRoot: %2")
				 .arg(imageSet->name())
				 .arg(imageSet->root().absolutePath()) );
		imageSetsItem->addChild(imageSetItem);

		// Add images
		foreach(ProjectImagePtr image, imageSet->images()) {
			QString camera = tr("<no camera>");
			if(image->camera())
				camera = image->camera()->name();

			QTreeWidgetItem *imageItem = new QTreeWidgetItem;
			imageItem->setText(0, imageSet->root().relativeFilePath(image->file().absoluteFilePath()));
			imageItem->setToolTip(0,
				QString("Path: %1\nCamera: %2\nExposure: %3")
					.arg(image->file().absoluteFilePath(), camera)
					.arg(image->exposure(), 0, 'f', 2));
			imageSetItem->addChild(imageItem);
		}
	}

	// Initially start expanded
	camerasItem->setExpanded(true);
	imageSetsItem->setExpanded(true);

	//
	this->project = project;
}

//---------------------------------------------------------------------

void ProjectExplorer::editCamera(CameraPtr cam) {
	for(int index = 0; index < camerasItem->childCount(); ++index) {
		QTreeWidgetItem *item = camerasItem->child(index);
		if(item->data(0, Qt::UserRole).toString() == cam->id()) {
			ui->treeWidget->editItem(item);
			break;
		}
	}
}

void ProjectExplorer::editImageSet(ImageSetPtr imageSet) {
	for(int index = 0; index < imageSetsItem->childCount(); ++index) {
		QTreeWidgetItem *item = imageSetsItem->child(index);
		if(item->data(0, Qt::UserRole).toString() == imageSet->id()) {
			ui->treeWidget->editItem(item);
			break;
		}
	}
}

//---------------------------------------------------------------------

void ProjectExplorer::on_treeWidget_itemChanged(QTreeWidgetItem *item, int column) {
	if(project) {
		if(::isAncestorOf(item, camerasItem)) {
			if(CameraPtr cam = project->camera(item->data(0, Qt::UserRole).toString()))
				cam->setName(item->text(column));
		} else if(::isAncestorOf(item, imageSetsItem)) {
			if(ImageSetPtr imageSet = project->imageSet(item->data(0, Qt::UserRole).toString()))
				imageSet->setName(item->text(column));
		}
	}
}

//---------------------------------------------------------------------

CameraPtr ProjectExplorer::selectedCamera() const {
	if(project && ui->treeWidget->selectedItems().count() > 0) {
		QTreeWidgetItem *selected = ui->treeWidget->selectedItems().at(0);
		if(::isAncestorOf(selected, camerasItem))
			return project->camera(selected->data(0, Qt::UserRole).toString());
	}
	return CameraPtr();
}

//---------------------------------------------------------------------

ImageSetPtr ProjectExplorer::selectedImageSet() const {
	if(project && ui->treeWidget->selectedItems().count() > 0) {
		QTreeWidgetItem *selected = ui->treeWidget->selectedItems().at(0);
		if(::isAncestorOf(selected->parent(), imageSetsItem)) // image selected, not image set
			selected = selected->parent();

		if(::isAncestorOf(selected, imageSetsItem))
			return project->imageSet(selected->data(0, Qt::UserRole).toString());
	}
	return ImageSetPtr();
}

//---------------------------------------------------------------------

ProjectImagePtr ProjectExplorer::selectedImage() const {
	if(project && ui->treeWidget->selectedItems().count() > 0) {
		QTreeWidgetItem *selected = ui->treeWidget->selectedItems().at(0);
		if(::isAncestorOf(selected->parent(), imageSetsItem)) {
			int index = selected->parent()->indexOfChild(selected);
			ImageSetPtr imageSet = project->imageSet(selected->parent()->data(0, Qt::UserRole).toString());
			return imageSet->images()[index];
		}
	}
	return ProjectImagePtr();
}

//---------------------------------------------------------------------

void ProjectExplorer::projectTreeSelectionChanged() {
	if(project && ui->treeWidget->selectedItems().count() > 0) {
		QTreeWidgetItem *selected = ui->treeWidget->selectedItems().at(0);
		if(::isAncestorOf(selected, camerasItem)) {
			int index = camerasItem->indexOfChild(selected);
			emit cameraSelected(index, project->camera(selected->data(0, Qt::UserRole).toString()));
		} else if(::isAncestorOf(selected, imageSetsItem)) {
			int index = imageSetsItem->indexOfChild(selected);
			if(index < 0) {
				selected = selected->parent();
				index = imageSetsItem->indexOfChild(selected);
			}

			emit imageSetSelected(index, project->imageSet(selected->data(0, Qt::UserRole).toString()));
		}
	}
}

//---------------------------------------------------------------------
