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
#include "gui/widgets/projectitem.hpp"
#include "project/project.hpp"
#include "project/camera.hpp"
#include "project/imageset.hpp"
#include "project/projectimage.hpp"

//---------------------------------------------------------------------

Q_DECLARE_METATYPE(ProjectImage *);

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

	//! Expands all parents of the given item so it becomes "exposed"
	void exposeItem(QTreeWidgetItem *item) {
		if(item) {
			while((item = item->parent()))
				item->setExpanded(true);
		}
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
	camerasItem->setExpanded(true);
	ui->treeWidget->addTopLevelItem(camerasItem);

	imageSetsItem->setText(0, tr("Image Sets"));
	imageSetsItem->setFont(0, boldFont);
	imageSetsItem->setExpanded(true);
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
	// Disconnect slots
	if(this->project)
		this->project->disconnect(this);

	//
	removeAllChildren(camerasItem);
	removeAllChildren(imageSetsItem);

	// Add stuff
	if(project) {
		foreach(CameraPtr cam, project->cameras())
			camerasItem->addChild(new ProjectItem(cam.get()));

		foreach(ImageSetPtr imageSet, project->imageSets()) {
			ProjectItem *imageSetItem = new ProjectItem(imageSet.get());
			imageSetsItem->addChild(imageSetItem);
			foreach(ProjectImagePtr image, imageSet->images())
				imageSetItem->addChild(new ProjectItem(image.get()));
		}

		connect(project.get(), SIGNAL(cameraAdded(CameraPtr)), SLOT(addCamera(CameraPtr)));
		connect(project.get(), SIGNAL(cameraRemoved(CameraPtr)), SLOT(removeCamera(CameraPtr)));
		connect(project.get(), SIGNAL(imageSetAdded(ImageSetPtr)), SLOT(addImageSet(ImageSetPtr)));
		connect(project.get(), SIGNAL(imageSetRemoved(ImageSetPtr)), SLOT(removeImageSet(ImageSetPtr)));
	}

	//
	this->project = project;
}

//---------------------------------------------------------------------

void ProjectExplorer::addCamera(CameraPtr cam) {
	camerasItem->addChild(new ProjectItem(cam.get()));
}

void ProjectExplorer::removeCamera(CameraPtr cam) {
	for(int index = 0; index < camerasItem->childCount(); ++index) {
		QTreeWidgetItem *item = camerasItem->child(index);
		if(item->data(0, Qt::UserRole).toString() == cam->id()) {
			camerasItem->removeChild(item);
			return;
		}
	}
}

void ProjectExplorer::addImageSet(ImageSetPtr imageSet) {
	imageSetsItem->addChild(new ProjectItem(imageSet.get()));

	// XXX Move to ProjectItem instead?
	connect(imageSet.get(), SIGNAL(imageAdded(ProjectImagePtr)), SLOT(addImage(ProjectImagePtr)));
	connect(imageSet.get(), SIGNAL(imageRemoved(ProjectImagePtr)), SLOT(removeImage(ProjectImagePtr)));
}

void ProjectExplorer::removeImageSet(ImageSetPtr imageSet) {
	for(int index = 0; index < imageSetsItem->childCount(); ++index) {
		QTreeWidgetItem *item = imageSetsItem->child(index);
		if(item->data(0, Qt::UserRole).toString() == imageSet->id()) {
			imageSetsItem->removeChild(item);
			imageSet->disconnect(this);
			return;
		}
	}
}

//---------------------------------------------------------------------

void ProjectExplorer::addImage(ProjectImagePtr image) {
	ImageSetPtr imageSet = image->imageSet();
	for(int index = 0; index < imageSetsItem->childCount(); ++index) {
		QTreeWidgetItem *item = imageSetsItem->child(index);
		if(item->data(0, Qt::UserRole).toString() == imageSet->id()) {
			item->addChild(new ProjectItem(image.get()));
			return;
		}
	}
}

void ProjectExplorer::removeImage(ProjectImagePtr image) {
	ImageSetPtr imageSet = image->imageSet();
	for(int index = 0; index < imageSetsItem->childCount(); ++index) {
		QTreeWidgetItem *item = imageSetsItem->child(index);
		if(item->data(0, Qt::UserRole).toString() == imageSet->id()) {
			for(int index = 0; index < item->childCount(); ++index) {
				QTreeWidgetItem *item2 = item->child(index);
				if(item2->data(0, Qt::UserRole).value<QObject *>() == image.get()) {
					item->removeChild(item2);
					return;
				}
			}
		}
	}
}

//---------------------------------------------------------------------

void ProjectExplorer::editCamera(CameraPtr cam) {
	if(QTreeWidgetItem *item = itemForCamera(cam)) {
		exposeItem(item);
		ui->treeWidget->selectionModel()->clear();
		item->setSelected(true);
		ui->treeWidget->editItem(item);
	}
}

void ProjectExplorer::editImageSet(ImageSetPtr imageSet) {
	if(QTreeWidgetItem *item = itemForImageSet(imageSet)) {
		exposeItem(item);
		ui->treeWidget->selectionModel()->clear();
		item->setSelected(true);
		ui->treeWidget->editItem(item);
	}
}

void ProjectExplorer::editImage(ProjectImagePtr image) {
	if(QTreeWidgetItem *item = itemForImage(image)) {
		exposeItem(item);
		ui->treeWidget->selectionModel()->clear();
		item->setSelected(true);
		ui->treeWidget->expandItem(item);
	}
}

//---------------------------------------------------------------------

QTreeWidgetItem * ProjectExplorer::itemForCamera(CameraPtr cam) {
	for(int index = 0; index < camerasItem->childCount(); ++index) {
		QTreeWidgetItem *item = camerasItem->child(index);
		if(item->data(0, Qt::UserRole).toString() == cam->id())
			return item;
	}
	return nullptr;
}

QTreeWidgetItem * ProjectExplorer::itemForImageSet(ImageSetPtr imageSet) {
	for(int index = 0; index < imageSetsItem->childCount(); ++index) {
		QTreeWidgetItem *item = imageSetsItem->child(index);
		if(item->data(0, Qt::UserRole).toString() == imageSet->id())
			return item;
	}
	return nullptr;
}

QTreeWidgetItem * ProjectExplorer::itemForImage(ProjectImagePtr image) {
	ImageSetPtr imageSet = image->imageSet();
	for(int index = 0; index < imageSetsItem->childCount(); ++index) {
		QTreeWidgetItem *item = imageSetsItem->child(index);
		if(item->data(0, Qt::UserRole).toString() == imageSet->id()) {
			for(int index = 0; index < item->childCount(); ++index) {
				QTreeWidgetItem *item2 = item->child(index);
				if(item2->data(0, Qt::UserRole).value<ProjectImage *>() == image.get())
					return item2;
			}
		}
	}
	return nullptr;
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
			emit cameraSelected(project->camera(selected->data(0, Qt::UserRole).toString()));
		} else if(::isAncestorOf(selected->parent(), imageSetsItem)) {
			//
			QTreeWidgetItem *parent = selected->parent();
			ImageSetPtr imageSet = project->imageSet(parent->data(0, Qt::UserRole).toString());
			emit imageSetSelected(imageSet);

			// XXX Could be dangerous if view and model indices don't line up
			int index = parent->indexOfChild(selected);
			emit imageSelected(imageSet->images()[index]);
		} else if(::isAncestorOf(selected, imageSetsItem)) {
			emit imageSetSelected(project->imageSet(selected->data(0, Qt::UserRole).toString()));
		}
	}
}

//---------------------------------------------------------------------
