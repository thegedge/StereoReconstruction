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
#include "imagesettable.hpp"

#include "gui/comboboxitemdelegate.hpp"
#include "gui/loadimagesthread.hpp"
#include "project/project.hpp"
#include "project/camera.hpp"
#include "project/imageset.hpp"
#include "project/projectimage.hpp"

#include <QComboBox>
#include <QDebug>
#include <QEvent>
#include <QHeaderView>
#include <QImage>
#include <QPixmap>
#include <QLabel>
#include <QList>
#include <QStringList>
#include <QStyledItemDelegate>
#include <QThread>

//---------------------------------------------------------------------
namespace {
	const int THUMBNAIL_COLUMN = 0;
	const int FNAME_COLUMN     = 1;
	const int CAMERA_COLUMN    = 2;
	const int EXPOSURE_COLUMN  = 3;
}

//---------------------------------------------------------------------

ImageSetTable::ImageSetTable(QWidget *parent)
    : QTableWidget(0, 4, parent)
	, loadImages(nullptr)
{
	setColumnWidth(THUMBNAIL_COLUMN, 110);
	setColumnWidth(CAMERA_COLUMN, 75);
	setColumnWidth(FNAME_COLUMN, 150);

	setShowGrid(true);
	setSortingEnabled(true);
	setAlternatingRowColors(true);
	setEditTriggers(AllEditTriggers);
	setSelectionBehavior(SelectRows);
	setCornerButtonEnabled(false);

	horizontalHeader()->setCascadingSectionResizes(true);
	horizontalHeader()->setStretchLastSection(true);
	verticalHeader()->setVisible(false);

	for(int i = 0; i <= 3; ++i)
		setHorizontalHeaderItem(i, new QTableWidgetItem);

	QEvent translateEvent(QEvent::LanguageChange);
	changeEvent(&translateEvent);

	//
	connect(this, SIGNAL(cellChanged(int,int)), SLOT(cellChanged(int,int)));
}

ImageSetTable::~ImageSetTable() {
	delete loadImages;
	loadImages = nullptr;
}

//---------------------------------------------------------------------

void ImageSetTable::setProject(ProjectPtr project) {
	this->project = project;

	// TODO update these when cameras change in project
	QStringList cameraNames, cameraIDs;
	cameraNames << tr("<no camera>");
	cameraIDs << "";

	foreach(CameraPtr cam, project->cameras()) {
		cameraNames << cam->name();
		cameraIDs << cam->id();
	}

	setItemDelegateForColumn(CAMERA_COLUMN, new ComboBoxItemDelegate(cameraNames, cameraIDs));
}

void ImageSetTable::setImageSet(int, ImageSetPtr imageSet) {
	if(project && !imageSet) {
		clearContents();
	} else if(project && imageSet) {
		clearContents();
		if(imageSet) {
			setRowCount(imageSet->images().size());

			// Update table
			for(size_t index = 0; index < imageSet->images().size(); ++index) {
				QString path = imageSet->root().relativeFilePath(imageSet->images()[index]->file().absoluteFilePath());
				QTableWidgetItem *fnameItem = new QTableWidgetItem(path);
				fnameItem->setFlags(fnameItem->flags() & ~Qt::ItemIsEditable);
				fnameItem->setTextAlignment(Qt::AlignCenter);

				CameraPtr cam = imageSet->images()[index]->camera();
				QTableWidgetItem *cameraItem = new QTableWidgetItem(cam ? cam->name() : tr("<no camera>"));
				//cameraItem->setFlags(fnameItem->flags() & ~Qt::ItemIsEditable);
				cameraItem->setTextAlignment(Qt::AlignCenter);

				QTableWidgetItem *exposureItem = new QTableWidgetItem;
				exposureItem->setFlags(fnameItem->flags() & ~Qt::ItemIsEditable);
				exposureItem->setText(QString::number(imageSet->images()[index]->exposure()));
				exposureItem->setTextAlignment(Qt::AlignCenter);

				setItem(index, FNAME_COLUMN, fnameItem);
				setItem(index, CAMERA_COLUMN, cameraItem);
				setItem(index, EXPOSURE_COLUMN, exposureItem);
			}

			// Load images
			QStringList images;
			foreach(ProjectImagePtr image, imageSet->images())
				images << image->file().absoluteFilePath();

			if(loadImages) {
				loadImages->stop();
				loadImages->wait();
				loadImages->deleteLater();
				loadImages = 0;
			}

			if(imageSet->images().size() > 0) {
				loadImages = new LoadImagesThread(images, this);
				connect(loadImages, SIGNAL(imageLoaded(int, QImage)), SLOT(imageLoaded(int, QImage)));
				loadImages->start();
			}
		}
	}

	currentSet = imageSet;
}

//---------------------------------------------------------------------

void ImageSetTable::cellChanged(int row, int column) {
	if(currentSet) {
		bool ok = false;
		switch(column) {
		case CAMERA_COLUMN: {
			QString name = item(row, column)->text().trimmed();
			CameraPtr cam = project->cameraFromName(name);
			if(cam) {
				foreach(const QModelIndex &index, selectedIndexes()) {
					currentSet->images()[index.row()]->camera() = cam;
					item(index.row(), column)->setText(name);
				}
			}
			break;
		}
		case EXPOSURE_COLUMN: {
			QString sExposure = item(row, column)->text();
			double exposure = sExposure.toDouble(&ok);
			if(ok) {
				foreach(const QModelIndex &index, selectedIndexes()) {
					currentSet->images()[index.row()]->exposure() = exposure;
					item(index.row(), column)->setText(sExposure);
				}
			}
			break;
		}
		default:
			break;
		}
	}
}

//---------------------------------------------------------------------

void ImageSetTable::imageLoaded(int index, QImage image) {
	QLabel *label = new QLabel(this);
	label->setAlignment(Qt::AlignCenter);

	if(image.isNull()) {
		label->setText(tr("<No Image>"));
	} else {
		if(image.width() > 200)
			image = image.scaledToWidth(100, Qt::SmoothTransformation);

		label->setPixmap(QPixmap::fromImage(image));
		setRowHeight(index, image.height() + 10);
	}

	setCellWidget(index, THUMBNAIL_COLUMN, label);
}

//---------------------------------------------------------------------

void ImageSetTable::changeEvent(QEvent *e) {
    QWidget::changeEvent(e);
    switch (e->type()) {
    case QEvent::LanguageChange: {
        QTableWidgetItem *item = horizontalHeaderItem(THUMBNAIL_COLUMN);
        item->setText(tr("Image"));
        item = horizontalHeaderItem(FNAME_COLUMN);
        item->setText(tr("Filename"));
        item = horizontalHeaderItem(CAMERA_COLUMN);
        item->setText(tr("Camera"));
        item = horizontalHeaderItem(EXPOSURE_COLUMN);
        item->setText(tr("Exposure Time (ms)"));
        break;
	}
    default:
        break;
    }
}

//---------------------------------------------------------------------
