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
#include "calibrationsetupdialog.hpp"
#include "ui_calibrationsetupdialog.h"

#include <QApplication>
#include <QPushButton>
#include <QSet>

#include "project/project.hpp"
#include "project/camera.hpp"
#include "project/imageset.hpp"
#include "project/projectimage.hpp"
#include "stereo/calibrate.hpp"

//---------------------------------------------------------------------

CalibrationSetupDialog::CalibrationSetupDialog(ProjectPtr project, QWidget *parent)
    : QDialog(parent)
    , ui(new Ui::CalibrationSetupDialog)
	, project(project)
{
    ui->setupUi(this);

	foreach(CameraPtr camera, project->cameras()) {
		QListWidgetItem *item = new QListWidgetItem(camera->name(), ui->camerasList);
		item->setFlags(item->flags() | Qt::ItemIsUserCheckable);
		item->setCheckState(Qt::Checked);
		item->setData(Qt::UserRole, camera->id());
	}

	foreach(ImageSetPtr imageSet, project->imageSets()) {
		QListWidgetItem *item = new QListWidgetItem(imageSet->name(), ui->imageSetsList);
		item->setFlags(item->flags() | Qt::ItemIsUserCheckable);
		item->setCheckState(Qt::Checked);
		item->setData(Qt::UserRole, imageSet->id());
	}

	ui->buttonBox->button(QDialogButtonBox::Ok)->setText(tr("Calibrate"));
	on_camerasList_itemChanged(nullptr);
}

CalibrationSetupDialog::~CalibrationSetupDialog() {
    delete ui;
}

//---------------------------------------------------------------------

void CalibrationSetupDialog::accept() {
	QDialog::accept();

	// Pull cameras and image sets from list
	std::vector<CameraPtr> cameras;
	for(int index = 0; index < ui->camerasList->count(); ++index) {
		QListWidgetItem *item = ui->camerasList->item(index);
		if(item->checkState() == Qt::Checked) {
			QString id = item->data(Qt::UserRole).toString();
			if(CameraPtr cam = project->camera(id))
				cameras.push_back(cam);
		}
	}

	std::vector<ImageSetPtr> imageSets;
	for(int index = 0; index < ui->imageSetsList->count(); ++index) {
		QListWidgetItem *item = ui->imageSetsList->item(index);
		if((item->flags() & Qt::ItemIsEnabled) && item->checkState() == Qt::Checked) {
			QString id = item->data(Qt::UserRole).toString();
			if(ImageSetPtr imageSet = project->imageSet(id))
				imageSets.push_back(imageSet);
		}
	}

	// Create background task
	std::shared_ptr<CameraCalibration> task(new CameraCalibration(project, cameras, imageSets));
    qApp->notify(parent(), new NewTaskEvent(task));
}

//---------------------------------------------------------------------

void CalibrationSetupDialog::on_camerasList_itemChanged(QListWidgetItem *) {
	QSet<QString> selectedCameras;
	for(int index = 0; index < ui->camerasList->count(); ++index) {
		QListWidgetItem *item = ui->camerasList->item(index);
		if(item->checkState() == Qt::Checked)
			selectedCameras << item->data(Qt::UserRole).toString();
	}

	// Only enable image sets that reference at least two selected cameras
	for(int index = 0; index < ui->imageSetsList->count(); ++index) {
		QListWidgetItem *item = ui->imageSetsList->item(index);
		QString id = item->data(Qt::UserRole).toString();
		if(ImageSetPtr imageSet = project->imageSet(id)) {
			int count = 0;
			foreach(ProjectImagePtr img, imageSet->images()) {
				if(selectedCameras.contains(img->camera()->id()))
					++count;
			}

			//if(count < 2)
			if(count < 1)
				item->setFlags(item->flags() & ~Qt::ItemIsEnabled);
			else
				item->setFlags(item->flags() | Qt::ItemIsEnabled);
		}
	}
}

//---------------------------------------------------------------------

void CalibrationSetupDialog::changeEvent(QEvent *e) {
    QDialog::changeEvent(e);
    switch (e->type()) {
    case QEvent::LanguageChange:
		ui->buttonBox->button(QDialogButtonBox::Ok)->setText(tr("Calibrate"));
        ui->retranslateUi(this);
        break;
    default:
        break;
    }
}

//---------------------------------------------------------------------

