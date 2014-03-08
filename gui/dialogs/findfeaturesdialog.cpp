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
#include "findfeaturesdialog.hpp"
#include "ui_findfeaturesdialog.h"

#include <QApplication>
#include <QPushButton>
#include <QSet>

#include "features/detector.hpp"
#include "features/checkerboard.hpp"
#include "features/surf.hpp"
#include "features/findfeaturestask.hpp"

#include "project/project.hpp"
#include "project/camera.hpp"
#include "project/imageset.hpp"
#include "project/projectimage.hpp"

//---------------------------------------------------------------------

FindFeaturesDialog::FindFeaturesDialog(ProjectPtr project, QWidget *parent)
    : QDialog(parent)
    , ui(new Ui::FindFeaturesDialog)
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
		item->setCheckState(Qt::Unchecked);
		item->setData(Qt::UserRole, imageSet->id());
	}

	ui->buttonBox->button(QDialogButtonBox::Ok)->setText(tr("Find Features"));
}

FindFeaturesDialog::~FindFeaturesDialog() {
    delete ui;
}

//---------------------------------------------------------------------

void FindFeaturesDialog::accept() {
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

	// TODO allow this to be setup via dialog
	std::shared_ptr<FeatureDetector> detector;
	detector.reset(new CheckerboardDetector(10, 12));

	// Create background task
	std::shared_ptr<FindFeaturesTask> task(new FindFeaturesTask(project, cameras, imageSets, detector));
	qApp->notify(QApplication::activeWindow(), new NewTaskEvent(task));
}

//---------------------------------------------------------------------

void FindFeaturesDialog::changeEvent(QEvent *e) {
    QDialog::changeEvent(e);
    switch (e->type()) {
    case QEvent::LanguageChange:
		ui->buttonBox->button(QDialogButtonBox::Ok)->setText(tr("Find Features"));
        ui->retranslateUi(this);
        break;
    default:
        break;
    }
}

//---------------------------------------------------------------------

