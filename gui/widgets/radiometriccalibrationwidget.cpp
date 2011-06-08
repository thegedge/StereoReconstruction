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
#include "radiometriccalibrationwidget.hpp"
#include "ui_radiometriccalibrationwidget.h"

#include <QDebug>
#include <QGraphicsItem>
#include <QGraphicsEllipseItem>
#include <QInputDialog>
#include <QLabel>

#include "hdr/radiometriccalibrationtask.hpp"
#include "gui/dialogs/progressdialog.hpp"
#include "project/project.hpp"
#include "project/imageset.hpp"
#include "project/camera.hpp"

//---------------------------------------------------------------------

const int THUMBNAIL_COLUMN = 0;
const int FNAME_COLUMN = 1;
const int EXPOSURE_COLUMN = 2;

RadiometricCalibrationWidget::RadiometricCalibrationWidget(QWidget *parent)
	: QWidget(parent)
    , ui(new Ui::RadiometricCalibrationWidget)
{
	ui->setupUi(this);
	curves[0] = curves[1] = curves[2] = 0;
}

RadiometricCalibrationWidget::~RadiometricCalibrationWidget() {
    delete ui;
}

//---------------------------------------------------------------------

void RadiometricCalibrationWidget::updateResponseCurve(const Task *task) {
	if(!camera || (task && task->isCancelled()))
		return;

	QGraphicsScene *scene = ui->graphicsView->scene();
	if(!scene) {
		scene = new QGraphicsScene(this);
		ui->graphicsView->setScene(scene);
	}
	scene->clear();

	//
	const Responses &response = camera->response();
	if(response.size() < 256)
		return;

	//
	const Qt::GlobalColor colors[3] = {Qt::red, Qt::green, Qt::blue};
	for(size_t color = 0; color < 3; ++color) {
		//
		double miny = 100000000;
		double maxy = -100000000;
		for(int i = 0; i < 256; ++i) {
			miny = qMin(miny, response[i][color]);
			maxy = qMax(maxy, response[i][color]);
		}

		//
		QList<QGraphicsItem *> items;

		QBrush brush(colors[color]);
		QPen pen(colors[color], 0.5, Qt::DotLine);
		QPen darkPen(pen.color().darker());

		double sy = 255.0 / (maxy - miny);
		for(int i = 0; i < 256; ++i) {
			double y = sy*(response[i][color] - miny);

			//
			QGraphicsEllipseItem *item = scene->addEllipse(i - 1.0, y - 1.0, 2.0, 2.0, darkPen, brush);
			item->setZValue(5.0);
			item->setToolTip(QString::number(response[i][color], 'f', 2));
			item->setCursor(QCursor(Qt::PointingHandCursor));

			//
			items << item;
			if(i > 0)
				items << scene->addLine(i - 1, sy*(response[i-1][color] - miny), i, y, pen);
		}

		curves[color] = scene->createItemGroup(items);
	}

	if(!ui->checkRed->isChecked()) curves[0]->setVisible(false);
	if(!ui->checkGreen->isChecked()) curves[1]->setVisible(false);
	if(!ui->checkBlue->isChecked()) curves[2]->setVisible(false);

	// Positive y-axis goes upwards
	ui->graphicsView->setTransform(QTransform(1, 0, 0, -1, 0, 0));
	ui->graphicsView->fitInView(scene->sceneRect(), Qt::KeepAspectRatio);
}

//---------------------------------------------------------------------

void RadiometricCalibrationWidget::calibrate() {
	std::shared_ptr<RadiometricCalibrationTask> calibrationTask(
			new RadiometricCalibrationTask(camera, imageSet) );

	connect(calibrationTask.get(),
			SIGNAL(finished(const Task *)),
			SLOT(updateResponseCurve(const Task *)));

	qApp->notify(QApplication::activeWindow(), new NewTaskEvent(calibrationTask));
}

//---------------------------------------------------------------------

void RadiometricCalibrationWidget::on_doCalibration_clicked() {
	//
	QList<ImageSetPtr> imageSets;
	QStringList imageSetNames;
	foreach(ImageSetPtr imageSet, project->imageSets()) {
		imageSets << imageSet;
		imageSetNames << imageSet->name();
	}

	//
	bool ok;
	QString result = QInputDialog::getItem(
			0,
			tr("Select Image Set"),
			tr("Select the image set to use for radiometric calibration:"),
			imageSetNames,
			0,
			false,
			&ok);

	if(ok) {
		int index = imageSetNames.indexOf(result);
		imageSet = imageSets[index];
		if(imageSet)
			calibrate();
	}
}

//---------------------------------------------------------------------

void RadiometricCalibrationWidget::setProject(ProjectPtr project) {
	this->project = project;
	ui->doCalibration->setEnabled(project.get() != 0);
}

void RadiometricCalibrationWidget::setCamera(CameraPtr camera) {
	this->camera = camera;
	updateResponseCurve();
}

void RadiometricCalibrationWidget::setImageSet(ImageSetPtr imageSet) {
	this->imageSet = imageSet;
}

//---------------------------------------------------------------------

void RadiometricCalibrationWidget::changeEvent(QEvent *e) {
    QWidget::changeEvent(e);
    switch (e->type()) {
    case QEvent::LanguageChange:
        ui->retranslateUi(this);
        break;
    default:
        break;
    }
}

//---------------------------------------------------------------------

void RadiometricCalibrationWidget::resizeEvent(QResizeEvent *) {
	ui->graphicsView->fitInView(ui->graphicsView->sceneRect(), Qt::KeepAspectRatio);
}

//---------------------------------------------------------------------

void RadiometricCalibrationWidget::on_checkRed_toggled(bool checked) {
	if(curves[0]) {
		curves[0]->setVisible(checked);
		ui->graphicsView->fitInView(ui->graphicsView->sceneRect(), Qt::KeepAspectRatio);
	}
}

void RadiometricCalibrationWidget::on_checkGreen_toggled(bool checked) {
	if(curves[1]) {
		curves[1]->setVisible(checked);
		ui->graphicsView->fitInView(ui->graphicsView->sceneRect(), Qt::KeepAspectRatio);
	}
}

void RadiometricCalibrationWidget::on_checkBlue_toggled(bool checked) {
	if(curves[2]) {
		curves[2]->setVisible(checked);
		ui->graphicsView->fitInView(ui->graphicsView->sceneRect(), Qt::KeepAspectRatio);
	}
}

//---------------------------------------------------------------------
