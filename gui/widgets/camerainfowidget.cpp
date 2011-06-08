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
#include "camerainfowidget.hpp"
#include "ui_camerainfowidget.h"

#include "project/project.hpp"
#include "project/camera.hpp"
#include "radiometriccalibrationwidget.hpp"

#include <QBoxLayout>

//---------------------------------------------------------------------

double toDouble(QString string, double defaultValueIfConversionFails) {
	bool ok = false;
	const double ret = string.toDouble(&ok);
	return (ok ? ret : defaultValueIfConversionFails);
}

//---------------------------------------------------------------------

CameraInfoWidget::CameraInfoWidget(QWidget *parent)
    : QWidget(parent)
    , ui(new Ui::CameraInfoWidget)
{
    ui->setupUi(this);
	ui->toolBox->setCurrentIndex(0);

	QBoxLayout *layout = new QBoxLayout(QBoxLayout::TopToBottom, ui->intrinsicParamsPage);
	layout->addWidget(ui->radialDistortionLabel);
	layout->addWidget(ui->radialDistortionLayout->parentWidget());
	layout->addWidget(ui->intrinsicParamsLabel);
	layout->addWidget(ui->intrinsicParamsLayout->parentWidget());
	layout->addStretch();

	layout = new QBoxLayout(QBoxLayout::TopToBottom, ui->extrinsicParamsPage);
	layout->addWidget(ui->rotationMatrixLabel);
	layout->addWidget(ui->rotationMatrixLayout->parentWidget());
	layout->addWidget(ui->copLabel);
	layout->addWidget(ui->copLayout->parentWidget());
	layout->addStretch();
}

CameraInfoWidget::~CameraInfoWidget() {
    delete ui;
}

//---------------------------------------------------------------------

void CameraInfoWidget::setProject(ProjectPtr project) {
	ui->radiometricCalibrationPage->setProject(project);
}

void CameraInfoWidget::setCamera(CameraPtr camera) {
	currentCamera = camera;
	if(camera) {
		ui->k1->setText(QString::number(camera->lensDistortion()[0], 'f'));
		ui->k2->setText(QString::number(camera->lensDistortion()[1], 'f'));
		ui->p1->setText(QString::number(camera->lensDistortion()[2], 'f'));
		ui->p2->setText(QString::number(camera->lensDistortion()[3], 'f'));
		ui->k3->setText(QString::number(camera->lensDistortion()[4], 'f'));

		ui->intrinsic11->setText(QString::number(camera->K()(0, 0), 'f'));
		ui->intrinsic12->setText(QString::number(camera->K()(0, 1), 'f'));
		ui->intrinsic13->setText(QString::number(camera->K()(0, 2), 'f'));
		ui->intrinsic21->setText(QString::number(camera->K()(1, 0), 'f'));
		ui->intrinsic22->setText(QString::number(camera->K()(1, 1), 'f'));
		ui->intrinsic23->setText(QString::number(camera->K()(1, 2), 'f'));
		ui->intrinsic31->setText(QString::number(camera->K()(2, 0), 'f'));
		ui->intrinsic32->setText(QString::number(camera->K()(2, 1), 'f'));
		ui->intrinsic33->setText(QString::number(camera->K()(2, 2), 'f'));

		ui->r11->setText(QString::number(camera->R()(0, 0), 'f'));
		ui->r12->setText(QString::number(camera->R()(0, 1), 'f'));
		ui->r13->setText(QString::number(camera->R()(0, 2), 'f'));
		ui->r21->setText(QString::number(camera->R()(1, 0), 'f'));
		ui->r22->setText(QString::number(camera->R()(1, 1), 'f'));
		ui->r23->setText(QString::number(camera->R()(1, 2), 'f'));
		ui->r31->setText(QString::number(camera->R()(2, 0), 'f'));
		ui->r32->setText(QString::number(camera->R()(2, 1), 'f'));
		ui->r33->setText(QString::number(camera->R()(2, 2), 'f'));

		ui->copX->setText(QString::number(camera->C().x(), 'f'));
		ui->copY->setText(QString::number(camera->C().y(), 'f'));
		ui->copZ->setText(QString::number(camera->C().z(), 'f'));
	}
	ui->radiometricCalibrationPage->setCamera(camera);
}

//---------------------------------------------------------------------

void CameraInfoWidget::changeEvent(QEvent *e) {
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

void CameraInfoWidget::updateCamera() {
	LensDistortions current = currentCamera->lensDistortion();
	current[0] = toDouble(ui->k1->text(), current[0]);
	current[1] = toDouble(ui->k2->text(), current[1]);
	current[4] = toDouble(ui->k3->text(), current[4]);
	current[2] = toDouble(ui->p1->text(), current[2]);
	current[3] = toDouble(ui->p2->text(), current[3]);
	currentCamera->setLensDistortion(current);

	Eigen::Matrix3d K = currentCamera->K();
	K(0, 0) = toDouble(ui->intrinsic11->text(), K(0, 0));
	K(0, 1) = toDouble(ui->intrinsic12->text(), K(0, 1));
	K(0, 2) = toDouble(ui->intrinsic13->text(), K(0, 2));
	K(1, 0) = toDouble(ui->intrinsic21->text(), K(1, 0));
	K(1, 1) = toDouble(ui->intrinsic22->text(), K(1, 1));
	K(1, 2) = toDouble(ui->intrinsic23->text(), K(1, 2));
	K(2, 0) = toDouble(ui->intrinsic31->text(), K(2, 0));
	K(2, 1) = toDouble(ui->intrinsic32->text(), K(2, 1));
	K(2, 2) = toDouble(ui->intrinsic33->text(), K(2, 2));
	currentCamera->setK(K);

	Eigen::Matrix3d R = currentCamera->R();
	R(0, 0) = toDouble(ui->r11->text(), R(0, 0));
	R(0, 1) = toDouble(ui->r12->text(), R(0, 1));
	R(0, 2) = toDouble(ui->r13->text(), R(0, 2));
	R(1, 0) = toDouble(ui->r21->text(), R(1, 0));
	R(1, 1) = toDouble(ui->r22->text(), R(1, 1));
	R(1, 2) = toDouble(ui->r23->text(), R(1, 2));
	R(2, 0) = toDouble(ui->r31->text(), R(2, 0));
	R(2, 1) = toDouble(ui->r32->text(), R(2, 1));
	R(2, 2) = toDouble(ui->r33->text(), R(2, 2));
	currentCamera->setR(R);

	Eigen::Vector3d C = currentCamera->C();
	K[0] = toDouble(ui->copX->text(), K[0]);
	K[1] = toDouble(ui->copY->text(), K[1]);
	K[2] = toDouble(ui->copZ->text(), K[2]);
	currentCamera->setC(C);
}

//---------------------------------------------------------------------
