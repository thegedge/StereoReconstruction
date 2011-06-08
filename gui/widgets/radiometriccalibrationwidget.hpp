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
#ifndef RADIOMETRICCALIBRATIONWIDGET_HPP
#define RADIOMETRICCALIBRATIONWIDGET_HPP

#include <QDialog>

#include "util/c++0x.hpp"

//
// Forward declarations
//
namespace Ui { class RadiometricCalibrationWidget; }

class QAbstractButton;
class QGraphicsItemGroup;
class QResizeEvent;
class Task;

FORWARD_DECLARE(Project);
FORWARD_DECLARE(Camera);
FORWARD_DECLARE(ImageSet);

//! A widget for performing radiometric calibration on a Camera
class RadiometricCalibrationWidget : public QWidget {
    Q_OBJECT

public:
    RadiometricCalibrationWidget(QWidget *parent = 0);
    ~RadiometricCalibrationWidget();

protected:
    void changeEvent(QEvent *e);
	void resizeEvent(QResizeEvent *);

public slots:
	void calibrate();

	void setCamera(CameraPtr camera);
	void setImageSet(ImageSetPtr imageSet);
	void setProject(ProjectPtr project);

private slots:
	void on_checkBlue_toggled(bool checked);
	void on_checkGreen_toggled(bool checked);
	void on_checkRed_toggled(bool checked);

	void on_doCalibration_clicked();
	void updateResponseCurve(const Task *task = nullptr);

private:
    Ui::RadiometricCalibrationWidget *ui;

	ProjectPtr project;
	CameraPtr camera;
	ImageSetPtr imageSet;

	QGraphicsItemGroup *curves[3];
};

#endif // PHOTOCALIBDIALOG_HPP
