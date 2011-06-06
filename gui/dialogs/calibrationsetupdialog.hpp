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
#ifndef CALIBRATIONSETUPDIALOG_HPP
#define CALIBRATIONSETUPDIALOG_HPP

#include <QDialog>
#include "util/c++0x.hpp"

//
// Forward declarations
//
namespace Ui {
    class CalibrationSetupDialog;
}

class QListWidgetItem;
FORWARD_DECLARE(Project);

/*!
 * A dialog that allows the user to select cameras and image sets to use
 * for camera calibration.
 */
class CalibrationSetupDialog : public QDialog {
    Q_OBJECT

public:
    CalibrationSetupDialog(ProjectPtr project, QWidget *parent = 0);
	~CalibrationSetupDialog();

public slots:
	void accept();

protected:
	void changeEvent(QEvent *);

protected slots:
	void on_camerasList_itemChanged(QListWidgetItem *);

private:
    Ui::CalibrationSetupDialog *ui;
	ProjectPtr project;
};

#endif // CALIBRATIONSETUPDIALOG_HPP
