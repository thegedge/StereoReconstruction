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
#ifndef FINDFEATURESDIALOG_HPP
#define FINDFEATURESDIALOG_HPP

#include <QDialog>

//
// Forward declarations
//
namespace Ui {
    class FindFeaturesDialog;
}

class QListWidgetItem;
FORWARD_DECLARE(Project);

/*!
 * A dialog that allows the user to select cameras and image sets to use
 * for finding features.
 */
class FindFeaturesDialog : public QDialog {
    Q_OBJECT

public:
    FindFeaturesDialog(ProjectPtr project, QWidget *parent = 0);
	~FindFeaturesDialog();

public slots:
	void accept();

protected:
	void changeEvent(QEvent *);

private:
    Ui::FindFeaturesDialog *ui;
	ProjectPtr project;
};

#endif // FINDFEATURESDIALOG_HPP
