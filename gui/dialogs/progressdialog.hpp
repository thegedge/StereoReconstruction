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
#ifndef PROGRESSDIALOG_HPP
#define PROGRESSDIALOG_HPP

#include <QDialog>
#include "util/c++0x.hpp"

//
// Forward declarations
//
namespace Ui {
    class ProgressDialog;
}

class QAbstractButton;
FORWARD_DECLARE(Task);

/*!
 * A dialog that attaches to an object which emits progress updates
 * (i.e., implements ProgressObject).
 */
class ProgressDialog : public QDialog {
    Q_OBJECT
public:
    ProgressDialog(TaskPtr task, QWidget *parent = 0);
    ~ProgressDialog();

private slots:
	void setProgress(int progress);
	void setStage(QString string);
	void on_buttonBox_clicked(QAbstractButton *button);

protected:
    void changeEvent(QEvent *e);
	void updateLabel();
	void reject();

private:
	QString stage;

    Ui::ProgressDialog *ui;
};

#endif // PROGRESSDIALOG_HPP
