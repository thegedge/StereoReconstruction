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
#ifndef TASKPROGRESSWIDGET_HPP
#define TASKPROGRESSWIDGET_HPP

#include <QString>
#include <QWidget>

//
// Forward declarations
//
namespace Ui {
    class TaskProgressWidget;
}

FORWARD_DECLARE(Task);

//! A widget to show progress info about a \a Task.
class TaskProgressWidget : public QWidget {
    Q_OBJECT

public:
    TaskProgressWidget(TaskPtr task, QWidget *parent = 0);

	TaskPtr task() const { return task_; }

private slots:
	//! .
	void on_cancelButton_clicked();

	//! .
	void updateProgress(int progress);

	//! .
	void updateStage(QString stage)	;

private:
    Ui::TaskProgressWidget *ui;
	TaskPtr task_;
};

#endif // TASKPROGRESSWIDGET_HPP
