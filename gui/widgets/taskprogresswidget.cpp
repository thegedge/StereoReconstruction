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
#include "taskprogresswidget.hpp"
#include "ui_taskprogresswidget.h"
#include "gui/task.hpp"

//---------------------------------------------------------------------

TaskProgressWidget::TaskProgressWidget(TaskPtr task, QWidget *parent)
	: QWidget(parent)
	, ui(new Ui::TaskProgressWidget)
	, task_(task)
{
	ui->setupUi(this);
	ui->cancelButton->setIcon(style()->standardIcon(QStyle::SP_BrowserStop));
	ui->cancelButton->setVisible(false);
	ui->progressBar->setRange(0, 0); // initially just a busy indicator

	if(task) {
		ui->cancelButton->setVisible(true);
		ui->progressBar->setMaximum(task->numSteps());
		connect(task.get(), SIGNAL(progressUpdate(int)), SLOT(updateProgress(int)));
		connect(task.get(), SIGNAL(stageUpdate(QString)), SLOT(updateStage(QString)));
	}
}

//---------------------------------------------------------------------

void TaskProgressWidget::on_cancelButton_clicked() {
	if(task_) {
		task_->disconnect(this, SLOT(updateProgress(int)));
		task_->disconnect(this, SLOT(updateStage(QString)));
		task_->cancel();
	}
	ui->progressBar->setRange(0, 0); // busy indicator instead of progress
	ui->labelStage->setText(tr("Cancelling..."));
}

//---------------------------------------------------------------------

void TaskProgressWidget::updateProgress(int progress) {
	// XXX If this function does nothing else, we can get rid of it and
	//     connect the tasks progressUpdate signal to the progress bar's
	//     setValue slot instead.
	ui->progressBar->setValue(progress);
}

//---------------------------------------------------------------------

void TaskProgressWidget::updateStage(QString stage) {
	// XXX Similar to updateProgress (see comment above)
	ui->labelStage->setText(stage);
	ui->labelStage->setToolTip(stage);
}

//---------------------------------------------------------------------
