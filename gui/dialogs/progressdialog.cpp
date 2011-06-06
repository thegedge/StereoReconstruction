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
#include "progressdialog.hpp"
#include "ui_progressdialog.h"

#include "gui/task.hpp"

//---------------------------------------------------------------------

ProgressDialog::ProgressDialog(TaskPtr task, QWidget *parent)
    : QDialog(parent)
    , ui(new Ui::ProgressDialog)
{
    ui->setupUi(this);

	//
	setWindowTitle(task->title());
	connect(task.get(), SIGNAL(progressUpdate(int)), ui->progressBar, SLOT(setValue(int)));
	connect(task.get(), SIGNAL(progressUpdate(int)), SLOT(setProgress(int)));
	connect(task.get(), SIGNAL(stageUpdate(QString)), SLOT(setStage(QString)));
	connect(task.get(), SIGNAL(destroyed()), SLOT(close()));
	connect(ui->buttonBox, SIGNAL(rejected()), task.get(), SLOT(cancel()));
	connect(ui->buttonBox, SIGNAL(accepted()), SLOT(close()));
}

ProgressDialog::~ProgressDialog() {
    delete ui;
}

//---------------------------------------------------------------------

void ProgressDialog::updateLabel() {
	ui->progressLabel->setText(
			QString("%1...%2%")
				.arg(stage)
				.arg(ui->progressBar->value()) );
}

void ProgressDialog::setProgress(int progress) {
	if(progress == 100) {
		ui->buttonBox->setStandardButtons(QDialogButtonBox::Ok);
		ui->buttonBox->setEnabled(true);
	}
	updateLabel();
}

void ProgressDialog::setStage(QString s) {
	stage = s;
	updateLabel();
}

//---------------------------------------------------------------------

void ProgressDialog::on_buttonBox_clicked(QAbstractButton *button) {
	if(ui->buttonBox->buttonRole(button) == QDialogButtonBox::RejectRole) {
		ui->buttonBox->setEnabled(false);
		ui->progressLabel->setText("Cancelling...");
	}
}

//---------------------------------------------------------------------

void ProgressDialog::reject() {
	// Let the connected signals take care of closing the dialog
}

//---------------------------------------------------------------------

void ProgressDialog::changeEvent(QEvent *e) {
    QDialog::changeEvent(e);
    switch (e->type()) {
    case QEvent::LanguageChange:
        ui->retranslateUi(this);
        break;
    default:
        break;
    }
}

//---------------------------------------------------------------------

