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
#include "pmvsdialog.hpp"
#include "ui_pmvsdialog.h"

#include <QAbstractButton>
#include <QDir>
#include <QProcess>
#include <QPushButton>
#include <QString>
#include <QStringList>
#include <QThread>

//---------------------------------------------------------------------

PMVSDialog::PMVSDialog(QWidget *parent)
    : QDialog(parent)
    , ui(new Ui::PMVSDialog)
{
    ui->setupUi(this);

	connect(this, SIGNAL(finished(int)), SLOT(pmvsTerminate(int)));
	connect(&process, SIGNAL(finished(int)), this, SLOT(pmvsFinished(int)));
	connect(&process, SIGNAL(readyReadStandardOutput()), this, SLOT(pmvsWroteToStdOut()));
	connect(&process, SIGNAL(readyReadStandardError()), this, SLOT(pmvsWroteToStdErr()));
}

PMVSDialog::~PMVSDialog() {
    delete ui;
}

//---------------------------------------------------------------------

void PMVSDialog::runPMVS(QString file) {
	// Break file into base name and directory for PMVS
	QDir dir(file);
	QString pmvs = "pmvs-2"; // TODO allow specifying path to PMVS in preferences
	QString input = QString("%1").arg( dir.dirName() );

	if(dir.cdUp()) {
		QString base = QString("%1/").arg( dir.path() );
		QStringList args;
		args << base << input;

		//
		ui->labelOutput->clear();;
		ui->labelOutput->setText(QString("<font color=\"#0F0\">%1 %2</font><br>").arg(pmvs, args.join(" ")));

		//
		process.start(pmvs, args, QProcess::ReadOnly);
	}
}

//---------------------------------------------------------------------

void PMVSDialog::changeEvent(QEvent *e) {
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

void PMVSDialog::pmvsFinished(int) {
	ui->buttonBox->removeButton(ui->buttonBox->button(QDialogButtonBox::Cancel));
	ui->buttonBox->addButton(QDialogButtonBox::Close);
}

void PMVSDialog::pmvsWroteToStdOut() {
	ui->labelOutput->setText(
			ui->labelOutput->toHtml()
			+ QString("<font color=\"#FFF\">%1</font>").arg( QString(process.readAllStandardOutput()).replace('\n', "<br/>") ));

	ui->labelOutput->moveCursor(QTextCursor::End);
	ui->labelOutput->ensureCursorVisible();
}

void PMVSDialog::pmvsWroteToStdErr() {
	ui->labelOutput->setText(
			ui->labelOutput->toHtml()
			+ QString("<font color=\"#F00\">%1</font>").arg( QString(process.readAllStandardError()).replace('\n', "<br/>") ));

	ui->labelOutput->moveCursor(QTextCursor::End);
	ui->labelOutput->ensureCursorVisible();
}

//---------------------------------------------------------------------

void PMVSDialog::pmvsTerminate(int) {
	if(process.isOpen()) {
		process.close();
		process.waitForFinished();
	}
}

void PMVSDialog::on_buttonBox_clicked(QAbstractButton*) {
	pmvsTerminate(0);
	close();
}

//---------------------------------------------------------------------
