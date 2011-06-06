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
#include "browseforfilelineedit.hpp"
#include "ui_browseforfilelineedit.h"

#include <QDebug>
#include <QDir>
#include <QFileDialog>
#include <QFileInfo>
#include <QSettings>

//---------------------------------------------------------------------

BrowseForFileLineEdit::BrowseForFileLineEdit(QWidget *parent)
    : QWidget(parent)
    , ui(new Ui::BrowseForFileLineEdit)
{
    ui->setupUi(this);
}

BrowseForFileLineEdit::BrowseForFileLineEdit(QString key, QWidget *parent)
    : QWidget(parent)
    , ui(new Ui::BrowseForFileLineEdit)
	, key_(key)
{
    ui->setupUi(this);

	//
	if(!key_.isNull()) {
		QString val = QSettings().value(key_).toString();
		ui->lineEdit->setText(val);
	}
}

BrowseForFileLineEdit::~BrowseForFileLineEdit() {
    delete ui;
}

//---------------------------------------------------------------------

QFileInfo BrowseForFileLineEdit::fileInfo() const {
	return QFileInfo(file);
}

QString BrowseForFileLineEdit::filename() const {
	return file;
}

QString BrowseForFileLineEdit::key() const {
	return key_;
}

void BrowseForFileLineEdit::setKey(QString key) {
	key_ = key;
	if(ui->lineEdit->text().isEmpty() && !key_.isNull()) {
		file = QSettings().value(key_).toString();
		ui->lineEdit->setText(file);
	}
}

//---------------------------------------------------------------------

void BrowseForFileLineEdit::on_browseButton_clicked() {
	QSettings settings;

	QString initialDir = file;
	if(!QFileInfo(file).exists()) {
		initialDir = QDir::homePath();
		if(!key_.isNull())
			initialDir = settings.value(key_, QDir::homePath()).toString();
	}

	QString selectedFile =
		QFileDialog::getOpenFileName(
			this,
			tr("Select File"),
			initialDir);

	if(!selectedFile.isNull()) {
		on_lineEdit_textEdited(selectedFile);
		if(!key_.isNull())
			settings.setValue(key_, selectedFile);
	}
}

//---------------------------------------------------------------------

void BrowseForFileLineEdit::on_lineEdit_editingFinished() {
	if(file != ui->lineEdit->text())
		on_lineEdit_textEdited(ui->lineEdit->text());
}

void BrowseForFileLineEdit::on_lineEdit_textEdited(QString txt) {
	file = txt;
	if(sender() != ui->lineEdit) {
		ui->lineEdit->setText(txt);
		ui->lineEdit->setFocus(Qt::OtherFocusReason);
	}
	ui->lineEdit->setStyleSheet(QFileInfo(txt).exists() ? "" : "color: red;");
	emit fileChanged(file);
}

//---------------------------------------------------------------------

void BrowseForFileLineEdit::changeEvent(QEvent *e) {
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
