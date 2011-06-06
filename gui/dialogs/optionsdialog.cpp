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
#include "optionsdialog.hpp"
#include "ui_optionsdialog.h"

//---------------------------------------------------------------------

OptionsDialog::OptionsDialog(const QStringList &options, QWidget *parent)
    : QDialog(parent)
    , ui(new Ui::OptionsDialog)
	, options(options)
{
    ui->setupUi(this);

	// Add options
	ui->optionsList->addItems(options);
}

OptionsDialog::~OptionsDialog() {
    delete ui;
}

//---------------------------------------------------------------------

QStringList OptionsDialog::getOptions(
		QWidget *parent,
		const QString &title,
		const QStringList &options,
		bool *ok)
{
	OptionsDialog dialog(options, parent);
	dialog.setWindowTitle(title);
	dialog.exec();

	if(ok)
		*ok = (dialog.result() == QDialog::Accepted);

	return dialog.selectedOptions();
}

QList<int> OptionsDialog::getOptionIndicies(
		QWidget *parent,
		const QString &title,
		const QStringList &options,
		bool *ok)
{
	OptionsDialog dialog(options, parent);
	dialog.setWindowTitle(title);
	dialog.exec();

	if(ok)
		*ok = (dialog.result() == QDialog::Accepted);

	return dialog.selectedOptionIndicies();
}

//---------------------------------------------------------------------

QStringList OptionsDialog::selectedOptions() const {
	QStringList ret;
	foreach(const QModelIndex &item, ui->optionsList->selectionModel()->selectedIndexes())
		ret << options.at(item.row());
	return ret;
}

QList<int> OptionsDialog::selectedOptionIndicies() const {
	QList<int> ret;
	foreach(const QModelIndex &item, ui->optionsList->selectionModel()->selectedIndexes())
		ret << item.row();
	return ret;
}

//---------------------------------------------------------------------

void OptionsDialog::changeEvent(QEvent *e) {
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

