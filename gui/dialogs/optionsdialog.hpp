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
#ifndef OPTIONSDIALOG_HPP
#define OPTIONSDIALOG_HPP

#include <QDialog>
#include <QList>
#include <QStringList>

//
// Forward declarations
//
namespace Ui {
    class OptionsDialog;
}

/*!
 * A dialog that attaches to an object which emits progress updates
 * (i.e., implements ProgressObject).
 */
class OptionsDialog : public QDialog {
    Q_OBJECT
public:
    OptionsDialog(const QStringList &options, QWidget *parent = 0);
    ~OptionsDialog();

public:
	static QStringList getOptions(
			QWidget *parent,
			const QString &title,
			const QStringList &options,
			bool *ok = nullptr);

	static QList<int> getOptionIndicies(
			QWidget *parent,
			const QString &title,
			const QStringList &options,
			bool *ok = nullptr);

	QStringList selectedOptions() const;
	QList<int> selectedOptionIndicies() const;

protected:
	void changeEvent(QEvent *);

private:
    Ui::OptionsDialog *ui;

	QStringList options;
};

#endif // OPTIONSDIALOG_HPP
