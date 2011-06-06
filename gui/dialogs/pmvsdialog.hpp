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
#ifndef PMVSDIALOG_HPP
#define PMVSDIALOG_HPP

#include <QDialog>
#include <QProcess>
#include <QString>

//
// Forward declarations
//
namespace Ui {
    class PMVSDialog;
}

class QAbstractButton;

//! A dialog for showing the execution of PMVS
class PMVSDialog : public QDialog {
    Q_OBJECT

public:
    PMVSDialog(QWidget *parent = 0);
    ~PMVSDialog();

	void runPMVS(QString file);

protected:
    void changeEvent(QEvent *e);

private:
    Ui::PMVSDialog *ui;

	QProcess process;

private slots:
	void pmvsFinished(int status);
	void pmvsWroteToStdOut();
	void pmvsWroteToStdErr();
	void pmvsTerminate(int status);

    void on_buttonBox_clicked(QAbstractButton* button);
};

#endif // PMVSDIALOG_HPP
