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
#ifndef BROWSEFORFILELINEEDIT_HPP
#define BROWSEFORFILELINEEDIT_HPP

#include <QFileInfo>
#include <QString>
#include <QWidget>

//
// Forward declarations
//
namespace Ui {
    class BrowseForFileLineEdit;
}

//! A simple widget that combines a textbox and browse button
class BrowseForFileLineEdit : public QWidget {
    Q_OBJECT

public:
    BrowseForFileLineEdit(QWidget *parent = 0);
	BrowseForFileLineEdit(QString key, QWidget *parent = 0);
    ~BrowseForFileLineEdit();

	QFileInfo fileInfo() const;
	QString filename() const;

	QString key() const;
	void setKey(QString key);

signals:
	void fileChanged(QString newFile);

protected:
    void changeEvent(QEvent *e);

private slots:
    void on_lineEdit_textEdited(QString );
    void on_lineEdit_editingFinished();
    void on_browseButton_clicked();

private:
    Ui::BrowseForFileLineEdit *ui;
	QString file;
	QString key_;
};

#endif // BROWSEFORFILELINEEDIT_HPP
