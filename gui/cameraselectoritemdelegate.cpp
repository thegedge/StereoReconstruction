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
#include "cameraselectoritemdelegate.hpp"
#include "project/project.hpp"
#include "project/camera.hpp"

#include <QComboBox>

//---------------------------------------------------------------------

CameraSelectorItemDelegate::CameraSelectorItemDelegate(ProjectPtr project, QObject *parent)
    : QStyledItemDelegate(parent)
    , project(project)
{ }

//---------------------------------------------------------------------

QWidget* CameraSelectorItemDelegate::createEditor(QWidget *parent, const QStyleOptionViewItem &, const QModelIndex &) const {
    QComboBox *cb = new QComboBox(parent);

	cb->addItem(tr("<no camera>"), QString());
	if(project) {
		foreach(CameraPtr cam, project->cameras())
			cb->addItem(cam->name(), cam->id());
	}

    return cb;
}

//---------------------------------------------------------------------

void CameraSelectorItemDelegate::setProject(ProjectPtr project) {
	this->project = project;
}

//---------------------------------------------------------------------

void CameraSelectorItemDelegate::setEditorData(QWidget *editor, const QModelIndex &index) const {
    if(QComboBox *cb = qobject_cast<QComboBox *>(editor)) {
        QString current = index.data(Qt::EditRole).toString();
        int currentIndex = cb->findText(current);
        if(currentIndex >= 0)
            cb->setCurrentIndex(currentIndex);
    } else {
        QStyledItemDelegate::setEditorData(editor, index);
    }
}

//---------------------------------------------------------------------

void CameraSelectorItemDelegate::setModelData(QWidget *editor, QAbstractItemModel *model, const QModelIndex &index) const {
    if(QComboBox *cb = qobject_cast<QComboBox *>(editor)) {
        model->setData(index, cb->currentText(), Qt::EditRole);
		model->setData(index, cb->itemData(cb->currentIndex()), Qt::UserRole);
    } else {
        QStyledItemDelegate::setModelData(editor, model, index);
	}
}

//---------------------------------------------------------------------
