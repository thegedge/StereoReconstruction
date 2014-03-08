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
#ifndef CAMERASELECTORITEMDELEGATE_HPP
#define CAMERASELECTORITEMDELEGATE_HPP

#include <QStringList>
#include <QStyledItemDelegate>


FORWARD_DECLARE(Project);

//! An item delegate that uses a combo box for editing.
class CameraSelectorItemDelegate : public QStyledItemDelegate {
public:
	CameraSelectorItemDelegate(ProjectPtr project = ProjectPtr(), QObject *parent = 0);

    QWidget * createEditor(QWidget *parent, const QStyleOptionViewItem &option, const QModelIndex &index) const;
    void setEditorData(QWidget *editor, const QModelIndex &index) const;
    void setModelData(QWidget *editor, QAbstractItemModel *model, const QModelIndex &index) const;

public:
	void setProject(ProjectPtr project);

private:
	ProjectPtr project;
};

#endif // CAMERASELECTORITEMDELEGATE_HPP
