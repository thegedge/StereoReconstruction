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
#include "comboboxitemdelegate.hpp"
#include <QComboBox>

//---------------------------------------------------------------------

ComboBoxItemDelegate::ComboBoxItemDelegate(QStringList options, QStringList data, QObject *parent)
    : QStyledItemDelegate(parent)
    , options(options)
    , data(data)
{ }

//---------------------------------------------------------------------

QWidget* ComboBoxItemDelegate::createEditor(QWidget *parent, const QStyleOptionViewItem &, const QModelIndex &) const {
    QComboBox *cb = new QComboBox(parent);

	cb->addItems(options);
	for(int index = 0; index < std::min(options.size(), data.size()); ++index)
		cb->setItemData(index, data[index]);

    return cb;
}

//---------------------------------------------------------------------

void ComboBoxItemDelegate::setEditorData(QWidget *editor, const QModelIndex &index) const {
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

void ComboBoxItemDelegate::setModelData(QWidget *editor, QAbstractItemModel *model, const QModelIndex &index) const {
    if(QComboBox *cb = qobject_cast<QComboBox *>(editor))
        model->setData(index, cb->currentText(), Qt::EditRole);
    else
        QStyledItemDelegate::setModelData(editor, model, index);
}

//---------------------------------------------------------------------
