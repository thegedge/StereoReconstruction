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
#include "saveablepixmapitem.hpp"

#include <QFileDialog>
#include <QGraphicsSceneContextMenuEvent>
#include <QMenu>
#include <QPixmap>

//---------------------------------------------------------------------

SaveablePixmapItem::SaveablePixmapItem(QGraphicsItem *parent)
	: QGraphicsPixmapItem(parent)
{
	menu.addAction(tr("Save Image..."), this, SLOT(askToSaveImage()));
}

SaveablePixmapItem::SaveablePixmapItem(const QPixmap &pixmap, QGraphicsItem *parent)
	: QGraphicsPixmapItem(pixmap, parent)
{
	menu.addAction(tr("Save Image..."), this, SLOT(askToSaveImage()));
}

//---------------------------------------------------------------------

QAction * SaveablePixmapItem::addMenuItem(QString text, QObject *receiver, const char *member) {
	return menu.addAction(text, receiver, member);
}

//---------------------------------------------------------------------

void SaveablePixmapItem::askToSaveImage() {
	QFileDialog fd(nullptr, tr("Save Image"), QString(), tr("Image Files (*.png *.bmp *.jpg *.jpeg)"));
	fd.setDefaultSuffix("png");
	fd.setAcceptMode(QFileDialog::AcceptSave);
	fd.exec();

	if(!fd.selectedFiles().isEmpty())
		pixmap().save(fd.selectedFiles().front());
}

//---------------------------------------------------------------------

void SaveablePixmapItem::contextMenuEvent(QGraphicsSceneContextMenuEvent *e) {
	menu.exec(e->screenPos());
}

//---------------------------------------------------------------------
