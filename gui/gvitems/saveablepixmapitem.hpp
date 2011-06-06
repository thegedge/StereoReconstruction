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
#ifndef SAVEABLEPIXMAPITEM_HPP
#define SAVEABLEPIXMAPITEM_HPP

#include <QGraphicsPixmapItem>
#include <QObject>
#include <QMenu>

class QAction;

//! A QGraphicsPixmapItem that can be saved (via a context menu)
class SaveablePixmapItem : public QObject, public QGraphicsPixmapItem {
	Q_OBJECT

public:
    SaveablePixmapItem(QGraphicsItem *parent = nullptr);
    SaveablePixmapItem(const QPixmap &pixmap, QGraphicsItem *parent = nullptr);

public:
	QAction * addMenuItem(QString text, QObject *receiver, const char *member);

public slots:
	void askToSaveImage();

protected:
	void contextMenuEvent(QGraphicsSceneContextMenuEvent *event);

private:
	QMenu menu;
};

#endif // SAVEABLEPIXMAPITEM_HPP
