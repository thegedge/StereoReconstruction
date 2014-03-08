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
#include "capturedimagesscene.hpp"

#include <QAction>
#include <QApplication>
#include <QBrush>
#include <QEvent>
#include <QGraphicsPixmapItem>
#include <QImage>

//---------------------------------------------------------------------
namespace {
	const int SPACING = 10; //! Spacing between images
}
//---------------------------------------------------------------------

CapturedImagesScene::CapturedImagesScene(QWidget *parent)
    : QGraphicsView(new QGraphicsScene, parent)
{
	//
	setFocusPolicy(Qt::WheelFocus);
	setBackgroundBrush( QBrush(QColor(0, 0, 0, 255), Qt::Dense7Pattern) );
    setRenderHints(QPainter::TextAntialiasing);

	// Actions
	actionZoom_In = new QAction(tr("Zoom In"), this);
	actionZoom_In->setObjectName(QString::fromUtf8("actionZoom_In"));
	actionZoom_In->setShortcut(QKeySequence::ZoomIn);
	addAction(actionZoom_In);

	actionZoom_Out = new QAction(tr("Zoom Out"), this);
	actionZoom_Out->setObjectName(QString::fromUtf8("actionZoom_Out"));
	actionZoom_Out->setShortcut(QKeySequence::ZoomOut);
	addAction(actionZoom_Out);

	//
	QMetaObject::connectSlotsByName(this);
}

//---------------------------------------------------------------------

void CapturedImagesScene::on_actionZoom_In_triggered() {
	scale(1.2, 1.2);
}

void CapturedImagesScene::on_actionZoom_Out_triggered() {
	if(transform().m11() >= 0.1)
		scale(1.0 / 1.2, 1.0 / 1.2);
}

//---------------------------------------------------------------------

void CapturedImagesScene::setImages(const std::vector<Image> &images) {
	scene()->clear();

	int currentX = 0;
	foreach(const Image &image, images) {
		QImage qimg(image.data.get(), image.width, image.height, QImage::Format_RGB888);
		scene()->addPixmap(QPixmap::fromImage(qimg))->setX(currentX);
		currentX += image.width + SPACING;
	}
}

void CapturedImagesScene::setImages(const std::vector<QImage> &images) {
	scene()->clear();

	int currentX = 0;
	foreach(const QImage &image, images) {
		scene()->addPixmap(QPixmap::fromImage(image))->setX(currentX);
		currentX += image.width() + SPACING;
	}
}

void CapturedImagesScene::setImages(const std::vector<QString> &images) {
	scene()->clear();

	int currentX = 0;
	foreach(const QString &imagePath, images) {
		QImage qimg(imagePath);
		scene()->addPixmap(QPixmap::fromImage(qimg))->setX(currentX);
		currentX += qimg.width() + SPACING;
	}
}

//---------------------------------------------------------------------

void CapturedImagesScene::changeEvent(QEvent *e) {
    QGraphicsView::changeEvent(e);
    switch (e->type()) {
    case QEvent::LanguageChange:
        actionZoom_In->setText(QApplication::translate("CapturedImagesScene", "Zoom In", 0));
        actionZoom_Out->setText(QApplication::translate("CapturedImagesScene", "Zoom Out", 0));
        break;
    default:
        break;
    }
}

//---------------------------------------------------------------------
