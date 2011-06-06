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
#include "elidedlabel.hpp"

#include <QDebug>
#include <QPainter>
#include <QResizeEvent>

//---------------------------------------------------------------------

ElidedLabel::ElidedLabel(QWidget *parent, Qt::WindowFlags f)
	: QLabel(parent, f)
	, elideMode_(Qt::ElideRight)
{
	setSizePolicy(QSizePolicy::Ignored, QSizePolicy::Preferred);
}

ElidedLabel::ElidedLabel(const QString &txt, QWidget *parent, Qt::WindowFlags f)
	: QLabel(txt, parent, f)
	, elideMode_(Qt::ElideRight)
{
	setSizePolicy(QSizePolicy::Ignored, QSizePolicy::Preferred);
}

ElidedLabel::ElidedLabel(const QString &txt,
						 Qt::TextElideMode elideMode,
						 QWidget *parent,
						 Qt::WindowFlags f)
	: QLabel(txt, parent, f)
	, elideMode_(elideMode)
{
	setSizePolicy(QSizePolicy::Ignored, QSizePolicy::Preferred);
}

//---------------------------------------------------------------------

void ElidedLabel::setText(const QString &txt) {
	QLabel::setText(txt);
	cacheElidedText(geometry().width());
	setSizePolicy(QSizePolicy::Ignored, QSizePolicy::Preferred);
}

//---------------------------------------------------------------------

void ElidedLabel::cacheElidedText(int w) {
	cachedElidedText = fontMetrics().elidedText(text(), elideMode_, w, Qt::TextShowMnemonic);
}

//---------------------------------------------------------------------

void ElidedLabel::resizeEvent(QResizeEvent *e) {
	QLabel::resizeEvent(e);
	cacheElidedText(e->size().width());
}

//---------------------------------------------------------------------

void ElidedLabel::paintEvent(QPaintEvent *e) {
	if(elideMode_ == Qt::ElideNone) {
		QLabel::paintEvent(e);
	} else {
		QPainter p(this);
		p.drawText(0, 0,
				   geometry().width(),
				   geometry().height(),
				   alignment(),
				   cachedElidedText);
	}
}

//---------------------------------------------------------------------
