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
#ifndef ELIDEDLABEL_HPP
#define ELIDEDLABEL_HPP

#include <QLabel>

/*!
 * A label that elides its text when not enough geometry is available to show
 * all of the text.
 *
 * \note currently only capable of one-line
 */
class ElidedLabel : public QLabel {
    Q_OBJECT

public:
	ElidedLabel(QWidget *parent = 0, Qt::WindowFlags f = 0);
	ElidedLabel(const QString &txt, QWidget * parent = 0, Qt::WindowFlags f = 0);
    ElidedLabel(const QString &txt,
				Qt::TextElideMode elideMode = Qt::ElideRight,
				QWidget * parent = 0,
				Qt::WindowFlags f = 0);

public:
	//! Set the elide mode used for displaying text.
	void setElideMode(Qt::TextElideMode elideMode) {
		elideMode_ = elideMode;
		updateGeometry();
	}

	//! Get the elide mode currently used to display text.
	Qt::TextElideMode elideMode() const { return elideMode_; }

public: // QLabel overrides
	void setText(const QString &);

protected: // QLabel overrides
	void paintEvent(QPaintEvent *);
	void resizeEvent(QResizeEvent *);

protected:
	//! Cache the elided text so as to not recompute it every paint event
	void cacheElidedText(int w);

private:
    Qt::TextElideMode elideMode_;
	QString cachedElidedText;
};

#endif // ELIDEDLABEL_HPP
