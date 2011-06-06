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
#ifndef FEATURE_HPP
#define FEATURE_HPP

#include <QDomElement>

//! A feature found in an image.
class Feature {
public:
	Feature(double x, double y) : x_(x), y_(y) { }

	//
	virtual ~Feature() { }

	//! .
	double x() const { return x_; }
	double y() const { return y_; }

public:
	//! Save this feature to the specified DOM element
	virtual void save(QDomElement &element) = 0;

	//! Load this feature from the specified DOM element
	virtual void load(const QDomElement &element) = 0;

	//! Return a unique type ID for this feature
	virtual int type() const = 0;

	//! Compare this feature to another
	virtual double compare(const Feature *other) const = 0;

	/*!
	 * A small piece of text that can be used to identify the feature. If
	 * such text is unnecessary then return a null string. This text can be
	 * used in various locations, such as StereoWidget's feature preview.
	 */
	virtual QString shortDescription() { return QString(); }

protected:
	double x_, y_;
};

#endif // FEATURE_HPP
