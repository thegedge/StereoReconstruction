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
#ifndef CAMERAINFOWIDGET_HPP
#define CAMERAINFOWIDGET_HPP

#include "util/c++0x.hpp"
#include <QWidget>


//
// Forward declarations
//
namespace Ui { class CameraInfoWidget; }

FORWARD_DECLARE(Camera);
FORWARD_DECLARE(Project);

//! A widget to display info about a camera
class CameraInfoWidget : public QWidget {
    Q_OBJECT
public:
    CameraInfoWidget(QWidget *parent = 0);
    ~CameraInfoWidget();

public slots:
	void setProject(ProjectPtr project);
	void setCamera(CameraPtr camera);
	void updateCamera();

protected:
    void changeEvent(QEvent *e);

private:
    Ui::CameraInfoWidget *ui;

	CameraPtr currentCamera;
};

#endif // CAMERAINFOWIDGET_HPP
