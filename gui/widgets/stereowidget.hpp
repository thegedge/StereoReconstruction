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
#ifndef STEREOWIDGET_HPP
#define STEREOWIDGET_HPP

#include <QDialog>
#include <QGraphicsScene>

#include "gui/gvitems/saveablepixmapitem.hpp"
#include "stereo/refractioncalibration.hpp"

//
// Forward declarations
//
namespace Ui {
    class StereoWidget;
}

class QAbstractButton;
class QGraphicsEllipseItem;
class QGraphicsLineItem;
class QGraphicsTextItem;
class QGraphicsPixmapItem;
class QListWidgetItem;

class FeatureGroup;
class Task;

FORWARD_DECLARE(Project);
FORWARD_DECLARE(Camera);
FORWARD_DECLARE(ProjectImage);
FORWARD_DECLARE(FeatureDetector);

FORWARD_DECLARE(RefractionCalibration);
FORWARD_DECLARE(MultiViewStereo);

/*!
 *
 */
class StereoWidget : public QWidget {
    Q_OBJECT

public:
    StereoWidget(QWidget *parent = 0);
    ~StereoWidget();

public slots:
	void setProject(ProjectPtr proj);

protected:
    void changeEvent(QEvent *);
	bool eventFilter(QObject *, QEvent *);

	//! .
	std::vector<ImageSetPtr> checkedImageSets() const;

	//! .
	void updateParameters();

	//! .
	void updateSceneRect();

	//! .
	QGraphicsItem * epipolarLineItem(CameraPtr left, CameraPtr right, QPen pen, int numDepths = 0);

public slots:
	//! .
	void updateFeatures();

	//! .
	void updateFeatures(int row);

	//! .
	void updateView();

private slots:
	//
	void parameterChanged();
	void cameraChanged(int index);

	//
	void checkAllImageSets();
	void uncheckAllImageSets();

	//
	void rotateLeftFeatures();
	void rotateRightFeatures();

	//
	void stereoCompleted(const Task *task);

	//
	void on_resetParametersButton_clicked();
	void on_calibrateButton_clicked();
	void on_imageSetsList_customContextMenuRequested(QPoint pos);
	void on_imageSetsList_currentItemChanged(QListWidgetItem*, QListWidgetItem*);
	void on_computeDepthMapsButton_clicked();

	void on_toolExport_clicked();
	void on_actionZoom_In_triggered();
	void on_actionZoom_Out_triggered();

private:
    Ui::StereoWidget *ui;

	//
	ProjectPtr project;
	CameraPtr leftView, rightView;

	//
	QGraphicsScene scene;

	//
	ProjectImagePtr leftImage, rightImage;
	SaveablePixmapItem *leftPixmap, *rightPixmap;
	SaveablePixmapItem *leftDepthMap, *rightDepthMap;

	// Epipolar line
	bool swap;
	QPointF point;
	QGraphicsEllipseItem *pointItem;
	QGraphicsEllipseItem *mp1, *mp2;
	QGraphicsItem *epipolarCurve, *epipolarLine;

	//! .
	Correspondence selectedCorrespondence;

	//! List of graphics view features for each image set
	std::vector<FeatureGroup *> featureItems;

	//! .
	RefractionCalibration calib;

	//! .
	MultiViewStereoPtr mvs;
};

#endif // STEREOWIDGET_HPP
