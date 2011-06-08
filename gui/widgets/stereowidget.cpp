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
//
// TODO
//  - See if any code here can be factored out somewhere else. Definitely
//    factor out the epipolar line computation to a file of its own (and
//    be sure to use this in stereo classes too)
//
//  - Get away from the two-view style interface and have a multi-view
//    approach. Perhaps tabs or just a horizontal display mode.
//
//  - Extract refractive calibration to somewhere else (CameraInfoWidget?)
//
//  - More interaction with MultiViewStereo. Allow selection of cameras,
//    easier tuning of parameters, complete set of tunable parameters,
//    and the ability to selectively view depth hypotheses from various
//    stages of the algorithm (e.g., before and after cross-checking)
//
#include "stereowidget.hpp"
#include "ui_stereowidget.h"

#include "gui/gvitems/saveablepixmapitem.hpp"

#include "features/checkerboard.hpp"
#include "features/correspondence.hpp"
#include "features/feature.hpp"
#include "features/surf.hpp"
#include "project/camera.hpp"
#include "project/imageset.hpp"
#include "project/project.hpp"
#include "project/projectimage.hpp"
#include "stereo/calibrate.hpp"
#include "stereo/multiviewstereo.hpp"
#include "stereo/badata.hpp"
#include "stereo/twoviewstereo.hpp"
#include "util/vectorimage.hpp"
#include <algorithm>
#include <map>

#include <QComboBox>
#include <QDebug>
#include <QFileDialog>
#include <QGraphicsDropShadowEffect>
#include <QGraphicsEllipseItem>
#include <QGraphicsLineItem>
#include <QGraphicsScene>
#include <QGraphicsSceneEvent>
#include <QGraphicsSceneMouseEvent>
#include <QGraphicsTextItem>
#include <QGraphicsWidget>
#include <QImage>
#include <QImageReader>
#include <QMenu>
#include <QMouseEvent>
#include <QWheelEvent>
#include <QPainter>
#include <QPrinter>
#include <QPushButton>
#include <QSet>
#include <QTransform>

//---------------------------------------------------------------------

using namespace Eigen;
using std::max;

namespace {
	const int SPACING = 10;
}

//---------------------------------------------------------------------

class FeatureItem {
public:
	FeatureItem() : l1(nullptr), l2(nullptr) { }

	FeatureItem(QGraphicsLineItem *l1, QGraphicsLineItem *l2, QGraphicsSimpleTextItem *txt)
		: l1(l1), l2(l2), txt(txt)
	{ }

	void setSelectedState(bool selected) {
		if(l1) l1->setPen(QPen(selected ? Qt::yellow: Qt::red, 0));
		if(l2) l2->setPen(QPen(selected ? Qt::yellow: Qt::red, 0));
		if(txt) txt->setBrush(QBrush(selected ? Qt::yellow : Qt::white));
	}

private:
	QGraphicsLineItem *l1, *l2;
	QGraphicsSimpleTextItem *txt;
};

//---------------------------------------------------------------------

class FeatureGroup : public QGraphicsItemGroup {
public:
	void addFeature(QGraphicsScene &s, FeaturePtr f, double xoff);
	void setSelectedState(FeaturePtr f, bool selectedState);

private:
	std::map<FeaturePtr, FeatureItem> features;
};

void FeatureGroup::addFeature(QGraphicsScene &s, FeaturePtr f, double xoff) {
	QGraphicsLineItem *l1 = s.addLine(xoff + f->x() - 5, f->y(), xoff + f->x() + 5, f->y());
	QGraphicsLineItem *l2 = s.addLine(xoff + f->x(), f->y() - 5, xoff + f->x(), f->y() + 5);
	addToGroup(l1);
	addToGroup(l2);
	l1->setCursor(QCursor(Qt::PointingHandCursor));
	l2->setCursor(QCursor(Qt::PointingHandCursor));

	QGraphicsSimpleTextItem *txt = nullptr;
	QString descr = f->shortDescription();
	if(!descr.isEmpty()) {
		txt = s.addSimpleText(descr);
		addToGroup(txt);
		txt->setPos(xoff + f->x() + 2, f->y() + 2);
		txt->setBrush(QBrush(Qt::white));
		txt->setPen(QPen(QBrush(Qt::black), 0.2));
	}

	features[f] = FeatureItem(l1, l2, txt);
	features[f].setSelectedState(false);
}

void FeatureGroup::setSelectedState(FeaturePtr f, bool selectedState) {
	std::map<FeaturePtr, FeatureItem>::iterator iter = features.find(f);
	if(iter != features.end())
		iter->second.setSelectedState(selectedState);
}

//---------------------------------------------------------------------

StereoWidget::StereoWidget(QWidget *parent)
	: QWidget(parent)
    , ui(new Ui::StereoWidget)
	, leftPixmap(nullptr)
	, rightPixmap(nullptr)
	, leftDepthMap(nullptr)
	, rightDepthMap(nullptr)
	, pointItem(nullptr)
	, epipolarCurve(nullptr)
	, epipolarLine(nullptr)
	, mvs(new MultiViewStereo)
{
    ui->setupUi(this);

	//
	ui->actionZoom_In->setShortcut(QKeySequence::ZoomIn);
	ui->actionZoom_Out->setShortcut(QKeySequence::ZoomOut);

	//
	ui->toolZoomIn->setDefaultAction(ui->actionZoom_In);
	ui->toolZoomOut->setDefaultAction(ui->actionZoom_Out);

	//
	ui->preview->setRenderHint(QPainter::Antialiasing);
	ui->preview->setScene(&scene);
	scene.installEventFilter(this);

	//
	addAction(ui->actionZoom_In);
	addAction(ui->actionZoom_Out);
}

StereoWidget::~StereoWidget() {
    delete ui;
}

//---------------------------------------------------------------------

void StereoWidget::setProject(ProjectPtr project) {
	//
	this->project = project;

	leftImage.reset();
	rightImage.reset();

	leftPixmap = nullptr;
	rightPixmap = nullptr;
	leftDepthMap = nullptr;
	rightDepthMap = nullptr;
	pointItem = mp1 = mp2 = nullptr;
	epipolarCurve = nullptr;
	epipolarLine = nullptr;

	scene.clear();
	featureItems.clear();

	ui->leftViewCombo->clear();
	ui->rightViewCombo->clear();
	ui->imageSetsList->clear();

	//
	if(project) {
		//
		featureItems.resize(project->imageSets().size(), nullptr);

		// Connect slots for left/right view combos before adding items
		// so that the cameraChanged slot is called while adding the items
		foreach(CameraPtr camera, project->cameras()) {
			ui->leftViewCombo->addItem(camera->name(), camera->id());
			ui->rightViewCombo->addItem(camera->name(), camera->id());
		}

		// Add image sets
		int index = 0;
		foreach(ImageSetPtr imageSet, project->imageSets()) {
			QListWidgetItem *item = new QListWidgetItem(imageSet->name(), ui->imageSetsList);
			item->setData(Qt::UserRole, imageSet->id());
			item->setFlags(item->flags() | Qt::ItemIsUserCheckable);
			item->setCheckState(Qt::Unchecked);
			item->setHidden(true);
			++index;
		}

		//
		ui->leftViewCombo->setCurrentIndex(0);
		ui->rightViewCombo->setCurrentIndex(1);

		ui->imageSetsList->setCurrentRow(0);
		ui->imageSetsList->setFocus();

		//
		updateFeatures();
		setEnabled(true);
	} else {
		setEnabled(false);
	}

	//
	//
	//
	updateParameters();
}

//---------------------------------------------------------------------

void StereoWidget::cameraChanged(int index) {
	//
	if(sender() == ui->leftViewCombo) {
		if(index < 0 || index >= ui->leftViewCombo->count())
			return;

		QString leftCamera = ui->leftViewCombo->itemData(index).toString();
		leftView = project->camera(leftCamera);
		if(leftDepthMap) {
			QImage img = mvs->depthMap(leftView);
			if(!img.isNull())
				leftDepthMap->setPixmap(QPixmap::fromImage(img));
		}
	} else if(sender() == ui->rightViewCombo){
		if(index < 0 || index >= ui->rightViewCombo->count())
			return;

		QString rightCamera = ui->rightViewCombo->itemData(index).toString();
		rightView = project->camera(rightCamera);
		if(rightDepthMap) {
			QImage img = mvs->depthMap(rightView);
			if(!img.isNull())
				rightDepthMap->setPixmap(QPixmap::fromImage(img));
		}
	}

	//
	QListWidgetItem *selected = ui->imageSetsList->currentItem();

	// Find image sets that have both the left and right view. Hide every
	// other image set from the user.
	const ImageSetMap &imageSets = project->imageSets();
	for(int index = 0; index < ui->imageSetsList->count(); ++index) {
		QString id = ui->imageSetsList->item(index)->data(Qt::UserRole).toString();
		ImageSetPtr imageSet = imageSets[id];

		bool isInvalidImageSet = !(imageSet->defaultImageForCamera(leftView)
								   && imageSet->defaultImageForCamera(rightView) );

		ui->imageSetsList->item(index)->setHidden(isInvalidImageSet);
		if(isInvalidImageSet)
			ui->imageSetsList->item(index)->setSelected(false);
	}

	//
	if(selected && !selected->isHidden())
		on_imageSetsList_currentItemChanged(ui->imageSetsList->currentItem(), nullptr);
	else
		ui->imageSetsList->setCurrentRow(0, QItemSelectionModel::ClearAndSelect);

	updateParameters();
	updateFeatures();
}

//---------------------------------------------------------------------

void StereoWidget::on_imageSetsList_currentItemChanged(
		QListWidgetItem *current,
		QListWidgetItem *previous)
{
	if(!current || !project)
		return;

	QString imageSetId = current->data(Qt::UserRole).toString();
	ImageSetPtr imageSet = project->imageSet(imageSetId);

	//
	if(leftDepthMap) leftDepthMap->setVisible(mvs->imageSet() == imageSet);
	if(rightDepthMap) rightDepthMap->setVisible(mvs->imageSet() == imageSet);

	// Load images
	QImage qleft, qright;
	if(leftView && rightView && imageSet) {
		leftImage = imageSet->defaultImageForCamera(leftView);
		if(leftImage)
			qleft = QImage(leftImage->file());

		rightImage = imageSet->defaultImageForCamera(rightView);
		if(rightImage)
			qright = QImage(rightImage->file());
	}

	//
	if(qleft.isNull() || qright.isNull()) {
		leftImage.reset();
		rightImage.reset();
		return;
	}
	{
		//
		if(!leftPixmap) {
			leftPixmap = new SaveablePixmapItem;
			leftPixmap->addMenuItem(tr("Rotate Features"), this, SLOT(rotateLeftFeatures()));
			scene.addItem(leftPixmap);
		}

		QPixmap pixmap = QPixmap::fromImage(qleft);
		leftPixmap->setPixmap(pixmap);
		leftPixmap->setPos(0, 0);
		leftPixmap->setZValue(200);

		//
		if(!rightPixmap) {
			rightPixmap = new SaveablePixmapItem;
			rightPixmap->addMenuItem(tr("Rotate Features"), this, SLOT(rotateRightFeatures()));
			scene.addItem(rightPixmap);
		}

		pixmap = QPixmap::fromImage(qright);
		rightPixmap->setPixmap(pixmap);
		rightPixmap->setPos(leftPixmap ? leftPixmap->boundingRect().right() + SPACING : 0, 0);
		rightPixmap->setZValue(0);

		//
		updateSceneRect();
	}

	if(QObject::sender() == ui->imageSetsList) {
		//
		if(previous) {
			int previousRow = ui->imageSetsList->row(previous);
			if(featureItems[previousRow]) featureItems[previousRow]->setVisible(false);
		}

		int currentRow = ui->imageSetsList->row(current);
		if(featureItems[currentRow]) featureItems[currentRow]->setVisible(true);

		//
		point = QPointF();
		updateView();
	}
}

//---------------------------------------------------------------------

std::vector<ImageSetPtr> StereoWidget::checkedImageSets() const {
	std::vector<ImageSetPtr> ret;
	for(int row = 0; row < ui->imageSetsList->count(); ++row) {
		const QListWidgetItem *item = ui->imageSetsList->item(row);
		if(item->checkState() == Qt::Checked)
			ret.push_back( project->imageSet(item->data(Qt::UserRole).toString()) );
	}
	return ret;
}

//---------------------------------------------------------------------

void StereoWidget::rotateLeftFeatures() {
	Features &features = project->features().features(leftImage);
	CheckerboardDetector::rotateIndicies(features);
	updateFeatures(ui->imageSetsList->currentRow());
}

void StereoWidget::rotateRightFeatures() {
	Features &features = project->features().features(rightImage);
	CheckerboardDetector::rotateIndicies(features);
	updateFeatures(ui->imageSetsList->currentRow());
}

//---------------------------------------------------------------------

void StereoWidget::updateFeatures() {
	for(int row = 0; row < ui->imageSetsList->count(); ++row)
		updateFeatures(row);
}

void StereoWidget::updateFeatures(int row) {
	//
	if(featureItems[row]) {
		scene.removeItem(featureItems[row]);
		featureItems[row] = nullptr;
	}

	//
	const QListWidgetItem *item = ui->imageSetsList->item(row);
	if(leftPixmap) {
		//
		FeatureGroup *group = new FeatureGroup;
		group->setVisible(row == ui->imageSetsList->currentRow());
		group->setPos(leftPixmap->sceneBoundingRect().topLeft());
		group->setZValue(500);
		scene.addItem(group);

		//
		ImageSetPtr imageSet = project->imageSet(item->data(Qt::UserRole).toString());
		if(!imageSet)
			return;

		ProjectImagePtr img1 = imageSet->defaultImageForCamera(leftView);
		ProjectImagePtr img2 = imageSet->defaultImageForCamera(rightView);

		if(img1 && img2) {
			// If features exist, don't detect them. Otherwise do so
			// and store them in the feature databse.
			Features &features1 = project->features().features(img1);
			Features &features2 = project->features().features(img2);

			//
			int index = 1;
			foreach(FeaturePtr feature, features1) {
				group->addFeature(scene, feature, leftPixmap->x());
				++index;
			}

			index = 1;
			foreach(FeaturePtr feature, features2) {
				group->addFeature(scene, feature, rightPixmap->x());
				++index;
			}
		}

		//
		featureItems[row] = group;
	}
}

//---------------------------------------------------------------------

void StereoWidget::updateParameters() {
	if(!leftView || !rightView)
		return;

	Vector3d pl = leftView->K() * leftView->plane().normal();
	Vector3d pr = rightView->K() * rightView->plane().normal();
	pl /= pl.z();
	pr /= pr.z();

	ui->lparamPixelXSpinner->disconnect(this);
	ui->lparamPixelYSpinner->disconnect(this);
	ui->lparamDistanceSpinner->disconnect(this);
	ui->rparamPixelXSpinner->disconnect(this);
	ui->rparamPixelYSpinner->disconnect(this);
	ui->rparamDistanceSpinner->disconnect(this);
	ui->refractiveIndexSpinner->disconnect(this);
	ui->leftViewRefractive->disconnect(this);
	ui->rightViewRefractive->disconnect(this);

	ui->lparamPixelXSpinner->setValue(pl.x());
	ui->lparamPixelYSpinner->setValue(pl.y());
	ui->lparamDistanceSpinner->setValue(leftView->plane().distance());
	ui->rparamPixelXSpinner->setValue(pr.x());
	ui->rparamPixelYSpinner->setValue(pr.y());
	ui->rparamDistanceSpinner->setValue(rightView->plane().distance());
	ui->leftViewRefractive->setChecked(leftView->isRefractive());
	ui->rightViewRefractive->setChecked(rightView->isRefractive());

	if(leftView->isRefractive())
		ui->refractiveIndexSpinner->setValue(leftView->refractiveIndex());
	else if(rightView->isRefractive())
		ui->refractiveIndexSpinner->setValue(rightView->refractiveIndex());

	connect(ui->lparamPixelXSpinner, SIGNAL(valueChanged(double)), SLOT(parameterChanged()));
	connect(ui->lparamPixelYSpinner, SIGNAL(valueChanged(double)), SLOT(parameterChanged()));
	connect(ui->lparamDistanceSpinner, SIGNAL(valueChanged(double)), SLOT(parameterChanged()));
	connect(ui->rparamPixelXSpinner, SIGNAL(valueChanged(double)), SLOT(parameterChanged()));
	connect(ui->rparamPixelYSpinner, SIGNAL(valueChanged(double)), SLOT(parameterChanged()));
	connect(ui->rparamDistanceSpinner, SIGNAL(valueChanged(double)), SLOT(parameterChanged()));
	connect(ui->refractiveIndexSpinner, SIGNAL(valueChanged(double)), SLOT(parameterChanged()));
	connect(ui->leftViewRefractive, SIGNAL(stateChanged(int)), SLOT(parameterChanged()));
	connect(ui->rightViewRefractive, SIGNAL(stateChanged(int)), SLOT(parameterChanged()));

	updateView();
}

//---------------------------------------------------------------------

void StereoWidget::parameterChanged() {
	using namespace Eigen;

	double x1 = ui->lparamPixelXSpinner->value();
	double y1 = ui->lparamPixelYSpinner->value();
	double x2 = ui->rparamPixelXSpinner->value();
	double y2 = ui->rparamPixelYSpinner->value();

	double leftRef = (ui->leftViewRefractive->isChecked() ? ui->refractiveIndexSpinner->value() : 1);
	double rightRef = (ui->rightViewRefractive->isChecked() ? ui->refractiveIndexSpinner->value() : 1);

	Vector3d leftNormal = leftView->Kinv() * Vector3d(x1, y1, 1);
	Vector3d rightNormal = rightView->Kinv() * Vector3d(x2, y2, 1);
	leftView->setPlane(Plane3d(leftNormal, ui->lparamDistanceSpinner->value()));
	leftView->setRefractiveIndex(leftRef);
	rightView->setPlane(Plane3d(rightNormal, ui->rparamDistanceSpinner->value()));
	rightView->setRefractiveIndex(rightRef);

	//
	calib.setImageSets(checkedImageSets());

	//
	updateView();

	// Show total and average calibration error
	double average_err = -1;
	const double total_err = calib.totalError(&average_err);
	QString errorString = QString("%1 (avg %2 per feature)").arg(total_err).arg(average_err);
	ui->totalErrorLabel->setText(errorString);
}

//---------------------------------------------------------------------

void StereoWidget::on_calibrateButton_clicked() {
	std::vector<ImageSetPtr> imageSets = checkedImageSets();
	std::vector<CameraPtr> views;
#if 1
	foreach(CameraPtr cam, project->cameras())
		views.push_back(cam);
#elif 1
	views.push_back(project->camera("7310085"));
	views.push_back(project->camera("7310087"));
	views.push_back(project->camera("7310095"));
	views.push_back(project->camera("7310099"));
#else
	views.push_back(leftView);
	views.push_back(rightView);
#endif

	// Set initial parameters
	typedef LevenbergMarquardt::Model Model;
	typedef LevenbergMarquardt::FixedParams FixedParams;

	Model model = Model::Zero(3*views.size());
	FixedParams fixed = FixedParams(3*views.size(), false);

	// Global parameters
	model[0] = ui->refractiveIndexSpinner->value();
	fixed[0] = true;

	// Camera-specific parameters
	for(size_t view_index = 0; view_index < views.size(); ++view_index) {
		CameraPtr cam = views[view_index];

		// If refractive camera then set initial parameters to existing
		// configuration. Otherwise, just keep parameters fixed.
		if(cam->isRefractive()) {
			Vector3d pixel = cam->K() * cam->plane().normal();
			pixel /= pixel.z();

			model[3*view_index + 1] = pixel.x();
			model[3*view_index + 2] = pixel.y();
			model[3*view_index + 3] = cam->plane().distance();
		} else {
			fixed[3*view_index + 1] = true;
			fixed[3*view_index + 2] = true;
			fixed[3*view_index + 3] = true;
		}
	}

	//
	calib.setProject(project);
	calib.setViews(views);
	calib.setImageSets(imageSets);
	calib.setModel(model, fixed);

	if(calib.calibrate()) {
		model = calib.model();
		for(size_t view_index = 0; view_index < views.size(); ++view_index) {
			CameraPtr view = views[view_index];
			Vector3d normal = view->Kinv() * Vector3d(model[3*view_index + 0], model[3*view_index + 1], 1);
			view->setPlane(Plane3d(normal, model[3*view_index + 2]));
		}
	}

	//
	updateParameters();
}

//---------------------------------------------------------------------

QGraphicsItem * StereoWidget::epipolarLineItem(CameraPtr left, CameraPtr right, QPen pen, int numDepths) {
	QList<QGraphicsItem *> epipolarLinePieces;

	const Ray3d &planeRay = left->principleRay();
	Vector3d p1(point.x(), point.y(), 1);
	Ray3d ray = left->unproject(p1);

	double minDepth = ui->minDepthSpinner->value();
	double maxDepth = ui->maxDepthSpinner->value();
	if(numDepths <= 0)
		numDepths = ui->numDepthLevelsSpinner->value();

	QPainterPath path;
	bool first = true;

	p1[0] = std::numeric_limits<double>::quiet_NaN();
	for(int dist = 0; dist < numDepths; ++dist) {
		double t = dist / (numDepths - 1.0);
		//t /= (5 - 4*t);
		const double depth = minDepth*(1 - t) + maxDepth*t;

		Ray3d::Point p2 = planeRay.point(depth);

		if(intersect(ray, Plane3d(planeRay.direction(), depth), p2)) {
			//Ray3d::Point p2 = ray.point(depth);
			if(right->project(p2)) {
				if(std::isnan(p1[0])) p1 = p2;

				//epipolarLinePieces << scene.addLine(p2[0], p2[1] - 5, p2[0], p2[1] + 5, pen2);

				// Only add lines that are at least a pixel long
				//
				// TODO
				//   - clip line segments instead of using this test
				//
				if((p2 - p1).squaredNorm() > 1){// && (p2 - p1).squaredNorm() < 1000) {
					if(first) {
						path.moveTo(p1[0], p1[1]);
						first = false;
					}

					path.lineTo(p2[0], p2[1]);
					p1 = p2;
				}
			}
		}
	}

	if(!first)
		return scene.addPath(path, pen);
	return nullptr;
}

//---------------------------------------------------------------------

void StereoWidget::updateView() {
	//
	if(pointItem) pointItem->setVisible(false);
	if(mp1) mp1->setVisible(false);
	if(mp2) mp2->setVisible(false);

	//
	if(epipolarCurve) {
		scene.removeItem(epipolarCurve);
		delete epipolarCurve;
		epipolarCurve = nullptr;
	}

	if(epipolarLine) {
		scene.removeItem(epipolarLine);
		delete epipolarLine;
		epipolarLine = nullptr;
	}

	if(!point.isNull()) {
		// If there is a selected correspondence then display its error value
		if(selectedCorrespondence.first) {
			Correspondence c = selectedCorrespondence;
			const double e = calib.error(c, leftView, rightView);
			QString errorString = QString("%1").arg(e);
			ui->errorLabel->setText(errorString);
		}

		// Switch the views depending on where we clicked
		CameraPtr tempLeftView, tempRightView;
		ProjectImagePtr tempLeftImage, tempRightImage;
		QGraphicsPixmapItem *tempLeftPixmap, *tempRightPixmap;

		tempLeftPixmap = tempRightPixmap = nullptr;
		if(swap) {
			tempLeftView = rightView;
			tempLeftImage = rightImage;
			tempLeftPixmap = rightPixmap;
			tempRightView = leftView;
			tempRightImage = leftImage;
			tempRightPixmap = leftPixmap;
		} else {
			tempLeftView = leftView;
			tempLeftImage = leftImage;
			tempLeftPixmap = leftPixmap;
			tempRightView = rightView;
			tempRightImage = rightImage;
			tempRightPixmap = rightPixmap;
		}

		//
		if(tempLeftPixmap) {
			//
			QPen curvePen(QBrush(Qt::green), 0);
			epipolarCurve = epipolarLineItem(tempLeftView, tempRightView, curvePen);
			if(epipolarCurve) {
				epipolarCurve->setParentItem(tempRightPixmap);
				epipolarCurve->setOpacity(1);
				epipolarCurve->setZValue(200);
			}

			// If refractive, show the non-refractive epipolar line too
			if(tempLeftView->isRefractive() || tempRightView->isRefractive()) {
				double currLeftRefractiveIndex = tempLeftView->refractiveIndex();
				double currRightRefractiveIndex = tempRightView->refractiveIndex();
				tempLeftView->setRefractiveIndex(1);
				tempRightView->setRefractiveIndex(1);

				QPen linePen(QBrush(Qt::yellow), 0, Qt::DashLine);
				epipolarLine = epipolarLineItem(tempLeftView, tempRightView, linePen);
				if(epipolarLine) {
					epipolarLine->setParentItem(tempRightPixmap);
					epipolarLine->setOpacity(1);
					epipolarLine->setZValue(200);
				}

				tempLeftView->setRefractiveIndex(currLeftRefractiveIndex);
				tempRightView->setRefractiveIndex(currRightRefractiveIndex);
			}

			//
			if(pointItem) {
				pointItem->setVisible(true);
				pointItem->setRect(point.x() - 4, point.y() - 4, 8, 8);
				//pointItem->setPos(point.x() - 4, point.y() - 4);
			} else {
				pointItem = scene.addEllipse(point.x() - 4, point.y() - 4, 8, 8, QPen(Qt::white, 0));
				//QGraphicsLineItem *l1 = scene.addLine(4, 0, 4, 8, QPen(Qt::white, 1));
				//QGraphicsLineItem *l2 = scene.addLine(0, 4, 8, 4, QPen(Qt::white, 1));
				//pointItem = scene.createItemGroup(QList<QGraphicsItem *>() << l1 << l2);
				//pointItem->setPos(point.x() - 4, point.y() - 4);
				pointItem->setZValue(100.0);
			}

			pointItem->setParentItem(tempLeftPixmap);
		}
	}
}

//---------------------------------------------------------------------

bool StereoWidget::eventFilter(QObject *source, QEvent *evt) {
	if(source == &scene) {
		if(evt->type() == QEvent::GraphicsSceneMousePress
		   || evt->type() == QEvent::GraphicsSceneMouseMove)
		{
			QGraphicsSceneMouseEvent *mouseEvt = static_cast<QGraphicsSceneMouseEvent *>(evt);
			if(leftPixmap && rightPixmap && mouseEvt->buttons().testFlag(Qt::LeftButton)) {
				bool lc = leftPixmap->contains( leftPixmap->mapFromScene(mouseEvt->scenePos()) );
				bool rc = rightPixmap->contains( rightPixmap->mapFromScene(mouseEvt->scenePos()) );
				if(lc || rc) {
					//
					swap = rc;
					point = (swap ? rightPixmap : leftPixmap)->mapFromScene( mouseEvt->scenePos() );

					//
					bool swap2 = false;
					Correspondences correspondences = project->features().correspondences(leftImage, rightImage, &swap2);
					swap2 ^= swap;

					// If it's line item, then that means we clicked one of the lines
					// that correspond to a feature location. The midpoint of the
					// line is esxactly where the feature lies, so modify the point
					// in question to be there.
					Correspondence newSelected;
					double min = 50;
					foreach(const Correspondence &corr, correspondences) {
						// XXX what if corr.second is for the left image and not the right,
						//     since the feature database considers ordering of images to
						//     be unimportant [i.e. correspondences(a,b) == correspondences(b,a)]
						const double x = (swap2 ? corr.second->x() : corr.first->x());
						const double y = (swap2 ? corr.second->y() : corr.first->y());
						const double dx = x - point.x();
						const double dy = y - point.y();
						const double d = dx*dx + dy*dy;
						if(d < min) {
							min = d;
							point.setX(x);
							point.setY(y);
							newSelected = corr;
						}
					}

					// Update the correspondence preview if the selected
					// correspondence has changed
					if(newSelected != selectedCorrespondence) {
						FeatureGroup *featureGroup = featureItems[ui->imageSetsList->currentRow()];
						if(featureGroup) {
							featureGroup->setSelectedState(selectedCorrespondence.first, false);
							featureGroup->setSelectedState(selectedCorrespondence.second, false);
						}

						swap2 ^= swap;
						if(swap2)
							newSelected.first.swap(newSelected.second);

						selectedCorrespondence = newSelected;
						if(featureGroup) {
							featureGroup->setSelectedState(selectedCorrespondence.first, true);
							featureGroup->setSelectedState(selectedCorrespondence.second, true);
						}
					}

					//
					updateView();
					return true;
				}
			}
		}
	}

	return QWidget::eventFilter(source, evt);
}

//---------------------------------------------------------------------

void StereoWidget::on_actionZoom_In_triggered() {
	ui->preview->scale(1.2, 1.2);
}

void StereoWidget::on_actionZoom_Out_triggered() {
	if(ui->preview->transform().m11() >= 0.1)
		ui->preview->scale(1.0 / 1.2, 1.0 / 1.2);
}

//---------------------------------------------------------------------

void StereoWidget::on_toolExport_clicked() {
	QString path = QFileDialog::getSaveFileName(
	                             this,
	                             tr("Export To File"),
	                             QDir::home().absolutePath(),
	                             tr("PDF Files (*.pdf); Image Files (*.png, *.bmp, *.jpg)"));

	if(!path.isNull()) {
		if(path.endsWith(".pdf")) {
			QPrinter printer;
			printer.setPaperSize(ui->preview->sceneRect().size(), QPrinter::Point);
			printer.setPageMargins(0, 0, 0, 0, QPrinter::Point);
			printer.setFullPage(true);
			printer.setOutputFormat(QPrinter::PdfFormat);
			printer.setOutputFileName(path);
			{
				QPainter p(&printer);
				scene.render(&p, ui->preview->sceneRect());
			}
		} else {
			QImage out(ui->preview->sceneRect().size().toSize(), QImage::Format_ARGB32);
			{
				QPainter p(&out);
				scene.render(&p);
			}
			out.save(path);
		}
	}
}

//---------------------------------------------------------------------

void StereoWidget::changeEvent(QEvent *e) {
    QWidget::changeEvent(e);
    switch (e->type()) {
    case QEvent::LanguageChange:
        ui->retranslateUi(this);
        break;
    default:
        break;
    }
}

//---------------------------------------------------------------------

void StereoWidget::checkAllImageSets() {
	for(int row = 0; row < ui->imageSetsList->count(); ++row)
		ui->imageSetsList->item(row)->setCheckState(Qt::Checked);
}

void StereoWidget::uncheckAllImageSets() {
	for(int row = 0; row < ui->imageSetsList->count(); ++row)
		ui->imageSetsList->item(row)->setCheckState(Qt::Unchecked);
}

//---------------------------------------------------------------------

void StereoWidget::updateSceneRect() {
	qreal ldmh = (leftDepthMap ? leftDepthMap->pixmap().height() : 0.0);
	qreal rdmh = (rightDepthMap ? rightDepthMap->pixmap().height() : 0.0);

	qreal x = 0.0;
	qreal y = 0.0;
	qreal w = leftPixmap->pixmap().width() + rightPixmap->pixmap().width() + SPACING;
	qreal h = max(leftPixmap->pixmap().height() + ldmh, rightPixmap->pixmap().height() + rdmh);
	ui->preview->setSceneRect(x, y, w, h);
}

//---------------------------------------------------------------------

void StereoWidget::on_imageSetsList_customContextMenuRequested(QPoint p) {
	QMenu menu(this);
	menu.addAction(tr("Check All"), this, SLOT(checkAllImageSets()));
	menu.addAction(tr("Uncheck All"), this, SLOT(uncheckAllImageSets()));
	menu.exec(ui->imageSetsList->mapToGlobal(p));
}

//---------------------------------------------------------------------

void StereoWidget::on_resetParametersButton_clicked() {
	ui->lparamPixelXSpinner->setValue(leftView->K()(0, 2));
	ui->lparamPixelYSpinner->setValue(leftView->K()(1, 2));
	ui->lparamDistanceSpinner->setValue(0.1);
	ui->rparamPixelXSpinner->setValue(rightView->K()(0, 2));
	ui->rparamPixelYSpinner->setValue(rightView->K()(1, 2));
	ui->rparamDistanceSpinner->setValue(0.1);
	ui->refractiveIndexSpinner->setValue(1.333);
}

//---------------------------------------------------------------------

void StereoWidget::stereoCompleted(const Task *task) {
	ui->computeDepthMapsButton->setEnabled(true);

	if(task->isCancelled())
		return;

	if(!leftDepthMap) leftDepthMap = new SaveablePixmapItem(leftPixmap);
	if(!rightDepthMap) rightDepthMap = new SaveablePixmapItem(rightPixmap);

	//
	leftDepthMap->setPixmap( QPixmap::fromImage(mvs->depthMap(leftView)) );
	leftDepthMap->setPos(leftPixmap->boundingRect().bottomLeft());
	rightDepthMap->setPixmap( QPixmap::fromImage(mvs->depthMap(rightView)) );
	rightDepthMap->setPos(rightPixmap->boundingRect().bottomLeft());

	updateSceneRect();
}

//---------------------------------------------------------------------

void StereoWidget::on_computeDepthMapsButton_clicked() {
	double mind = ui->minDepthSpinner->value();
	double maxd = ui->maxDepthSpinner->value();

	if(mind >= 0 && maxd > 0 && maxd > mind + 1e-10
	   && leftImage && rightImage
	   && leftPixmap && rightPixmap)
	{
		ui->computeDepthMapsButton->setEnabled(false);

		// TODO allow selection of cameras to use
		std::vector<CameraPtr> views;
		foreach(CameraPtr cam, project->cameras())
			views.push_back(cam);

		// Create task and notify application
		mvs->initialize(project, leftImage->imageSet(), views,
		                mind, maxd,
		                ui->numDepthLevelsSpinner->value(),
		                ui->crossCheckSpinner->value(),
		                ui->scaleSpinner->value() );

		connect(mvs.get(),
				SIGNAL(finished(const Task *)),
				SLOT(stereoCompleted(const Task *)));

		qApp->notify(QApplication::activeWindow(), new NewTaskEvent(mvs));
	}
}

//---------------------------------------------------------------------
