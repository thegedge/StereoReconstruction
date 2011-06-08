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
#include "mainwindow.hpp"
#include "ui_mainwindow.h"

#include <QDebug>
#include <QDirIterator>
#include <QDockWidget>
#include <QDomDocument>
#include <QFile>
#include <QFileDialog>
#include <QFileInfo>
#include <QImageReader>
#include <QInputDialog>
#include <QListWidget>
#include <QMessageBox>
#include <QSettings>
#include <QSignalMapper>
#include <QStackedWidget>
#include <QSystemTrayIcon>
#include <QTextStream>
#include <QThread>
#include <QTimer>
#include <QVBoxLayout>

#include <boost/scoped_array.hpp>
#include <opencv/cv.h>
#include <opencv/highgui.h>

#ifdef HAS_HDR
#   include "hdr/hdr.hpp"
#endif

#include "gui/task.hpp"
#include "gui/dialogs/aboutdialog.hpp"
#include "gui/dialogs/calibrationsetupdialog.hpp"
#include "gui/dialogs/findfeaturesdialog.hpp"
#include "gui/dialogs/optionsdialog.hpp"
#include "gui/dialogs/pmvsdialog.hpp"
#include "gui/dialogs/progressdialog.hpp"
#include "gui/scene/capturedimagesscene.hpp"
#include "gui/scene/pointsviewscene.hpp"
#include "gui/scene/cameralayoutscene.hpp"
#include "gui/widgets/camerainfowidget.hpp"
#include "gui/widgets/imagesettable.hpp"
#include "gui/widgets/projectexplorer.hpp"
#include "gui/widgets/stereowidget.hpp"
#include "gui/widgets/taskprogresswidget.hpp"

#include "project/project.hpp"
#include "project/camera.hpp"
#include "project/imageset.hpp"
#include "project/projectimage.hpp"

#include "util/plane.hpp"
#include "util/rawimages/rawimagereader.hpp"

#include "stereo/calibrate.hpp"
#include "stereo/refractioncalibration.hpp"

//---------------------------------------------------------------------

QWidget * createWidgetWithBoxLayout(QWidget *parent, QWidget *child) {
	QVBoxLayout *layout = new QVBoxLayout;
	layout->setMargin(0);
	layout->addWidget(child);

	QWidget *widget = new QWidget(parent);
	child->setParent(widget);
	widget->setLayout(layout);
	return widget;
}

//---------------------------------------------------------------------

MainWindow::MainWindow(QWidget *parent)
	: QMainWindow(parent)
	, ui(new Ui::MainWindow)
	, userSettings(QSettings::UserScope, "Jason Gedge", "StereoReconstruction")
	, trayIcon(nullptr)
    , recentFiles(userSettings.value("recentFileList").toStringList())
{
    ui->setupUi(this);

	//
	// Recent Files
	//
	QSignalMapper *recentFilesMapper = new QSignalMapper(this);
	connect(recentFilesMapper, SIGNAL(mapped(int)), SLOT(openRecentFile(int)));

	for(int index = 0; index < NUM_RECENT_FILES; ++index) {
		recentFileActions[index] = ui->menuOpen_Recent->addAction(
				QString("%1: <No File>").arg(index + 1),
				recentFilesMapper,
				SLOT(map()) );

		recentFileActions[index]->setVisible(false);
		recentFilesMapper->setMapping(recentFileActions[index], index);
	}
	{
		// Move separator + "clear recent files" to end
		QAction *act = ui->menuOpen_Recent->actions().first();
		ui->menuOpen_Recent->removeAction(act);
		ui->menuOpen_Recent->addAction(act);

		act = ui->menuOpen_Recent->actions().first();
		ui->menuOpen_Recent->removeAction(act);
		ui->menuOpen_Recent->addAction(act);
	}

	//
	//
	//
	trayIcon = new QSystemTrayIcon(this);

#ifndef HAS_IMAGE_CAPTURE
	ui->menuBar->removeAction(ui->menuCapture->menuAction());
#endif
#ifndef HAS_HDR
	ui->menuBar->removeAction(ui->menuHDR->menuAction());
#endif

	//
	//
	//
	projectExplorerDock = new QDockWidget(tr("Project Explorer"), this);
	projectExplorerDock->setObjectName(QString::fromUtf8("projectExplorerDock"));
	projectExplorerDock->setMinimumWidth(200);
	projectExplorerDock->setAllowedAreas(Qt::LeftDockWidgetArea | Qt::RightDockWidgetArea);
	addDockWidget(Qt::LeftDockWidgetArea, projectExplorerDock);
	projectExplorerDock->hide();

	projectExplorer = new ProjectExplorer(this);
	projectExplorerDock->setWidget(projectExplorer);
	connect(projectExplorer, SIGNAL(customContextMenuRequested(QPoint)), SLOT(showProjectMenu(QPoint)));

	//
	//
	//
	imageSetTable = new ImageSetTable(this);
	imageSetTable->setEnabled(false);

	cameraInfoWidget = new CameraInfoWidget(this);
	cameraInfoWidget->setEnabled(false);

	inspectorDock = new QDockWidget(tr("Inspector"), this);
	inspectorDock->setObjectName(QString::fromUtf8("inspectorDock"));
	inspectorDock->setMinimumWidth(350);
	inspectorDock->setAllowedAreas(Qt::LeftDockWidgetArea | Qt::RightDockWidgetArea);
	addDockWidget(Qt::RightDockWidgetArea, inspectorDock);
	inspectorDock->hide();

	inspector = new QStackedWidget(inspectorDock);
	inspector->addWidget(new QWidget(this));
	inspector->addWidget(cameraInfoWidget);
	inspector->addWidget(imageSetTable);
	inspectorDock->setWidget(inspector);

	//
	//
	//
	taskList = new QWidget(this);
	taskList->setAutoFillBackground(true);
	taskList->setBackgroundRole(QPalette::Light);
	taskList->setObjectName(QString::fromUtf8("taskList"));
	taskList->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
	taskList->setMinimumHeight(50); // XXX necessary?

	taskListLayout = new QVBoxLayout(taskList);
	taskListLayout->setMargin(0);
	taskListLayout->addStretch(1);

	taskListDock = new QDockWidget(tr("Task List"), this);
	taskListDock->setObjectName(QString::fromUtf8("taskListDock"));
	taskListDock->setWidget(taskList);
	taskListDock->setAllowedAreas(Qt::AllDockWidgetAreas);
	addDockWidget(Qt::BottomDockWidgetArea, taskListDock);
	taskListDock->hide();

	//
	// TODO move these to the ui files
	//
	connect(projectExplorer,
			SIGNAL(cameraSelected(CameraPtr)),
			SLOT(cameraSelected(CameraPtr)) );

	connect(projectExplorer,
			SIGNAL(imageSetSelected(ImageSetPtr)),
			SLOT(imageSetSelected(ImageSetPtr)) );

	connect(projectExplorerDock,
			SIGNAL(visibilityChanged(bool)),
			SLOT(on_actionShowHide_Project_Explorer_triggered(bool)) );

	connect(inspectorDock,
			SIGNAL(visibilityChanged(bool)),
			SLOT(on_actionShowHide_Inspector_triggered(bool)) );

	projectExplorer->connect(this,
			SIGNAL(projectLoaded(ProjectPtr)),
			SLOT(setProject(ProjectPtr)) );

	cameraInfoWidget->connect(this,
							  SIGNAL(projectLoaded(ProjectPtr)),
							  SLOT(setProject(ProjectPtr)) );

	cameraInfoWidget->connect(projectExplorer,
							  SIGNAL(cameraSelected(CameraPtr)),
							  SLOT(setCamera(CameraPtr)) );

	imageSetTable->connect(this,
						   SIGNAL(projectLoaded(ProjectPtr)),
						   SLOT(setProject(ProjectPtr)) );

	imageSetTable->connect(projectExplorer,
						   SIGNAL(imageSetSelected(ImageSetPtr)),
						   SLOT(setImageSet(ImageSetPtr)) );

	ui->stereoWidget->connect(this,
							  SIGNAL(projectLoaded(ProjectPtr)),
							  SLOT(setProject(ProjectPtr)) );

	//
	// Set up initial window state
	//
	restoreGeometry(userSettings.value("mainWindowGeometry").toByteArray());
	restoreState(userSettings.value("mainWindowState").toByteArray());
	updateRecentFiles();
	ui->actionView_Nothing->trigger();
	ui->actionNew->trigger();
}

MainWindow::~MainWindow() {
	userSettings.setValue("mainWindowGeometry", saveGeometry());
	userSettings.setValue("mainWindowState", saveState());
	delete ui;
}

//---------------------------------------------------------------------

void MainWindow::on_actionExit_triggered() {
	QApplication::quit();
}

//---------------------------------------------------------------------

void crossProduct(const float *p1, const float *p2, const float *p3, float *n) {
	float x2 = p2[0] - p1[0]; float x3 = p3[0] - p1[0];
	float y2 = p2[1] - p1[1]; float y3 = p3[1] - p1[1];
	float z2 = p2[2] - p1[2]; float z3 = p3[2] - p1[2];

	n[0] = y2*z3 - z2*y3;
	n[1] = x3*z2 - z3*x2;
	n[2] = x2*y3 - y2*x3;

	float d = 1.0 / std::sqrt(n[0]*n[0] + n[1]*n[1] + n[2]*n[2]);
	n[0] *= d;
	n[1] *= d;
	n[2] *= d;
}

//---------------------------------------------------------------------
// TODO factor code out of here and into its own class for loading and
//      saving PLY files (additional code in stereo/multiviewstereo.cpp)
//
void MainWindow::on_actionView_PLY_File_triggered() {
	QString initialDir = userSettings.contains("InitialPLYDir")
	                     ? userSettings.value("InitialPLYDir").toString()
	                     : QDir::homePath();

	// TODO sheets would be nice for Mac users :)
	QString fname = QFileDialog::getOpenFileName(this,
	                                             tr("Open File"),
	                                             initialDir,
	                                             "PLY Files (*.ply)");

	if(!fname.isNull()) {
		QFile file(fname);
		if(file.open(QFile::ReadOnly)) {
			userSettings.setValue("InitialPLYDir", QDir(fname).absolutePath());

			QTextStream textStream(&file);

			// For now we'll ignore the header and assume a certain format
			QString line;
			bool hasNormal = false;
			bool hasColor = false;
			bool isBinary = false;

			unsigned int numVerticies = 0;
			unsigned int numFaces = 0;

			static QRegExp typeTest("n[xyz]");
			static QRegExp normalTest("n[xyz]");
			static QRegExp colorTest("diffuse_(red|blue|green)");
			static QRegExp elementTest("element (vertex|face) (\\d+)");

			while((line = textStream.readLine().trimmed()) != "end_header") {
				if(line.startsWith("property")) {
					if( line.contains(normalTest) )
						hasNormal = true;

					if( line.contains(colorTest) )
						hasColor = true;
				} else if(elementTest.indexIn(line) != -1) {
					if(elementTest.cap(1) == "face")
						numFaces = elementTest.cap(2).toUInt();
					else if(elementTest.cap(1) == "vertex")
						numVerticies = elementTest.cap(2).toUInt();
				} else if(line.startsWith("format")) {
					isBinary = line.contains("binary");
				}
			}

			QDataStream dataStream;
			if(isBinary) {
				qint64 pos = textStream.pos();
				file.close();
				file.open(QFile::ReadOnly);
				dataStream.setDevice(&file);
				dataStream.skipRawData(pos);
			}

			//
			// Read in the verticies
			//
			GLfloat d[9] = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
			std::vector<GLfloat> verticies(numVerticies * 9);
			std::vector<GLfloat>::iterator vertexIter = verticies.begin();
			if(isBinary) {
				uint len = 12 + (hasNormal ? 12 : 0) + (hasColor ? 3 : 0);

				// TODO more efficient to read in larger chunk of data
				for(unsigned int vertex = 0; vertex < numVerticies && !textStream.atEnd(); ++vertex) {
					dataStream.readRawData(reinterpret_cast<char *>(d), len);
					if(!hasNormal)
						d[3] = d[4] = d[5] = 0;

					if(!hasColor)
						d[6] = d[7] = d[8] = 1;

					if(dataStream.status() != QDataStream::ReadPastEnd)
						vertexIter = std::copy(d, d + 9, vertexIter);
				}
			} else {
				for(unsigned int vertex = 0; vertex < numVerticies && !textStream.atEnd(); ++vertex) {
					textStream >> d[0] >> d[1] >> d[2];
					if(hasNormal)
						textStream >> d[3] >> d[4] >> d[5];
					else
						d[3] = d[4] = d[5] = 0.0f;

					if(hasColor) {
						textStream >> d[6] >> d[7] >> d[8];
						d[6] /= 255.0;
						d[7] /= 255.0;
						d[8] /= 255.0;
					} else {
						d[6] = d[7] = d[8] = 0.5f;
						//d[6] = qrand() / (1.0*RAND_MAX);
						//d[7] = qrand() / (1.0*RAND_MAX);
						//d[8] = qrand() / (1.0*RAND_MAX);
					}

					if(textStream.status() != QTextStream::ReadPastEnd)
						vertexIter = std::copy(d, d + 9, vertexIter);

					textStream.readLine();
				}
			}

			//
			// Read in the indicies
			//
			GLuint dd[3] = {0, 0, 0};
			std::vector<GLuint> indicies(numFaces * 3, 0);
			std::vector<GLuint>::iterator indexIter = indicies.begin();

			if(isBinary) {
				// TODO more efficient to read in larger chunk of data
				char num;
				for(unsigned int face = 0; face < numFaces && !textStream.atEnd(); ++face) {
					dataStream.readRawData(&num, 1);
					if(num == 3) {
						dataStream.readRawData(reinterpret_cast<char *>(dd), 12);

						if(textStream.status() != QTextStream::ReadPastEnd)
							indexIter = std::copy(dd, dd + 3, indexIter);
					} else {
						indexIter += 3;
						dataStream.skipRawData(num*4);
					}
				}
			} else {
				for(unsigned int face = 0; face < numFaces && !textStream.atEnd(); ++face) {
					textStream >> dd[0];
					if(dd[0] != 3) {
						textStream.readLine();
						dd[0] = dd[1] = dd[2] = 0;
					} else {
						textStream >> dd[0] >> dd[1] >> dd[2];
					}

					if(textStream.status() != QTextStream::ReadPastEnd)
						indexIter = std::copy(dd, dd + 3, indexIter);
				}
			}

			//
			// If no normals, generate some. Normals for each vertex are
			// averaged over all faces, so as to produce a "smooth" look
			//
			if(!hasNormal) {
				if(numFaces > 0) {
					GLuint i1, i2, i3;
					float n[3];
					for(unsigned int face = 0; face < numFaces; ++face) {
						i1 = indicies[face*3 + 0];
						i2 = indicies[face*3 + 1];
						i3 = indicies[face*3 + 2];

						// TODO determine vertex winding order
						crossProduct(&verticies[i1*9], &verticies[i2*9], &verticies[i3*9], n);
						for(int i = 0; i < 3; ++i) {
							verticies[i1*9 + 3 + i] += n[i];
							verticies[i2*9 + 3 + i] += n[i];
							verticies[i3*9 + 3 + i] += n[i];
						}
					}

					for(unsigned int vertex = 0; vertex < numVerticies; ++vertex) {
						float d = std::sqrt(verticies[vertex*9 + 3]*verticies[vertex*9 + 3]
											+ verticies[vertex*9 + 4]*verticies[vertex*9 + 4]
											+ verticies[vertex*9 + 5]*verticies[vertex*9 + 5]);

						if(d > 1e-10) {
							d = 1.0 / d;
							verticies[vertex*9 + 3] *= d;
							verticies[vertex*9 + 4] *= d;
							verticies[vertex*9 + 5] *= d;
						}
					}
				} else {
					for(unsigned int vertex = 0; vertex < numVerticies; ++vertex) {
						verticies[vertex*9 + 3] = 0;
						verticies[vertex*9 + 4] = 0;
						verticies[vertex*9 + 5] = 1;
					}
				}
			}

			//
			//
			//
			ui->actionView_Points->trigger();
			ui->pointsViewScene->setPoints(verticies, indicies);
		} else {
			QMessageBox::warning(
					this,
					tr("Cannot Perform Action"),
					tr("Could not open the specified file.") );
		}
	}
}

#ifdef HAS_IMAGE_CAPTURE
//---------------------------------------------------------------------

void MainWindow::captureImages_Finished() {
	ui->statusBar->showMessage(tr("Images captured! (count = %1)").arg(ciThread.images().size()), 2000);
	ui->actionView_Images->trigger();
	ui->capturedImagesScene->setImages(ciThread.images());
}

//---------------------------------------------------------------------

void MainWindow::captureCalibrationImages_Finished() {
	QString fname = userSettings.value("InitialCaptureMultipleImagesName").toString();

	if(ciThread.images().size() > 0) {
		++currentImageIndex;
		saveCapturedImages(fname.arg("%1", QString::number(currentImageIndex)), 50);
	}

	hide();
	qApp->processEvents();
	ciThread.msleep(2000);
	show();
	qApp->processEvents();

	if(currentImageIndex < numImagesToCapture) {
		if(currentImageIndex + 1 == numImagesToCapture)
			ciThread.setShouldPowerDown(true);

		ui->statusBar->showMessage(QString("Capturing image %1 of %2. Please wait...").arg(currentImageIndex + 1).arg(numImagesToCapture));
		ciThread.msleep(250);
		ciThread.start();
	} else {
		ui->statusBar->showMessage("Images captured!", 1000);
		ui->actionView_Images->trigger();
		ui->capturedImagesScene->setImages(ciThread.images());
	}
}

//---------------------------------------------------------------------

void MainWindow::on_actionSave_Captured_Images_triggered() {
	QString initialDir =
			userSettings.contains("InitialCapturedImagesName")
			? userSettings.value("InitialCapturedImagesName").toString()
			: QDir::homePath() + "/cam%1.jpg";

	bool ok;
	QString fname = QInputDialog::getText(
			this,
			tr("Enter Filename"),
			tr("Enter filename pattern (use %1 to specify camera index; required):"),
			QLineEdit::Normal,
			initialDir,
			&ok);

	if(ok && !fname.isEmpty()) {
		if(!fname.contains("%1")) {
			QMessageBox::critical(
					this,
					tr("Improper Format"),
					tr("Your specified filename must contain '%1', which will specify where to place the camera index in the filename."));
			return;
		}

		userSettings.setValue("InitialCapturedImagesName", fname);


		QStringList list = fname.split('/', QString::SkipEmptyParts);
		list.removeLast();
		QDir(list.join("/")).mkpath(".");
		saveCapturedImages(fname);
	}
}

//---------------------------------------------------------------------

void MainWindow::saveCapturedImages(QString filenameTemplate, int darkThreshold) const {
	typedef CaptureImagesThread<unsigned char>::ImageType ImageType;
	std::vector<ImageType> &image_data = ciThread.images();

	for(size_t index = 0; index < image_data.size(); ++index) {
		ImageType img = image_data[index];

		// Get rid of most of the noise in the dark regions. Dramatically improves the compression
		// ratio of PNGs (since RLE is used)
		if(false && darkThreshold > 1) {
			for(int i = 0; i < 3*img.width*img.height; i += 3) {
				unsigned char &r = img.data[i + 0];
				unsigned char &g = img.data[i + 1];
				unsigned char &b = img.data[i + 2];
				if(r < darkThreshold || r + 15 < g || r + 15 < b || g + 15 < b || (r > 150 && b > 150))
					r = g = b = 0;
			}
		}

		//
		QImage qimg(img.data.get(), img.width, img.height, QImage::Format_RGB888);
		qimg.save( filenameTemplate.arg(index, 8, 10, QChar('0')) );
	}
}

//---------------------------------------------------------------------

void MainWindow::on_actionCapture_Images_triggered() {
	ui->statusBar->showMessage("Capturing images. Please wait...");

	connect(&ciThread, SIGNAL(finished()), this, SLOT(captureImages_Finished()));
	ciThread.setFPS(-1);
	ciThread.setExposureTime(100);
	ciThread.setGain(15);
	ciThread.msleep(10000);
	ciThread.start();
}

//---------------------------------------------------------------------

void
MainWindow::on_actionCapture_Calibration_Images_triggered() {
	bool ok;
	numImagesToCapture = QInputDialog::getInt(
			this,
			tr("Enter Number"),
			tr("Enter the number of images to capture:"),
			1,
			1, 500, 1,
			&ok);

	if(!ok)
		return;

	QString initialDir =
			userSettings.contains("InitialCaptureMultipleImagesName")
			? userSettings.value("InitialCaptureMultipleImagesName").toString()
			: QDir::homePath() + "/cam%1_%2.jpg";

	QString fname = QInputDialog::getText(
			this,
			tr("Enter Filename"),
			tr("Enter filename pattern (use %1 to specify camera index and %2 to specify image index; both are required):"),
			QLineEdit::Normal,
			initialDir,
			&ok);

	if(ok && !fname.isEmpty()) {
		if(!fname.contains("%1") || !fname.contains("%2")) {
			QMessageBox::critical(
					this,
					tr("Improper Format"),
					tr("Your specified filename must contain '%1', which will specify where to place the camera index in the filename, and '%2', which will specify the image index."));
			return;
		}

		userSettings.setValue("InitialCaptureMultipleImagesName", fname);

		QStringList list = fname.split('/', QString::SkipEmptyParts);
		list.removeLast();
		QDir(list.join("/")).mkpath(".");

		connect(&ciThread, SIGNAL(finished()), this, SLOT(captureCalibrationImages_Finished()));

		currentImageIndex = 0;
		ui->statusBar->showMessage(QString("Capturing image 1 of %1. Please wait...").arg(numImagesToCapture));

		ciThread.setShouldPowerDown(false);
		ciThread.setExposureTime(50);
		ciThread.setGain(15);
		ciThread.msleep(5000);
		ciThread.start();
	}
}

//---------------------------------------------------------------------

typedef struct fpair_ {
	float exposure, gain;
	fpair_(float exposure, float gain) : exposure(exposure), gain(gain) { }
} fpair;

void
MainWindow::on_actionCapture_Multi_Exposure_Images_triggered() {
	bool ok;
	QString stringTimes = QInputDialog::getText(
			this,
			tr("Enter Exposure Times"),
			tr("Enter the exposure times/gains to capture:"),
			QLineEdit::Normal,
			"1/5,2/5,4/5,8/5,16/5,32/5,64/5",
			&ok);

	if(!ok)
		return;

	QStringList listPairs = stringTimes.split(',');
	QList<fpair> times;
	foreach(const QString &stime, listPairs) {
		QStringList listExposureGain = stime.split('/');
		if(listExposureGain.size() == 2) {
			bool ok1, ok2;
			float time = listExposureGain[0].trimmed().toFloat(&ok1);
			float gain = listExposureGain[1].trimmed().toFloat(&ok2);
			if(ok1 && ok2)
				times.append(fpair(time, gain));
		}
	}

	if(times.size() == 0)
		return;

	//
	QString initialDir =
			userSettings.contains("InitialCaptureMultipleExposureName")
			? userSettings.value("InitialCaptureMultipleExposureName").toString()
			: QDir::homePath() + "/cam%1_%2.jpg";

	QString fname = QInputDialog::getText(
			this,
			tr("Enter Filename"),
			tr("Enter filename pattern (use %1 to specify camera index and %2 to specify exposure index; both are required):"),
			QLineEdit::Normal,
			initialDir,
			&ok);

	if(ok && !fname.isEmpty()) {
		if(!fname.contains("%1") || !fname.contains("%2")) {
			QMessageBox::critical(
					this,
					tr("Improper Format"),
					tr("Your specified filename must contain '%1', which will specify where to place the camera index in the filename, and '%2', which will specify the exposure index."));
			return;
		}

		userSettings.setValue("InitialCaptureMultipleExposureName", fname);

		QStringList list = fname.split('/', QString::SkipEmptyParts);
		list.removeLast();
		QDir(list.join("/")).mkpath(".");

		ui->statusBar->showMessage(QString("Capturing images. Please wait..."));

		int index = 1;
		foreach(const fpair &pair, times) {
			ciThread.setFPS(5);
			ciThread.setGain(pair.gain);
			ciThread.setExposureTime(pair.exposure);
			ciThread.msleep(1000);
			ciThread.start();
			ciThread.wait();
			saveCapturedImages(fname.arg("%1", QString::number(index)));
			index++;
		}

		captureImages_Finished();
	}
}
#endif
//---------------------------------------------------------------------

void MainWindow::on_actionView_Nothing_triggered() {
	ui->actionView_Nothing->setEnabled(false);
	ui->actionView_Camera_Layout->setEnabled(true);
	ui->actionView_Images->setEnabled(true);
	ui->actionView_Points->setEnabled(true);
	ui->stackedWidget->setCurrentWidget(ui->stereoScrollArea);
}

//---------------------------------------------------------------------

void MainWindow::on_actionView_Camera_Layout_triggered() {
	ui->actionView_Nothing->setEnabled(true);
	ui->actionView_Camera_Layout->setEnabled(false);
	ui->actionView_Images->setEnabled(true);
	ui->actionView_Points->setEnabled(true);
	ui->stackedWidget->setCurrentWidget(ui->cameraLayoutScene);
}

//---------------------------------------------------------------------

void MainWindow::on_actionView_Images_triggered() {
	ui->actionView_Nothing->setEnabled(true);
	ui->actionView_Camera_Layout->setEnabled(true);
	ui->actionView_Images->setEnabled(false);
	ui->actionView_Points->setEnabled(true);
	ui->stackedWidget->setCurrentWidget(ui->capturedImagesScene);
}

//---------------------------------------------------------------------

void MainWindow::on_actionView_Points_triggered() {
	ui->actionView_Nothing->setEnabled(true);
	ui->actionView_Camera_Layout->setEnabled(true);
	ui->actionView_Images->setEnabled(true);
	ui->actionView_Points->setEnabled(false);
	ui->stackedWidget->setCurrentWidget(ui->pointsViewScene);
}

//---------------------------------------------------------------------

void MainWindow::on_actionShowHide_Project_Explorer_triggered(bool vis) {
	if(sender() == ui->actionShowHide_Project_Explorer) {
		vis = !projectExplorerDock->isVisible();
		projectExplorerDock->setVisible(vis);
	}

	if(vis)
		ui->actionShowHide_Project_Explorer->setText(tr("Hide Project Explorer"));
	else
		ui->actionShowHide_Project_Explorer->setText(tr("Show Project Explorer"));
}

//---------------------------------------------------------------------

void MainWindow::on_actionShowHide_Inspector_triggered(bool vis) {
	if(sender() == ui->actionShowHide_Inspector) {
		vis = !inspectorDock->isVisible();
		inspectorDock->setVisible(vis);
	}

	if(vis)
		ui->actionShowHide_Inspector->setText(tr("Hide Inspector"));
	else
		ui->actionShowHide_Inspector->setText(tr("Show Inspector"));
}

//---------------------------------------------------------------------

void MainWindow::on_actionShowHide_Task_List_triggered(bool vis) {
	if(sender() == ui->actionShowHide_Task_List) {
		vis = !taskListDock->isVisible();
		taskListDock->setVisible(vis);
	}

	if(vis)
		ui->actionShowHide_Task_List->setText(tr("Hide Task List"));
	else
		ui->actionShowHide_Task_List->setText(tr("Show Task List"));
}

//---------------------------------------------------------------------

void MainWindow::setProject(QString fname) {
	try {
		project.reset(new Project(fname));

		// Show windows and such
		if(!projectExplorer->isVisible())
			ui->actionShowHide_Project_Explorer->trigger();

		// Set camera locations
		ui->cameraLayoutScene->setProject(project);

		//
		ui->actionFind_Features->setEnabled(true);
		ui->actionFind_Feature_Correspondences->setEnabled(true);
		ui->actionCalibrate_Cameras->setEnabled(true);
		ui->actionCreate_HDR_Image->setEnabled(true);
		ui->actionSave->setEnabled(true);

		//
		setWindowFilePath(fname);
		addRecentFile(fname);

		//
		emit projectLoaded(project);
	} catch(std::runtime_error &e) {
		QMessageBox::warning(
				this,
				tr("Error Loading Project"),
				tr(e.what()) );
	}
}

//---------------------------------------------------------------------

void MainWindow::on_actionNew_triggered() {
	if(project) {
		// TODO ask whether or not to save existing project
	}

	setProject(QString());
}

//---------------------------------------------------------------------

void MainWindow::on_actionOpen_triggered() {
	// Ask for filename
	QString initialDir =
			userSettings.contains("InitialProjectDir")
			? userSettings.value("InitialProjectDir").toString()
			: QDir::homePath();

	QString fname = QFileDialog::getOpenFileName(this,
	                                             tr("Open Project File"),
	                                             initialDir,
	                                             tr("XML Files (*.xml)"));

	//
	if(!fname.isNull()) {
		userSettings.setValue("InitialProjectDir", QDir(fname).absolutePath());
		setProject(fname);
	}
}

//---------------------------------------------------------------------

void MainWindow::openRecentFile(int index) {
	// TODO check for modified document
	if(index >= 0 && index < recentFiles.size())
		setProject(recentFiles.at(index));
}

void MainWindow::addRecentFile(const QString &fname) {
	if(fname.isNull())
		return;

	//
	recentFiles.removeAll(fname);
	recentFiles.prepend(fname);
	userSettings.setValue("recentFileList", recentFiles);
	updateRecentFiles();
}

void MainWindow::updateRecentFiles() {
	// Limit list size to the number of stored actions
	while(recentFiles.size() > NUM_RECENT_FILES)
		recentFiles.removeLast();

	// Update menu
	for(int index = 0; index < recentFiles.size(); ++index) {
		recentFileActions[index]->setText(recentFiles.at(index));
		recentFileActions[index]->setVisible(true);
	}

	for(int index = recentFiles.size(); index < NUM_RECENT_FILES; ++index)
		recentFileActions[index]->setVisible(false);
}

void MainWindow::clearRecentFiles() {
	recentFiles.clear();
	updateRecentFiles();
}

//---------------------------------------------------------------------

void MainWindow::saveProjectToFile(QString path) const {
	QDomDocument *xml = project->toXML();

	QFile fout(path);
	if(fout.open(QFile::WriteOnly)) {
		fout.write(xml->toByteArray());
		fout.close();
	}
}

//---------------------------------------------------------------------

void MainWindow::on_actionSave_triggered() {
	if(windowFilePath().isNull())
		ui->actionSave_As->trigger();
	else
		saveProjectToFile(windowFilePath());
}

//---------------------------------------------------------------------

void MainWindow::on_actionSave_As_triggered() {
	QString filePath = QFileDialog::getSaveFileName(this,
	                                                tr("Save Project As"),
	                                                windowFilePath(),
	                                                "XML Files (*.xml)");

	if(!filePath.isNull()) {
		saveProjectToFile(filePath);
		setWindowFilePath(filePath);
		project->setProjectPath(filePath);
		addRecentFile(filePath);
	}
}

//---------------------------------------------------------------------

void MainWindow::cameraSelected(CameraPtr cam) {
	ui->cameraLayoutScene->setSelectedCamera(cam);
	if(cam) {
		cameraInfoWidget->setEnabled(true);
		inspector->setCurrentWidget(cameraInfoWidget);
	}
}

//---------------------------------------------------------------------

void MainWindow::imageSetSelected(ImageSetPtr imageSet) {
	ui->cameraLayoutScene->setSelectedCamera(CameraPtr());
	if(imageSet) {
		imageSetTable->setEnabled(true);
		inspector->setCurrentWidget(imageSetTable);

		std::vector<QString> imagePaths;
		foreach(ProjectImagePtr image, imageSet->images())
			imagePaths.push_back(image->file());
		ui->capturedImagesScene->setImages(imagePaths);
	}
}

//---------------------------------------------------------------------

void MainWindow::on_actionRun_PMVS_triggered() {
	// Ask for filename
	QString initialDir =
			userSettings.contains("InitialPMVSDir")
			? userSettings.value("InitialPMVSDir").toString()
			: QDir::homePath();

	QString fname = QFileDialog::getOpenFileName(
			this,
			tr("Open PMVS Input File"),
			initialDir);

	if(!fname.isNull()) {
		userSettings.setValue("InitialPMVSDir", fname);

		PMVSDialog dialog;
		dialog.runPMVS(fname);
		dialog.exec();
	}
}

//---------------------------------------------------------------------

void MainWindow::on_actionDump_Projection_Data_For_PMVS_triggered() {
	// Ask for filename
	QString initialDir =
			userSettings.contains("InitialPMVSDir")
			? userSettings.value("InitialPMVSDir").toString()
			: QDir::homePath();

	QString fname = QFileDialog::getExistingDirectory(
			this,
			tr("Open PMVS Input File"),
			initialDir);

	if(!fname.isNull()) {
		userSettings.setValue("InitialPMVSDir", fname);

		foreach(CameraPtr cam, project->cameras()) {
			QFile projectionFile(QString("%1/%2.txt").arg(fname, cam->id()));
			if(projectionFile.open(QFile::WriteOnly)) {
				QTextStream projectionStream(&projectionFile);
				projectionStream << "CONTOUR\n";
				for(int row = 0; row < 3; ++row) {
					projectionStream << cam->P()(row, 0) << ' '
					                 << cam->P()(row, 1) << ' '
					                 << cam->P()(row, 2) << ' '
					                 << cam->P()(row, 3) << '\n';
				}
			}
		}
	}
}

//---------------------------------------------------------------------
#ifdef HAS_HDR
void MainWindow::on_actionCreate_HDR_Image_triggered() {
	// TODO non-fixed values (selection dialog)
	CameraPtr camera = project->camera("1");
	ImageSetPtr imageSet = project->imageSet(0);

	if(camera && imageSet) {
		MultiExposureToHDR hdrCreator(camera, imageSet);
		if(!hdrCreator.writeHDR("out.hdr")) // TODO allow specifying output file
			qDebug() << "failed";
	} else
		qDebug() << "failed";
}
#endif
//---------------------------------------------------------------------

void MainWindow::on_actionConvert_RAW_images_triggered() {
	bool ok = false;
	int w = QInputDialog::getInt(this, tr("Enter Width"), tr("Enter RAW image width:"), 1, 1, 0xFFFF, 1, &ok);
	if(ok) {
		int h = QInputDialog::getInt(this, tr("Enter height"), tr("Enter RAW image height:"), 1, 1, 0xFFFF, 1, &ok);
		if(ok) {
			QString dir =
				QFileDialog::getExistingDirectory(
					this,
					tr("Select Directory Containing RAW images"),
					QDir::homePath());

			QDirIterator it(dir, QDirIterator::Subdirectories);
			while(it.hasNext()) {
				it.next();

				QFileInfo finfo = it.fileInfo();
				if(!finfo.isFile() || !finfo.isReadable())
					continue;

				if(finfo.suffix() != "raw")
					continue;

				if(finfo.size() != w*h) {
					QDir(finfo.absolutePath()).remove(it.fileName());
					continue;
				}

				//
				QFile fin(it.filePath());
				if(fin.open(QFile::ReadOnly)) {
					QByteArray data = fin.readAll();
					if(!data.isEmpty()) {
						boost::scoped_array<unsigned char> outData(new unsigned char[3*w*h]);
						rawImageToRGB_es(
								reinterpret_cast<const unsigned char *>(data.constData()),
								outData.get(),
								w, h);

						// Change filename to have png as the extension
						QString outFileName = QString("%1.png").arg(finfo.completeBaseName());
						QString outFile = finfo.absoluteDir().absoluteFilePath(outFileName);
						QImage(outData.get(), w, h, QImage::Format_RGB888).save(outFile);
					}

					QDir(finfo.absolutePath()).remove(it.fileName());
				}
			}
		}
	}
}

//---------------------------------------------------------------------

void MainWindow::on_actionFind_Features_triggered() {
	FindFeaturesDialog dlg(project, this);
	dlg.exec();
}

//---------------------------------------------------------------------

void MainWindow::on_actionFind_Feature_Correspondences_triggered() {
	//
	std::vector<CameraPtr> cameras;
	foreach(CameraPtr cam, project->cameras())
		cameras.push_back(cam);

	//
	foreach(ImageSetPtr imageSet, project->imageSets()) {
		for(size_t cam1_index = 0; cam1_index < cameras.size(); ++cam1_index) {
			CameraPtr cam1 = cameras[cam1_index];
			for(size_t cam2_index = cam1_index + 1; cam2_index < cameras.size(); ++cam2_index) {
				CameraPtr cam2 = cameras[cam2_index];

				ProjectImagePtr img1 = imageSet->defaultImageForCamera(cam1);
				ProjectImagePtr img2 = imageSet->defaultImageForCamera(cam2);

				Features &features1 = project->features().features(img1);
				Features &features2 = project->features().features(img2);

				Correspondences &correspondences = project->features().correspondences(img1, img2);
				correspondences = findCorrespondences(features1, features2);
			}
		}
	}
}

//---------------------------------------------------------------------

void MainWindow::on_actionCalibrate_Cameras_triggered() {
	CalibrationSetupDialog dlg(project, this);
	dlg.exec();
}

//---------------------------------------------------------------------

void MainWindow::removeTask(const Task *task) {
	// TODO consider not removing, but rather having a means of clearing
	//      completed tasks

	if(QSystemTrayIcon::supportsMessages())
		trayIcon->showMessage(tr("Task completed"),
							 tr("The \"%1\" task has completed.").arg(task->title()),
							 QSystemTrayIcon::NoIcon,
							 5000);
	else
		QApplication::alert(this, 1000);

	// TODO Needs some fixin'
	QLayout *layout = taskListLayout;
	for(int row = 0; row < layout->count() - 1; ++row) {
		if(static_cast<TaskProgressWidget *>(layout->itemAt(row)->widget())->task().get() == task) {
			layout->takeAt(row)->widget()->deleteLater();
			break;
		}
	}
}

//---------------------------------------------------------------------

void MainWindow::customEvent(QEvent *e) {
	if(e->type() == NewTaskEvent::TYPE) {
		TaskPtr task = static_cast<NewTaskEvent *>(e)->task();

		// Create the widget for showing progress
		TaskProgressWidget *tpw = new TaskProgressWidget(task, taskList);
		taskListLayout->insertWidget(taskListLayout->count() - 1, tpw);
		taskListDock->show();

		// Throw task into a thread
		QThread *taskThread = new QThread;
		taskThread->connect(taskThread, SIGNAL(finished()), SLOT(deleteLater()));
		task->connect(taskThread, SIGNAL(started()), SLOT(run()));
		connect(task.get(), SIGNAL(finished(const Task *)), SLOT(removeTask(const Task *)));

		// Start the task
		task->moveToThread(taskThread);
		taskThread->start();

		// TODO Consider using QtConcurrent namespace

		// TODO consider showing a blocking dialog first, with a button to
		//      allow the user to place this task in the background
	}
}

//---------------------------------------------------------------------

void MainWindow::prepareProjectMenu() {
	bool camSelected = (projectExplorer->selectedCamera() ? true : false);
	bool imageSetSelected = (projectExplorer->selectedImageSet() ? true : false);
	bool imageSelected = (projectExplorer->selectedImage() ? true : false);

	ui->actionRemove_Camera->setEnabled(camSelected);
	ui->actionRemove_Image_Set->setEnabled(imageSetSelected);
	ui->actionNew_Image->setEnabled(imageSetSelected);
	ui->actionRemove_Image->setEnabled(imageSelected);
}

//---------------------------------------------------------------------

void MainWindow::showProjectMenu(const QPoint &p) {
	ui->menuProject->exec(projectExplorer->mapToGlobal(p));
}

//---------------------------------------------------------------------

void MainWindow::on_actionNew_Camera_triggered() {
	QString id = QInputDialog::getText(this,
	                                   tr("New Camera"),
	                                   tr("Enter an identifier for the camera:"));

	id = id.trimmed();
	if(id.isNull()) {
		QMessageBox::critical(this,
		                      tr("Empty Identifier"),
		                      tr("The camera's identifier cannot be empty!"));
	} else if(project->cameras().contains(id)) {
		QMessageBox::critical(this,
		                      tr("Camera Exists"),
		                      tr("A camera already exists with identitifer `%1`").arg(id));
	} else {
		CameraPtr cam = std::make_shared<Camera>(id, id);
		project->addCamera(cam);
		projectExplorer->editCamera(cam);
	}
}

//---------------------------------------------------------------------

void MainWindow::on_actionRemove_Camera_triggered() {
	if(CameraPtr cam = projectExplorer->selectedCamera()) {
		// Check if an image references this camera, and if so be sure to ask
		// the user whether or not s/he wants to proceed with the removal
		QMessageBox::StandardButton ret = QMessageBox::No;
		foreach(ImageSetPtr imageSet, project->imageSets()) {
			if(imageSet->hasImageForCamera(cam)) {
				ret = QMessageBox::information(this,
				                               tr("Remove Images"),
				                               tr("Also remove images that reference this camera?"),
				                               QMessageBox::Yes | QMessageBox::No);

				break;
			}
		}

		project->removeCamera(cam, ret == QMessageBox::Yes);
	}
}

//---------------------------------------------------------------------

void MainWindow::on_actionNew_Image_Set_triggered() {
	//
	QString id = QInputDialog::getText(this,
	                                   tr("New Image Set"),
	                                   tr("Enter an identifier for the image set:"));

	id = id.trimmed();
	if(id.isNull()) {
		QMessageBox::critical(this,
		                      tr("Empty Identifier"),
		                      tr("The image set's identifier cannot be empty!"));
	} else if(project->imageSets().contains(id)) {
		QMessageBox::critical(this,
		                      tr("Image Set Exists"),
		                      tr("An image set already exists with identitifer `%1`").arg(id));
	} else {
		ImageSetPtr imageSet = std::make_shared<ImageSet>(id, id);
		project->addImageSet(imageSet);
		projectExplorer->editImageSet(imageSet);
	}
}

//---------------------------------------------------------------------

void MainWindow::on_actionNew_Image_Set_From_Files_triggered() {
	//
	QStringList imageFilter;
	foreach(QByteArray format, QImageReader::supportedImageFormats())
		imageFilter << format;

	//
	QString initialDir =
			userSettings.contains("InitialNewImageSetFromDirectoryDir")
			? userSettings.value("InitialNewImageSetFromDirectoryDir").toString()
			: QDir::homePath();

	QStringList images = QFileDialog::getOpenFileNames(this,
	                                                   tr("New Image Set"),
	                                                   initialDir,
	                                                   tr("Images (*.%1)").arg(imageFilter.join(" *.")));

	if(!images.isEmpty()) {
		QDir path(images.front());
		path.cdUp();

		userSettings.setValue("InitialNewImageSetFromDirectoryDir", path.path());

		// Find an id that doesn't exist
		QString base = path.dirName();
		QString id = base;
		int index = 1;
		while(project->imageSet(id)) {
			id = base + QString::number(index);
			++index;
		}

		// Try to relate cameras to images. We use a simple check where, if
		// an image's path contains a camera's name/id, that camera is
		// associated to the camera
		ImageSetPtr imageSet = std::make_shared<ImageSet>(id, id);
		imageSet->setRoot(path.absolutePath());

		foreach(QString imagePath, images) {
			CameraPtr imageCam;
			foreach(CameraPtr cam, project->cameras()) {
				if(imagePath.contains(cam->name(), Qt::CaseInsensitive)) {
					imageCam = cam;
					break;
				} else if(imagePath.contains(cam->id(), Qt::CaseInsensitive)) {
					imageCam = cam;
					break;
				}
			}

			ProjectImagePtr image = std::make_shared<ProjectImage>(imagePath);
			imageSet->addImageForCamera(imageCam, image);
		}

		//
		project->addImageSet(imageSet);
		projectExplorer->editImageSet(imageSet);
	}
}

//---------------------------------------------------------------------

void MainWindow::on_actionRemove_Image_Set_triggered() {
	if(ImageSetPtr imageSet = projectExplorer->selectedImageSet())
		project->removeImageSet(imageSet);
}

//---------------------------------------------------------------------

void MainWindow::on_actionNew_Image_triggered() {
	//
	ImageSetPtr imageSet = projectExplorer->selectedImageSet();
	if(!imageSet) {
		QMessageBox::critical(this,
		                      tr("Image Set Required"),
		                      tr("You must select an image set before you can add an image!"));
		return;
	}

	//
	QString initialDir =
			userSettings.contains("InitialNewImageDir")
			? userSettings.value("InitialNewImageDir").toString()
			: QDir::homePath();

	//
	QStringList imageFilter;
	foreach(QByteArray format, QImageReader::supportedImageFormats())
		imageFilter << format;

	QString filter = tr("Images (*.%1)").arg(imageFilter.join(" *."));

	//
	QString path = QFileDialog::getOpenFileName(this,
	                                            tr("New Image"),
	                                            initialDir,
	                                            filter);

	if(!path.isNull()) {
		userSettings.setValue("InitialNewImageDir", path);

		CameraPtr cam;
		if(!project->cameras().isEmpty())
			cam = project->cameras().begin().value();

		ProjectImagePtr image = std::make_shared<ProjectImage>(path);
		imageSet->addImageForCamera(cam, image);
		projectExplorer->editImage(image);
	}
}

//---------------------------------------------------------------------

void MainWindow::on_actionRemove_Image_triggered() {
	if(ImageSetPtr imageSet = projectExplorer->selectedImageSet()) {
		if(ProjectImagePtr image = projectExplorer->selectedImage())
			imageSet->removeImage(image);
	}
}

//---------------------------------------------------------------------

void MainWindow::on_actionAbout_StereoReconstruction_triggered() {
	AboutDialog dlg(this);
	dlg.exec();
}

//---------------------------------------------------------------------
