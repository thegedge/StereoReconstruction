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
#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QSettings>

#include "captureimagesthread.hpp"
#include "util/c++0x.hpp"

//
// Forward declarations
//
namespace Ui { class MainWindow; }

class QListWidget;
class QSystemTrayIcon;
class QStackedWidget;
class QVBoxLayout;
class StereoWidget;
class SceneViewer;
class Task;

class CameraLayoutScene;
class CapturedImagesScene;
class PointsViewScene;

class ProjectExplorer;
class ImageSetTable;
class CameraInfoWidget;

FORWARD_DECLARE(Project);
FORWARD_DECLARE(Camera);
FORWARD_DECLARE(ImageSet);

//! Our main window class
class MainWindow : public QMainWindow {
    Q_OBJECT

signals:
	void projectLoaded(ProjectPtr project);

public:
    MainWindow(QWidget *parent = 0);
    ~MainWindow();

private:
	void setProject(QString projectPath);
	void saveProjectToFile(QString path) const;
	void saveCapturedImages(QString filenameTemplate, int darkThreshold = 0) const;

protected:
	void customEvent(QEvent *);

	//! Add a file to the recent files list
	void addRecentFile(const QString &fname);

	//! Update the recent file list to reflect MainWindow's stored recent list
	void updateRecentFiles();

private slots:
	//! .
	void on_actionDump_Projection_Data_For_PMVS_triggered();
	void removeTask(const Task *task);

	//! .
	void prepareProjectMenu();

	//! .
	void showProjectMenu(const QPoint &);

	//! .
	void openRecentFile(int index);
	void clearRecentFiles();

	// File menu
	void on_actionNew_triggered();
	void on_actionOpen_triggered();
	void on_actionSave_triggered();
	void on_actionSave_As_triggered();
	void on_actionConvert_RAW_images_triggered();
	void on_actionView_PLY_File_triggered();
    void on_actionExit_triggered();

	// View menu
	void on_actionView_Nothing_triggered();
	void on_actionView_Camera_Layout_triggered();
	void on_actionView_Images_triggered();
	void on_actionView_Points_triggered();
	void on_actionShowHide_Project_Explorer_triggered(bool vis = false);
	void on_actionShowHide_Inspector_triggered(bool vis = false);
	void on_actionShowHide_Task_List_triggered(bool vis = false);

	// Project menu
	void on_actionNew_Camera_triggered();
	void on_actionRemove_Camera_triggered();
	void on_actionNew_Image_Set_triggered();
	void on_actionNew_Image_Set_From_Files_triggered();
	void on_actionRemove_Image_Set_triggered();
	void on_actionNew_Image_triggered();
	void on_actionRemove_Image_triggered();

#ifdef HAS_IMAGE_CAPTURE
	// Images menu
	void on_actionCapture_Images_triggered();
	void on_actionCapture_Images_Bumblebee_triggered();
	void on_actionSave_Captured_Images_triggered();
	void on_actionCapture_Calibration_Images_triggered();
	void on_actionCapture_Multi_Exposure_Images_triggered();
#endif

	// Stereo menu
	void on_actionRun_PMVS_triggered();
	void on_actionFind_Features_triggered();
	void on_actionFind_Feature_Correspondences_triggered();
	void on_actionCalibrate_Cameras_triggered();

#ifdef HAS_HDR
	// HDR menu
	void on_actionCreate_HDR_Image_triggered();
#endif

private slots:
#ifdef HAS_IMAGE_CAPTURE
	// Image capture events
	void captureImages_Finished();
	void captureCalibrationImages_Finished();
#endif
	// Project explorer events
	void cameraSelected(CameraPtr cam);
	void imageSetSelected(ImageSetPtr imageSet);

	// Help menu
	void on_actionAbout_StereoReconstruction_triggered();

private:
	//
	Ui::MainWindow *ui;

	static const int NUM_RECENT_FILES = 10;
	QAction *recentFileActions[NUM_RECENT_FILES];

	//
	QSettings userSettings;
	QSystemTrayIcon *trayIcon;
	QStringList recentFiles;

	// Dockable wigets
	QStackedWidget *inspector;
	QWidget *taskList;
	QVBoxLayout *taskListLayout;

	QDockWidget *inspectorDock;
	QDockWidget *projectExplorerDock;
	QDockWidget *taskListDock;

	// Central widgets
	ProjectExplorer *projectExplorer;
	ImageSetTable *imageSetTable;
	CameraInfoWidget *cameraInfoWidget;

	//
	ProjectPtr project;

	//
#ifdef HAS_IMAGE_CAPTURE
	CaptureImagesThread<unsigned char> ciThread;
	int currentImageIndex, numImagesToCapture;
	int numCameras;
#endif
	//
	std::shared_ptr<CapturedImagesScene> sceneImages;
	std::shared_ptr<PointsViewScene>     scenePoints;
	std::shared_ptr<CameraLayoutScene>   sceneCameraLayout;
};


#endif // MAINWINDOW_H
