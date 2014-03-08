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
#include "gui/mainwindow.hpp"
#include "ui_mainwindow.h"

#include "gui/captureimagesthread.hpp"

#include <digiclops.h>
#include <triclops.h>
#include <QDebug>

//---------------------------------------------------------------------
// Macro to check, report on, and handle Digiclops API error codes.
//
#define _HANDLE_DIGICLOPS_ERROR(functionname) \
	if ( de != DIGICLOPS_OK ) { \
		qDebug() <<  functionname << "() failed: \"" \
		         << ::digiclopsErrorToString(de) << '"'; \
		assert( false ); \
		return; \
	} \

//---------------------------------------------------------------------

typedef CaptureImagesThread<unsigned char>::ImageType  ImageType;
typedef CaptureImagesThread<unsigned char>::DataType   DataType;

ImageType convertTriclopsImage(const TriclopsColorImage &img) {
	DataType data(new unsigned char[3 * img.ncols * img.nrows]);
	ImageType ret(img.ncols, img.nrows, data);

	int index = 0;

	unsigned char *red = img.red;
	unsigned char *blue = img.blue;
	unsigned char *green = img.green;
	for(int row = 0; row < img.nrows; ++row) {
		unsigned char *red2 = red;
		unsigned char *blue2 = blue;
		unsigned char *green2 = green;
		for(int col = 0; col < img.ncols; ++col, index += 3) {
			data[index + 0] = red2[col];
			data[index + 1] = green2[col];
			data[index + 2] = blue2[col];
		}

		red += img.rowinc;
		blue += img.rowinc;
		green += img.rowinc;
	}
	return ret;
}

//---------------------------------------------------------------------


void MainWindow::on_actionCapture_Images_Bumblebee_triggered() {
	DigiclopsContext  digiclops;
	DigiclopsError    de;

	TriclopsContext   triclops;
	TriclopsInput     triclopsInput;
	TriclopsError     te;

	de  = digiclopsCreateContext( &digiclops );
	_HANDLE_DIGICLOPS_ERROR( "digiclopsCreateContext()" );

	// initialize Digiclops device 0
	de  = digiclopsInitialize( digiclops, 0 );
	_HANDLE_DIGICLOPS_ERROR( "digiclopsInitialize()" );

	// extract Triclops context from camera
	de = digiclopsGetTriclopsContextFromCamera( digiclops, &triclops );
	_HANDLE_DIGICLOPS_ERROR( "digiclopsGetTriclopsContextFromCamera()" );

	// start the Digiclops device streaming images
	de  = digiclopsStart( digiclops );
	_HANDLE_DIGICLOPS_ERROR( "digiclopsStart()" );

	// tell the Digiclops to provide left/right images
	de  = digiclopsSetImageTypes( digiclops, LEFT_IMAGE | RIGHT_IMAGE );
	_HANDLE_DIGICLOPS_ERROR( "digiclopsSetImageTypes()" );

	//de  = digiclopsSetImageResolution( digiclops, DIGICLOPS_HALF );
	de  = digiclopsSetImageResolution( digiclops, DIGICLOPS_FULL );
	_HANDLE_DIGICLOPS_ERROR( "digiclopsSetImageResolution()" );

	de  = digiclopsSetColorProcessing(digiclops, DIGICLOPS_EDGE_SENSING);
	_HANDLE_DIGICLOPS_ERROR( "digiclopsSetColorProcessing()" );

	// set the stereo resolution and perform required setup
	printf( "Setting up Triclops library...\n" );

	te = triclopsSetResolutionAndPrepare(triclops, 768, 1024, 768, 1024);
	//te = triclopsSetResolutionAndPrepare(triclops, 384, 512, 384, 512);
	if(te != TriclopsErrorOk) {
		printf( "TriclopsError: triclopsSetResolutionAndPrepare : %s\n", triclopsErrorToString( te ) );
		return;
	}

	// perform a simple profiling
	// grab images for approximately 10 seconds and determine the frame rate
	de = digiclopsGrabImage( digiclops );
	_HANDLE_DIGICLOPS_ERROR( "digiclopsGrabImage()" );

	//
	std::vector<ImageType> images(2);
	{
		de = digiclopsExtractTriclopsInput( digiclops, LEFT_IMAGE , &triclopsInput );
		_HANDLE_DIGICLOPS_ERROR( "digiclopsExtractTriclopsInput" );

		// process color images so that they are rectified
		TriclopsColorImage triclopsImage;
		triclopsRectifyColorImage(triclops, TriCam_LEFT, &triclopsInput, &triclopsImage);
		images[0] = convertTriclopsImage(triclopsImage);
	}
	{
		de = digiclopsExtractTriclopsInput( digiclops, RIGHT_IMAGE , &triclopsInput );
		_HANDLE_DIGICLOPS_ERROR( "digiclopsExtractTriclopsInput" );

		// process color images so that they are rectified
		TriclopsColorImage triclopsImage;
		triclopsRectifyColorImage(triclops, TriCam_RIGHT, &triclopsInput, &triclopsImage);
		images[1] = convertTriclopsImage(triclopsImage);
	}

	float val;
	triclopsGetBaseline(triclops, &val);
	qDebug() << val;
	triclopsGetFocalLength(triclops, &val);
	qDebug() << val;

	float row, col;
	triclopsGetImageCenter(triclops, &row, &col);
	qDebug() << col << row;

	ciThread.setImages(images);
	captureImages_Finished();

	// stop the Digiclops device streaming images
	de  = digiclopsStop( digiclops );
	_HANDLE_DIGICLOPS_ERROR( "digiclopsStop()" );

	de  = digiclopsDestroyContext( digiclops );
	_HANDLE_DIGICLOPS_ERROR( "digiclopsDestroyContext()" );
}
