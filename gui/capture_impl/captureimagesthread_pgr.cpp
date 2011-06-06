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
#include "../captureimagesthread.hpp"
#include "rawimagereader.h"

#include <QDebug>
#include <FlyCapture2.h>

//---------------------------------------------------------------------

template <class ImageDataType>
CaptureImagesThread<ImageDataType>::CaptureImagesThread()
	: width(1024), height(768)
	, fps(-1)
	, exposure(-1)
	, gain(-1)
	, powerDown(true)
{ }

//---------------------------------------------------------------------

int numChannels(FlyCapture2::PixelFormat fmt) {
	switch(fmt) {
	case FlyCapture2::PIXEL_FORMAT_RAW8:
	case FlyCapture2::PIXEL_FORMAT_RAW12:
	case FlyCapture2::PIXEL_FORMAT_RAW16:
	case FlyCapture2::PIXEL_FORMAT_MONO8:
	case FlyCapture2::PIXEL_FORMAT_MONO12:
	case FlyCapture2::PIXEL_FORMAT_MONO16:
	case FlyCapture2::PIXEL_FORMAT_S_MONO16:
		return 1;
	default:
		return 3;
	}
}
//---------------------------------------------------------------------

template <class T>
struct ImageFormat {
	static const FlyCapture2::PixelFormat Source;
	static const FlyCapture2::PixelFormat Dest;
};

template <>
const FlyCapture2::PixelFormat ImageFormat<unsigned char>::Source = FlyCapture2::PIXEL_FORMAT_RAW8;

template <>
const FlyCapture2::PixelFormat ImageFormat<unsigned char>::Dest   = FlyCapture2::PIXEL_FORMAT_RGB8;

//---------------------------------------------------------------------

template <>
const FlyCapture2::PixelFormat ImageFormat<unsigned short>::Source = FlyCapture2::PIXEL_FORMAT_RAW16;

template <>
const FlyCapture2::PixelFormat ImageFormat<unsigned short>::Dest   = FlyCapture2::PIXEL_FORMAT_RGB16;

//---------------------------------------------------------------------
namespace {
	// Misc
	const unsigned int MAX_CAMERAS = 16;
	const unsigned int MAX_DMA_CONTEXTS = 16;
	const unsigned int NUM_BUFFERS = 4;

	// Camera registers
	const int TRIGGER_MODE       = 0x830;
	const int IMAGE_RETRANSMIT   = 0x12E8;
	const int FRAME_INFO         = 0x12F8;
	const int CAMERA_POWER       = 0x610;
	const int SOFTWARE_TRIGGER   = 0x62C;
}
//---------------------------------------------------------------------

void deleteCamera(FlyCapture2::Camera *cam) {
	cam->StopCapture();
	cam->Disconnect();
	delete cam;
}

//---------------------------------------------------------------------

template <FlyCapture2::PixelFormat InType, FlyCapture2::PixelFormat OutType>
bool convertImage(FlyCapture2::Image &in, FlyCapture2::Image &out) {
	out.ReleaseBuffer();
	return (in.Convert(OutType, &out) == FlyCapture2::PGRERROR_OK);
}

// FlyCapture SDK doesn't do conversion to RGB16, so take care of this
template <>
bool convertImage
	<FlyCapture2::PIXEL_FORMAT_MONO16, FlyCapture2::PIXEL_FORMAT_RGB16>
	(FlyCapture2::Image &in, FlyCapture2::Image &out)
{
	size_t dataSize = in.GetRows() * in.GetCols();
	unsigned char *data = new unsigned char[dataSize * 3 * sizeof(unsigned short)];

	const unsigned short *srcData = reinterpret_cast<const unsigned short*>(in.GetData());
	unsigned short *destData = reinterpret_cast<unsigned short*>(data);

	for(size_t dataIndex = 0; dataIndex < dataSize; ++dataIndex, ++srcData, destData += 3) {
		destData[0] = destData[1] = destData[2] = (*srcData >> 4); // shift by 4 because only high 12 bits are useful
	}

	out.ReleaseBuffer();
	out.SetDimensions(
		in.GetRows(),
		in.GetCols(),
		in.GetCols()*sizeof(unsigned short)*3,
		FlyCapture2::PIXEL_FORMAT_RGB16,
		FlyCapture2::NONE);

	out.SetData(data, dataSize * 3 * sizeof(unsigned short));

	return true;
}

template <>
bool convertImage
	<FlyCapture2::PIXEL_FORMAT_RAW16, FlyCapture2::PIXEL_FORMAT_RGB16>
	(FlyCapture2::Image &in, FlyCapture2::Image &out)
{
	size_t dataSize = in.GetRows() * in.GetCols();
	unsigned char *data = new unsigned char[dataSize * 3 * sizeof(unsigned short)];

	unsigned short* srcData = reinterpret_cast<unsigned short*>(in.GetData());
	unsigned short* destData = reinterpret_cast<unsigned short*>(data);

	//
	for(size_t dataIndex = 0; dataIndex < dataSize; ++dataIndex, ++srcData)
		*srcData = (*srcData) >> 4; // shift by 4 because only high 12 bits are useful
	srcData = reinterpret_cast<unsigned short*>(in.GetData());

	//
	rawImageToRGB_es(srcData, destData, in.GetCols(), in.GetRows());

	out.ReleaseBuffer();
	out.SetDimensions(
		in.GetRows(),
		in.GetCols(),
		in.GetCols()*sizeof(unsigned short)*3,
		FlyCapture2::PIXEL_FORMAT_RGB16,
		FlyCapture2::NONE);

	out.SetData(data, dataSize * 3 * sizeof(unsigned short));

	return true;
}

//---------------------------------------------------------------------

template <class ImageDataType>
void CaptureImagesThread<ImageDataType>::run() {
	const FlyCapture2::PixelFormat IMG_SRC_FORMAT = ImageFormat<ImageDataType>::Source;
	const FlyCapture2::PixelFormat IMG_DST_FORMAT = ImageFormat<ImageDataType>::Dest;

	static int inImageLength = width * height * numChannels(IMG_SRC_FORMAT);
	static int outImageLength = width * height * numChannels(IMG_DST_FORMAT);

	std::shared_ptr<FlyCapture2::Camera> cameras[MAX_CAMERAS];
	FlyCapture2::BusManager bus;

	//
	// Find number of cameras
	//
	unsigned int numCameras = 0;
	if(bus.GetNumOfCameras(&numCameras) != FlyCapture2::PGRERROR_OK || numCameras == 0) {
		qDebug() << "Could not enumerate or find any cameras";
		return;
	}

	image_data.swap(std::vector<ImageType>(numCameras));


	//
	// Initialize buffers
	//
	DataType buffers(new ImageDataType[(inImageLength + 8192) * NUM_BUFFERS * numCameras]);
	ImageDataType *bufferPointers[NUM_BUFFERS * MAX_CAMERAS];
	for(unsigned int buffer = 0; buffer < NUM_BUFFERS * numCameras; ++buffer)
		bufferPointers[buffer] = buffers.get() + (inImageLength + 8192)*(buffer);

	//
	// Initialize camera contexts
	//
	{
		FlyCapture2::Property shutterProp;
		shutterProp.type = FlyCapture2::SHUTTER;
		shutterProp.absControl = (exposure >= 0);
		shutterProp.autoManualMode = (exposure < 0);
		shutterProp.absValue = exposure;
		shutterProp.onOff = true;

		FlyCapture2::Property gainProp;
		gainProp.type = FlyCapture2::GAIN;
		gainProp.absControl = (gain >= 0);
		gainProp.autoManualMode = (gain < 0);
		gainProp.absValue = gain;
		gainProp.onOff = true;

		FlyCapture2::Property wbProp;
		gainProp.type = FlyCapture2::WHITE_BALANCE;
		gainProp.absControl = false;
		gainProp.autoManualMode = false;
		gainProp.valueA = 576;
		gainProp.valueB = 640;
		gainProp.onOff = true;

		FlyCapture2::Property fpsProp;
		fpsProp.type = FlyCapture2::FRAME_RATE;
		fpsProp.absControl = (fps > 0);
		fpsProp.autoManualMode = (fps <= 0);
		fpsProp.absValue = fps;
		fpsProp.onOff = true;

		FlyCapture2::FC2Config cfg;
		cfg.grabMode = FlyCapture2::BUFFER_FRAMES;
		cfg.numBuffers = NUM_BUFFERS;
		cfg.grabTimeout = 1000;

		qDebug() << "Found " << numCameras << " cameras!\nInitializing cameras...";
		for(unsigned int camera = 0; camera < numCameras; ++camera) {
			qDebug() << "\tInitializing camera" << camera;

			// Initialize camera
			FlyCapture2::PGRGuid guid;
			if(bus.GetCameraFromIndex(camera, &guid) != FlyCapture2::PGRERROR_OK) {
				qDebug() << "\tCould not create camera context";
				continue;
			}

			cameras[camera] = std::shared_ptr<FlyCapture2::Camera>(new FlyCapture2::Camera, deleteCamera);
			if(cameras[camera]->Connect(&guid) != FlyCapture2::PGRERROR_OK) {
				qDebug() << "\tCould not initialize camera with bus index" << camera;
				continue;
			}

			// Power up cameras
			cameras[camera]->WriteRegister(CAMERA_POWER, 0x80000000);

			// Store timestamp
			cameras[camera]->WriteRegister(FRAME_INFO, 0x80000001);

			//
			cameras[camera]->SetProperty(&fpsProp);
			cameras[camera]->SetProperty(&shutterProp);
			cameras[camera]->SetProperty(&gainProp);
			cameras[camera]->SetProperty(&wbProp);

			//
			cameras[camera]->SetConfiguration(&cfg);
			cameras[camera]->SetUserBuffers(
					reinterpret_cast<unsigned char *>(bufferPointers[NUM_BUFFERS*camera]),
					inImageLength + 8192,
					NUM_BUFFERS);

			// TODO non-fixed width/height. Try to find max allowable.
			DataType data(new ImageDataType[3 * width * height]);
			image_data[camera] = ImageType(width, height, data);
		}
	}

	//
	// Start cameras
	//
	{
		// XXX maybe scan for possible settings (like max width/height)?
		FlyCapture2::Format7ImageSettings imageSettings;
		imageSettings.offsetX = 0;
		imageSettings.offsetY = 0;
		imageSettings.width = width;
		imageSettings.height = height;
		imageSettings.pixelFormat = IMG_SRC_FORMAT;

		qDebug() << "Grabbing images...";
		for(unsigned int camera = 0; camera < std::min(numCameras, MAX_DMA_CONTEXTS); ++camera) {
			if(image_data[camera].data) {
				// Start image capture
				if(cameras[camera]->SetFormat7Configuration(&imageSettings, 50.0f) != FlyCapture2::PGRERROR_OK
				   || cameras[camera]->StartCapture() != FlyCapture2::PGRERROR_OK)
				{
					qDebug() << "\tCould not start camera with bus index " << camera;
					image_data[camera].data.reset(0);
					continue;
				}
			}
		}
	}

	//
	// Sleep for a bit to allow cameras to sync and for internal processing to occur
	//
	msleep(1000);

	//
	// Grab images
	//
	std::vector<FlyCapture2::Image> images(numCameras);
	for(unsigned int camera = 0; camera < numCameras; ++camera) if(image_data[camera].data) {
		qDebug() << "\tGrabbing images for camera " << camera;

		if(cameras[camera]->RetrieveBuffer(&images[camera]) != FlyCapture2::PGRERROR_OK) {
			qDebug() << "\tCould not lock camera for latest with bus index " << camera;
			image_data[camera].data.reset();
			cameras[camera].reset();
			continue;
		}

		if(images[camera].GetData()) {
			unsigned long seconds = images[camera].GetTimeStamp().cycleSeconds;
			unsigned long count = images[camera].GetTimeStamp().cycleCount;
			qDebug() << '\t' << (seconds + count/8000.0);
		} else
			image_data[camera].data.reset(0);
	}

	//
	// Copy data to internal buffer
	//
	for(unsigned int camera = 0; camera < numCameras; ++camera) {
		// Power down
		if(cameras[camera]) {
			cameras[camera]->StopCapture();
			if(powerDown)
				cameras[camera]->WriteRegister(CAMERA_POWER, 0x00000000);
			cameras[camera]->Disconnect();
		}

		if(image_data[camera].data) {
			// The first four bytes contain the FRAME_INFO information, so get rid of this
			images[camera].GetData()[0] = 0;
			images[camera].GetData()[1] = 0;
			images[camera].GetData()[2] = 0;
			images[camera].GetData()[3] = 0;
			images[camera].SetColorProcessing(FlyCapture2::HQ_LINEAR);

			// Convert raw Bayer-tiled image to RGB
			FlyCapture2::Image tmp;
			if(convertImage<IMG_SRC_FORMAT, IMG_DST_FORMAT>(images[camera], tmp)) {
				const ImageDataType *data = reinterpret_cast<ImageDataType *>(tmp.GetData());
				std::copy(data, data + outImageLength, image_data[camera].data.get());
			} else {
				image_data[camera].data.reset();
			}
			tmp.ReleaseBuffer();
		}
	}
}

template class CaptureImagesThread<unsigned char>;
template class CaptureImagesThread<unsigned short>;

//---------------------------------------------------------------------
