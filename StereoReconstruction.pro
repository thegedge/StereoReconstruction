#---------------------------------------------------------------------
#
# Copyright © 2011, Jason Gedge <gedge -at- ualberta -dot- ca>
#
# This file is part of StereoReconstruction.
#
# StereoReconstruction is free software: you can redistribute it and/or
# modify it under the terms of the GNU General Public License as published
# by the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# StereoReconstruction is distributed in the hope that it will be
# useful, but WITHOUT ANY WARRANTY; without even the implied warranty
# of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License along
# with StereoReconstruction. If not, see <http:www.gnu.org/licenses/>.
#
#---------------------------------------------------------------------
# See README for build instructions
#---------------------------------------------------------------------
TEMPLATE = app
TARGET = StereoReconstruction
CONFIG += c++11 warn_on precompile_header
QT += widgets opengl xml xmlpatterns printsupport
QMAKE_MAC_SDK = macosx10.9

exists(UserConfig.pri):include(UserConfig.pri)

#---------------------------------------------------------------------
isEmpty(MAIN_CPP):MAIN_CPP = main.cpp

SOURCES += $${MAIN_CPP} \
    $$files(gui/*.cpp) \
    $$files(gui/dialogs/*.cpp) \
    $$files(gui/widgets/*.cpp) \
    $$files(gui/gvitems/*.cpp) \
    $$files(project/*.cpp) \
    $$files(features/*.cpp) \
    $$files(hdr/*.c*) \
    $$files(stereo/*.cpp) \
    $$files(util/*.cpp) \
    $$files(util/rawimages/*.cpp)

HEADERS += $$files(gui/*.h*) \
    $$files(gui/dialogs/*.hpp) \
    $$files(gui/widgets/*.hpp) \
    $$files(gui/gvitems/*.hpp) \
    $$files(project/*.hpp) \
    $$files(features/*.hpp) \
    $$files(hdr/*.h*) \
    $$files(stereo/*.hpp) \
    $$files(util/*.hpp) \
    $$files(util/rawimages/*.hpp)

PRECOMPILED_HEADER = util/precompiled.hpp

FORMS += $$files(gui/forms/*.ui)

OTHER_FILES += shaders/*.* project/project.xsd GPL_HEADER LICENSE README.md

RESOURCES += resources.qrc
INCLUDEPATH += .

#---------------------------------------------------------------------
CONFIG(debug, debug|release) { 
	DEFINES *= DEBUG _DEBUG
    BUILD_PREFIX = debug
} else {
	DEFINES *= NDEBUG _NDEBUG
    BUILD_PREFIX = release
}

#---------------------------------------------------------------------

QMAKE_CXXFLAGS += -isystem /usr/local/include
QMAKE_CXXFLAGS += -Wno-c++98-compat -Wno-c++98-compat-pedantic -Wno-padded  \
                  -Wno-exit-time-destructors -Wno-unused-private-field
LIBS += -L/usr/local/lib

#---------------------------------------------------------------------
win32 { 
	DEFINES *= PLATFORM_WIN
	LIBS *= -lglew32
}
else:macx { 
	DEFINES *= PLATFORM_MAC
	LIBS *= -lGLEW
}

#---------------------------------------------------------------------
LIBS *= -lopencv_core -lopencv_highgui -lopencv_legacy \
        -lopencv_imgproc -lopencv_features2d -lopencv_calib3d \
        -lopencv_contrib -lopencv_nonfree

LIBS *= -lgsl

#---------------------------------------------------------------------
mrf {
	LIBS *= -lMRF
	DEFINES *= USE_MRF
}

#---------------------------------------------------------------------
sba {
	DEFINES *= USE_SBA
	LIBS *= -lsba
}

#---------------------------------------------------------------------
hdr { 
	LIBS *= -llapack
	LIBS *= -lIlmImf -lHalf # OpenEXR
	LIBS *= -lboost_system-mt -lboost_filesystem-mt
	DEFINES *= HAS_HDR
}

#---------------------------------------------------------------------
splats {
	DEFINES += USE_SPLATS
	DEFINES *= USE_OPENGL
}

#---------------------------------------------------------------------
tbb {
	DEFINES *= USE_TBB
	LIBS *= -ltbb -ltbbmalloc
} else:openmp {
	QMAKE_CXXFLAGS *= -fopenmp
	QMAKE_LFLAGS *= -fopenmp
	!msvc:LIBS *= -lgomp
	DEFINES *= USE_OPENMP
}

#---------------------------------------------------------------------
pgr { 
	DEFINES *= USING_PGR
	DEFINES *= HAS_IMAGE_CAPTURE
	SOURCES *= gui/capture_impl/*.cpp
	LIBS *= -lFlyCapture2 -ldigiclops -ltriclops
}

#---------------------------------------------------------------------
