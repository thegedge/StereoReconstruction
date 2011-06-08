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
#ifndef SHADERS_HPP
#define SHADERS_HPP

//! Load a shader of a specified type from a file
GLuint loadShader(GLenum type, const char *file);

//! Create an OpenGL program from the specified vertex/fragment shader files
void loadProgram(const char *vsFile,
                 const char *fsFile,
                 GLuint &vs,
                 GLuint &fs,
                 GLuint &program);

#endif // SHADERS_HPP
