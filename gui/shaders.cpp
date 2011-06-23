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
#ifdef PLATFORM_WIN
#    include <windows.h>
#endif
#include <GL/glew.h>
#include <GL/gl.h>

#ifdef USE_OPENGL
#include <fstream>
#include <QDebug>

//---------------------------------------------------------------------

GLuint loadShader(GLenum type, const char *file) {
	std::ifstream fin(file);

	GLint numBytes = 0;
	GLchar *fileData = NULL;
	if(fin) {
		fin.seekg(0, std::ios::end);
		numBytes = fin.tellg();
		fin.seekg(0, std::ios::beg);

		fileData = new GLchar[numBytes];
		fin.read(reinterpret_cast<char *>(fileData), numBytes);
		fin.close();
	} else
		return 0;

	GLuint shader = glCreateShader(type);
	glShaderSource(shader, 1, const_cast<const GLchar**>(&fileData), &numBytes);
	glCompileShader(shader);

	GLint status;
	glGetShaderiv(shader, GL_COMPILE_STATUS, &status);
	if(status == GL_FALSE) {
		GLchar log[1024];
		int len;
		glGetShaderInfoLog(shader, 1024, &len, log);
		qDebug() << len << "Errors in file " << file << ":\n" << log;
	}

	delete [] fileData;

	return shader;
}

//---------------------------------------------------------------------

void loadProgram(const char *vsFile,
                 const char *fsFile,
                 GLuint &vs,
                 GLuint &fs,
                 GLuint &program)
{
	vs = loadShader(GL_VERTEX_SHADER, vsFile);
	fs = loadShader(GL_FRAGMENT_SHADER, fsFile);

	program = glCreateProgram();
	glAttachShader(program, vs);
	glAttachShader(program, fs);
	glLinkProgram(program);
}

//---------------------------------------------------------------------
#else
GLuint loadShader(GLenum, const char *) { return 0; }
void loadProgram(const char *, const char *, GLuint &, GLuint &, GLuint &) { }
#endif
