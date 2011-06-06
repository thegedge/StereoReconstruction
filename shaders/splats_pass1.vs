//---------------------------------------------------------------------
//
// Copyright ¬© 2011, Jason Gedge <gedge -at- ualberta -dot- ca>
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
// Splat Renderer - Vertex Shader
//
// Based on the paper "High-Quality Point-Based Rendering on Modern GPUs"
//   written by Mario Botsch and Leif Kobbelt (2003)
//---------------------------------------------------------------------
uniform float splatRadius;
uniform vec2 windowSize;

varying vec3 normal;

void main() {
	// Standard stuff
	gl_Position = gl_ModelViewProjectionMatrix * gl_Vertex;
	normal = normalize(gl_NormalMatrix * gl_Normal).xyz;

	// Point size calculation
	float tMinusB = 2.0;
	float nOverZ = 0.1 / gl_Position.w;
	float hOverTB = windowSize.y / tMinusB;

	gl_PointSize = 2.0;//max(2.0, splatRadius * nOverZ * hOverTB);
}
