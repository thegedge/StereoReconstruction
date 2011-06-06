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
// Splat Renderer - Fragment Shader
//
// Based on the paper "High-Quality Point-Based Rendering on Modern GPUs"
//   written by Mario Botsch and Leif Kobbelt (2003)
//---------------------------------------------------------------------
uniform sampler2D colorTexture;

void main() {
	//
	vec4 color = texture2D(colorTexture, gl_TexCoord[0].st);
	if(color.a < 1e-10)
		discard;

	gl_FragColor = color / color.a;
}
