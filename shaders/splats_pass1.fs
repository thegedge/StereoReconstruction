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
uniform vec3 viewDir;
uniform vec2 windowSize;

varying vec3 normal;

const float near = 0.1;
const float far = 10.0;
const float epsilon = 1.0;

void main() {
	// Discard if point is on a back-facing splat
	// TODO: more efficient or better way of doing this?
	if(dot(normal, viewDir) < 1e-3)
		discard;

	//
	vec3 nn = normalize(normal.xyz);

	// Check to see if this fragment should be part of the splat
	vec2 pos = 2.0*gl_TexCoord[0].xy - 1.0;

	gl_FragColor = vec4(1.0, 0.0, 0.0, 1.0);
#if 0
	float deltaZ = 10.0;
	if(abs(nn.z) > 1e-5)
		deltaZ = -(nn.x/nn.z)*pos.x - (nn.y/nn.z)*pos.y;

	// Verify this fragment is within the splat
	vec3 p = vec3(pos, deltaZ);
	float dz = dot(p, p);
	if(dz > 1.0)
		discard;

	// Calculate per-pixel depth with offset and normalize
	float z = mix(near, far, gl_FragCoord.z) + deltaZ + epsilon;
	gl_FragDepth = (z - near) / (far - near);
#endif
}
