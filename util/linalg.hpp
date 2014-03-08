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
#ifndef LINALG_HPP
#define LINALG_HPP


namespace linalg {

//! Projects \a vectorA onto \a vectorB
template <class DataType, int Size>
Eigen::Matrix<DataType, Size, 1>
project(
		const Eigen::Matrix<DataType, Size, 1> &vectorA,
		const Eigen::Matrix<DataType, Size, 1> &vectorB)
{
	const Eigen::Matrix<DataType, Size, 1> bn = vectorB.normalized();
	return bn.dot(vectorA)*bn;
}

}


#endif // LINALG_HPP
