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
#ifndef CPP0X_HPP
#define CPP0X_HPP

#if defined(__GXX_EXPERIMENTAL_CXX0X__)
#   include <memory>
#   include <tuple>
#   include <functional>
#elif defined(_MSC_VER)
#   include <memory>
#   include <float.h>
#   include <tuple>
#   include <functional>
	namespace std {
		using namespace std::tr1;

		inline bool isnan(double x) {
			return (_isnan(x) != 0);
		}

		inline bool isfinite(double x) {
			return (_finite(x) != 0);
		}

		inline bool isinf(double x) {
			return (_finite(x) == 0 && _isnan(x) == 0);
		}
	}
#else
#   include <tr1/memory>
#   include <tr1/tuple>
#   include <tr1/functional>
	namespace std {
		using namespace std::tr1;
		using namespace boost;
	}
#endif

#define FORWARD_DECLARE(cls) \
	class cls; \
	typedef std::shared_ptr<cls> cls##Ptr; \
	typedef std::weak_ptr<cls> cls##WeakPtr


#endif // CPP0X_HPP
