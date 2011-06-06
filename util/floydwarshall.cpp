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
#include "floydwarshall.hpp"
#include <cmath>
#include <QDebug>
#include "util/c++0x.hpp"

//---------------------------------------------------------------------

FloydWarshall::FloydWarshall(const std::vector<std::vector<double> > &graph)
	: graph(graph)
	, vals(graph)
{
	// Set up vectors
	next.resize(graph.size());
	for(size_t i = 0; i < graph.size(); ++i) {
		next[i].resize(graph.size(), -1);
		for(size_t j = 0; j < graph.size(); ++j) {
			if(std::isfinite(graph[i][j]))
				next[i][j] = -1;
		}
	}

	// FW algorithm
	for(size_t k = 0; k < graph.size(); ++k) {
		for(size_t i = 0; i < graph.size(); ++i) {
			for(size_t j = 0; j < graph.size(); ++j) {
				const double cost = vals[i][k] + vals[k][j];
				if(cost < vals[i][j]) {
					vals[i][j] = cost;
					next[i][j] = k;
				}
			}
		}
	}
}

//---------------------------------------------------------------------

std::deque<int> FloydWarshall::path(size_t i, size_t j) const {
	std::deque<int> path = inner_path(i, j);

	if(path.size() == 0) {
		// no inner nodes, so determine if this means there is no path
		// or the shortest path from node i to node j is simply travelling
		// a single edge that joins them
		if(std::isfinite(vals[i][j])) {
			path.push_back(i);
			path.push_back(j);
		}
	} else {
		path.push_front(i);
		path.push_back(j);
	}

	return path;
}

//---------------------------------------------------------------------

std::deque<int> FloydWarshall::inner_path(size_t i, size_t j) const {
	int k = next[i][j];
	if(i == j || k < 0)
		return std::deque<int>();

	std::deque<int> p1 = inner_path(i, k);
	std::deque<int> p2 = inner_path(k, j);

	p1.push_back(k);
	p1.insert(p1.end(), p2.begin(), p2.end());
	return p1;
}

//---------------------------------------------------------------------

double FloydWarshall::cost(size_t i, size_t j) const {
	return vals[i][j];
}

//---------------------------------------------------------------------
