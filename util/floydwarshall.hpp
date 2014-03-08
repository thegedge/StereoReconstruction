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
#ifndef FLOYDWARSHALL_HPP
#define FLOYDWARSHALL_HPP


/*!
 * Implementation of Floyd-Warshall's method for computing all-pairs
 * shortest path on a graph.
 */
class FloydWarshall {
public:
	/*!
	 * All-pairs shortest path for the graph given in \a graph (adjacency
	 * matrix representation) Edges that cannot be traveresed should be
	 * specified with infinity values.
	 */
	FloydWarshall(const std::vector<std::vector<double> > &graph);

public:
	/*!
	 * Get the shortest path from node i to node j.
	 * \note returned vector will be empty if no path exists
	 * \note returned path, if one exists, will include nodes i and j
	 */
	std::deque<int> path(size_t i, size_t j) const;

	//! Get the cost of the shortest path from node i to node j.
	double cost(size_t i, size_t j) const;

private:
	//! Get the "inner" nodes along the path from i to j
	std::deque<int> inner_path(size_t i, size_t j) const;

private:
	//! Reference to the graph to apply FW to.
	const std::vector<std::vector<double> > &graph;

	//! \a path[i][j] = cost of shortest path from node \a i to node \a j
	std::vector<std::vector<double> > vals;

	//! \a next[i][j] = next node on the shortest path from node \a i to node \a j
	std::vector<std::vector<int> > next;
};

#endif // #ifndef FLOYDWARSHALL_HPP
