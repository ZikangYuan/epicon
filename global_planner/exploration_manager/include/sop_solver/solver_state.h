#ifndef SOLVER_STATE_H
#define SOLVER_STATE_H

#include <vector>

#include "edge.h"

class SolverState{
	public:
		SolverState(Edge next_edge, std::vector<Edge> path, int cost, int bound);
		SolverState(const SolverState& state);
		Edge next_edge;
		std::vector<Edge> path;
		int cost;
		int lower_bound;
};

class SolverStateCompare{
	public:
		bool operator()(const SolverState& s1, const SolverState& s2) const{
			return s1.lower_bound > s2.lower_bound;
		}
};

#endif
