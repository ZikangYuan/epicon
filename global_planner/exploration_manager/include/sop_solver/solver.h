#ifndef SOLVER_H
#define SOLVER_H

#include <vector>
#include <utility>
#include <atomic>
#include <thread>

#include "edge.h"
#include "digraph.h"
#include "history_node.h"
#include "hash_map.h"
#include "solver_state.h"
#include "hungarian.h"

namespace std
{
    template<>
    struct hash<std::pair<vector<bool>, int>> {
        size_t operator () (std::pair<vector<bool>, int> const& p) const {
            return (std::hash<vector<bool>>()(p.first) ^ std::hash<int>()(p.second));
        }
    };
}

class Solver {
	public:
		Solver(Digraph const * cost_graph, Digraph const * precedance_graph);
		Solver(Digraph const * cost_graph, Digraph const * precedance_graph, Hungarian h);
		void clear();
		void solve_sop_parallel(int nub_threads);
		void solve_sop(SolverState first_visit, int thread_id);
		void print_solution();
		void nearest_neighbor();
		void set_time_limit(int limit, bool per_node);
		void set_hash_size(size_t size);
		int get_static_lower_bound();
		int best_solution_cost();
		std::vector<Edge> best_solution_path();
		std::vector<unsigned long int> get_enumerated_nodes();
		std::vector<unsigned long int> get_bound_calculations();
		static void set_cost_matrix(vector<vector<int>> matrix);
		void set_initial_solution(vector<Edge> path, int cost);
	private:
		std::vector<Edge> solution;
		int solution_weight;
		std::vector<bool> visited_nodes;
		int last_visited_node;
		Digraph const * cost_graph;
		Digraph const * precedance_graph;
		static HashMap<std::pair<std::vector<bool>, int>, HistoryNode> history;
		static int thread_count;
		bool valid_node(int node);
		bool predecessors_visited(int node);
		void update_best_solution();
		void reset_solution(SolverState state);
		int edge_bound(int current_node);
		void backtrack(int source);
		int get_lower_bound(int cost, int current_node, std::vector<bool>);
		void split_visits();
		static vector<vector<int>> cost_matrix;
		static int max_edge_weight;
		Hungarian hungarian_solver;
		static std::atomic<int> active_threads;
		static std::atomic<bool> wait_thread;
		//vector<std::thread> spawned_threads;
		SolverState generate_solver_state(Edge starting_edge);
		int thread_id;
		static vector<unsigned long int> enumerated_nodes;
		static vector<unsigned long int> bound_calculations;
};

struct {
	bool operator()(const pair<Edge, int>& s1, const pair<Edge, int>& s2) const{
		return s1.second > s2.second;
	}
} EdgeBoundCompare;

#endif
