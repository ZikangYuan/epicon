#ifndef DIGPRAPH_H
#define DIGPRAPH_H

#include <vector>
#include "edge.h"

class Digraph {
	public:
		Digraph();
		Digraph(int node_count);
		void set_size(int size);
		void add_edge (int source, int target, int weight);
		int node_count() const;
		const std::vector<Edge>& adj_outgoing(int node) const;
		const std::vector<Edge>& adj_incoming(int node) const;
		const std::vector<Edge>& sorted_adj_outgoing(int node) const;
		const std::vector<Edge>& sorted_adj_incoming(int node) const;
		void sort_edges();
		std::vector<std::vector<int>> dense_hungarian() const;
		int get_max_edge_weight() const;
		void remove_edge(int source, int dest);
	private:
		void remove_outgoing_edge(std::vector<std::vector<Edge>>& edges, int source, int dest);
		void remove_incoming_edge(std::vector<std::vector<Edge>>& edges, int source, int dest);
		std::vector<std::vector<Edge>> outgoing_edges;
		std::vector<std::vector<Edge>> incoming_edges;
		std::vector<std::vector<Edge>> sorted_outgoing_edges;
		std::vector<std::vector<Edge>> sorted_incoming_edges;
		int max_edge_weight;
		int num_nodes;
};

#endif
