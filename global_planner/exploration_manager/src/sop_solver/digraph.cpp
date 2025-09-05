#include <vector>
#include <algorithm>

#include <sop_solver/digraph.h>
#include <sop_solver/edge.h>

using std::vector;
using std::max;

Digraph::Digraph(){
	num_nodes = 0;
	max_edge_weight = 0;
}

Digraph::Digraph(int node_count){
	outgoing_edges.resize(node_count);
	incoming_edges.resize(node_count);
	sorted_outgoing_edges.resize(node_count);
	sorted_incoming_edges.resize(node_count);
	max_edge_weight = 0;
	num_nodes = node_count;
}

void Digraph::set_size(int size){
	outgoing_edges.resize(size);
	incoming_edges.resize(size);
	sorted_outgoing_edges.resize(size);
	sorted_incoming_edges.resize(size);
	num_nodes = size;
}


void Digraph::add_edge(int source, int dest, int weight){
	Edge out(source, dest, weight);
	outgoing_edges[source].push_back(out);
	incoming_edges[dest].push_back(out);
	sorted_outgoing_edges[source].push_back(out);
	sorted_incoming_edges[dest].push_back(out);
	max_edge_weight = max(max_edge_weight, weight+1);
}

void Digraph::remove_edge(int source, int dest){
	remove_outgoing_edge(outgoing_edges, source, dest);
	remove_incoming_edge(incoming_edges, source, dest);
	remove_outgoing_edge(sorted_outgoing_edges, source, dest);
	remove_incoming_edge(sorted_incoming_edges, source, dest);
}

void Digraph::remove_outgoing_edge(vector<vector<Edge>>& edges, int source, int dest){
    int index = -1;
    for(int i = 0; i < edges[source].size(); ++i){
        if(edges[source][i].dest == dest){
            index = i;
            break;
        }
    }
    if(index >= 0){
        edges[source].erase(edges[source].begin() + index);
    }
}

void Digraph::remove_incoming_edge(vector<vector<Edge>>& edges, int source, int dest){
    int index = -1;
    for(int i = 0; i < edges[dest].size(); ++i){
        if(edges[dest][i].source == source){
            index = i;
            break;
        }
    }
    if(index >= 0){
        edges[dest].erase(edges[dest].begin() + index);
    }
}

int Digraph::node_count() const{
	return num_nodes;
}

const vector<Edge>& Digraph::adj_outgoing(int node) const{
	return outgoing_edges[node];
}

const vector<Edge>& Digraph::adj_incoming(int node) const{
	return incoming_edges[node];
}

void Digraph::sort_edges(){
	for(int i = 0; i < num_nodes; ++i){
		std::stable_sort(sorted_outgoing_edges[i].begin(), sorted_outgoing_edges[i].end());
		std::stable_sort(sorted_incoming_edges[i].begin(), sorted_incoming_edges[i].end());
	}
	
}
const vector<Edge>& Digraph::sorted_adj_outgoing(int node) const{
	return sorted_outgoing_edges[node];
}

const vector<Edge>& Digraph::sorted_adj_incoming(int node) const{
	return sorted_incoming_edges[node];
}

vector<vector<int>> Digraph::dense_hungarian() const{
	vector<vector<int>> matrix(num_nodes);
	for(int i = 0; i < num_nodes; ++i){
		matrix[i] = vector<int>(num_nodes, max_edge_weight*2);
	}
	for(vector<Edge> edges : outgoing_edges){
		for(Edge e : edges){
			matrix[e.source][e.dest] = e.weight*2;
		}
	}
	return matrix;
}

int Digraph::get_max_edge_weight() const{
	return max_edge_weight;
}

