#include <sop_solver/edge.h>

Edge::Edge(int source, int dest, int weight){
	this->source = source;
	this->dest = dest;
	this->weight = weight;
}

Edge::Edge(){
	this->source = 0;
	this->dest = 0;
	this->weight = 0;
}

bool operator<(const Edge& first, const Edge& second){
	return first.weight < second.weight;
}
