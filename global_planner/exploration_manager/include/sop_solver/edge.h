#ifndef EDGE_H
#define EDGE_H

class Edge {
	public:
		Edge(int source, int dest, int weight);
		Edge();
		int source;
		int dest;
		int weight;
};

bool operator<(const Edge& first, const Edge& second);

#endif
