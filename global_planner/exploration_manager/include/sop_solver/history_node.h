#ifndef HISTORY_NODE_H
#define HISTORY_NODE_H

#include <vector>

#include "edge.h"

class HistoryNode {
	public:
		HistoryNode(int prefix_cost, int bound);
		HistoryNode(int prefix_cost);
		HistoryNode();
		int prefix_cost;
		int lower_bound;
};

#endif
