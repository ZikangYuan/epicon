#ifndef HUNGARIAN_H
#define HUNGARIAN_H

#include <vector>

using std::max;
using std::min;
using std::vector;


class Hungarian {

public:
	Hungarian();
	Hungarian(int node_count, int max_weight, vector<vector<int>> cost_graph);
	int start();
	void fix_row(int i, int to);
	void fix_column(int j, int from);	
	void undue_row(int i, int to);
	void undue_column(int j, int from);	
	int get_matching_cost();
	int solve_dynamic(void);

private:
	int max_edge_weight;
	int N;
	vector<vector<int>> cost;
	vector<vector<int>> original_cost;
	int n, max_match;
	vector<int> lx; vector<int> ly;
	vector<int> xy;
	vector<int> yx;
	vector<bool> S; vector<bool> T;
	vector<int> slack;
	vector<int> slackx;
	vector<int> prev;
	vector<int> fixed_rows;
	vector<int> fixed_columns;

	void init_labels();
	void update_labels();
	void add_to_tree(int x, int prevx);
	void augment();
};

#endif
