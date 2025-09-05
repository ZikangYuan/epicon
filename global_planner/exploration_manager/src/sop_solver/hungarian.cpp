#include <vector>
#include <algorithm>
#include <limits>

#include <sop_solver/hungarian.h>

using std::max;
using std::min;
using std::vector;

Hungarian::Hungarian(int node_count, int max_weight, vector<vector<int>> cost_graph) {

	n = node_count;
	N = node_count;
	max_edge_weight = max_weight*2;
	cost = vector<vector<int>>(N, vector<int>(N));
	lx = vector<int>(N);
	ly = vector<int>(N);
	xy = vector<int>(N);
	yx = vector<int>(N);
	S = vector<bool>(N);
	T = vector<bool>(N);
	slack = vector<int>(N);
	slackx = vector<int>(N);
	prev = vector<int>(N);
	original_cost = cost_graph;
	fixed_rows = vector<int>(N);
	fixed_columns = vector<int>(N);

	for (int i = 0 ; i < N ; ++i) {
		for (int j = 0 ; j < N ; ++j) {
		    cost[i][j] = -cost_graph[i][j];
		}
	}
}

Hungarian::Hungarian() {
	n = 0;
	N = 0;
}

int Hungarian::start(void) {

	int ret = 0;
	max_match = 0;
	for (int i = 0 ; i < n ; ++i) {
		xy[i] = yx[i] = -1;
		fixed_rows[i] = 0;
		fixed_columns[i] = 0;
	}
	init_labels();
	augment();
	for (int x = 0, c; x < n; ++x) {
		c = cost[x][xy[x]];
		if (c != -max_edge_weight){
			ret += cost[x][xy[x]];
		}
	}

	return -ret;
}

void Hungarian::fix_row(int i, int to) {
	if(i == to){
		return;
	}
	fixed_rows[i] = 1;
	for (int j = 0 ; j < n ; ++j) {
		if (j != to) {
			cost[i][j] = -1 * max_edge_weight;
		}
	}
	if(xy[i] != -1){
	    --max_match;
	    yx[xy[i]] = -1;
	    xy[i] = -1;
	}
	int minimum_alpha = cost[i][0] - ly[0];
	for (int j = 1 ; j < n ; ++j) {
		minimum_alpha = max(minimum_alpha, cost[i][j] - ly[j]);
	}
	lx[i] = minimum_alpha;
}

void Hungarian::fix_column(int j, int from) {
	if(j == from){
		return;
	}
	fixed_columns[j] = 1;
	for (int i = 0 ; i < n ; ++i) {
		if (i != from) {
			cost[i][j] = -1 * max_edge_weight;
		}
	}
	if(yx[j] != -1){
	    --max_match;
	    xy[yx[j]] = -1;
	    yx[j] = -1;
	}
	int minimum_beta = cost[0][j] - lx[0];
	for (int i = 1 ; i < n ; ++i) {
		minimum_beta = max(minimum_beta, cost[i][j] - lx[i]);
	}
	ly[j] = minimum_beta;
}

void Hungarian::undue_row(int i, int to){
	fixed_rows[i] = 0;
	for (int j = 0 ; j < n ; ++j) {
		if (j != to && fixed_rows[j] == 0) {
			cost[i][j] = -1 * original_cost[i][j];
		}
	}
	if(xy[i] != -1){
	    --max_match;
	    yx[xy[i]] = -1;
	    xy[i] = -1;
	}
	int minimum_alpha = cost[i][0] - ly[0];
	for (int j = 1 ; j < n ; ++j) {
		minimum_alpha = max(minimum_alpha, cost[i][j] - ly[j]);
	}
	lx[i] = minimum_alpha;
}

void Hungarian::undue_column(int j, int from) {
	fixed_columns[j] = 0;
	for (int i = 0 ; i < n ; ++i) {
		if (i != from && fixed_columns[i] == 0) {
			cost[i][j] = -1 * original_cost[i][j];
		}
	}
	if(yx[j] != -1){
	    --max_match;
	    xy[yx[j]] = -1;
	    yx[j] = -1;
	}
	int minimum_beta = cost[0][j] - lx[0];
	for (int i = 1 ; i < n ; ++i) {
		minimum_beta = max(minimum_beta, cost[i][j] - lx[i]);
	}
	ly[j] = minimum_beta;
}

int Hungarian::get_matching_cost() {

	int ret = 0; int c;
	for (int x = 0; x < n; ++x) {
		c = cost[x][xy[x]];
		if (c != -max_edge_weight)
			ret += c;
	}
	return -ret;
}

void Hungarian::init_labels() {
	for (int i = 0 ; i < n ; ++i) {
		lx[i] = ly[i] = 0;
	}

	for (int x = 0; x < n; ++x) {
		for (int y = 0; y < n; ++y) {
			lx[x] = max(lx[x], cost[x][y]);
		}
	}
}

void Hungarian::update_labels() {

	int x, y, delta = std::numeric_limits<int>::max();

	for (y = 0; y < n; ++y) {
		if (!T[y]) {
			delta = min(delta, slack[y]);
		}
	}

	for (x = 0; x < n; ++x) {
		if (S[x]) { 
			lx[x] -= delta;
		}
	}
	
	for (y = 0; y < n; ++y) {
		if (T[y]) {
			ly[y] += delta;
		}
	}

	for (y = 0; y < n; ++y) {
		if (!T[y]) {
			slack[y] -= delta;
		}
	}
}


void Hungarian::add_to_tree(int x, int prevx) {
	
	S[x] = true;
	prev[x] = prevx;
	for (int y = 0; y < n; ++y) {
		if (lx[x] + ly[y] - cost[x][y] < slack[y]) {
			slack[y] = lx[x] + ly[y] - cost[x][y];
			slackx[y] = x;
		}
	}
}

void Hungarian::augment() {

	if (max_match == n){
	    return;
	}

	int x, y, root;
	vector<int> q(N);
	int wr = 0;
	int rd = 0;

	for (int i = 0 ; i < n ; ++i) {
		S[i] = T[i] = false;
		prev[i] = -1;
	}

	for (x = 0; x < n; ++x) {
		if (xy[x] == -1) {
			q[wr++] = root = x;
			prev[x] = -2;
			S[x] = true;
			break;
		}
	}

	for (y = 0; y < n; ++y) {
		slack[y] = lx[root] + ly[y] - cost[root][y];
		slackx[y] = root;
	}

	while (true) {
		while (rd < wr) {
			x = q[rd++];
			for (y = 0; y < n; ++y) {
				if (cost[x][y] == lx[x] + ly[y] && !T[y]) {
					if (yx[y] == -1){
					    break;
					}
					T[y] = true;
					q[wr++] = yx[y];
					add_to_tree(yx[y], x);
				}
			}
			if (y < n) break;
		}

		if (y < n) break;

		update_labels();

		wr = rd = 0; 
		for (y = 0; y < n; ++y) {
			if (!T[y] && slack[y] == 0) {
				if (yx[y] == -1) {
					x = slackx[y];
					break;
				} else {
					T[y] = true;
					if (!S[yx[y]]) {
						q[wr++] = yx[y];
						add_to_tree(yx[y], slackx[y]);
					}
				}
			}
		}
		if (y < n) break;
	}

	if (y < n) {
		max_match++;
		for (int cx = x, cy = y, ty; cx != -2; cx = prev[cx], cy = ty) {
			ty = xy[cx];
			yx[cy] = cx;
			xy[cx] = cy;
		}
		augment();
	}
}

int Hungarian::solve_dynamic(void) {
    augment();
		return 0;
}

