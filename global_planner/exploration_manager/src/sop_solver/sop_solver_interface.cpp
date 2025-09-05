#include <Eigen/Eigen>
#include <chrono>
#include <dirent.h>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <sys/types.h>
#include <unordered_set>
#include <vector>

#include <sop_solver/digraph.h>
#include <sop_solver/edge.h>
#include <sop_solver/solver.h>
#include <sop_solver/sop_solver_interface.h>

using std::cout;
using std::ifstream;
using std::string;
using std::unordered_set;
using std::vector;

void creat_graphs_from_file(string file, Digraph &g, Digraph &p);
void creat_graphs_from_matrix(const Eigen::MatrixXi &cost_matrix, Digraph &g, Digraph &p);
void remove_redundant_edges(Digraph &g, Digraph &p);
void remove_redundant_edge_successors(Digraph &g, Digraph &p);
void print_solution_path(const vector<Edge> &path);

struct tour {
  vector<Edge> path;
  int cost;
};

struct tour read_tour_file(string file, const Digraph &g);
string match_file(const string &directory, string pattern);

int solveSOP(const Eigen::MatrixXi &cost_matrix, vector<int> &path) {
  // std::cout << "solve sop, size: " << cost_matrix.rows() << std::endl;

  // save cost mat to file
  // std::ofstream cost_mat_file =
  //     std::ofstream("/home/eason/workspace/exploration_ws/sop_cost_mat.txt");
  // for (int i = 0; i < cost_matrix.rows(); ++i) {
  //   for (int j = 0; j < cost_matrix.cols(); ++j) {
  //     cost_mat_file << "\t" << cost_matrix(i, j);
  //   }
  //   cost_mat_file << std::endl;
  // }

  // Time analysis
  std::chrono::time_point<std::chrono::system_clock> start, end;
  std::chrono::duration<double> elapsed_time;

  // start = std::chrono::system_clock::now();
  Digraph g;
  Digraph p;
  creat_graphs_from_matrix(cost_matrix, g, p);

  g.sort_edges();
  remove_redundant_edges(g, p);
  remove_redundant_edge_successors(g, p);
  // end = std::chrono::system_clock::now();
  // elapsed_time = end - start;
  // std::cout << "preprocessing time duration (seconds): " << elapsed_time.count() << std::endl;

  // start = std::chrono::system_clock::now();
  Solver::set_cost_matrix(g.dense_hungarian());
  // end = std::chrono::system_clock::now();
  // elapsed_time = end - start;
  // std::cout << "hungarian time duration (seconds): " << elapsed_time.count() << std::endl;

  // std::cout << "g size: " << g.node_count() << std::endl;
  // std::cout << "p size: " << p.node_count() << std::endl;
  Solver s = Solver(&g, &p);
  s.set_time_limit(1, false);
  // start = std::chrono::system_clock::now();
  s.set_hash_size(10000);
  // end = std::chrono::system_clock::now();
  s.nearest_neighbor();
  // elapsed_time = end - start;
  // std::cout << "NN time duration (seconds): " << elapsed_time.count() << std::endl;

  start = std::chrono::system_clock::now();

  int static_lower_bound = s.get_static_lower_bound();
  int nearest_neighbor_cost = s.best_solution_cost();

  // std::cout << "static lower bound: " << static_lower_bound << std::endl;
  // std::cout << "NN solution is" << std::endl;
  // print_solution_path(s.best_solution_path());
  // std::cout << "with cost: " << nearest_neighbor_cost << std::endl;

  // std::chrono::time_point<std::chrono::system_clock> start, end;
  // start = std::chrono::system_clock::now();
  s.solve_sop_parallel(1);
  // end = std::chrono::system_clock::now();
  // elapsed_time = end - start;
  // std::cout << "sop time duration (seconds): " << elapsed_time.count() << std::endl;

  int best_solution_cost = s.best_solution_cost();

  // vector<unsigned long int> enumerated_nodes = s.get_enumerated_nodes();
  // vector<unsigned long int> bound_calculations = s.get_bound_calculations();
  // std::cout << "best solution found" << std::endl;
  // print_solution_path(s.best_solution_path());
  // std::cout << "with cost: " << s.best_solution_cost() << std::endl;
  // std::cout << "time duration (seconds): " << elapsed_time.count() << std::endl;
  // for (int i = 0; i < 1; ++i) {
  //   std::cout << "enumerated nodes: " << enumerated_nodes[i] << std::endl;
  //   std::cout << "calculated bounds: " << bound_calculations[i] << std::endl;
  // }

  path.clear();
  for (Edge e : s.best_solution_path()) {
    path.push_back(e.dest);
  }

  // // print path
  // std::cout << "path: ";
  // for (int i = 0; i < path.size(); ++i) {
  //   std::cout << path[i] << " ";
  // }
  // std::cout << std::endl;

  // this will cost 1ms
  s.clear();

  // end = std::chrono::system_clock::now();

  // elapsed_time = end - start;
  // std::cout << "misc time duration (seconds): " << elapsed_time.count() << std::endl;

  return s.best_solution_cost();
}

void creat_graphs_from_file(string file, Digraph &g, Digraph &p) {
  ifstream graph_file(file);
  if (graph_file.fail()) {
    std::cout << "failed to open file at " << file << std::endl;
    exit(1);
  }
  string line;
  int source = 0;
  bool set_size = true;
  while (getline(graph_file, line)) {
    std::istringstream iss(line);
    vector<string> words;
    for (std::string s; iss >> s;) {
      words.push_back(s);
    }
    if (set_size) {
      g.set_size(words.size());
      p.set_size(words.size());
      set_size = false;
    }
    for (int i = 0; i < words.size(); ++i) {
      int dest = i;
      int weight = std::stoi(words[i]);
      if (weight < 0) {
        p.add_edge(source, dest, 0);
      } else if (source != dest) {
        g.add_edge(source, dest, weight);
      }
    }
    ++source;
  }
}

void creat_graphs_from_matrix(const Eigen::MatrixXi &cost_matrix, Digraph &g, Digraph &p) {
  g.set_size(cost_matrix.rows());
  p.set_size(cost_matrix.rows());
  for (int i = 0; i < cost_matrix.rows(); ++i) {
    for (int j = 0; j < cost_matrix.cols(); ++j) {
      if (cost_matrix(i, j) < 0) {
        p.add_edge(i, j, 0);
      } else if (i != j) {
        g.add_edge(i, j, cost_matrix(i, j));
      }
    }
  }
}

void remove_redundant_edges(Digraph &g, Digraph &p) {
  for (int i = 0; i < p.node_count(); ++i) {
    const vector<Edge> &preceding_nodes = p.adj_outgoing(i);
    unordered_set<int> expanded_nodes;
    for (int j = 0; j < preceding_nodes.size(); ++j) {
      vector<Edge> st;
      st.push_back(preceding_nodes[j]);
      while (!st.empty()) {
        Edge dependence_edge = st.back();
        st.pop_back();
        if (dependence_edge.source != i) {
          g.remove_edge(dependence_edge.dest, i);
          expanded_nodes.insert(dependence_edge.dest);
        }
        for (const Edge &e : p.adj_outgoing(dependence_edge.dest)) {
          if (expanded_nodes.find(e.dest) == expanded_nodes.end()) {
            st.push_back(e);
            expanded_nodes.insert(e.dest);
          }
        }
      }
    }
  }
}

void remove_redundant_edge_successors(Digraph &g, Digraph &p) {
  for (int i = 0; i < p.node_count(); ++i) {
    const vector<Edge> &preceding_nodes = p.adj_incoming(i);
    unordered_set<int> expanded_nodes;
    for (int j = 0; j < preceding_nodes.size(); ++j) {
      vector<Edge> st;
      st.push_back(preceding_nodes[j]);
      while (!st.empty()) {
        Edge dependence_edge = st.back();
        st.pop_back();
        if (dependence_edge.source != i) {
          g.remove_edge(i, dependence_edge.dest);
          expanded_nodes.insert(dependence_edge.dest);
        }
        for (const Edge &e : p.adj_outgoing(dependence_edge.dest)) {
          if (expanded_nodes.find(e.dest) == expanded_nodes.end()) {
            st.push_back(e);
            expanded_nodes.insert(e.dest);
          }
        }
      }
    }
  }
}

void print_solution_path(const vector<Edge> &path) {
  for (Edge e : path) {
    cout << e.dest << " -> ";
  }
  cout << std::endl;
}

struct tour read_tour_file(string file, const Digraph &g) {
  vector<vector<int>> cost_matrix = g.dense_hungarian();
  struct tour t;
  t.path = vector<Edge>();
  t.cost = 0;
  ifstream tour_file(file);
  if (tour_file.fail()) {
    std::cout << "failed to open file at " << file << std::endl;
    exit(1);
  }
  string line;
  int row = 0;
  int prev = 0;
  int next = 0;
  bool set_size = true;
  while (getline(tour_file, line)) {
    if (row > 5 && line != "EOF") {
      next = std::stoi(line);
      next -= 1;
      if (next >= 0) {
        int weight = cost_matrix[prev][next] / 2;
        if (next == prev) {
          weight = 0;
        }
        t.path.push_back(Edge(prev, next, weight));
        t.cost += weight;
      }
      prev = next;
    }
    ++row;
  }
  return t;
}

string match_file(const string &directory, string pattern) {
  string file_match = "";
  DIR *dirp = opendir(directory.c_str());
  struct dirent *dp;
  if (dirp != NULL) {
    while ((dp = readdir(dirp)) != NULL) {
      string file = string(dp->d_name);
      size_t found = file.find(pattern);
      if (found != string::npos) {
        file_match = file;
      }
    }
    closedir(dirp);
  }

  return file_match;
}
