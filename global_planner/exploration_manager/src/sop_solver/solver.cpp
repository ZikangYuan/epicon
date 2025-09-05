#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <limits>
#include <mutex>
#include <queue>
#include <string>
#include <thread>
#include <vector>

#include <iostream>

#include <sop_solver/digraph.h>
#include <sop_solver/edge.h>
#include <sop_solver/hash_map.h>
#include <sop_solver/history_node.h>
#include <sop_solver/hungarian.h>
#include <sop_solver/solver.h>
#include <sop_solver/solver_state.h>

using std::atomic;
using std::condition_variable;
using std::mutex;
using std::pair;
using std::priority_queue;
using std::thread;
using std::unique_lock;
using std::vector;

static int best_solution = std::numeric_limits<int>::max();
static vector<Edge> best_solution_nodes;
static int static_lower_bound = 0;
static int time_limit = 100;
static std::chrono::time_point<std::chrono::system_clock> start_time;
static mutex best_solution_mutex;
static priority_queue<SolverState, vector<SolverState>, SolverStateCompare> first_visits;
static mutex first_visits_mutex;
static int main_enumerated_nodes = 0;
static int main_bound_calculations = 0;

HashMap<pair<vector<bool>, int>, HistoryNode> Solver::history;
vector<unsigned long int> Solver::enumerated_nodes;
vector<unsigned long int> Solver::bound_calculations;
int Solver::thread_count = 0;
int Solver::max_edge_weight = 0;
vector<vector<int>> Solver::cost_matrix;

static int active_thread_count = 1;
static condition_variable cv;
static mutex cv_lock;
static mutex active_lock;
static volatile bool terminate = false;

int history_put_count = 0;

void Solver::set_cost_matrix(vector<vector<int>> matrix) { cost_matrix = matrix; }

Solver::Solver(Digraph const *cost_graph, Digraph const *precedance_graph) {
  this->cost_graph = cost_graph;
  this->precedance_graph = precedance_graph;
  visited_nodes.assign(cost_graph->node_count(), false);
  solution_weight = 0;
  max_edge_weight = this->cost_graph->get_max_edge_weight();
  hungarian_solver = Hungarian(cost_graph->node_count(), max_edge_weight, cost_matrix);
  this->thread_id = -1;
}

Solver::Solver(Digraph const *cost_graph, Digraph const *precedance_graph, Hungarian h) {
  this->cost_graph = cost_graph;
  this->precedance_graph = precedance_graph;
  visited_nodes.assign(cost_graph->node_count(), false);
  solution_weight = 0;
  hungarian_solver = h;
}

void Solver::clear() {
  // Clear
  // static HashMap<std::pair<std::vector<bool>, int>, HistoryNode> history;
  // static int thread_count;
  // static vector<vector<int>> cost_matrix;
  // static int max_edge_weight;
  // static std::atomic<int> active_threads;
  // static std::atomic<bool> wait_thread;
  // static vector<unsigned long int> enumerated_nodes;
  // static vector<unsigned long int> bound_calculations;
  // static int best_solution = std::numeric_limits<int>::max();
  // static vector<Edge> best_solution_nodes;
  // static int static_lower_bound = 0;
  // static int time_limit = 100;
  // static std::chrono::time_point<std::chrono::system_clock> start_time;
  // static mutex best_solution_mutex;
  // static priority_queue<SolverState, vector<SolverState>, SolverStateCompare> first_visits;
  // static mutex first_visits_mutex;
  // static int main_enumerated_nodes = 0;
  // static int main_bound_calculations = 0;

  // std::cout << "Clearing Solver" << std::endl;

  history.clear();
  thread_count = 0;
  cost_matrix.clear();
  max_edge_weight = 0;
  enumerated_nodes.clear();
  bound_calculations.clear();

	best_solution = std::numeric_limits<int>::max();
	best_solution_nodes.clear();
	static_lower_bound = 0;
	time_limit = 100;
	start_time = std::chrono::system_clock::now();
	first_visits = priority_queue<SolverState, vector<SolverState>, SolverStateCompare>();
	main_enumerated_nodes = 0;
	main_bound_calculations = 0;

  // std::cout << "history_put_count: " << history_put_count << std::endl;
  history_put_count = 0;

	// std::cout << "Cleared Solver" << std::endl;
}

void Solver::set_time_limit(int limit, bool per_node) {
  if (per_node) {
    time_limit = limit * cost_graph->node_count();
  } else {
    time_limit = limit;
  }
}

void Solver::set_hash_size(size_t size) {
  size_t bit_vector_mem =
      sizeof(visited_nodes) +
      std::max((size_t)std::ceil((float)cost_graph->node_count() / 8), (size_t)32);
  history.set_size(size, bit_vector_mem);
}

void Solver::solve_sop_parallel(int num_threads) {
  start_time = std::chrono::system_clock::now();
  thread_count = num_threads;
  active_thread_count = 0;
  enumerated_nodes.assign(num_threads, 0);
  bound_calculations.assign(num_threads, 0);
  if (static_lower_bound < best_solution) {
    for (int i = 0; i < cost_graph->node_count(); ++i) {
      solution.clear();
      solution.push_back(Edge(i, i, 0));
      reset_solution(SolverState(Edge(i, i, 0), vector<Edge>(1, Edge(i, i, 0)), 0, 0));
      for (const Edge &e : cost_graph->adj_outgoing(i)) {
        if (valid_node(e.dest)) {
          first_visits.push(SolverState(e, vector<Edge>(1, Edge(i, i, 0)), 0, static_lower_bound));
        }
      }
      if (!first_visits.empty()) {
        vector<thread> solver_threads(thread_count);
        vector<Solver> solvers;
        first_visits_mutex.lock();
        for (int j = 0; j < thread_count; ++j) {
          solvers.push_back(std::move(Solver(cost_graph, precedance_graph)));
          if (first_visits.size() < thread_count) {
            split_visits();
          }
          if (!first_visits.empty()) {
            SolverState first_visit = first_visits.top();
            first_visits.pop();
            active_lock.lock();
            ++active_thread_count;
            active_lock.unlock();
            solver_threads[j] = thread(&Solver::solve_sop, std::move(solvers[j]), first_visit, j);
          }
        }
        first_visits_mutex.unlock();
        for (int j = 0; j < thread_count; ++j) {
          if (solver_threads[j].joinable()) {
            solver_threads[j].join();
          }
        }
      }
    }
  }
}

void Solver::split_visits() {
  int initial_size = first_visits.size();
  bool split = true;
  int index = 0;
  int lower_bound;
  while (split && index < first_visits.size()) {
    if (first_visits.empty() || first_visits.size() > thread_count) {
      split = false;
    } else {
      SolverState first_visit = first_visits.top();
      visited_nodes.assign(cost_graph->node_count(), false);
      for (const Edge &e : first_visit.path) {
        visited_nodes[e.dest] = true;
      }
      if (first_visit.path.size() < cost_graph->node_count() - 1) {
        first_visits.pop();
        --index;
        first_visit.path.push_back(first_visit.next_edge);
        first_visit.cost += first_visit.next_edge.weight;
        visited_nodes[first_visit.next_edge.dest] = true;
        last_visited_node = first_visit.next_edge.dest;
        reset_solution(first_visit);
        if (thread_id > -1) {
          enumerated_nodes[thread_id] += 1;
        } else {
          main_enumerated_nodes += 1;
        }
        if ((lower_bound = get_lower_bound(first_visit.cost, first_visit.next_edge.dest,
                                           visited_nodes)) < best_solution) {
          for (const Edge &e : cost_graph->adj_outgoing(first_visit.next_edge.dest)) {
            if (valid_node(e.dest)) {
              first_visits.push(SolverState(e, first_visit.path, first_visit.cost, lower_bound));
            }
          }
        }
      }
    }
    ++index;
  }
}

void Solver::solve_sop(SolverState first_visit, int thread_id) {
  std::chrono::time_point<std::chrono::system_clock> current_time;
  this->thread_id = thread_id;
  int solution_size = cost_graph->node_count();
  bool start_new_search = true;
  int lower_bound;

  while (start_new_search) {
    bool create_thread = true;
    vector<Edge> st;
    reset_solution(first_visit);
    st.push_back(first_visit.next_edge);
    while (!st.empty()) {
      Edge last_edge = st.back();
      st.pop_back();
      solution.push_back(last_edge);
      solution_weight += last_edge.weight;

      visited_nodes[last_edge.dest] = true;
      last_visited_node = last_edge.dest;
      hungarian_solver.fix_row(last_edge.source, last_edge.dest);
      hungarian_solver.fix_column(last_edge.dest, last_edge.source);
      if (thread_id > -1) {
        enumerated_nodes[thread_id] += 1;
      } else {
        main_enumerated_nodes += 1;
      }
      if (solution.size() == solution_size) {
        update_best_solution();
        if (!st.empty()) {
          backtrack(st.back().source);
        }
      } else {
        const vector<Edge> &sorted_outgoing = cost_graph->sorted_adj_outgoing(last_edge.dest);
        vector<pair<Edge, int>> priority_edges;
        for (int i = sorted_outgoing.size() - 1; i >= 0; --i) {
          if (valid_node(sorted_outgoing[i].dest)) {
            visited_nodes[sorted_outgoing[i].dest] = true;
            solution_weight += sorted_outgoing[i].weight;
            solution.push_back(sorted_outgoing[i]);
            pair<Edge, int> node_pair(
                sorted_outgoing[i],
                get_lower_bound(solution_weight, sorted_outgoing[i].dest, visited_nodes));
            visited_nodes[sorted_outgoing[i].dest] = false;
            hungarian_solver.undue_row(sorted_outgoing[i].source, sorted_outgoing[i].dest);
            hungarian_solver.undue_column(sorted_outgoing[i].dest, sorted_outgoing[i].source);
            solution_weight -= sorted_outgoing[i].weight;
            solution.pop_back();
            if (node_pair.second < best_solution) {
              priority_edges.push_back(node_pair);
            }
          }
        }
        if (priority_edges.size() > 0) {
          std::sort(priority_edges.begin(), priority_edges.end(), EdgeBoundCompare);
          for (int i = 0; i < priority_edges.size(); ++i) {
            st.push_back(priority_edges[i].first);
          }
        } else {
          if (!st.empty()) {
            backtrack(st.back().source);
            if (create_thread && st.size() > 1 && active_thread_count < thread_count) {
              Edge next_edge = st.front();
              SolverState state = generate_solver_state(next_edge);
              if ((active_thread_count < thread_count) && (state.path.size() <= 5)) {
                st.erase(st.begin());
                first_visits_mutex.lock();
                first_visits.push(state);
                first_visits_mutex.unlock();
              } else {
                if (active_thread_count < thread_count) {
                  create_thread = false;
                }
              }
              cv.notify_all();
            }
          }
        }
      }
      current_time = std::chrono::system_clock::now();
      std::chrono::duration<double> elapsed_time = current_time - start_time;
      if (elapsed_time.count() > time_limit) {
        unique_lock<mutex> lk(cv_lock);
        terminate = true;
        lk.unlock();
        start_new_search = false;
        cv.notify_all();
        break;
      }
    }
    first_visits_mutex.lock();
    if (!first_visits.empty()) {
      if (first_visits.size() < thread_count) {
        split_visits();
      }
      if (!first_visits.empty()) {
        first_visit = first_visits.top();
        first_visits.pop();
      } else {
        start_new_search = false;
      }
    } else {
      start_new_search = false;
    }
    first_visits_mutex.unlock();
    if (start_new_search == false) {
      active_lock.lock();
      --active_thread_count;
      if (active_thread_count == 0) {
        unique_lock<mutex> lk(cv_lock);
        terminate = true;
        lk.unlock();
        cv.notify_all();
      }
      active_lock.unlock();
      if (!terminate) {
        unique_lock<mutex> lk(cv_lock);
        cv.wait(lk, [] { return terminate; });
      }
      if (!terminate) {
        first_visits_mutex.lock();
        if (!first_visits.empty()) {
          if (first_visits.size() < thread_count) {
            split_visits();
          }
          if (!first_visits.empty()) {
            first_visit = first_visits.top();
            first_visits.pop();
            start_new_search = true;
            active_lock.lock();
            ++active_thread_count;
            active_lock.unlock();
          }
        } else {
          start_new_search = false;
        }
        first_visits_mutex.unlock();
      } else {
        start_new_search = false;
      }
    }
  }
}

void Solver::backtrack(int source) {
  while (solution.back().dest != source) {
    Edge e = solution.back();
    solution.pop_back();
    visited_nodes[e.dest] = false;
    hungarian_solver.undue_row(e.source, e.dest);
    hungarian_solver.undue_column(e.dest, e.source);
    last_visited_node = e.source;
    solution_weight -= e.weight;
  }
}

bool Solver::valid_node(int node) { return !visited_nodes[node] && predecessors_visited(node); }

SolverState Solver::generate_solver_state(Edge next_edge) {
  int source = next_edge.source;
  vector<Edge> path;
  int cost = 0;
  int i = 0;
  vector<bool> visits(cost_graph->node_count());
  visits.assign(cost_graph->node_count(), false);
  while (solution[i].source != source) {
    path.push_back(solution[i]);
    cost += solution[i].weight;
    visits[solution[i].dest] = true;
    Edge e = solution.back();
    ++i;
  }
  // lower_bound is not important because the state is used when the first_visits are empty
  return SolverState(next_edge, path, cost, cost);
}

bool Solver::predecessors_visited(int node) {
  bool all_visited = true;
  for (Edge e : precedance_graph->adj_outgoing(node)) {
    if (!visited_nodes[e.dest]) {
      all_visited = false;
      break;
    }
  }
  return all_visited;
}

void Solver::update_best_solution() {
  best_solution_mutex.lock();
  if (solution_weight < best_solution) {
    best_solution = solution_weight;
    best_solution_nodes = solution;
  }
  best_solution_mutex.unlock();
}

void Solver::reset_solution(SolverState state) {
  solution.clear();
  solution = state.path;
  solution_weight = state.cost;
  visited_nodes.assign(cost_graph->node_count(), false);
  hungarian_solver = Hungarian(cost_matrix.size(), max_edge_weight, cost_matrix);
  hungarian_solver.start();
  for (const Edge &e : state.path) {
    visited_nodes[e.dest] = true;
    hungarian_solver.fix_row(e.source, e.dest);
    hungarian_solver.fix_column(e.dest, e.source);
  }
  hungarian_solver.solve_dynamic();
}

int Solver::get_static_lower_bound() { return static_lower_bound; }

int Solver::best_solution_cost() { return best_solution; }

vector<Edge> Solver::best_solution_path() { return best_solution_nodes; }

int Solver::edge_bound(int current_node) {
  int largest_out_weight = 0;
  int outgoing_edge_weights = 0;
  int largest_in_weight = 0;
  int incoming_edge_weights = 0;
  for (int i = 0; i < cost_graph->node_count(); ++i) {
    if ((!visited_nodes[i] || i == current_node)) {
      int min_out_weight = -1;
      for (const Edge &e : cost_graph->sorted_adj_outgoing(i)) {
        if (!visited_nodes[e.dest]) {
          min_out_weight = e.weight;
          break;
        }
      }
      if (min_out_weight >= 0) {
        outgoing_edge_weights += min_out_weight;
      } else {
        min_out_weight = max_edge_weight;
        outgoing_edge_weights += min_out_weight;
      }
      if (min_out_weight > largest_out_weight) {
        largest_out_weight = min_out_weight;
      }
      if (i != current_node) {
        int min_in_weight = -1;
        for (const Edge &e : cost_graph->sorted_adj_incoming(i)) {
          if ((!visited_nodes[e.source] || e.source == current_node)) {
            min_in_weight = e.weight;
            break;
          }
        }
        if (min_in_weight >= 0) {
          incoming_edge_weights += min_in_weight;
        } else {
          min_in_weight = max_edge_weight;
          incoming_edge_weights += min_in_weight;
        }
        if (min_in_weight > largest_in_weight) {
          largest_in_weight = min_in_weight;
        }
      }
    }
  }
  int outgoing_bound = solution_weight + outgoing_edge_weights - largest_out_weight;
  int incoming_bound = solution_weight + incoming_edge_weights;
  return std::max(outgoing_bound, incoming_bound);
}

int Solver::get_lower_bound(int cost, int current_node, vector<bool> visits) {
  pair<vector<bool>, int> history_pair = pair<vector<bool>, int>(visits, current_node);
  auto p = history.find(history_pair);
  int lower_bound = std::numeric_limits<int>::max();
  if (p.first != history.end()) {
    if (p.first->second.prefix_cost > cost) {
      history.lock_bucket(p.second);
      int improvement = p.first->second.prefix_cost - cost;
      lower_bound = p.first->second.lower_bound - improvement;
      if (lower_bound < best_solution) {
        Edge last_edge = solution.back();
        hungarian_solver.fix_row(last_edge.source, last_edge.dest);
        hungarian_solver.fix_column(last_edge.dest, last_edge.source);
        p.first->second.prefix_cost = cost;
        p.first->second.lower_bound = lower_bound;
      } else {
        if (p.first->second.prefix_cost > cost) {
          p.first->second.prefix_cost = cost;
          p.first->second.lower_bound = lower_bound;
        }
      }
      history.unlock_bucket(p.second);
    }
  } else {
    Edge last_edge = solution.back();
    hungarian_solver.fix_row(last_edge.source, last_edge.dest);
    hungarian_solver.fix_column(last_edge.dest, last_edge.source);
    hungarian_solver.solve_dynamic();
    lower_bound = hungarian_solver.get_matching_cost() / 2;
    if (thread_id > -1) {
      bound_calculations[thread_id] += 1;
    } else {
      main_bound_calculations += 1;
    }

    p = history.find(history_pair);
    if (p.first == history.end()) {
      history.put(history_pair, HistoryNode(cost, lower_bound));
      history_put_count += 1;
    } else {
      if (p.first->second.prefix_cost > cost) {
        history.lock_bucket(p.second);
        int improvement = p.first->second.prefix_cost - cost;
        lower_bound = p.first->second.lower_bound - improvement;
        if (lower_bound < best_solution) {
          hungarian_solver.fix_row(last_edge.source, last_edge.dest);
          hungarian_solver.fix_column(last_edge.dest, last_edge.source);
          p.first->second.prefix_cost = cost;
          p.first->second.lower_bound = lower_bound;
        } else {
          p.first->second.prefix_cost = cost;
          p.first->second.lower_bound = lower_bound;
        }
        history.unlock_bucket(p.second);
      } else {
        lower_bound = std::numeric_limits<int>::max();
      }
    }
  }
  return lower_bound;
}

void Solver::nearest_neighbor() {
  Edge current_node(0, 0, 0);
  static_lower_bound = hungarian_solver.start() / 2;
  reset_solution(SolverState(current_node, vector<Edge>(), 0, 0));
  visited_nodes[0] = true;
  solution.push_back(current_node);
  bool continue_search = true;
  while (continue_search) {
    for (const Edge &e : cost_graph->sorted_adj_outgoing(current_node.dest)) {
      if (!visited_nodes[e.dest] && predecessors_visited(e.dest)) {
        solution.push_back(e);
        solution_weight += e.weight;
        visited_nodes[e.dest] = true;
        current_node = e;
        break;
      }
    }
    if (solution.size() == cost_graph->node_count()) {
      continue_search = false;
    }
  }
  if (solution_weight < best_solution) {
    best_solution = solution_weight;
    best_solution_nodes = solution;
  }
}

vector<unsigned long int> Solver::get_enumerated_nodes() {
  enumerated_nodes[0] += main_enumerated_nodes;
  return enumerated_nodes;
}

vector<unsigned long int> Solver::get_bound_calculations() {
  bound_calculations[0] += main_bound_calculations;
  return bound_calculations;
}

void Solver::set_initial_solution(vector<Edge> path, int cost) {
  if (cost < best_solution) {
    best_solution = cost;
    best_solution_nodes = path;
  }
}
