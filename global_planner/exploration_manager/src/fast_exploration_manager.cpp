#include <boost/lexical_cast.hpp>
#include <epic_planner/expl_data.h>
#include <epic_planner/fast_exploration_manager.h>
#include <fstream>
#include <iostream>
#include <lkh_tsp_solver/lkh_interface.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <plan_manage/planner_manager.h>
#include <sys/types.h>
#include <sys/wait.h>
#include <thread>
#include <unistd.h>
#include <visualization_msgs/Marker.h>
#include <sop_solver/sop_solver_interface.h>

using namespace std;
using namespace Eigen;

namespace fast_planner
{
FastExplorationManager::FastExplorationManager() {}

FastExplorationManager::~FastExplorationManager() {}

void FastExplorationManager::initialize(
    ros::NodeHandle &nh, FrontierManager::Ptr frt_manager,
    FastPlannerManager::Ptr planner_manager) {

  frontier_manager_ptr_ = frt_manager;
  planner_manager_ = planner_manager;

  ed_.reset(new ExplorationData);
  ep_.reset(new ExplorationParam);
  ed_->next_goal_node_ = make_shared<TopoNode>();

  ep_->a_avg_ = tan(planner_manager_->gcopter_config_->maxTiltAngle) *
                planner_manager_->gcopter_config_->gravAcc;
  ep_->v_max_ = planner_manager_->gcopter_config_->maxVelMag;
  ep_->yaw_v_max_ = planner_manager_->gcopter_config_->yaw_max_vel;
  nh.param("exploration/tsp_dir", ep_->tsp_dir_, string("null"));
  nh.getParam("viewpoint_param/global_viewpoint_num",
              ep_->global_viewpoint_num_);
  nh.getParam("view_graph", ep_->view_graph_);
  nh.getParam("viewpoint_param/local_viewpoint_num", ep_->local_viewpoint_num_);
  nh.getParam("global_planning/w_vdir", ep_->w_vdir_);
  nh.getParam("global_planning/w_yawdir", ep_->w_yawdir_);
  Eigen::Vector3d origin, size;
  ofstream par_file(ep_->tsp_dir_ + "/single.par");
  par_file << "PROBLEM_FILE = " << ep_->tsp_dir_ << "/single.tsp\n";
  par_file << "GAIN23 = NO\n";
  par_file << "MOVE_TYPE = 2\n";
  par_file << "OUTPUT_TOUR_FILE =" << ep_->tsp_dir_ << "/single.txt\n";
  par_file << "RUNS = 10\n";
  ros::Duration(1.0).sleep();
}

void FastExplorationManager::goalCallback(
    const geometry_msgs::PoseStampedConstPtr &msg) {
  double roll, pitch;
  tf::Quaternion quat;
  tf::quaternionMsgToTF(msg->pose.orientation, quat);
  tf::Matrix3x3(quat).getRPY(roll, pitch, goal_yaw);
}

double FastExplorationManager::getPathCost(TopoNode::Ptr &n1,
                                           Eigen::Vector3d v1, float yaw1,
                                           TopoNode::Ptr &n2, float yaw2) {
  auto estimateCost = [&](TopoNode::Ptr &n1, Eigen::Vector3d v1, float &yaw1,
                          TopoNode::Ptr &n2, float &yaw2, int res,
                          vector<Eigen::Vector3f> &path) -> double {
    double len_cost, yaw_cost, dir_cost;
    len_cost = yaw_cost = dir_cost = 0.0;
    if (res == BubbleAstar::NO_PATH)
      return 2e3 + (n1->center_ - n2->center_)
                       .norm();
    if (res == BubbleAstar::START_FAIL || res == BubbleAstar::END_FAIL)
      return 2e3 +
             (n1->center_ - n2->center_).norm();

    len_cost = 0.0;
    for (int i = 0; i < path.size() - 1; ++i)
      len_cost += ((path[i + 1] - path[i]).norm() +
                   0.5 * fabs(path[i + 1].z() - path[i].z()));
    len_cost /= (ep_->v_max_ / 2.0);

    return len_cost + dir_cost;
  };
  vector<Eigen::Vector3f> path;
  int res = planner_manager_->fast_searcher_->topoSearch(n1, n2, 1e-2, path);
  return estimateCost(n1, v1, yaw1, n2, yaw2, res, path);
}

double FastExplorationManager::getPathCostWithoutTopo(TopoNode::Ptr &n1,
                                                      Eigen::Vector3d v1,
                                                      float &yaw1,
                                                      TopoNode::Ptr &n2,
                                                      float &yaw2) {
  vector<Eigen::Vector3f> path;
  int res = planner_manager_->parallel_path_finder_->search(
      n1->center_, n2->center_, path, 1.0, false);
  if (res != ParallelBubbleAstar::REACH_END)
    return 2e3;
  double cost;
  planner_manager_->parallel_path_finder_->calculatePathCost(path, cost);
  return cost;
}

std::vector<int> FastExplorationManager::dijkstra(std::vector<std::vector<double>> &graph, int start, int end)
{
  int N = graph.size();
  double INF = 2e3;
  std::vector<double> dist(N, INF);
  std::vector<int> prev(N, -1);
  priority_queue<pair<double, int>, std::vector<pair<double, int>>, greater<pair<double, int>>> pq;
  dist[start] = 0;
  pq.push(make_pair(0, start));
  while (!pq.empty()) {
    int u = pq.top().second;
    pq.pop();
    if (u == end) {
      break;
    }
    for (int v = 0; v < N; v++) {
      if (graph[u][v] < INF && dist[u] + graph[u][v] < dist[v]) {
        dist[v] = dist[u] + graph[u][v];
        prev[v] = u;
        pq.push(make_pair(dist[v], v));
      }
    }
  }
  std::vector<int> path;
  for (int u = end; u != -1; u = prev[u]) {
    path.push_back(u);
  }
  reverse(path.begin(), path.end());
  return path;
}

void FastExplorationManager::refineLocalTourHGrid(Eigen::Vector3d &cur_vel, float cur_yaw, TopoNode::Ptr &next_pos, 
  std::vector<TopoNode::Ptr> &viewpoints, std::vector<double> &distances, 
  std::vector<TopoNode::Ptr> &refined_viewpoints, std::vector<double> &refined_distances)
{
  int dim = 0;
  dim += viewpoints.size();
  dim += 2;

  std::vector<std::vector<double>> cost_matrix(dim, std::vector<double>(dim, 2e3));

  bool is_all_inf = true;
  for (unsigned int j = 0; j < viewpoints.size(); j++)
  {
    cost_matrix[0][j + 1] = getPathCost(planner_manager_->topo_graph_->odom_node_, cur_vel, cur_yaw, viewpoints[j], viewpoints[j]->yaw_);

    if (cost_matrix[0][j + 1] < 2e3) is_all_inf = false;
  }

  if (is_all_inf)
  {
    for (unsigned int j = 0; j < viewpoints.size(); j++)
      cost_matrix[0][j + 1] -= 2e3;
  }

  is_all_inf = true;

  for (unsigned int j = 0; j < viewpoints.size(); j++)
  {
    cost_matrix[dim - viewpoints.size() - 1 + j][dim - 1] = getPathCost(viewpoints[j], Eigen::Vector3d::Zero(), viewpoints[j]->yaw_, 
                                                                        next_pos, 0.0);

    if (cost_matrix[dim - viewpoints.size() - 1 + j][dim - 1] < 2e3) is_all_inf = false;
  }

  if (is_all_inf)
  {
    for (unsigned int j = 0; j < viewpoints.size(); j++)
      cost_matrix[dim - viewpoints.size() - 1 + j][dim - 1] -= 2e3;
  }

  for (int j = 0; j < viewpoints.size(); j++)
  {
    is_all_inf = true;
    for (int k = 0; k < viewpoints.size(); k++)
    {
      if (j != k)
        cost_matrix[j + 1][k + 1] = 
          getPathCost(viewpoints[j], Eigen::Vector3d::Zero(), viewpoints[j]->yaw_, viewpoints[k], viewpoints[k]->yaw_);
      else
        cost_matrix[j + 1][k + 1] = 0;

      if (cost_matrix[j + 1][k + 1] < 2e3) is_all_inf = false;
    }

    if (is_all_inf)
    {
      for (int k = 0; k < viewpoints.size(); k++)
      {
        cost_matrix[j + 1][k + 1] -= 2e3;
      }
    }
  }

  std::vector<int> path_idx = dijkstra(cost_matrix, 0, dim - 1);

  refined_viewpoints.clear();

  for (int idx : path_idx)
  {
    if (idx == 0) continue;
    else if (idx == dim - 1) continue;
    else
    {
      refined_viewpoints.push_back(viewpoints[idx - 1]);
      refined_distances.push_back(distances[idx - 1]);
    }
  }
}

void FastExplorationManager::removeUnreachableNodes(std::vector<TopoNode::Ptr> &node_points)
{
  if (planner_manager_->topo_graph_->odom_node_->neighbors_.empty())
    return;

  planner_manager_->topo_graph_->insertNodes(node_points, true);

  std::vector<bool> node_kept(node_points.size(), true);

  for (int i = 0; i < node_points.size(); i++)
  {
    if (node_points[i]->neighbors_.empty())
    {
      node_kept[i] = false;
      continue;
    }

    std::vector<TopoNode::Ptr> topo_path;
    auto closest_node = planner_manager_->topo_graph_->odom_node_;
    float closest_dis = (closest_node->center_ - node_points[i]->center_).squaredNorm();
    
    for (auto& hodom : planner_manager_->topo_graph_->history_odom_nodes_)
    {
      float dis = (hodom->center_ - node_points[i]->center_).squaredNorm();
      if (dis < closest_dis)
      {
        closest_dis = dis;
        closest_node = hodom;
      }
    }

    if (!planner_manager_->topo_graph_->graphSearch(closest_node, node_points[i], topo_path, 3e-4))
    {
      node_kept[i] = false;
    }
  }

  planner_manager_->topo_graph_->removeNodes(node_points);

  std::vector<TopoNode::Ptr> kept_node_points;

  for (int i = 0; i < node_kept.size(); i++)
  {
    if (node_kept[i])
      kept_node_points.push_back(node_points[i]);
    else
    {
      TopoNode::Ptr node_temp = make_shared<TopoNode>();
      node_temp->is_unkownpoint_ = true;
      node_temp->center_ = node_points[i]->center_;

      std::vector<TopoNode::Ptr> v_node_temp;
      v_node_temp.push_back(node_temp);
      planner_manager_->topo_graph_->insertNodes(v_node_temp);

      auto closest_node = planner_manager_->topo_graph_->odom_node_;
      float closest_dis = (closest_node->center_ - node_points[i]->center_).squaredNorm();
      
      for (auto& hodom : planner_manager_->topo_graph_->history_odom_nodes_)
      {
        float dis = (hodom->center_ - node_points[i]->center_).squaredNorm();
        if (dis < closest_dis)
        {
          closest_dis = dis;
          closest_node = hodom;
        }
      }
      planner_manager_->topo_graph_->removeNodes(v_node_temp);

      Eigen::Vector3f direction = closest_node->center_ - node_temp->center_;
      Eigen::Vector3f step_size = direction.normalized();

      while (!node_kept[i])
      {
        node_temp->center_ = node_temp->center_ + step_size;

        if ((node_temp->center_ - closest_node->center_).norm() < 3.0) break;

        std::vector<TopoNode::Ptr>().swap(v_node_temp);
        v_node_temp.push_back(node_temp);
        planner_manager_->topo_graph_->insertNodes(v_node_temp);

        if (node_points[i]->neighbors_.empty())
        {
          planner_manager_->topo_graph_->removeNodes(v_node_temp);
          continue;
        }
        else
        {
          std::vector<TopoNode::Ptr> topo_path;

          if (!planner_manager_->topo_graph_->graphSearch(closest_node, node_temp, topo_path, 3e-4))
          {
            planner_manager_->topo_graph_->removeNodes(v_node_temp);
            continue;
          }
        }

        node_kept[i] = true;
        planner_manager_->topo_graph_->removeNodes(v_node_temp);
        break;
      }

      if (node_kept[i])
        kept_node_points.push_back(node_temp);
    }
  }

  node_points.swap(kept_node_points);
}

void FastExplorationManager::calculateCostMatrix(const Eigen::Vector3d &pos, const Eigen::Vector3d &vel, Eigen::MatrixXd &cost_matrix,
    std::vector<TopoNode::Ptr> &unkownpoints, std::vector<TopoNode::Ptr> &activepoints, double hybrid_search_radius, double unknown_penalty_factor)
{
  int dim = 1;
  int mat_idx = 1;

  dim += activepoints.size();
  dim += unkownpoints.size();

  cost_matrix = Eigen::MatrixXd::Zero(dim, dim);

  float curr_yaw = (float)planner_manager_->local_data_.curr_yaw_;
  double cost = 0.0;

  for (int i = 0; i < activepoints.size(); i++)
  {
    double line_distance = (pos - activepoints[i]->center_.cast<double>()).norm();

    if (line_distance > hybrid_search_radius)
      cost = 2e3 + line_distance;
    else
      cost = getPathCost(planner_manager_->topo_graph_->odom_node_, vel, 0.0, activepoints[i], 0.0);

    cost_matrix(0, mat_idx) = cost;

    mat_idx++;
  }

  for (int i = 0; i < unkownpoints.size(); i++)
  {
    cost = 2e3 + (pos - unkownpoints[i]->center_.cast<double>()).norm();

    cost_matrix(0, mat_idx) = cost * unknown_penalty_factor;

    mat_idx++;
  }

  int mat_idx1 = 1;
  std::vector<TopoNode::Ptr> centers1;
  centers1.insert(centers1.end(), activepoints.begin(), activepoints.end());
  centers1.insert(centers1.end(), unkownpoints.begin(), unkownpoints.end());

  for (int i = 0; i < (int)centers1.size(); i++)
  {
    int mat_idx2 = 1;
    std::vector<TopoNode::Ptr> centers2;
    centers2.insert(centers2.end(), activepoints.begin(), activepoints.end());
    centers2.insert(centers2.end(), unkownpoints.begin(), unkownpoints.end());

    for (int j = 0; j < (int)centers2.size(); j++)
    {
      if (mat_idx2 <= mat_idx1)
      {
        mat_idx2++;
        continue;
      }

      double cost = 0.0;
      double line_distance = (centers1[i]->center_ - centers2[j]->center_).norm();

      if (line_distance > hybrid_search_radius)
      {
        cost = 2e3 + line_distance;
      }
      else
      {
        cost = getPathCost(centers1[i], Eigen::Vector3d::Zero(), 0.0, centers2[j], 0.0);

        if (cost > 1e3) cost = 2e3 + line_distance;
      }

      if (i >= (int)activepoints.size() || j >= (int)activepoints.size())
        cost *= unknown_penalty_factor;

      cost_matrix(mat_idx1, mat_idx2) = cost_matrix(mat_idx2, mat_idx1) = cost;

      mat_idx2++;
    }

    mat_idx1++;
  }
}

int FastExplorationManager::planGlobalPath(const Eigen::Vector3d &pos,
                                           const Eigen::Vector3d &vel,
                                           std::vector<Eigen::Vector3f, Eigen::aligned_allocator<Eigen::Vector3f>> &unkownpoints_,
                                           std::vector<Eigen::Vector3f, Eigen::aligned_allocator<Eigen::Vector3f>> &activepoints_, 
                                           double hybrid_search_radius, double unknown_penalty_factor)
{
  bool bm_without_topo = false;
  auto estimiateVdirCost = [&](const TopoNode::Ptr &n1,
                               const Eigen::Vector3d &v1,
                               const TopoNode::Ptr &n2) -> double {
    Eigen::Vector3f dir = n2->center_ - n1->center_;
    dir.normalize();
    Eigen::Vector3f v_dir = v1.normalized().cast<float>();
    float yaw1 = atan2(dir.y(), dir.x());
    float yaw2 = atan2(v_dir.y(), v_dir.x());
    float diff = yaw1 - yaw2;
    while (diff > M_PI)
      diff -= 2.0 * M_PI;
    while (diff < -M_PI)
      diff += 2.0 * M_PI;
    return ep_->w_vdir_ *
           (fabs(diff) / planner_manager_->gcopter_config_->yaw_max_vel);
  };
  ros::Time start = ros::Time::now();
  vector<TopoNode::Ptr> viewpoints;
  frontier_manager_ptr_->generateTSPViewpoints(
      planner_manager_->topo_graph_->odom_node_->center_, viewpoints);

  if (viewpoints.empty()) {
    planner_manager_->graph_visualizer_->vizTour({}, VizColor::RED, "global");
    return NO_FRONTIER;
  }

  std::vector<TopoNode::Ptr> unkownpoints;

  if (planner_manager_->topo_graph_->history_odom_nodes_.size() > 10)
  {
    for (int i = 0; i < (int)unkownpoints_.size(); i++)
    {
      TopoNode::Ptr up_node = make_shared<TopoNode>();
      up_node->is_unkownpoint_ = true;
      up_node->center_ = unkownpoints_[i];
      // up_node->yaw_ = 0.0;
      unkownpoints.push_back(up_node);
    }

    removeUnreachableNodes(unkownpoints);
  }

  std::vector<TopoNode::Ptr> activepoints;

  if (planner_manager_->topo_graph_->history_odom_nodes_.size() > 10)
  {
    for (int i = 0; i < (int)activepoints_.size(); i++)
    {
      TopoNode::Ptr ap_node = make_shared<TopoNode>();
      ap_node->is_activepoint_ = true;
      ap_node->center_ = activepoints_[i];
      // ap_node->yaw_ = 0.0;
      activepoints.push_back(ap_node);
    }

    removeUnreachableNodes(activepoints);
  }

  std::vector<TopoNode::Ptr> allpoints;
  allpoints.insert(allpoints.end(), unkownpoints.begin(), unkownpoints.end());
  //allpoints.insert(allpoints.end(), activepoints.begin(), activepoints.end());
  allpoints.insert(allpoints.end(), viewpoints.begin(), viewpoints.end());

  std::vector<float> distance_odom2points;

  for (int i = 0; i < allpoints.size(); i++)
  {
    float distance = (pos.cast<float>() - allpoints[i]->center_).norm();
    distance_odom2points.push_back(distance);
  }

  std::vector<int> idx;
  for (int i = 0; i < distance_odom2points.size(); i++) {
    idx.push_back(i);
  }

  sort(idx.begin(), idx.end(), [&](int a, int b) { return distance_odom2points[a] < distance_odom2points[b]; });

  int consider_range = min(20, (int)idx.size());

  std::vector<TopoNode::Ptr> unkownpoints_temp;
  std::vector<TopoNode::Ptr> activepoints_temp;
  std::vector<TopoNode::Ptr> viewpoints_temp;

  for (int i = 0; i < consider_range; i++)
  {
    if ((allpoints[idx[i]]->center_ - planner_manager_->topo_graph_->odom_node_->center_).norm() < 0.5)
    {
      continue;
    }

    if (allpoints[idx[i]]->is_unkownpoint_)
      unkownpoints_temp.push_back(allpoints[idx[i]]);
    else if (allpoints[idx[i]]->is_activepoint_)
      activepoints_temp.push_back(allpoints[idx[i]]);
    else if (allpoints[idx[i]]->is_viewpoint_)
      viewpoints_temp.push_back(allpoints[idx[i]]);
  }

  unkownpoints.swap(unkownpoints_temp);
  activepoints.swap(activepoints_temp);
  viewpoints.swap(viewpoints_temp);

  viewpoints.insert(viewpoints.end(), unkownpoints.begin(), unkownpoints.end());
  // viewpoints.insert(viewpoints.end(), activepoints.begin(), activepoints.end());
  planner_manager_->topo_graph_->insertNodes(viewpoints, false);

  ros::Time t1 = ros::Time::now();

  updateGoalNode();
  ros::Time t2 = ros::Time::now();

  float curr_yaw = (float)planner_manager_->local_data_.curr_yaw_;
  vector<double> distance_odom2vp(viewpoints.size(), 0);
  vector<double> distance_lastgoal2vp(viewpoints.size(), 0);
  double dis2last_goal = 5e3;
  if (planner_manager_->lidar_map_interface_->getDisToOcc(
          ed_->next_goal_node_->center_) >
      planner_manager_->parallel_path_finder_->safe_distance_ + 0.1) {
    dis2last_goal = getPathCost(planner_manager_->topo_graph_->odom_node_,
                                Eigen::Vector3d::Zero(), curr_yaw,
                                ed_->next_goal_node_, curr_yaw);
  }
  static double last_frame_value = dis2last_goal;
  bool last_goal_reachable = dis2last_goal < 2e3;

  if (last_goal_reachable && (dis2last_goal < 1.5 * last_frame_value)) {
    last_frame_value = dis2last_goal;
  } else {
    last_goal_reachable = false;
  }

  ros::Time t_start_cvp_1 = ros::Time::now();
  omp_set_num_threads(4);
  // clang-format off
  #pragma omp parallel for
  // clang-format on
  for (int i = 0; i < viewpoints.size(); ++i) {
    if (last_goal_reachable) {
      distance_lastgoal2vp[i] =
          getPathCost(ed_->next_goal_node_, Eigen::Vector3d::Zero(),
                      viewpoints[i]->yaw_, viewpoints[i], viewpoints[i]->yaw_);
      distance_odom2vp[i] =
          getPathCost(planner_manager_->topo_graph_->odom_node_, vel, curr_yaw,
                      viewpoints[i], viewpoints[i]->yaw_);

      if (viewpoints[i]->is_unkownpoint_)
      {
        distance_lastgoal2vp[i] *= unknown_penalty_factor;
        distance_odom2vp[i] *= unknown_penalty_factor;
      }
    }
    else
    {
      distance_lastgoal2vp[i] =
          getPathCost(planner_manager_->topo_graph_->odom_node_, vel, curr_yaw,
                      viewpoints[i], viewpoints[i]->yaw_);
      distance_odom2vp[i] = distance_lastgoal2vp[i];

      if (viewpoints[i]->is_unkownpoint_)
      {
        distance_lastgoal2vp[i] *= unknown_penalty_factor;
        distance_odom2vp[i] *= unknown_penalty_factor;
      }
    }
  }
  ros::Time t_end_cvp_1 = ros::Time::now();

  if (bm_without_topo) {
    omp_set_num_threads(4);
    // clang-format off
    #pragma omp parallel for
    // clang-format on
    for (int i = 0; i < viewpoints.size(); ++i) {
      if (last_goal_reachable) {
        distance_lastgoal2vp[i] = getPathCostWithoutTopo(
            ed_->next_goal_node_, Eigen::Vector3d::Zero(), viewpoints[i]->yaw_,
            viewpoints[i], viewpoints[i]->yaw_);
        distance_odom2vp[i] = getPathCostWithoutTopo(
            planner_manager_->topo_graph_->odom_node_, vel, curr_yaw,
            viewpoints[i], viewpoints[i]->yaw_);

      } else {
        distance_lastgoal2vp[i] = getPathCostWithoutTopo(
            planner_manager_->topo_graph_->odom_node_, vel, curr_yaw,
            viewpoints[i], viewpoints[i]->yaw_);
        distance_odom2vp[i] = distance_lastgoal2vp[i];
      }
    }
    ros::Time t_end_cvp_2 = ros::Time::now();
    double cost_mat_with_topo = (t_end_cvp_1 - t_start_cvp_1).toSec() * 1000;
    double cost_mat_without_topo = (t_end_cvp_2 - t_end_cvp_1).toSec() * 1000;
    cout << "cost mat topo: " << cost_mat_with_topo << "ms" << endl;
    cout << "cost mat point cloud: " << cost_mat_without_topo << "ms" << endl;
  }

  vector<TopoNode::Ptr> viewpoint_reachable;
  vector<double> viewpoint_reachable_distance, viewpoint_reachable_distance2;
  for (int i = 0; i < distance_lastgoal2vp.size(); ++i) {
    if (distance_odom2vp[i] > 2e3)
      continue;
    if (last_goal_reachable) {
      viewpoint_reachable_distance.emplace_back(distance_lastgoal2vp[i]);
    } else {
      viewpoint_reachable_distance.emplace_back(distance_odom2vp[i]);
    }
    viewpoint_reachable_distance2.emplace_back(distance_odom2vp[i]);
    viewpoint_reachable.emplace_back(viewpoints[i]);
  }

  vector<TopoNode::Ptr> raw_viewpoint_reachable = viewpoint_reachable;

  if (viewpoint_reachable.empty())
  {
    planner_manager_->topo_graph_->removeNodes(viewpoints);
    planner_manager_->graph_visualizer_->vizTour({}, VizColor::RED, "global");

    if (raw_viewpoint_reachable.empty())
      return NO_FRONTIER;
    else
      return SUCCEED;
  }

  if (viewpoint_reachable.size() == 1)
  {
    ed_->global_tour_.clear();
    ed_->global_tour_.emplace_back(pos.cast<float>());
    ed_->global_tour_.emplace_back(viewpoint_reachable.front()->center_);

    planner_manager_->local_data_.end_yaw_ = viewpoint_reachable.front()->yaw_;
    planner_manager_->graph_visualizer_->vizTour(ed_->global_tour_, VizColor::RED,
                                               "global");
    planner_manager_->topo_graph_->removeNodes(viewpoints);
    return SUCCEED;
  }

  int dim = viewpoint_reachable.size() + 1;
  Eigen::MatrixXd mat;
  mat.resize(dim, dim);
  mat.setZero();
  for (int i = 1; i < dim; ++i) {
    mat(0, i) = viewpoint_reachable_distance[i - 1];

    if (viewpoint_reachable[i - 1]->is_unkownpoint_)
      mat(0, i) *= unknown_penalty_factor;
  }

  omp_set_num_threads(4);
  // clang-format off
  #pragma omp parallel for
  // clang-format on
  for (int i = 1; i < dim; i++) {
    for (int j = i + 1; j < dim; j++) {
      mat(i, j) = mat(j, i) = getPathCost(
          viewpoint_reachable[i - 1], Eigen::Vector3d(0, 0, 0),
          viewpoint_reachable[i - 1]->yaw_, viewpoint_reachable[j - 1],
          viewpoint_reachable[j - 1]->yaw_);

      if (viewpoint_reachable[i - 1]->is_unkownpoint_ || viewpoint_reachable[j - 1]->is_unkownpoint_)
      {
        mat(i, j) *= unknown_penalty_factor;
        mat(j, i) *= unknown_penalty_factor;
      }
    }
  }

  for (int i = 1; i < dim; ++i) {
    mat(i, 0) = 2e3 - viewpoint_reachable_distance2[i - 1] * 0.2;
  }
  for (int i = 0; i < dim; ++i) {
    for (int j = 1; j < dim; ++j) {
      for (int k = 1; k < dim; ++k) {
        if (mat(i, j) > mat(i, k) + mat(k, j)) {
          mat(i, j) = mat(i, k) + mat(k, j) + 1e-2;
        }
      }
    }
  }

  vector<int> indices;
  indices.reserve(dim);
  ros::Time start_tsp = ros::Time::now();
  cout << "calculate tsp cost matrix cost " << (start_tsp - t2).toSec() * 1000
       << "ms" << endl;
  solveLHK(mat, indices);
  ros::Time end_tsp = ros::Time::now();
  cout << "lkh solver cost: " << (end_tsp - start_tsp).toSec() * 1000 << "ms"
       << endl;

  std::vector<TopoNode::Ptr> tsp_path;
  std::vector<double> tsp_distance;
  TopoNode::Ptr next_grid_node = nullptr;

  for (auto &i : indices)
  {
    if (i == 0)
      continue;

    if (viewpoint_reachable[i - 1]->is_viewpoint_)
    {
      tsp_path.push_back(viewpoint_reachable[i - 1]);
      tsp_distance.push_back(viewpoint_reachable_distance[i - 1]);
    }
    else if (viewpoint_reachable[i - 1]->is_unkownpoint_)
    {
      double threshold = vel.norm() > 2 ? 5.0 : 2.0;

      if ((viewpoint_reachable[i - 1]->center_ - planner_manager_->topo_graph_->odom_node_->center_).norm() < threshold)
      {
        continue;
      }
      else if (tsp_distance.size() < 2)
      {
        tsp_path.push_back(viewpoint_reachable[i - 1]);
        tsp_distance.push_back(viewpoint_reachable_distance[i - 1]);
      }
      else
      {
        next_grid_node = viewpoint_reachable[i - 1];
        break;
      }
    }
  }

  if (next_grid_node != nullptr)
  {
    std::vector<TopoNode::Ptr> refined_viewpoints;
    std::vector<double> refined_distances;

    float curr_yaw = (float)planner_manager_->local_data_.curr_yaw_;
    Eigen::Vector3d curr_vel = vel;
    refineLocalTourHGrid(curr_vel, curr_yaw, next_grid_node, tsp_path, tsp_distance, refined_viewpoints, refined_distances);

    tsp_path = refined_viewpoints;
    tsp_distance = refined_distances;
  }

  ed_->global_tour_.clear();
  ed_->global_tour_.push_back(planner_manager_->topo_graph_->odom_node_->center_);

  for (int i = 0; i < tsp_path.size(); i++)
    ed_->global_tour_.emplace_back(tsp_path[i]->center_);

  if (next_grid_node != nullptr)
    ed_->global_tour_.push_back(next_grid_node->center_);

  if (!last_goal_reachable)
  {
    last_frame_value = tsp_distance[1];
  }

  ros::Time end = ros::Time::now();

  planner_manager_->topo_graph_->removeNodes(viewpoints);
  planner_manager_->graph_visualizer_->vizTour(ed_->global_tour_, VizColor::RED,
                                               "global");

  planner_manager_->local_data_.end_yaw_ =
      viewpoint_reachable[indices[1] - 1]->yaw_;

  updateGoalNode();
  return SUCCEED;
}

void FastExplorationManager::solveLHK(Eigen::MatrixXd &cost_mat, vector<int> &indices,
                                      bool skip_first, bool skip_last, int result_id_offset)
{
  int dimension = cost_mat.rows();
  if (dimension < 3)
    return;
  ofstream prob_file(ep_->tsp_dir_ + "/single.tsp");

  string prob_spec =
      "NAME : single\nTYPE : ATSP\nDIMENSION : " + to_string(dimension) +
      "\nEDGE_WEIGHT_TYPE : "
      "EXPLICIT\nEDGE_WEIGHT_FORMAT : FULL_MATRIX\nEDGE_WEIGHT_SECTION\n";

  prob_file << prob_spec;

  const int scale = 100;

  for (int i = 0; i < dimension; ++i) {
    for (int j = 0; j < dimension; ++j) {
      int int_cost = cost_mat(i, j) * scale;
      prob_file << int_cost << " ";
    }
    prob_file << "\n";
  }

  prob_file << "EOF";
  prob_file.close();

  solveTSPLKH((ep_->tsp_dir_ + "/single.par").c_str());

  ifstream res_file(ep_->tsp_dir_ + "/single.txt");
  string res;
  while (getline(res_file, res)) {
    if (res.compare("TOUR_SECTION") == 0)
      break;
  }

  while (getline(res_file, res)) {

    int id = stoi(res);

    if (id == 1 && skip_first) continue;

    if (id == dimension && skip_last) break;

    if (id == -1)
      break;

    indices.push_back(id - result_id_offset);
  }

  res_file.close();
}

void FastExplorationManager::updateGoalNode()
{
  if (ed_->global_tour_.empty())
    return;

  Eigen::Vector3f goal = ed_->global_tour_[1];

  struct PairPtrHash {
    std::size_t
    operator()(const std::pair<TopoNode::Ptr, TopoNode::Ptr> &p) const {
      return std::hash<TopoNode::Ptr>()(p.first) ^
             std::hash<TopoNode::Ptr>()(p.second);
    }
  };

  Eigen::Vector3i idx;
  planner_manager_->topo_graph_->getIndex(goal, idx);
  vector<TopoNode::Ptr> pre_nbrs;

  for (int i = -1; i <= 1; i++)
    for (int j = -1; j <= 1; j++)
      for (int k = -1; k <= 1; k++) {
        Eigen::Vector3i tmp_idx = idx;
        tmp_idx(0) = idx(0) + i;
        tmp_idx(1) = idx(1) + j;
        tmp_idx(2) = idx(2) + k;

        auto region = planner_manager_->topo_graph_->getRegionNode(tmp_idx);
        if (region) {
          for (auto &topo : region->topo_nodes_) {
            if (topo == ed_->next_goal_node_)
              continue;
            pre_nbrs.emplace_back(topo);
          }
        }
      }
  std::unordered_map<std::pair<TopoNode::Ptr, TopoNode::Ptr>,
                     vector<Eigen::Vector3f>, PairPtrHash>
      edge2insert;
  mutex edge2insert_mtx;
  omp_set_num_threads(4);
  // clang-format off
  #pragma omp parallel for
  // clang-format on
  for (auto &nbr : pre_nbrs)
  {
    vector<Eigen::Vector3f> path;
    int res = planner_manager_->topo_graph_->parallel_bubble_astar_->search(
        goal, nbr->center_, path, 1e-3);
    if (res == ParallelBubbleAstar::REACH_END &&
        planner_manager_->topo_graph_->parallel_bubble_astar_
            ->collisionCheck_shortenPath(path)) {
      edge2insert_mtx.lock();
      edge2insert.insert({std::make_pair(ed_->next_goal_node_, nbr), path});
      edge2insert_mtx.unlock();
    }
  }

  ed_->next_goal_node_->center_ = goal;
  ed_->next_goal_node_->is_viewpoint_ = true;
  if (edge2insert.size() > 0) {
    planner_manager_->topo_graph_->removeNode(ed_->next_goal_node_);
    for (auto &edge : edge2insert) {
      ed_->next_goal_node_->neighbors_.insert(edge.first.second);
      ed_->next_goal_node_->paths_.insert({edge.first.second, edge.second});
      double cost;
      planner_manager_->topo_graph_->parallel_bubble_astar_->calculatePathCost(
          edge.second, cost);
      ed_->next_goal_node_->weight_[edge.first.second] = cost;
      auto nbr = edge.first.second;
      nbr->neighbors_.insert(ed_->next_goal_node_);
      nbr->weight_[ed_->next_goal_node_] = cost;
      vector<Eigen::Vector3f> path = edge.second;
      std::reverse(path.begin(), path.end());
      nbr->paths_[ed_->next_goal_node_] = path;
    }
  }
}
}