#include <cbf/cbf_filter.h>
#include <optimizer/poly_traj_utils.hpp>
#include <geometry_msgs/Vector3Stamped.h>
#include <algorithm>
#include <sstream>
#include <iomanip>

namespace cbf {

// ── Lifecycle ────────────────────────────────────────────────────────────────

CbfFilter::~CbfFilter() {
  if (work_) osqp_cleanup(work_);
}


void CbfFilter::init(ros::NodeHandle& nh, int self_id) {
  self_id_  = self_id;

  nh.param("cbf/enabled",  enabled_,  false);
  nh.param("cbf/gamma",    gamma_,    1.0);
  nh.param("cbf/r_safe",     r_safe_,     0.3);
  nh.param("cbf/v_max",      v_max_,      3.0);
  nh.param("cbf/d_obs_safe",    d_obs_safe_,    0.3);
  nh.param("cbf/debug_markers", debug_markers_, false);

  if (debug_markers_) {
    cbf_obs_marker_pub_ = nh.advertise<visualization_msgs::MarkerArray>(
        "cbf/debug/obs_barrier_markers", 10);
    cbf_vel_marker_pub_ = nh.advertise<visualization_msgs::MarkerArray>(
        "cbf/debug/vel_markers", 10);
  }

  cbf_blocked_dir_pub_ = nh.advertise<geometry_msgs::Vector3Stamped>(
      "cbf/blocked_direction", 10);

  swarm_traj_sub_ = nh.subscribe<traj_utils::PolyTraj>(
      "planning/broadcast_traj_recv", 100, &CbfFilter::swarmTrajCb, this);

  P_csc_.nzmax = 3; P_csc_.m = 3; P_csc_.n = 3;
  P_csc_.p = P_indptr_; P_csc_.i = P_indices_; P_csc_.x = P_data_;
  P_csc_.nz = -1;

  esdf_cloud_.reset(new pcl::PointCloud<pcl::PointXYZI>());
  std::string esdf_topic = "/drone_" + std::to_string(self_id_) +
      "_ego_planner_node/grid_map/esdf";
  esdf_sub_ = nh.subscribe<sensor_msgs::PointCloud2>(
      esdf_topic, 1, &CbfFilter::esdfCallback, this);

  ready_ = true;
}

// ── Swarm trajectory callback ─────────────────────────────────────────────────

void CbfFilter::swarmTrajCb(const traj_utils::PolyTrajConstPtr& msg) {
  if (msg->drone_id < 0 || msg->order != 5) return;
  if (msg->duration.size() * (msg->order + 1) != msg->coef_x.size()) return;
  if (msg->drone_id == self_id_) return;

  const size_t id = (size_t)msg->drone_id;
  int piece_nums  = (int)msg->duration.size();

  std::vector<double> dura(piece_nums);
  std::vector<poly_traj::CoefficientMat> cMats(piece_nums);
  for (int i = 0; i < piece_nums; ++i) {
    int i6 = i * 6;
    cMats[i].row(0) << msg->coef_x[i6+0], msg->coef_x[i6+1], msg->coef_x[i6+2],
                       msg->coef_x[i6+3], msg->coef_x[i6+4], msg->coef_x[i6+5];
    cMats[i].row(1) << msg->coef_y[i6+0], msg->coef_y[i6+1], msg->coef_y[i6+2],
                       msg->coef_y[i6+3], msg->coef_y[i6+4], msg->coef_y[i6+5];
    cMats[i].row(2) << msg->coef_z[i6+0], msg->coef_z[i6+1], msg->coef_z[i6+2],
                       msg->coef_z[i6+3], msg->coef_z[i6+4], msg->coef_z[i6+5];
    dura[i] = msg->duration[i];
  }

  ego_planner::LocalTrajData entry;
  entry.drone_id   = msg->drone_id;
  entry.traj_id    = msg->traj_id;
  entry.start_time = msg->start_time.toSec();
  entry.traj       = poly_traj::Trajectory(dura, cMats);
  entry.duration   = entry.traj.getTotalDuration();
  entry.start_pos  = entry.traj.getPos(0.0);

  std::lock_guard<std::mutex> lk(swarm_mtx_);
  if (swarm_traj_.size() <= id) {
    ego_planner::LocalTrajData blank;
    blank.drone_id = -1;
    swarm_traj_.resize(id + 1, blank);
  }
  swarm_traj_[id] = entry;
}

// ── ESDF callback ─────────────────────────────────────────────────────────────

void CbfFilter::esdfCallback(const sensor_msgs::PointCloud2ConstPtr& msg) {
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(
      new pcl::PointCloud<pcl::PointXYZI>());
  pcl::fromROSMsg(*msg, *cloud);
  if (cloud->empty()) return;

  std::lock_guard<std::mutex> lk(esdf_mtx_);
  esdf_cloud_ = cloud;
  esdf_kdtree_.setInputCloud(esdf_cloud_);
  esdf_ready_ = true;
}

// ── Barrier construction ─────────────────────────────────────────────────────

void CbfFilter::buildBarriers(const Eigen::Vector3d& pos, double t_abs,
                               std::vector<Eigen::Vector3d>& grads,
                               std::vector<double>& hs) {
  grads.clear();
  hs.clear();


  // ── Swarm drones ────────────────────────────────────────────────────────
  {
    std::lock_guard<std::mutex> lk(swarm_mtx_);
    for (size_t i = 0; i < swarm_traj_.size(); ++i) {
      const auto& t = swarm_traj_[i];
      if (t.drone_id < 0) continue;

      double rel_t = t_abs - t.start_time;
      Eigen::Vector3d p_other;
      if (rel_t < 0.0) {
        p_other = t.start_pos;
      } else if (rel_t <= t.duration) {
        p_other = t.traj.getPos(rel_t);
      } else {
        // Linear extrapolation from end of trajectory
        double exceed = rel_t - t.duration;
        p_other = t.traj.getPos(t.duration) + t.traj.getVel(t.duration) * exceed;
      }

      Eigen::Vector3d diff = pos - p_other;
      double dist = diff.norm();
      if (dist < 1e-6) continue;  // same position, skip

      double h_swm = dist - r_safe_;

      // ∇_pos h_swm = (pos - p_other) / ||pos - p_other||
      Eigen::Vector3d grad_swm = diff / dist;

      grads.push_back(grad_swm);
      hs.push_back(h_swm);
    }
  }

  // ── Static obstacles (ESDF KDTree) ───────────────────────────────────────
  {
    std::lock_guard<std::mutex> lk(esdf_mtx_);
    if (esdf_ready_ && !esdf_cloud_->empty()) {
      pcl::PointXYZI search_pt;
      search_pt.x = pos(0);
      search_pt.y = pos(1);
      search_pt.z = pos(2);

      std::vector<int> idx(1);
      std::vector<float> dist2(1);
      if (esdf_kdtree_.nearestKSearch(search_pt, 1, idx, dist2) > 0) {
        double dist_obs = esdf_cloud_->points[idx[0]].intensity;

        if (dist_obs < d_obs_safe_ * 3.0) {
          Eigen::Vector3d nearest(
              esdf_cloud_->points[idx[0]].x,
              esdf_cloud_->points[idx[0]].y,
              esdf_cloud_->points[idx[0]].z);
          Eigen::Vector3d grad_obs = pos - nearest;
          if (grad_obs.norm() > 1e-6) grad_obs.normalize();

          double h_obs = dist_obs - d_obs_safe_;
          // >>> MOD: low-pass filter on h_obs to reduce jumps
          static double h_obs_filt = 0.0;
          static bool   h_obs_init = false;
          const double alpha = 0.2;
          if (!h_obs_init) {
            h_obs_filt = h_obs;
            h_obs_init = true;
          } else {
            h_obs_filt = alpha * h_obs + (1.0 - alpha) * h_obs_filt;
          }
          grads.push_back(grad_obs);
          hs.push_back(h_obs_filt);

          ROS_INFO_THROTTLE(1.0, "[CBF] dist=%.3f h=%.3f grad=(%.3f,%.3f,%.3f)",
                            dist_obs, h_obs_filt, grad_obs(0), grad_obs(1), grad_obs(2));

          if (debug_markers_) {
            double t = std::max(0.0, std::min(1.0, h_obs / (2.0 * d_obs_safe_)));

            visualization_msgs::MarkerArray ma;
            ros::Time now = ros::Time::now();

            visualization_msgs::Marker arrow;
            arrow.header.frame_id = "world";
            arrow.header.stamp    = now;
            arrow.ns              = "cbf_obs_grad";
            arrow.id              = 0;
            arrow.type            = visualization_msgs::Marker::ARROW;
            arrow.action          = visualization_msgs::Marker::ADD;
            arrow.lifetime        = ros::Duration(0.1);
            geometry_msgs::Point p0, p1;
            p0.x = pos(0); p0.y = pos(1); p0.z = pos(2);
            Eigen::Vector3d tip  = pos + grad_obs * 0.5;
            p1.x = tip(0); p1.y = tip(1); p1.z = tip(2);
            arrow.points.push_back(p0);
            arrow.points.push_back(p1);
            arrow.scale.x = 0.05;
            arrow.scale.y = 0.10;
            arrow.scale.z = 0.12;
            arrow.color.r = (float)(1.0 - t);
            arrow.color.g = (float)t;
            arrow.color.b = 0.0f;
            arrow.color.a = 0.9f;
            ma.markers.push_back(arrow);

            visualization_msgs::Marker text;
            text.header    = arrow.header;
            text.ns        = "cbf_obs_h";
            text.id        = 0;
            text.type      = visualization_msgs::Marker::TEXT_VIEW_FACING;
            text.action    = visualization_msgs::Marker::ADD;
            text.lifetime  = ros::Duration(0.1);
            text.pose.position.x = tip(0);
            text.pose.position.y = tip(1);
            text.pose.position.z = tip(2) + 0.15;
            text.pose.orientation.w = 1.0;
            text.scale.z   = 0.15;
            text.color.r   = (float)(1.0 - t);
            text.color.g   = (float)t;
            text.color.b   = 0.0f;
            text.color.a   = 0.9f;
            std::ostringstream ss;
            ss << "h=" << std::fixed << std::setprecision(3) << h_obs;
            text.text = ss.str();
            ma.markers.push_back(text);

            cbf_obs_marker_pub_.publish(ma);
          }
        }
      }
    }
  }
}

// ── OSQP solver ──────────────────────────────────────────────────────────────

bool CbfFilter::solveQP(const Eigen::Vector3d& vel_ref,
                         const std::vector<Eigen::Vector3d>& grads,
                         const std::vector<double>& hs,
                         Eigen::Vector3d& vel_out) {
  const int N_cbf = (int)grads.size();
  const int m     = N_cbf + 3;  // CBF rows + box rows
  const int nnz_A = N_cbf * 3 + 3;

  // ── Build A (CSC column-major), l, u ─────────────────────────────────────
  // Column j layout: N_cbf barrier entries (rows 0..N_cbf-1) + 1 box entry (row N_cbf+j)
  A_data_.resize(nnz_A);
  A_indices_.resize(nnz_A);
  A_indptr_.resize(4);  // 3 columns + 1

  for (int j = 0; j < 3; ++j) {
    A_indptr_[j] = j * (N_cbf + 1);
    // Barrier rows
    for (int i = 0; i < N_cbf; ++i) {
      int pos_in_data       = j * (N_cbf + 1) + i;
      A_data_[pos_in_data]   = (c_float)grads[i](j);
      A_indices_[pos_in_data] = (c_int)i;
    }
    // Box row
    int box_pos             = j * (N_cbf + 1) + N_cbf;
    A_data_[box_pos]         = 1.0;
    A_indices_[box_pos]      = (c_int)(N_cbf + j);
  }
  A_indptr_[3] = nnz_A;

  l_vec_.resize(m);
  u_vec_.resize(m);
  for (int i = 0; i < N_cbf; ++i) {
    l_vec_[i] = (c_float)(-gamma_ * hs[i]);
    u_vec_[i] = (c_float)OSQP_INFTY;
  }
  for (int j = 0; j < 3; ++j) {
    l_vec_[N_cbf + j] = (c_float)(-v_max_);
    u_vec_[N_cbf + j] = (c_float)v_max_;
  }

  q_[0] = (c_float)(-vel_ref(0));
  q_[1] = (c_float)(-vel_ref(1));
  q_[2] = (c_float)(-vel_ref(2));

  // ── OSQP setup or update ─────────────────────────────────────────────────
  if (work_ == nullptr || osqp_m_ != m) {
    if (work_) { osqp_cleanup(work_); work_ = nullptr; }

    A_csc_.nzmax = nnz_A; A_csc_.m = m; A_csc_.n = 3;
    A_csc_.p = A_indptr_.data(); A_csc_.i = A_indices_.data();
    A_csc_.x = A_data_.data(); A_csc_.nz = -1;

    data_.n = 3; data_.m = m;
    data_.P = &P_csc_; data_.q = q_;
    data_.A = &A_csc_;
    data_.l = l_vec_.data(); data_.u = u_vec_.data();

    osqp_set_default_settings(&settings_);
    settings_.warm_start = 1;
    settings_.verbose    = 0;
    settings_.polish     = 0;
    settings_.max_iter   = 200;
    settings_.eps_abs    = 1e-4;
    settings_.eps_rel    = 1e-4;

    if (osqp_setup(&work_, &data_, &settings_) != 0) return false;
    osqp_m_ = m;
  } else {
    // Warm-start: update q, A values (same sparsity), and bounds
    osqp_update_lin_cost(work_, q_);
    osqp_update_A(work_, A_data_.data(), nullptr, (c_int)nnz_A);
    osqp_update_bounds(work_, l_vec_.data(), u_vec_.data());
  }

  if (osqp_solve(work_) != 0) return false;
  if (work_->info->status_val != OSQP_SOLVED &&
      work_->info->status_val != OSQP_SOLVED_INACCURATE) return false;

  vel_out(0) = work_->solution->x[0];
  vel_out(1) = work_->solution->x[1];
  vel_out(2) = work_->solution->x[2];
  return true;
}

// ── Scale fallback ────────────────────────────────────────────────────────────

Eigen::Vector3d CbfFilter::scaleFallback(const Eigen::Vector3d& vel_ref,
                                          const std::vector<Eigen::Vector3d>& grads,
                                          const std::vector<double>& hs) {
  double alpha = 1.0;
  for (size_t i = 0; i < grads.size(); ++i) {
    double Lf = grads[i].dot(vel_ref);
    if (Lf >= 0.0) continue;       // constraint satisfied regardless of scale
    if (hs[i] < 0.0) return Eigen::Vector3d::Zero();  // already violated, stop
    double alpha_i = gamma_ * hs[i] / (-Lf);
    alpha = std::min(alpha, alpha_i);
  }
  return alpha * vel_ref;
}

// ── Main filter entry point ───────────────────────────────────────────────────

// ── 修改後的 filter ───────────────────────────────────────────────────────
bool CbfFilter::filter(const Eigen::Vector3d& pos,
                        Eigen::Vector3d& vel,
                        Eigen::Vector3d& acc,
                        double t_abs) {
  if (!enabled_ || !ready_) return false;

  std::vector<Eigen::Vector3d> grads;
  std::vector<double> hs;
  buildBarriers(pos, t_abs, grads, hs);

  if (grads.empty()) {
    last_h_min_      = 1e9;
    last_triggered_  = false;
    last_infeasible_ = false;
    return false;
  }

  last_h_min_ = *std::min_element(hs.begin(), hs.end());

  bool any_violated = false;
  for (size_t i = 0; i < grads.size(); ++i) {
    if (grads[i].dot(vel) + gamma_ * hs[i] < 0.0) {
      any_violated = true;
      break;
    }
  }
  if (!any_violated) {
    last_triggered_  = false;
    last_infeasible_ = false;
    return false;
  }

  Eigen::Vector3d vel_ref = vel;
  Eigen::Vector3d vel_safe;
  bool qp_ok = solveQP(vel, grads, hs, vel_safe);
  last_infeasible_ = !qp_ok;
  if (!qp_ok) {
    vel_safe = scaleFallback(vel, grads, hs);
  }

  // 計算速度被削減的比例 (0.0 ~ 1.0)
  double speed_norm = vel.norm();
  double scale = (speed_norm > 1e-4) ? (vel_safe.norm() / speed_norm) : 0.0;

  // 計算被封鎖的方向並發布
  {
    Eigen::Vector3d blocked = vel - vel_safe;
    if (blocked.norm() > 1e-4) {
      last_blocked_dir_ = blocked.normalized();
    } else {
      last_blocked_dir_ = Eigen::Vector3d::Zero();
    }
    geometry_msgs::Vector3Stamped dir_msg;
    dir_msg.header.stamp    = ros::Time::now();
    dir_msg.header.frame_id = "world";
    dir_msg.vector.x = last_blocked_dir_(0);
    dir_msg.vector.y = last_blocked_dir_(1);
    dir_msg.vector.z = last_blocked_dir_(2);
    cbf_blocked_dir_pub_.publish(dir_msg);
  }

  // 更新速度
  vel = vel_safe;

  // X, Y 軸加速度配合速度等比例縮減 (避免急煞時翻滾)
  acc(0) *= scale;
  acc(1) *= scale;
  // Z 軸加速度 (acc(2)) 保持原樣，維持高度控制器所需的前饋推力！

  last_triggered_ = true;

  if (debug_markers_) {
    visualization_msgs::MarkerArray ma;
    ros::Time now = ros::Time::now();
    geometry_msgs::Point p0;
    p0.x = pos(0); p0.y = pos(1); p0.z = pos(2);

    if (vel_ref.norm() >= 1e-4) {
      visualization_msgs::Marker m;
      m.header.frame_id = "world";
      m.header.stamp    = now;
      m.ns              = "cbf_vel_before";
      m.id              = 0;
      m.type            = visualization_msgs::Marker::ARROW;
      m.action          = visualization_msgs::Marker::ADD;
      m.lifetime        = ros::Duration(0.1);
      geometry_msgs::Point p1;
      Eigen::Vector3d tip = pos + vel_ref * 0.3;
      p1.x = tip(0); p1.y = tip(1); p1.z = tip(2);
      m.points.push_back(p0);
      m.points.push_back(p1);
      m.scale.x = 0.05; m.scale.y = 0.10; m.scale.z = 0.10;
      m.color.r = 0.5f; m.color.g = 0.5f; m.color.b = 0.5f; m.color.a = 0.8f;
      ma.markers.push_back(m);
    }

    if (vel_safe.norm() >= 1e-4) {
      visualization_msgs::Marker m;
      m.header.frame_id = "world";
      m.header.stamp    = now;
      m.ns              = "cbf_vel_after";
      m.id              = 1;
      m.type            = visualization_msgs::Marker::ARROW;
      m.action          = visualization_msgs::Marker::ADD;
      m.lifetime        = ros::Duration(0.1);
      geometry_msgs::Point p1;
      Eigen::Vector3d tip = pos + vel_safe * 0.3;
      p1.x = tip(0); p1.y = tip(1); p1.z = tip(2);
      m.points.push_back(p0);
      m.points.push_back(p1);
      m.scale.x = 0.05; m.scale.y = 0.10; m.scale.z = 0.10;
      m.color.r = 0.0f; m.color.g = 1.0f; m.color.b = 0.3f; m.color.a = 0.8f;
      ma.markers.push_back(m);
    }

    cbf_vel_marker_pub_.publish(ma);
  }

  return true;
}

CbfFilter::CbfStats CbfFilter::getStats() const {
  return {last_h_min_, last_triggered_, last_infeasible_};
}

}  // namespace cbf
