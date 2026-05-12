#pragma once

#include <ros/ros.h>
#include <Eigen/Eigen>
#include <traj_utils/PolyTraj.h>
#include <traj_utils/plan_container.hpp>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <visualization_msgs/MarkerArray.h>
#include <mutex>
#include <unordered_map>
#include <vector>

extern "C" {
#include <osqp/osqp.h>
}

namespace cbf {

// CBF-QP safety filter for swarm drones (Dynamic Obstacle Avoidance).
//
// QP solved at each active call:
//   min  (1/2) ||v_safe - v_nom||^2
//   s.t. ∇h_i' v_safe + γ h_i >= 0     for each active barrier i
//        |v_safe_j| <= v_max           for j = x, y, z
//
// Swarm drone barrier: h_swm = ||p_self - p_i|| - r_safe
//
// OSQP is kept warm across calls (workspace reused; q, A, and bounds
// are updated per call based on active constraints).
//
// n = 3 vars (v_safe), m = N_swarm + 3 box constraints.
class CbfFilter {
public:
  bool enabled_ = false;

  CbfFilter() = default;
  ~CbfFilter();

  // Must be called once before filter().
  // Reads params from "cbf/" namespace on nh.
  void init(ros::NodeHandle& nh, int self_id);

  // Modifies vel and acc in-place.
  // When CBF constraint is active, vel is modified, X/Y acc are scaled,
  // and Z acc is preserved for hover thrust.
  // t_abs: ros::Time::now().toSec(), used for swarm trajectory prediction.
  // Returns true if vel was changed.
  bool filter(const Eigen::Vector3d& pos,
              Eigen::Vector3d& vel,
              Eigen::Vector3d& acc,
              double t_abs);

  bool   isEnabled() const { return enabled_; }
  double getRSafe()  const { return r_safe_;  }

  Eigen::Vector3d getBlockedDir() const { return last_blocked_dir_; }

  struct CbfStats { double h_min; bool cbf_triggered; bool infeasible; };
  CbfStats getStats() const;

private:
  void swarmTrajCb(const traj_utils::PolyTrajConstPtr& msg);
  void esdfCallback(const sensor_msgs::PointCloud2ConstPtr& msg);

  // Populates grads and hs for swarm constraints at (pos, t_abs).
  void buildBarriers(const Eigen::Vector3d& pos,
                     double t_abs,
                     std::vector<Eigen::Vector3d>& grads,
                     std::vector<double>& hs);

  // Solves the CBF-QP via OSQP; returns true on success.
  bool solveQP(const Eigen::Vector3d& vel_ref,
               const std::vector<Eigen::Vector3d>& grads,
               const std::vector<double>& hs,
               Eigen::Vector3d& vel_out);

  // Closed-form fallback: scale vel_ref to satisfy all barriers.
  Eigen::Vector3d scaleFallback(const Eigen::Vector3d& vel_ref,
                                const std::vector<Eigen::Vector3d>& grads,
                                const std::vector<double>& hs);

  // ── Inputs ──────────────────────────────────────────────────────────────
  std::vector<ego_planner::LocalTrajData> swarm_traj_;
  ros::Subscriber swarm_traj_sub_;
  std::mutex swarm_mtx_;

  ros::Subscriber esdf_sub_;
  pcl::KdTreeFLANN<pcl::PointXYZI> esdf_kdtree_;
  pcl::PointCloud<pcl::PointXYZI>::Ptr esdf_cloud_;
  std::mutex esdf_mtx_;
  bool esdf_ready_ = false;

  ros::Publisher cbf_obs_marker_pub_;
  ros::Publisher cbf_vel_marker_pub_;
  ros::Publisher cbf_blocked_dir_pub_;

  // ── State ────────────────────────────────────────────────────────────────
  bool ready_{false};

  // ── Params ───────────────────────────────────────────────────────────────
  int    self_id_    = -1;
  double gamma_      = 1.0;  // class-K gain
  double r_safe_     = 0.3;  // swarm safety radius [m]
  double v_max_      = 3.0;  // per-axis velocity bound [m/s]
  double d_obs_safe_ = 0.3;  // static obstacle safety distance [m]

  // ── OSQP persistent state ────────────────────────────────────────────────
  OSQPWorkspace* work_{nullptr};
  OSQPData       data_;
  OSQPSettings   settings_;
  int            osqp_m_{0};  // number of constraints in current workspace

  // P = I₃ (upper triangular), nnz=3, constant
  c_float P_data_[3]    {1.0, 1.0, 1.0};
  c_int   P_indices_[3] {0, 1, 2};
  c_int   P_indptr_[4]  {0, 1, 2, 3};
  csc     P_csc_;

  // A and associated dense vectors are rebuilt in solveQP from the barrier list.
  std::vector<c_float> A_data_;
  std::vector<c_int>   A_indices_;
  std::vector<c_int>   A_indptr_;
  csc                  A_csc_;

  c_float q_[3]{0, 0, 0};
  std::vector<c_float> l_vec_;
  std::vector<c_float> u_vec_;

  bool   debug_markers_{false};

  // Per-drone low-pass filter state for h_swm (keyed by swarm_traj_ index).
  std::unordered_map<size_t, double> h_swm_filt_;

  double last_h_min_{0.0};
  bool   last_triggered_{false};
  bool   last_infeasible_{false};
  Eigen::Vector3d last_blocked_dir_ = Eigen::Vector3d::Zero();
};

}  // namespace cbf