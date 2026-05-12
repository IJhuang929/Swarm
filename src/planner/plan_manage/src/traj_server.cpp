#include <nav_msgs/Odometry.h>
#include <traj_utils/PolyTraj.h>
#include <cbf/cbf_filter.h>
#include <optimizer/poly_traj_utils.hpp>
#include <quadrotor_msgs/PositionCommand.h>
#include <std_msgs/Empty.h>
#include <visualization_msgs/Marker.h>
#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/String.h>
#include <std_msgs/Float64.h>
#include <geometry_msgs/TwistStamped.h>
#include <fstream>
#include <sstream>
#include <iomanip>
#include <sys/stat.h>
#include <chrono>

ros::Publisher pos_cmd_pub;
ros::Publisher replan_trigger_pub_;
ros::Time last_replan_trigger_(0);

quadrotor_msgs::PositionCommand cmd;
double pos_gain[3] = {0, 0, 0};
double vel_gain[3] = {0, 0, 0};

bool receive_traj_ = false;
boost::shared_ptr<poly_traj::Trajectory> traj_;
double traj_duration_;
ros::Time start_time_;
int traj_id_;

// yaw control
double last_yaw_, last_yaw_dot_;
double time_forward_;

Eigen::Vector3d last_vel(Eigen::Vector3d::Zero()), last_acc(Eigen::Vector3d::Zero()), last_jerk(Eigen::Vector3d::Zero());
bool flag = false;
double jerk2_inter = 0, acc2_inter = 0;
int cnt = 0;
ros::Time global_start_time;
int drone_id_;
Eigen::Vector3d odom_pos_(Eigen::Vector3d::Zero());
cbf::CbfFilter cbf_filter_;
std::string result_dir = "/home/ubuntu/catkin_ws/src/Swarm-Formation/results/";
std::fstream result_file;
std::fstream cbf_debug_file;
std::vector<Eigen::Vector3d> pos_vec_, vel_vec_, acc_vec_, jerk_vec_;
std::vector<double> time_vec_;

ros::Publisher cbf_debug_pub_;
ros::Publisher cbf_state_pub_;
ros::Publisher cbf_h_min_pub_;
double dbg_h_min_      = 0.0;
bool   dbg_triggered_  = false;
bool   dbg_infeasible_ = false;
double dbg_vel_before_ = 0.0;
double dbg_vel_after_  = 0.0;

int    cbf_trigger_count_    = 0;
int    cbf_infeasible_count_ = 0;
double global_h_min_         = 1e9;
std::vector<double> exec_time_vec_;

const std::vector<std::string> explode(const std::string& s, const char& c)
{
  std::string buff{""};
  std::vector<std::string> v;
  
  for(auto n:s)
  {
    if(n != c) buff+=n; else
    if(n == c && buff != "") { v.push_back(buff); buff = ""; }
  }
  if(buff != "") v.push_back(buff);
  
  return v;
}

void odomCallback(const nav_msgs::OdometryConstPtr& msg)
{
  odom_pos_(0) = msg->pose.pose.position.x;
  odom_pos_(1) = msg->pose.pose.position.y;
  odom_pos_(2) = msg->pose.pose.position.z;
}

void polyTrajCallback(traj_utils::PolyTrajPtr msg)
{
  if (msg->order != 5)
  {
    ROS_ERROR("[traj_server] Only support trajectory order equals 5 now!");
    return;
  }
  if (msg->duration.size() * (msg->order + 1) != msg->coef_x.size())
  {
    ROS_ERROR("[traj_server] WRONG trajectory parameters, ");
    return;
  }

  int piece_nums = msg->duration.size();
  std::vector<double> dura(piece_nums);
  std::vector<poly_traj::CoefficientMat> cMats(piece_nums);
  for (int i = 0; i < piece_nums; ++i)
  {
    int i6 = i * 6;
    cMats[i].row(0) << msg->coef_x[i6 + 0], msg->coef_x[i6 + 1], msg->coef_x[i6 + 2],
        msg->coef_x[i6 + 3], msg->coef_x[i6 + 4], msg->coef_x[i6 + 5];
    cMats[i].row(1) << msg->coef_y[i6 + 0], msg->coef_y[i6 + 1], msg->coef_y[i6 + 2],
        msg->coef_y[i6 + 3], msg->coef_y[i6 + 4], msg->coef_y[i6 + 5];
    cMats[i].row(2) << msg->coef_z[i6 + 0], msg->coef_z[i6 + 1], msg->coef_z[i6 + 2],
        msg->coef_z[i6 + 3], msg->coef_z[i6 + 4], msg->coef_z[i6 + 5];

    dura[i] = msg->duration[i];
  }

  traj_.reset(new poly_traj::Trajectory(dura, cMats));

  start_time_ = msg->start_time;
  traj_duration_ = traj_->getTotalDuration();
  traj_id_ = msg->traj_id;

  receive_traj_ = true;
}

void finishCallback(const std_msgs::Bool::ConstPtr& msg) {
  if (msg->data == true) {
    ROS_WARN("total_time, cnt, acc2_inter, jerk2_inter = %lf \t %d \t %lf \t %lf", (ros::Time::now() - global_start_time).toSec(), cnt, acc2_inter, jerk2_inter);
    // ROS_WARN("time.size = %d, %d, %d, %d", time_vec_.size(), vel_vec_.size(), acc_vec_.size(), jerk_vec_.size());
    auto t = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());
    //转为字符串
    std::stringstream ss;
    ss << std::put_time(std::localtime(&t), "%Y-%m-%d-%H-%M-%S");
    std::string str_time = ss.str();
    result_file << str_time << "\n";
    double max_vel2 = 0;
    for (int i = 0; i < time_vec_.size(); i++) {
      double tmp_vel2 = (vel_vec_[i](0))*(vel_vec_[i](0)) + (vel_vec_[i](1))*(vel_vec_[i](1)) + (vel_vec_[i](2))*(vel_vec_[i](2));
      max_vel2 = (tmp_vel2 > max_vel2) ? tmp_vel2 : max_vel2; 
    }
    result_file << "cbf_enabled=" << (cbf_filter_.isEnabled() ? 1 : 0) << "\n";
    result_file << "r_safe=" << cbf_filter_.getRSafe() << "\n";
    result_file << "max_vel = " << sqrt(max_vel2) << "\n";
    result_file << "cbf_triggers=" << cbf_trigger_count_ << "\n";
    result_file << "cbf_infeasible=" << cbf_infeasible_count_ << "\n";
    result_file << "global_h_min=" << (global_h_min_ < 1e9 ? global_h_min_ : -1.0) << "\n";
    double sum_exec = 0.0;
    for (auto t : exec_time_vec_) sum_exec += t;
    double mean_exec = exec_time_vec_.empty() ? 0.0 : sum_exec / exec_time_vec_.size();
    double max_exec = 0.0;
    for (auto t : exec_time_vec_) if (t > max_exec) max_exec = t;
    result_file << "cmd_exec_ms=" << mean_exec << "(mean) " << max_exec << "(max)\n";
    result_file << "\n";
  }
}

void startCallback(const std_msgs::Bool::ConstPtr& msg) {
  if (msg->data == true) {
    ROS_WARN("START!!!!");
    global_start_time = ros::Time::now();
  }
}

void debugCallback(const ros::TimerEvent&) {
  std_msgs::String msg;
  std::ostringstream oss;
  oss << std::fixed << std::setprecision(2)
      << "h_min=" << dbg_h_min_
      << " triggered=" << (int)dbg_triggered_
      << " infeasible=" << (int)dbg_infeasible_
      << " vel_before=" << dbg_vel_before_
      << " vel_after=" << dbg_vel_after_;
  msg.data = oss.str();
  cbf_debug_pub_.publish(msg);

  if (cbf_debug_file.is_open()) {
    cbf_debug_file << std::fixed << std::setprecision(3)
                   << ros::Time::now().toSec() << " " << oss.str() << "\n";
    cbf_debug_file.flush();
  }
}

std::pair<double, double> calculate_yaw(double t_cur, Eigen::Vector3d &pos, ros::Time &time_now, ros::Time &time_last)
{
  constexpr double PI = 3.1415926;
  constexpr double YAW_DOT_MAX_PER_SEC = PI;
  // constexpr double YAW_DOT_DOT_MAX_PER_SEC = PI;
  std::pair<double, double> yaw_yawdot(0, 0);
  double yaw = 0;
  double yawdot = 0;

  Eigen::Vector3d dir = t_cur + time_forward_ <= traj_duration_
                            ? traj_->getPos(t_cur + time_forward_) - pos
                            : traj_->getPos(traj_duration_) - pos;
  double yaw_temp = dir.norm() > 0.1
                        ? atan2(dir(1), dir(0))
                        : last_yaw_;
  double max_yaw_change = YAW_DOT_MAX_PER_SEC * (time_now - time_last).toSec();
  if (yaw_temp - last_yaw_ > PI)
  {
    if (yaw_temp - last_yaw_ - 2 * PI < -max_yaw_change)
    {
      yaw = last_yaw_ - max_yaw_change;
      if (yaw < -PI)
        yaw += 2 * PI;

      yawdot = -YAW_DOT_MAX_PER_SEC;
    }
    else
    {
      yaw = yaw_temp;
      if (yaw - last_yaw_ > PI)
        yawdot = -YAW_DOT_MAX_PER_SEC;
      else
        yawdot = (yaw_temp - last_yaw_) / (time_now - time_last).toSec();
    }
  }
  else if (yaw_temp - last_yaw_ < -PI)
  {
    if (yaw_temp - last_yaw_ + 2 * PI > max_yaw_change)
    {
      yaw = last_yaw_ + max_yaw_change;
      if (yaw > PI)
        yaw -= 2 * PI;

      yawdot = YAW_DOT_MAX_PER_SEC;
    }
    else
    {
      yaw = yaw_temp;
      if (yaw - last_yaw_ < -PI)
        yawdot = YAW_DOT_MAX_PER_SEC;
      else
        yawdot = (yaw_temp - last_yaw_) / (time_now - time_last).toSec();
    }
  }
  else
  {
    if (yaw_temp - last_yaw_ < -max_yaw_change)
    {
      yaw = last_yaw_ - max_yaw_change;
      if (yaw < -PI)
        yaw += 2 * PI;

      yawdot = -YAW_DOT_MAX_PER_SEC;
    }
    else if (yaw_temp - last_yaw_ > max_yaw_change)
    {
      yaw = last_yaw_ + max_yaw_change;
      if (yaw > PI)
        yaw -= 2 * PI;

      yawdot = YAW_DOT_MAX_PER_SEC;
    }
    else
    {
      yaw = yaw_temp;
      if (yaw - last_yaw_ > PI)
        yawdot = -YAW_DOT_MAX_PER_SEC;
      else if (yaw - last_yaw_ < -PI)
        yawdot = YAW_DOT_MAX_PER_SEC;
      else
        yawdot = (yaw_temp - last_yaw_) / (time_now - time_last).toSec();
    }
  }

  if (fabs(yaw - last_yaw_) <= max_yaw_change)
    yaw = 0.5 * last_yaw_ + 0.5 * yaw; // nieve LPF
  yawdot = 0.5 * last_yaw_dot_ + 0.5 * yawdot;
  last_yaw_ = yaw;
  last_yaw_dot_ = yawdot;

  yaw_yawdot.first = yaw;
  yaw_yawdot.second = yawdot;

  return yaw_yawdot;
}

void cmdCallback(const ros::TimerEvent &e)
{
  /* no publishing before receive traj_ */
  if (!receive_traj_)
    return;

  ros::Time t_exec_start = ros::Time::now();
  ros::Time time_now = ros::Time::now();
  double t_cur = (time_now - start_time_).toSec();


  Eigen::Vector3d pos(Eigen::Vector3d::Zero()), vel(Eigen::Vector3d::Zero()), acc(Eigen::Vector3d::Zero()), jerk(Eigen::Vector3d::Zero()), pos_f;
  std::pair<double, double> yaw_yawdot(0, 0);

  static ros::Time time_last = ros::Time::now();
  if (flag == false) {
    flag = true;
  } else {
    cnt++;
    acc2_inter += last_acc.norm()*last_acc.norm()*(time_now-time_last).toSec();
    jerk2_inter += last_jerk.norm()*last_jerk.norm()*(time_now-time_last).toSec();
    
  }
  if (t_cur < traj_duration_ && t_cur >= 0.0)
  {
    pos = traj_->getPos(t_cur);
    vel = traj_->getVel(t_cur);
    acc = traj_->getAcc(t_cur);
    jerk = traj_->getJer(t_cur);

    /*** calculate yaw ***/
    yaw_yawdot = calculate_yaw(t_cur, pos, time_now, time_last);
    /*** calculate yaw ***/

    double tf = std::min(traj_duration_, t_cur + 2.0);
    pos_f = traj_->getPos(tf);
  }
  else if (t_cur >= traj_duration_)
  {
    /* hover when finish traj_ */
    pos = traj_->getPos(traj_duration_);
    vel.setZero();
    acc.setZero();

    yaw_yawdot.first = last_yaw_;
    yaw_yawdot.second = 0;

    pos_f = pos;

    // When CBF is in active violation (h < 0), suppress hover corrections so the
    // CBF's QP recovery (vel_ref=0 → minimum-norm repulsion) can separate the drones.
    // When h >= 0, restore normal P-controller homing.
    auto prev_cbf = cbf_filter_.getStats();
    const bool cbf_violating = (prev_cbf.h_min < 0.0 && prev_cbf.h_min > -1e8);
    if (!cbf_violating) {
      Eigen::Vector3d pos_err = pos - odom_pos_;
      Eigen::Vector3d ref_vel = 0.8 * pos_err;
      double v_max = 0.5;
      double v_norm = ref_vel.norm();
      if (v_norm > v_max) ref_vel = ref_vel / v_norm * v_max;
      const double hover_deadband = 0.05;
      if (pos_err.norm() > hover_deadband) vel = ref_vel;
    }
    // else: vel stays zero; CBF QP will produce minimal repulsion to separate drones
  }

  time_last = time_now;
  time_vec_.push_back((ros::Time::now() - global_start_time).toSec());
  pos_vec_.push_back(pos);
  vel_vec_.push_back(vel);
  acc_vec_.push_back(acc);
  jerk_vec_.push_back(jerk);
  last_vel = vel;
  last_acc = acc;
  last_jerk = jerk;

  cmd.header.stamp = time_now;
  cmd.header.frame_id = "world";
  cmd.trajectory_flag = quadrotor_msgs::PositionCommand::TRAJECTORY_STATUS_READY;
  cmd.trajectory_id = traj_id_;

  cmd.position.x = pos(0);
  cmd.position.y = pos(1);
  cmd.position.z = pos(2);
  
  cmd.velocity.x = vel(0);
  cmd.velocity.y = vel(1);
  cmd.velocity.z = vel(2);

  cmd.acceleration.x = acc(0);
  cmd.acceleration.y = acc(1);
  cmd.acceleration.z = acc(2);

  cmd.yaw = yaw_yawdot.first;
  cmd.yaw_dot = yaw_yawdot.second;

  last_yaw_ = cmd.yaw;

  bool in_hover = (t_cur >= traj_duration_);
  dbg_vel_before_ = vel.norm();
  bool cbf_active = cbf_filter_.filter(odom_pos_, vel, acc, time_now.toSec());
  {
    auto s = cbf_filter_.getStats();
    dbg_h_min_      = s.h_min;
    dbg_triggered_  = s.cbf_triggered;
    dbg_infeasible_ = s.infeasible;
    dbg_vel_after_  = vel.norm();
    std_msgs::Float64 h_min_msg;
    h_min_msg.data = s.h_min;
    cbf_h_min_pub_.publish(h_min_msg);
  }

  // In hover mode, vel_ref was either 0 (violation suppressed) or small (P-ctrl).
  // CBF-generated repulsion from vel_ref=0 must NOT be commanded: it would cause
  // perpetual motion since the drones can never settle at overlapping targets.
  // Just lock the drone in place; the collision is already recorded in global_h_min_.
  if (in_hover && dbg_vel_before_ <= 1e-6 && cbf_active) {
    vel.setZero();
    acc.setZero();
    cbf_active = false;
  }

  if (cbf_active) {
    cmd.position.x = odom_pos_(0) + vel(0) * 0.05;
    cmd.position.y = odom_pos_(1) + vel(1) * 0.05;
    cmd.position.z = odom_pos_(2) + vel(2) * 0.05;
    cmd.velocity.x = vel(0);
    cmd.velocity.y = vel(1);
    cmd.velocity.z = vel(2);
    cmd.acceleration.x = acc(0);
    cmd.acceleration.y = acc(1);
    // acc(2) already preserved by filter; cmd.acceleration.z unchanged

    // Only trigger replan during active flight (not hover): hover replanning jumps
    // the FSM from WAIT_TARGET to GEN_NEW_TRAJ causing spin loops.
    if (!in_hover && dbg_infeasible_ &&
        (time_now - last_replan_trigger_).toSec() > 0.5) {
      ROS_WARN("[REPLAN] trigger=cbf state=executing drone=%d h_min=%.3f vel=(%.2f,%.2f,%.2f)",
               drone_id_, dbg_h_min_, vel(0), vel(1), vel(2));
      replan_trigger_pub_.publish(std_msgs::Empty());
      last_replan_trigger_ = time_now;
    }

    geometry_msgs::TwistStamped cbf_state;
    cbf_state.header.stamp = time_now;
    cbf_state.twist.linear.x = vel(0);
    cbf_state.twist.linear.y = vel(1);
    cbf_state.twist.linear.z = vel(2);
    cbf_state.twist.angular.x = acc(0);
    cbf_state.twist.angular.y = acc(1);
    cbf_state.twist.angular.z = acc(2);
    cbf_state_pub_.publish(cbf_state);
  }

  if (cbf_active) cbf_trigger_count_++;
  if (dbg_infeasible_) cbf_infeasible_count_++;
  if (dbg_h_min_ < 1e9 && dbg_h_min_ < global_h_min_) global_h_min_ = dbg_h_min_;
  exec_time_vec_.push_back((ros::Time::now() - t_exec_start).toSec() * 1000.0);
  // >>> MOD: lock position to target if CBF not active
  if (!cbf_active) {
    cmd.position.x = pos(0);
    cmd.position.y = pos(1);
    cmd.position.z = pos(2);
  }
  pos_cmd_pub.publish(cmd);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "traj_server");
  // ros::NodeHandle node;
  ros::NodeHandle nh("~");
  // get drone num
  std::string name_drone = ros::this_node::getName();
  std::vector<std::string> v{explode(name_drone, '_')};
  drone_id_ = std::stoi(v[1]);

  {
    auto _ts = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());
    std::stringstream _ss;
    _ss << std::put_time(std::localtime(&_ts), "%Y-%m-%d_%H-%M-%S");
    std::string run_dir = result_dir + _ss.str() + "/";
    mkdir(result_dir.c_str(), 0755);
    mkdir(run_dir.c_str(),    0755);

    result_file.open(    run_dir + "result_drone_" + v[1] + ".txt", std::ios::out);
    cbf_debug_file.open( run_dir + "cbf_debug_"   + v[1] + ".txt", std::ios::out);

    double gamma, r_safe, v_max, swarm_clearance, obstacle_clearance, replan_cooldown;
    nh.param("cbf/gamma",                       gamma,             1.0);
    nh.param("cbf/r_safe",                      r_safe,      0.3);
    nh.param("cbf/v_max",                       v_max,             3.0);
    nh.param("optimization/swarm_clearance",    swarm_clearance,   0.4);
    nh.param("optimization/obstacle_clearance", obstacle_clearance,0.5);
    nh.param("cbf/replan_cooldown",             replan_cooldown,   0.5);
    std::ofstream params_f(run_dir + "params.txt");
    params_f << "cbf/gamma="              << gamma             << "\n"
             << "cbf/r_safe="       << r_safe      << "\n"
             << "cbf/v_max="              << v_max             << "\n"
             << "swarm_clearance="        << swarm_clearance   << "\n"
             << "obstacle_clearance="     << obstacle_clearance<< "\n"
             << "replan_cooldown="        << replan_cooldown   << "\n";
  }

  ros::Subscriber poly_traj_sub = nh.subscribe("planning/trajectory", 10, polyTrajCallback);
  ros::Subscriber reached_sub = nh.subscribe("planning/finish", 10, finishCallback);
  ros::Subscriber start_sub = nh.subscribe("planning/start", 10, startCallback);
  ros::Subscriber odom_sub = nh.subscribe("odom_world", 1, odomCallback);
  pos_cmd_pub = nh.advertise<quadrotor_msgs::PositionCommand>("/position_cmd", 50);
  replan_trigger_pub_ = nh.advertise<std_msgs::Empty>("planning/trigger_replan", 1);
  cbf_debug_pub_ = nh.advertise<std_msgs::String>("cbf_debug", 10);
  cbf_state_pub_ = nh.advertise<geometry_msgs::TwistStamped>("planning/cbf_corrected_state", 10);
  cbf_h_min_pub_ = nh.advertise<std_msgs::Float64>("planning/cbf_h_min", 10);
  

  ros::Timer cmd_timer   = nh.createTimer(ros::Duration(0.01), cmdCallback);
  ros::Timer debug_timer = nh.createTimer(ros::Duration(0.1),  debugCallback);

  /* control parameter */
  cmd.kx[0] = pos_gain[0];
  cmd.kx[1] = pos_gain[1];
  cmd.kx[2] = pos_gain[2];

  cmd.kv[0] = vel_gain[0];
  cmd.kv[1] = vel_gain[1];
  cmd.kv[2] = vel_gain[2];

  cbf_filter_.init(nh, drone_id_);

  nh.param("traj_server/time_forward", time_forward_, -1.0);
  last_yaw_ = 0.0;
  last_yaw_dot_ = 0.0;

  ros::Duration(1.0).sleep();

  ROS_WARN("[Traj server]: ready.");

  ros::spin();

  return 0;
}