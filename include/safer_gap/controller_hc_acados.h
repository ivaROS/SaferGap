/*********************************************************************
 *
 *  Software License Agreement
 *
 *  Copyright (c) 2022, Shiyu Feng, All rights reserved.
 *
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program.  If not, see <https://www.gnu.org/licenses/>.
 *
 *  Authors: Shiyu Feng
 *********************************************************************/

#ifndef PG_MPC_HARD_CSTR_CONTROLLER_ACADOS_H_
#define PG_MPC_HARD_CSTR_CONTROLLER_ACADOS_H_

#include <corbo-controllers/predictive_controller.h>

#include <corbo-optimal-control/structured_ocp/structured_optimal_control_problem.h>
#include <mpc_local_planner/optimal_control/stage_inequality_se2.h>
#include <mpc_local_planner/systems/robot_dynamics_interface.h>
#include <teb_local_planner/obstacles.h>
#include <teb_local_planner/pose_se2.h>

#include <mpc_local_planner_msgs/StateFeedback.h>

#include <base_local_planner/costmap_model.h>

#include <ros/subscriber.h>
#include <ros/time.h>
#include <nav_msgs/Odometry.h>

#include <memory>
#include <mutex>

// #include <Eigen/Geometry>

#include <casadi/casadi.hpp>
#include <safer_gap/keyhole.hpp>
#include <potential_gap/gap.h>
#include <safer_gap/utils.h>
#include <potential_gap/robot_geo_parser.h>

#include <pips_trajectory_msgs/trajectory_point.h>
#include <pips_trajectory_msgs/trajectory_points.h>

#include <safer_gap/TimingStats.h>

// acados
#include "acados/utils/print.h"
#include "acados_c/ocp_nlp_interface.h"
#include "acados_c/external_function_interface.h"
#include "acados/ocp_nlp/ocp_nlp_constraints_bgh.h"
#include "acados/ocp_nlp/ocp_nlp_cost_ls.h"

// blasfeo
#include "blasfeo/include/blasfeo_d_aux.h"
#include "blasfeo/include/blasfeo_d_aux_ext_dep.h"

// crazyflie specific
#include "unicycle_keyhole_mpc_model/unicycle_keyhole_mpc_model.h"
#include "unicycle_keyhole_mpc_constraints/unicycle_keyhole_mpc_constraints.h"
#include "acados_solver_unicycle_keyhole_mpc.h"

namespace pg_mpc_local_planner {

using namespace mpc_local_planner;
using namespace casadi;

/**
 * @brief Potential Gap MPC controller for mobile robots
 *
 * @ingroup controllers
 *
 * @author Shiyu Feng (shiyufeng@gatech.edu)
 */

class PGHCAcadosController : public corbo::PredictiveController
{
public:
   using Ptr     = std::shared_ptr<PGHCAcadosController>;
   using PoseSE2 = teb_local_planner::PoseSE2;

   PGHCAcadosController() = default;

   bool configure(ros::NodeHandle& nh);
   bool configure(ros::NodeHandle& nh, double controller_frequency, bool ni_enabled=false);

   bool stepHC(const potential_gap::StaticInfGap& gap, const std::pair<double, double>& ego_min, potential_gap::RobotGeoProc& robot_geo, const pips_trajectory_msgs::trajectory_points& initial_plan, const PoseSE2& robot_pose, const geometry_msgs::Twist& vel, ros::Duration t_diff,
            DM& u_opt, DM& x_seq);

   bool stepPCont(const potential_gap::StaticInfGap& gap, const std::pair<double, double>& ego_min, const pips_trajectory_msgs::trajectory_points& initial_plan, const PoseSE2& robot_pose, const geometry_msgs::Twist& vel, ros::Duration t_diff,
            bool has_feedforward, Eigen::Vector2f& u_opt);

   DM projectionOperator(const std::pair<double, double>& ego_min, DM& u);
   Eigen::Vector3d projection_method(double min_diff_x, double min_diff_y);

   pips_trajectory_msgs::trajectory_point interpPose(const pips_trajectory_msgs::trajectory_points& traj, double t);
   pips_trajectory_msgs::trajectory_point findNearestPose(const pips_trajectory_msgs::trajectory_points& traj, const PoseSE2& robot_pose);
   Eigen::Matrix2cd getComplexMatrix(double x, double y, double quat_w, double quat_z);

//  bool step(const PoseSE2& start, const PoseSE2& goal, const geometry_msgs::Twist& vel, double dt, ros::Time t, corbo::TimeSeries::Ptr u_seq,
//            corbo::TimeSeries::Ptr x_seq);

   // implements interface method
   corbo::ControllerInterface::Ptr getInstance() const override { return std::make_shared<PGHCAcadosController>(); }
   static corbo::ControllerInterface::Ptr getInstanceStatic() { return std::make_shared<PGHCAcadosController>(); }

//  void setOptimalControlProblem(corbo::OptimalControlProblemInterface::Ptr ocp) = delete;

   void stateFeedbackCallback(const mpc_local_planner_msgs::StateFeedback::ConstPtr& msg);

//  void publishOptimalControlResult();

   void setInitialPlanEstimateOrientation(bool estimate) { _initial_plan_estimate_orientation = estimate; }

   /**
    * @brief Check whether the planned trajectory is feasible or not.
    *
    * This method currently checks only that the trajectory, or a part of the trajectory is collision free.
    * Obstacles are here represented as costmap instead of the internal ObstacleContainer.
    * @param costmap_model Pointer to the costmap model
    * @param footprint_spec The specification of the footprint of the robot in world coordinates
    * @param inscribed_radius The radius of the inscribed circle of the robot
    * @param circumscribed_radius The radius of the circumscribed circle of the robot
    * @param min_resolution_collision_check_angular Min angular resolution during the costmap collision check:
    *        if not respected intermediate samples are added. [rad]
    * @param look_ahead_idx Number of poses along the trajectory that should be verified, if -1, the complete trajectory will be checked.
    * @return \c true, if the robot footprint along the first part of the trajectory intersects with
    *         any obstacle in the costmap, \c false otherwise.
    */
//  virtual bool isPoseTrajectoryFeasible(base_local_planner::CostmapModel* costmap_model, const std::vector<geometry_msgs::Point>& footprint_spec,
//                                        double inscribed_radius = 0.0, double circumscribed_radius = 0.0,
//                                        double min_resolution_collision_check_angular = M_PI, int look_ahead_idx = -1);

   // implements interface method
   void reset() override;

   bool forceReplan()
   {
      if(prev_u_.nnz() == 0)
         return true;
      else
         return false;
   }

   std::shared_ptr<keyhole::Keyhole> getKeyhole()
   {
      return keyhole_;
   }

   void setPrintFlags(printFlags flags)
   {
      print_debug_info_ = flags.print_debug_info;
      print_timing_ = flags.print_timing;
   }

   uint64_t getMPCFailNum()
   {
      return mpc_fail_num_;
   }

   void resetMPCFailNum()
   {
      mpc_fail_num_ = 0;
   }

   uint64_t getKeyholeFailNum()
   {
      return keyhole_fail_num_;
   }

   void resetKeyholeFailNum()
   {
      keyhole_fail_num_ = 0;
   }

   void getTiming(safer_gap::TimingStats& time_stats)
   {
      time_stats.keyhole_time = keyhole_time_;
      time_stats.mpc_time = mpc_time_;
      time_stats.po_time = po_time_;
   }

   void resetTimingStats()
   {
      keyhole_time_ = 0;
      mpc_time_ = 0;
      po_time_ = 0;
   }

protected:
   void configureParams(const ros::NodeHandle& nh);
   void configureRobotDynamics(const ros::NodeHandle& nh);
   void configureNMPC(const ros::NodeHandle& nh);

//  bool generateInitialStateTrajectory(const Eigen::VectorXd& x0, const Eigen::VectorXd& xf,
//                                      const std::vector<geometry_msgs::PoseStamped>& initial_plan, bool backward);

//  MX getState(const geometry_msgs::PoseStamped& input_pose);

   std::string _robot_type;

   ros::Publisher _ocp_result_pub;
   bool _ocp_successful      = false;
   std::size_t _ocp_seq      = 0;
   bool _publish_ocp_results = false;
   // bool _print_cpu_time      = false;

   bool print_debug_info_    = false;
   bool print_timing_        = false;

   bool _prefer_x_feedback = false;  // prefer state feedback over odometry feedback
   ros::Subscriber _x_feedback_sub;
   std::mutex _x_feedback_mutex;
   ros::Time _recent_x_time;
   Eigen::VectorXd _recent_x_feedback;

   teb_local_planner::PoseSE2 _last_goal;
   double _force_reinit_new_goal_dist    = 1.0;
   double _force_reinit_new_goal_angular = 0.5 * M_PI;
   corbo::DiscreteTimeReferenceTrajectory _x_seq_init;
   bool _initial_plan_estimate_orientation = true;
   bool _guess_backwards_motion            = true;
   int _force_reinit_num_steps             = 0;
 
 // Casadi
protected:
   dynamicsParams dyn_params_;
   nmpcParams nmpc_params_;

   int nx, nu;
   double u_lin_ref_, time_int_;
   nav_msgs::Odometry::ConstPtr odom_msg_;
   SX x_, u_, robot_dynamics_, X_, U_;
   Function state_func_, intg_func_, dynamics_func_, opti_func_;
   Dict intg_options_, opti_options_, qpsol_options_;
   DMDict const_args_;
   Opti opti_;
   int T_, N_;

   DM prev_u_, prev_opt_u_;

   bool use_keyhole_ = false, use_cbf_ = false;
   std::shared_ptr<keyhole::Keyhole> keyhole_;
   bool init_keyhole_;
   keyhole::KeyholeParam prev_suc_keyhole_;

   bool ni_enabled_, fail_stop_;

   double cbf_gamma_, cbf_kw_;

   double k_drive_x_, k_drive_y_, k_turn_;
   int ctrl_ahead_pose_;

   //PO
   bool use_po_ = false;
   double r_norm_, r_norm_offset_, r_inscr_, r_min_;
   double k_po_, k_po_turn_;

   // Save stats results
   uint64_t mpc_fail_num_ = 0, keyhole_fail_num_ = 0;

   // Timing
   double keyhole_time_, mpc_time_, po_time_;

   // acados
   int nx_aug, nu_aug, ny;
   unicycle_keyhole_mpc_solver_capsule *acados_ocp_capsule;
   ocp_nlp_config *nlp_config;
   ocp_nlp_dims *nlp_dims;
   ocp_nlp_in *nlp_in;
   ocp_nlp_out *nlp_out;
   ocp_nlp_solver *nlp_solver;
   void *nlp_opts;
};

}  // namespace pg_mpc_local_planner

#endif  // PG_MPC_HARD_CSTR_CONTROLLER_ACADOS_H_
