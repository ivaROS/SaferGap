/*********************************************************************
 *
 *  Software License Agreement
 *
 *  Copyright (c) 2020, Christoph Rösmann, All rights reserved.
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
 *  Authors: Christoph Rösmann
 *********************************************************************/

#include <potential_gap_mpc/controller_hc.h>

#include <corbo-optimal-control/functions/hybrid_cost.h>
#include <corbo-optimal-control/functions/minimum_time.h>
#include <corbo-optimal-control/functions/quadratic_control_cost.h>
#include <corbo-optimal-control/structured_ocp/discretization_grids/finite_differences_variable_grid.h>
#include <corbo-optimal-control/structured_ocp/structured_optimal_control_problem.h>
#include <corbo-optimization/hyper_graph/hyper_graph_optimization_problem_edge_based.h>
#include <corbo-optimization/solver/levenberg_marquardt_sparse.h>
#include <corbo-optimization/solver/nlp_solver_ipopt.h>
#include <corbo-optimization/solver/qp_solver_osqp.h>
#include <mpc_local_planner/optimal_control/fd_collocation_se2.h>
#include <mpc_local_planner/optimal_control/final_state_conditions_se2.h>
#include <mpc_local_planner/optimal_control/finite_differences_variable_grid_se2.h>
#include <mpc_local_planner/optimal_control/min_time_via_points_cost.h>
#include <mpc_local_planner/optimal_control/quadratic_cost_se2.h>
#include <mpc_local_planner/optimal_control/stage_inequality_se2.h>
#include <mpc_local_planner/systems/kinematic_bicycle_model.h>
#include <mpc_local_planner/systems/simple_car.h>
#include <mpc_local_planner/systems/unicycle_robot.h>
#include <mpc_local_planner/utils/time_series_se2.h>

#include <mpc_local_planner_msgs/OptimalControlResult.h>
#include <mpc_local_planner_msgs/StateFeedback.h>

#include <corbo-communication/utilities.h>
#include <corbo-core/console.h>

#include <tf2/utils.h>

#include <memory>
#include <mutex>

#include <Eigen/Dense>
#include <Eigen/Geometry>

namespace pg_mpc_local_planner {

bool PGHCController::configure(ros::NodeHandle& nh)
{
    mpc_fail_num_ = 0;

    init_keyhole_ = false;
    keyhole_ = std::make_shared<keyhole::Keyhole>();

    nh.param("controller/use_keyhole", use_keyhole_, use_keyhole_);
    nh.param("controller/use_cbf", use_cbf_, use_cbf_);

    fail_stop_ = false;
    nh.param("controller/fail_stop", fail_stop_, fail_stop_);

    k_drive_x_ = 1, k_drive_y_ = 1, k_turn_ = 1;
    ctrl_ahead_pose_ = 2;
    nh.param("controller/pose_con/k_drive_x", k_drive_x_, k_drive_x_);
    nh.param("controller/pose_con/k_drive_y", k_drive_y_, k_drive_y_);
    nh.param("controller/pose_con/k_turn", k_turn_, k_turn_);
    nh.param("controller/pose_con/ctrl_ahead_pose", ctrl_ahead_pose_, ctrl_ahead_pose_);

    nh.param("controller/use_po", use_po_, use_po_);
    r_norm_ = 1, r_norm_offset_ = 0, r_inscr_ = 0.2, r_min_ = 0.36;
    k_po_ = 1, k_po_turn_ = 1;
    nh.param("controller/po/r_norm", r_norm_, r_norm_);
    nh.param("controller/po/r_norm_offset", r_norm_offset_, r_norm_offset_);
    nh.param("controller/po/r_inscr", r_inscr_, r_inscr_);
    nh.param("controller/po/r_min", r_min_, r_min_);
    nh.param("controller/po/k_po", k_po_, k_po_);
    nh.param("controller/po/k_po_turn", k_po_turn_, k_po_turn_);

    configureParams(nh);
    configureRobotDynamics(nh);
    configureNMPC(nh);

    // further goal opions
    nh.param("controller/force_reinit_new_goal_dist", _force_reinit_new_goal_dist, _force_reinit_new_goal_dist);
    nh.param("controller/force_reinit_new_goal_angular", _force_reinit_new_goal_angular, _force_reinit_new_goal_angular);

    nh.param("controller/allow_init_with_backward_motion", _guess_backwards_motion, _guess_backwards_motion);
    nh.param("controller/force_reinit_num_steps", _force_reinit_num_steps, _force_reinit_num_steps);

    // custom feedback:
    nh.param("controller/prefer_x_feedback", _prefer_x_feedback, _prefer_x_feedback);
    _x_feedback_sub = nh.subscribe("state_feedback", 1, &PGHCController::stateFeedbackCallback, this);
    
    return true;
}

bool PGHCController::configure(ros::NodeHandle& nh, double controller_frequency, bool ni_enabled)
{
    time_int_ = 1.0 / controller_frequency;
    ni_enabled_ = ni_enabled;
    return configure(nh);
}

// bool PGHCController::step(const PGHCController::PoseSE2& start, const PGHCController::PoseSE2& goal, const geometry_msgs::Twist& vel, double dt, ros::Time t,
//                       corbo::TimeSeries::Ptr u_seq, corbo::TimeSeries::Ptr x_seq)
// {
//     std::vector<geometry_msgs::PoseStamped> initial_plan(2);
//     start.toPoseMsg(initial_plan.front().pose);
//     goal.toPoseMsg(initial_plan.back().pose);
//     return step(initial_plan, vel, dt, t, u_seq, x_seq);
// }

bool PGHCController::stepHC(const potential_gap::StaticInfGap& gap, const std::pair<double, double>& ego_min, potential_gap::RobotGeoProc& robot_geo, const pips_trajectory_msgs::trajectory_points& initial_plan, const PoseSE2& robot_pose, const geometry_msgs::Twist& vel, ros::Duration t_diff,
                      DM& u_opt, DM& x_seq)
{
    if (initial_plan.points.size() < 2)
    {
        ROS_ERROR("Controller::step(): initial plan must contain at least two poses.");
        if(!fail_stop_)
        {
            u_opt = prev_u_(Slice(0, nu));
            ROS_WARN_STREAM("From prev u: " << u_opt);
        }
        else
        {
            u_opt = DM::zeros(1, nu);
            ROS_WARN_STREAM("Stop robot.");
        }

        if(use_po_)
        {
            u_opt = projectionOperator(ego_min, u_opt);
            ROS_INFO_STREAM("Control after projection operator: " << u_opt);
        }
        return false;
    }
    
    pips_trajectory_msgs::trajectory_point start_pose(initial_plan.points.front());
    pips_trajectory_msgs::trajectory_point goal_pose(initial_plan.points.back());

    DM u0 = prev_u_;
    DM x_0 = DM(std::vector<double>{robot_pose.x(), robot_pose.y(), robot_pose.theta()});
    DM X0 = DM::zeros(nx, N_+1);

    for(size_t i = 0; i < N_+1; i++)
    {
        X0(Slice(i*nx)) = x_0(0);
        X0(Slice(i*nx+1)) = x_0(1);
        X0(Slice(i*nx+2)) = x_0(2);
    }
    X0 = X0.T();
    
    const_args_["p"](Slice(0,nx)) = x_0;
    
    ROS_INFO_STREAM_COND(print_debug_info_, "x0: " << x_0);
    // ROS_INFO_STREAM("X0: " << X0);

    for(size_t i = 0; i < N_; i++)
    {
        double t = t_diff.toSec() + i * time_int_;
        pips_trajectory_msgs::trajectory_point interp_pose = interpPose(initial_plan, t);

        // std::cout << interp_pose << std::endl;

        const_args_["p"]((5*i+3)) = interp_pose.x;
        const_args_["p"]((5*i+4)) = interp_pose.y;
        const_args_["p"]((5*i+5)) = interp_pose.theta;
        const_args_["p"]((5*i+6)) = interp_pose.v;
        const_args_["p"]((5*i+7)) = interp_pose.w;
    }

    bool mpc_ready = false;
    bool optimal_found = false;
    bool keyhole_found = false;
    if(use_keyhole_)
    {
        Eigen::Vector2d xc = gap.getOrigMapPoseVec();
        double r = gap.getMinDist();
        Eigen::Vector2d l_int = gap.getLIntMapVec();
        Eigen::Vector2d r_int = gap.getRIntMapVec();
        Eigen::Vector2d lgap = gap.getLMapVec();
        Eigen::Vector2d rgap = gap.getRMapVec();
        
        // ROS_INFO_STREAM("Gap input to keyhole: \n" << xc << "\n" << r << "\n" << l_int << "\n" << r_int << "\n" << lgap << "\n" << rgap);

        ros::WallTime start = ros::WallTime::now();
        
        keyhole_->set_keyhole(xc, r, l_int, r_int, lgap, rgap);
        bool rbf_on = false;
        keyhole_found = keyhole_->synthesize(rbf_on);

        ros::WallDuration keyhole_elapsed = ros::WallTime::now() - start;
        float keyhole_time_elapsed = float(keyhole_elapsed.toNSec())/1000000;
        ROS_INFO_STREAM_COND(print_timing_, "Keyhole time: " << keyhole_time_elapsed << " ms. Found keyhole: " << keyhole_found);

        if(keyhole_found)
        {
            Eigen::VectorXd a;
            Eigen::Vector2d c1, c2, c3, c4, c5;
            double d1, d2, d3, d4, d5;
            keyhole_->get_param(a, c1, c2, c3, c4, c5, d1, d2, d3, d4, d5);

            init_keyhole_ = true;
            prev_suc_keyhole_ = keyhole::KeyholeParam(a, c1, c2, c3, c4, c5, d1, d2, d3, d4, d5);

            // ROS_INFO_STREAM("Keyhole params: \n " << a << " \n " << c1 << " \n " << c2 << " \n " << c3 << " \n " << d1 << " \n " << d2 << " \n " << d3);

            int ref_num = nx + N_ * (nx + nu);
            const_args_["p"](ref_num) = c1(0);
            const_args_["p"](ref_num+1) = c1(1);
            const_args_["p"](ref_num+2) = d1;

            const_args_["p"](ref_num+3) = c2(0);
            const_args_["p"](ref_num+4) = c2(1);
            const_args_["p"](ref_num+5) = d2;

            const_args_["p"](ref_num+6) = c3(0);
            const_args_["p"](ref_num+7) = c3(1);
            const_args_["p"](ref_num+8) = d3;

            const_args_["p"](ref_num+9) = c4(0);
            const_args_["p"](ref_num+10) = c4(1);
            const_args_["p"](ref_num+11) = d4;

            const_args_["p"](ref_num+12) = c5(0);
            const_args_["p"](ref_num+13) = c5(1);
            const_args_["p"](ref_num+14) = d5;

            const_args_["p"](ref_num+15) = xc(0);
            const_args_["p"](ref_num+16) = xc(1);
            const_args_["p"](ref_num+17) = r;

            for(size_t i = 0; i < 16; i++)
            {
                const_args_["p"](ref_num+18+i) = a(i);
            }

            mpc_ready = true;
        }
        else
        {
            // Use previous when keyhole is not found.
            // int ref_num = nx + N_ * (nx + nu);
            // for(size_t i = 0; i < 33; i++)
            // {
            //     const_args_["p"](ref_num+i) = 0;
            // }
            // const_args_["p"](ref_num+33) = 1.;
            bool use_prev_keyhole = true;
            if(use_prev_keyhole && init_keyhole_)
            {
                int ref_num = nx + N_ * (nx + nu);
                const_args_["p"](ref_num) = prev_suc_keyhole_.c1(0);
                const_args_["p"](ref_num+1) = prev_suc_keyhole_.c1(1);
                const_args_["p"](ref_num+2) = prev_suc_keyhole_.d1;

                const_args_["p"](ref_num+3) = prev_suc_keyhole_.c2(0);
                const_args_["p"](ref_num+4) = prev_suc_keyhole_.c2(1);
                const_args_["p"](ref_num+5) = prev_suc_keyhole_.d2;

                const_args_["p"](ref_num+6) = prev_suc_keyhole_.c3(0);
                const_args_["p"](ref_num+7) = prev_suc_keyhole_.c3(1);
                const_args_["p"](ref_num+8) = prev_suc_keyhole_.d3;

                const_args_["p"](ref_num+9) = prev_suc_keyhole_.c4(0);
                const_args_["p"](ref_num+10) = prev_suc_keyhole_.c4(1);
                const_args_["p"](ref_num+11) = prev_suc_keyhole_.d4;

                const_args_["p"](ref_num+12) = prev_suc_keyhole_.c5(0);
                const_args_["p"](ref_num+13) = prev_suc_keyhole_.c5(1);
                const_args_["p"](ref_num+14) = prev_suc_keyhole_.d5;

                const_args_["p"](ref_num+15) = xc(0);
                const_args_["p"](ref_num+16) = xc(1);
                const_args_["p"](ref_num+17) = r;

                for(size_t i = 0; i < 16; i++)
                {
                    const_args_["p"](ref_num+18+i) = prev_suc_keyhole_.a(i);
                }

                mpc_ready = true;
            }
            else
            {
                optimal_found = false;
                mpc_ready = false;
            }
        }
    }
    else
    {
        int ref_num = nx + N_ * (nx + nu);
        for(size_t i = 0; i < 33; i++)
        {
            const_args_["p"](ref_num+i) = 0;
        }
        const_args_["p"](ref_num+33) = 1.;

        mpc_ready = true;
    }

    // ROS_INFO_STREAM("ref: " << const_args_["p"]);

    DM u_opt_seq, raw_u_opt;
    if(mpc_ready)
    {
        DM X0_reshape = reshape(X0, nx*(N_+1), 1);
        DM u0_reshape = reshape(u0, nu*N_, 1);
        const_args_["x0"] = vertcat(X0_reshape, u0_reshape);

        const_args_["p"] = const_args_["p"].T();
        const_args_["lbg"] = const_args_["lbg"].T();
        const_args_["ubg"] = const_args_["ubg"].T();

        // ROS_INFO_STREAM("x0: " << const_args_["x0"]);

        DMDict sol = opti_func_(DMDict{{"x0", const_args_["x0"]}, {"lbx", const_args_["lbx"]}, {"ubx", const_args_["ubx"]}, {"lbg", const_args_["lbg"]},{"ubg", const_args_["ubg"]}, {"p", const_args_["p"]}});

        DM full_sol = sol["x"];
        u_opt_seq = reshape(full_sol(Slice(nx*(N_+1), full_sol.size1())), nu, N_);
        x_seq = reshape(full_sol(Slice(0, nx*(N_+1))), nx, N_+1);
        ROS_INFO_STREAM_COND(print_debug_info_, "x_seq: " << x_seq);
        ROS_INFO_STREAM_COND(print_debug_info_, "u_seq: " << u_opt_seq);
        ROS_INFO_STREAM_COND(print_debug_info_, "Obj: " << sol["f"]);
        
        raw_u_opt = u_opt_seq(Slice(0, nu));
        optimal_found = opti_func_.stats()["success"];
    }

    // if((!mpc_ready && optimal_found) || (use_keyhole_ && !keyhole_found && optimal_found))
    if((!mpc_ready && optimal_found))
    {
        ROS_ERROR_STREAM("FATAL: should not happen.");
        throw;
    }

    if(optimal_found)
    {
        u_opt = raw_u_opt;
    }
    else
    {
        if(use_keyhole_ && keyhole_found)
            ROS_WARN_STREAM("No optimal found from NMPC.");
        else if(use_keyhole_ && !keyhole_found)
            ROS_WARN_STREAM("No optimal found for keyhole.");
        else
            ROS_WARN_STREAM("Keyhole is disabled. No optimal found from NMPC.");

        if(!fail_stop_)
        {
            u_opt = prev_u_(Slice(0, nu));
            ROS_WARN_STREAM("From prev u: " << u_opt);
        }
        else
        {
            u_opt = DM::zeros(1, nu);
            ROS_WARN_STREAM("Stop robot.");
        }
        mpc_fail_num_++;
    }

    // CBF
    // if(use_keyhole_ && use_cbf_)
    // {
    //     Eigen::Vector3d rbt_p(robot_pose.x(), robot_pose.y(), robot_pose.theta());
    //     // Eigen::Vector2d cbf_f(0, 0);
    //     // Eigen::MatrixXd cbf_g(2, 2);
    //     // cbf_g << cos(robot_pose.theta()), 0, sin(robot_pose.theta()), 0;
    //     Eigen::Vector2d u_vec(u_opt.get_elements()[0], u_opt.get_elements()[1]);

    //     double th_max = M_PI / 2;
    //     Eigen::Vector2d u_cbf = keyhole_->cbf_qp(rbt_p, u_vec, cbf_gamma_, cbf_kw_, th_max);

    //     u_cbf(0) = u_cbf(0) < dyn_params_.v_min ? dyn_params_.v_min : u_cbf(0);
    //     u_cbf(0) = u_cbf(0) > dyn_params_.v_max ? dyn_params_.v_max : u_cbf(0);
    //     u_cbf(1) = u_cbf(1) < dyn_params_.w_min ? dyn_params_.w_min : u_cbf(1);
    //     u_cbf(1) = u_cbf(1) > dyn_params_.w_max ? dyn_params_.w_max : u_cbf(1);
    //     u_opt = DM(std::vector<double>{u_cbf(0), u_cbf(1)});
    // }

    // Projection Operator
    bool compare_inf_dist = true;
    if(use_po_ && (!optimal_found || (compare_inf_dist && gap.getMinDist() < robot_geo.getRobotMaxRadius())))
    {
        u_opt = projectionOperator(ego_min, u_opt);
        ROS_INFO_STREAM("Control after projection operator: " << u_opt);
    }

    if(mpc_ready && optimal_found)
    {
        DM u_opt_back = reshape(u_opt_seq(Slice(nu, nu*N_)), N_-1, nu);
        DM u_add = DM::zeros(1, nu);
        prev_u_ = vertcat(u_opt_back, u_add);
        _ocp_successful = true;
    }
    else
    {
        DM u_opt_back = reshape(prev_u_(Slice(nu, nu*N_)), N_-1, nu);
        DM u_add = DM::zeros(1, nu);
        prev_u_ = vertcat(u_opt_back, u_add);
        _ocp_successful = false;
    }

    prev_opt_u_ = u_opt;

    // ROS_INFO_STREAM_COND(print_debug_info_, "u: " << u_opt);
    ROS_INFO_STREAM("u: " << u_opt);

    // publish results if desired
    // if (_publish_ocp_results) publishOptimalControlResult();  // TODO(roesmann): we could also pass time t from above
    // ROS_INFO_STREAM_COND(_print_cpu_time, "Cpu time: " << _statistics.step_time.toSec() * 1000.0 << " ms.");
    ++_ocp_seq;
    // _last_goal = goal_pose;
    return _ocp_successful;
}

DM PGHCController::projectionOperator(const std::pair<double, double>& ego_min, DM& u)
{
    double u_add_x = 0;
    double u_add_y = 0;

    double vx = u.get_elements()[0];
    double vy = 0;
    double w = u.get_elements()[1];

    double min_dist_ang = ego_min.second;
    double min_dist = ego_min.first;

    double min_diff_x = 0;
    double min_diff_y = 0;

    // ROS_INFO_STREAM(init_pose.pose);
    
    Eigen::Vector3d comp;
    double prod_mul;
    Eigen::Vector2d si_der;
    Eigen::Vector2d v_err(vx, vy);

    {
        double r_max = double(r_norm_ + r_norm_offset_);
        min_dist = min_dist >= r_max ? r_max : min_dist;
        if (min_dist <= 0) ROS_INFO_STREAM("Min dist <= 0, : " << min_dist);
        min_dist = min_dist <= 0 ? 0.01 : min_dist;
        // min_dist -= cfg_->rbt.r_inscr / 2;

        double min_x = min_dist * cos(min_dist_ang);
        double min_y = min_dist * sin(min_dist_ang);

        // ROS_INFO_STREAM("min_dist: " << min_dist);

        {
            // ROS_INFO_STREAM("Point Ver");
            min_diff_x = - min_x;
            min_diff_y = - min_y;
            comp = projection_method(min_diff_x, min_diff_y);
            si_der = Eigen::Vector2d(comp(0), comp(1));
            si_der(1) /= 3;
            prod_mul = v_err.dot(si_der);
        }

        if(comp(2) >= 0 && prod_mul <= 0)
        {
            u_add_x = comp(2) * prod_mul * - comp(0);
            u_add_y = comp(2) * prod_mul * - comp(1);
            // ROS_INFO_STREAM("comp y" << comp(1));
        }

        ROS_INFO_STREAM("u add x: " << u_add_x << ", y: " << u_add_y << ", min_dist: " << min_dist << ", min_dist_ang: " << min_dist_ang <<
                         ", comp: [" << comp(0) << "," << comp(1) << "," << comp(2) << "], prod_mul: " << prod_mul);
    }

    // Make sure no ejection
    // u_add_x = std::min(u_add_x, double(0));

    // ROS_INFO_STREAM("Mid vlinx: " << v_lin_x_fb);

    {
        // ROS_INFO_STREAM("Non holonomic");
        w = w + k_po_turn_ * u_add_y;
        vx = vx + k_po_ * u_add_x;

        if (min_dist_ang > - M_PI / 4 && min_dist_ang < M_PI / 4 && min_dist < r_inscr_)
        {
            // ROS_INFO_STREAM("No x linear");
            vx = 0;
            w *= 2;
        }

        if(vx < 0)
            vx = 0;
    }

    vx = vx < dyn_params_.v_min ? dyn_params_.v_min : vx;
    vx = vx > dyn_params_.v_max ? dyn_params_.v_max : vx;
    w = w < dyn_params_.w_min ? dyn_params_.w_min : w;
    w = w > dyn_params_.w_max ? dyn_params_.w_max : w;
    
    return DM(std::vector<double>{vx, w});
}

Eigen::Vector3d PGHCController::projection_method(double min_diff_x, double min_diff_y)
{
    double r_min = r_min_;
    double r_norm = r_norm_;

    double min_dist = sqrt(pow(min_diff_x, 2) + pow(min_diff_y, 2));
    double si = (r_min / min_dist - r_min / r_norm) / (1. - r_min / r_norm);
    double base_const = sqrt(pow(min_dist, 3)) * (r_min - r_norm);
    double up_const = r_min * r_norm;
    double si_der_x = up_const * - min_diff_x / base_const;
    double si_der_y = up_const * - min_diff_y / base_const;

    double norm_si_der = sqrt(pow(si_der_x, 2) + pow(si_der_y, 2));
    double norm_si_der_x = si_der_x / norm_si_der;
    double norm_si_der_y = si_der_y / norm_si_der;
    return Eigen::Vector3d(norm_si_der_x, norm_si_der_y, si);
}

bool PGHCController::stepPCont(const potential_gap::StaticInfGap& gap, const std::pair<double, double>& ego_min, const pips_trajectory_msgs::trajectory_points& initial_plan, const PoseSE2& robot_pose, const geometry_msgs::Twist& vel, ros::Duration t_diff,
                               bool has_feedforward, Eigen::Vector2f& u_opt)
{
    if (initial_plan.points.size() < 2)
    {
        ROS_ERROR("Controller::step(): initial plan must contain at least two poses.");
        return false;
    }
    
    pips_trajectory_msgs::trajectory_point start_pose(initial_plan.points.front());
    pips_trajectory_msgs::trajectory_point goal_pose(initial_plan.points.back());
    
    pips_trajectory_msgs::trajectory_point interp_pose;

    if(has_feedforward)
    {
        double t = t_diff.toSec();
        interp_pose = interpPose(initial_plan, t);
    }
    else
    {
        interp_pose = findNearestPose(initial_plan, robot_pose);
    }

    geometry_msgs::Quaternion curr_orient = euler2Quat(robot_pose.theta());
    Eigen::Matrix2cd g_curr = getComplexMatrix(robot_pose.x(), robot_pose.y(), curr_orient.w, curr_orient.z);
    geometry_msgs::Quaternion des_orient = euler2Quat(interp_pose.theta);
    Eigen::Matrix2cd g_des = getComplexMatrix(interp_pose.x, interp_pose.y, des_orient.w, des_orient.z);
    Eigen::Matrix2cd g_error = g_curr.inverse() * g_des;

    double theta_error = std::arg(g_error(0,0));
    double x_error = g_error.real()(0,1);
    double y_error = g_error.imag()(0,1);

    double v_lin_fb, v_lin_fb_y, v_ang_fb;
    
    {
      v_ang_fb = theta_error * k_turn_ + y_error*k_drive_y_;
      v_lin_fb = x_error * k_drive_x_;
    }

    double v_ang_ff = interp_pose.w;
    double v_lin_ff = interp_pose.v;

    if(!has_feedforward)
    {
        v_ang_ff = 0;
        v_lin_ff = 0;
    }

    double v_ang = v_ang_fb + v_ang_ff;
    double v_lin = v_lin_fb + v_lin_ff;

    ROS_INFO_STREAM_COND(print_debug_info_, "Raw u: [" << v_lin << ", " << v_ang << "].");

    if(use_keyhole_ && use_cbf_)
    {
        Eigen::Vector2d xc = gap.getOrigMapPoseVec();
        double r = gap.getMinDist();
        Eigen::Vector2d l_int = gap.getLIntMapVec();
        Eigen::Vector2d r_int = gap.getRIntMapVec();
        Eigen::Vector2d lgap = gap.getLMapVec();
        Eigen::Vector2d rgap = gap.getRMapVec();
        
        ROS_INFO_STREAM_COND(print_debug_info_, "Gap input to keyhole: \n" << xc << "\n" << r << "\n" << l_int << "\n" << r_int << "\n" << lgap << "\n" << rgap);

        ros::WallTime start = ros::WallTime::now();
        
        keyhole_->set_keyhole(xc, r, l_int, r_int, lgap, rgap);
        bool rbf_on = false;
        bool keyhole_found = keyhole_->synthesize(rbf_on);

        ros::WallDuration keyhole_elapsed = ros::WallTime::now() - start;
        float keyhole_time_elapsed = float(keyhole_elapsed.toNSec())/1000000;
        ROS_INFO_STREAM_COND(print_timing_, "Keyhole time: " << keyhole_time_elapsed << " ms.");

        // CBF
        Eigen::Vector3d rbt_p(robot_pose.x(), robot_pose.y(), robot_pose.theta());
        // Eigen::Vector2d cbf_f(0, 0);
        // Eigen::MatrixXd cbf_g(2, 2);
        // cbf_g << cos(robot_pose.theta()), 0, sin(robot_pose.theta()), 0;
        Eigen::Vector2d u_vec(v_lin, v_ang);

        double cbf_gamma = 1, k_w = 1, th_max = M_PI / 2;;
        Eigen::Vector2d u_cbf = keyhole_->cbf_qp(rbt_p, u_vec, cbf_gamma, k_w, th_max);
        v_lin = u_cbf(0);
        v_ang = u_cbf(1);
    }

    // Projection Operator
    if(use_po_)
    {
        DM u = DM(std::vector<double>{v_lin, v_ang});
        u = projectionOperator(ego_min, u);
        ROS_INFO_STREAM("Control after projection operator: " << u);
        v_lin = u.get_elements()[0];
        v_ang = u.get_elements()[1];
    }

    if(v_lin > dyn_params_.v_max) v_lin = dyn_params_.v_max;
    if(v_lin < dyn_params_.v_min) v_lin = dyn_params_.v_min;
    if(v_ang > dyn_params_.w_max) v_ang = dyn_params_.w_max;
    if(v_ang < dyn_params_.w_min) v_ang = dyn_params_.w_min;

    ROS_INFO_STREAM_COND(print_debug_info_, "u: [" << v_lin << ", " << v_ang << "].");
    u_opt = Eigen::Vector2f(v_lin, v_ang);

    _ocp_successful = true;
    // publish results if desired
    // if (_publish_ocp_results) publishOptimalControlResult();  // TODO(roesmann): we could also pass time t from above
    // ROS_INFO_STREAM_COND(_print_cpu_time, "Cpu time: " << _statistics.step_time.toSec() * 1000.0 << " ms.");
    ++_ocp_seq;
    // _last_goal = goal_pose;
    return _ocp_successful;
}

pips_trajectory_msgs::trajectory_point PGHCController::interpPose(const pips_trajectory_msgs::trajectory_points& traj, double t)
{
    pips_trajectory_msgs::trajectory_point interp_pose;
    ros::Duration last_time = traj.points.back().time;
    for(size_t i = 0; i < traj.points.size() - 1; i++)
    {
        ros::Duration curr_time = traj.points[i].time;
        ros::Duration next_time = traj.points[i+1].time;
        if(ros::Duration(t) >= curr_time && ros::Duration(t) <= next_time)
        {
            pips_trajectory_msgs::trajectory_point curr_pose = traj.points[i];
            pips_trajectory_msgs::trajectory_point next_pose = traj.points[i+1];
            double scale = (t - curr_pose.time.toSec()) / (next_pose.time.toSec() - curr_pose.time.toSec());
            double interp_x = curr_pose.x + scale * (next_pose.x - curr_pose.x);
            double interp_y = curr_pose.y + scale * (next_pose.y - curr_pose.y);

            // double curr_yaw = quat2Euler(curr_pose.pose.orientation);
            // double next_yaw = quat2Euler(next_pose.pose.orientation);
            double curr_yaw = curr_pose.theta;
            double next_yaw = next_pose.theta;
            double interp_yaw = curr_yaw + scale * (next_yaw - curr_yaw);

            double interp_v = curr_pose.v + scale * (next_pose.v - curr_pose.v);
            double interp_w = curr_pose.w + scale * (next_pose.w - curr_pose.w);

            // geometry_msgs::Quaternion interp_orient = euler2Quat(interp_yaw);

            // ROS_INFO_STREAM(curr_pose << " " << next_pose);
            // ROS_INFO_STREAM(curr_yaw << " " << interp_yaw << " " << next_yaw);

            // interp_pose.header.stamp = ros::Time(t);
            // interp_pose.header.frame_id = curr_pose.header.frame_id;
            // interp_pose.pose.position.x = interp_x;
            // interp_pose.pose.position.y = interp_y;
            // interp_pose.pose.position.z = 0;
            // interp_pose.pose.orientation = interp_orient;

            interp_pose.time = ros::Duration(t);
            interp_pose.x = interp_x;
            interp_pose.y = interp_y;
            interp_pose.theta = interp_yaw;
            interp_pose.v = interp_v;
            interp_pose.w = interp_w;

            break;
        }
        else if(ros::Duration(t) > last_time)
        {
            interp_pose = traj.points.back();

            break;
        }
    }

    return interp_pose;
}

pips_trajectory_msgs::trajectory_point PGHCController::findNearestPose(const pips_trajectory_msgs::trajectory_points& traj, const PoseSE2& robot_pose)
{
    std::vector<double> pose_diff(traj.points.size());
    for (int i = 0; i < pose_diff.size(); i++) // i will always be positive, so this is fine
    {
        geometry_msgs::Quaternion curr_orient = euler2Quat(robot_pose.theta());
        geometry_msgs::Quaternion pose_orient = euler2Quat(traj.points[i].theta);
        pose_diff[i] = sqrt(pow(robot_pose.x() - traj.points[i].x, 2) + 
                            pow(robot_pose.y() - traj.points[i].y, 2)) + 
                            0.5 * (1 - (   curr_orient.x * pose_orient.x + 
                                    curr_orient.y * pose_orient.y +
                                    curr_orient.z * pose_orient.z +
                                    curr_orient.w * pose_orient.w)
                            );
    }

    auto min_element_iter = std::min_element(pose_diff.begin(), pose_diff.end());
    int target_pose = std::distance(pose_diff.begin(), min_element_iter) + ctrl_ahead_pose_;
    target_pose = std::min(target_pose, int(traj.points.size() - 1));
    return traj.points[target_pose];
}

Eigen::Matrix2cd PGHCController::getComplexMatrix(double x, double y, double quat_w, double quat_z)
{
    std::complex<double> phase(quat_w, quat_z);
    phase = phase*phase;

    Eigen::Matrix2cd g(2,2);
    //g.real()(0,0) = phase.real();
    g.real()(0,1) = x;
    g.real()(1,0) = 0;
    g.real()(1,1) = 1;

    //g.imag()(0,0) = phase.imag();
    g.imag()(0,1) = y;
    g.imag()(1,0) = 0;
    g.imag()(1,1) = 0;

    g(0,0) = phase;

    return g;
}

double PGHCController::quat2Euler(geometry_msgs::Quaternion quat)
{
    Eigen::Quaterniond q(quat.w, quat.x, quat.y, quat.z);
    auto euler = q.toRotationMatrix().eulerAngles(0, 1, 2);
    return euler[2];
}

geometry_msgs::Quaternion PGHCController::euler2Quat(double euler)
{
    double roll = 0, pitch = 0, yaw = euler;    
    Eigen::Quaterniond q;
    q = Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX())
        * Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY())
        * Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ());
    
    geometry_msgs::Quaternion quat;
    quat.x = q.x();
    quat.y = q.y();
    quat.z = q.z();
    quat.w = q.w();

    return quat;
}

void PGHCController::stateFeedbackCallback(const mpc_local_planner_msgs::StateFeedback::ConstPtr& msg)
{
    // if ((int)msg->state.size() != _dynamics->getStateDimension())
    // {
    //     ROS_ERROR_STREAM("stateFeedbackCallback(): state feedback dimension does not match robot state dimension: "
    //                      << msg->state.size() << " != " << _dynamics->getStateDimension());
    //     return;
    // }

    std::lock_guard<std::mutex> lock(_x_feedback_mutex);
    _recent_x_time     = msg->header.stamp;
    _recent_x_feedback = Eigen::Map<const Eigen::VectorXd>(msg->state.data(), (int)msg->state.size());
}

// void PGHCController::publishOptimalControlResult()
// {
//     if (!_dynamics) return;
//     mpc_local_planner_msgs::OptimalControlResult msg;
//     msg.header.stamp           = ros::Time::now();
//     msg.header.seq             = static_cast<unsigned int>(_ocp_seq);
//     msg.dim_states             = _dynamics->getStateDimension();
//     msg.dim_controls           = _dynamics->getInputDimension();
//     msg.optimal_solution_found = _ocp_successful;
//     msg.cpu_time               = _statistics.step_time.toSec();

//     if (_x_ts && !_x_ts->isEmpty())
//     {
//         msg.time_states = _x_ts->getTime();
//         msg.states      = _x_ts->getValues();
//     }

//     if (_u_ts && !_u_ts->isEmpty())
//     {
//         msg.time_controls = _u_ts->getTime();
//         msg.controls      = _u_ts->getValues();
//     }

//     _ocp_result_pub.publish(msg);
// }

void PGHCController::reset() 
{ 
    PredictiveController::reset();
    mpc_fail_num_ = 0;
}

void PGHCController::configureParams(const ros::NodeHandle& nh)
{
    // Dynamics

    nx = 3;
    nu = 2;

    double v_min = 0, v_max = 0.5, w_min = -3, w_max = 3, v_a_max = 0.5, w_a_max = 0.5;
    nh.param("robot/v_min", v_min, v_min);
    nh.param("robot/v_max", v_max, v_max);
    nh.param("robot/w_min", w_min, w_min);
    nh.param("robot/w_max", w_max, w_max);
    nh.param("robot/v_a_max", v_a_max, v_a_max);
    nh.param("robot/w_a_max", w_a_max, w_a_max);

    dyn_params_.v_min = v_min;
    dyn_params_.v_max = v_max;
    dyn_params_.w_min = w_min;
    dyn_params_.w_max = w_max;
    dyn_params_.v_a_max = v_a_max;
    dyn_params_.w_a_max = w_a_max;

    // NMPC

    N_ = 8;
    // time_int_ = 0.5;
    nh.param("solver/N", N_, N_);
    // nh.param("solver/T", time_int_, time_int_);

    nmpc_params_.N = N_;
    nmpc_params_.T = time_int_;

    double Q1 = 1, Q2 = 1, Q3 = 0., R1 = 0.5, R2 = 0.05, terminal_weight = 1;
    nh.param("solver/Q1", Q1, Q1);
    nh.param("solver/Q2", Q2, Q2);
    nh.param("solver/Q3", Q3, Q3);
    nh.param("solver/R1", R1, R1);
    nh.param("solver/R2", R2, R2);
    nh.param("solver/terminal_weight", terminal_weight, terminal_weight);

    nmpc_params_.Q1 = Q1;
    nmpc_params_.Q2 = Q2;
    nmpc_params_.Q3 = Q3;
    nmpc_params_.R1 = R1;
    nmpc_params_.R2 = R2;
    nmpc_params_.terminal_weight = terminal_weight;

    u_lin_ref_ = 0.3;
    nh.param("solver/u_lin_ref", u_lin_ref_, u_lin_ref_);

    nmpc_params_.u_lin_ref = u_lin_ref_;

    cbf_gamma_ = 1, cbf_kw_ = 1;
    nh.param("cbf/cbf_gamma", cbf_gamma_, cbf_gamma_);
    nh.param("cbf/cbf_kw", cbf_kw_, cbf_kw_);
}

void PGHCController::configureRobotDynamics(const ros::NodeHandle& nh)
{
    _robot_type = "unicycle";
    nh.param("robot/type", _robot_type, _robot_type);

    x_ = SX::sym("x", nx);
    u_ = SX::sym("u", nu);

    if (_robot_type == "unicycle")
    {
        robot_dynamics_ = vertcat(u_(0) * cos(x_(2)), u_(0) * sin(x_(2)), u_(1));
        state_func_ = Function("state_func", {x_, u_}, {robot_dynamics_}, {"x", "u"}, {"robot_dynamics"});
    }
    else if (_robot_type == "simple_car")
    {
        double wheelbase = 0.5;
        nh.param("robot/simple_car/wheelbase", wheelbase, wheelbase);
        bool front_wheel_driving = false;
        nh.param("robot/simple_car/front_wheel_driving", front_wheel_driving, front_wheel_driving);
        // if (front_wheel_driving)
        //     return std::make_shared<SimpleCarFrontWheelDrivingModel>(wheelbase);
        // else
        //     return std::make_shared<SimpleCarModel>(wheelbase);
    }
    else if (_robot_type == "kinematic_bicycle_vel_input")
    {
        double length_rear = 1.0;
        nh.param("robot/kinematic_bicycle_vel_input/length_rear", length_rear, length_rear);
        double length_front = 1.0;
        nh.param("robot/kinematic_bicycle_vel_input/length_front", length_front, length_front);
        // return std::make_shared<KinematicBicycleModelVelocityInput>(length_rear, length_front);
    }
    else
    {
        ROS_ERROR_STREAM("Unknown robot type '" << _robot_type << "' specified.");
    }
}

void PGHCController::configureNMPC(const ros::NodeHandle& nh)
{
    std::string solver_type = "ipopt";
    nh.param("solver/type", solver_type, solver_type);
    std::string ipopt_linear_solver = "ma27";
    nh.param("solver/linear_solver", ipopt_linear_solver, ipopt_linear_solver);

    X_ = SX::sym("X", nx, N_+1);
    U_ = SX::sym("U", nu, N_);
    SX P_ = SX::sym("P", nx + N_ * (nx + nu) + 3*5 + 3 + 16);

    SX obj = 0;

    SX g = X_(Slice(0, nx)) - P_(Slice(0, nx));

    for(int k = 0; k < N_; k++)
    {
        SX st = X_(Slice(k*nx, (k+1)*nx));
        SX con = U_(Slice(k*nu, (k+1)*nu));

        SX st_next = X_(Slice((k+1)*nx, (k+2)*nx));

        if(ni_enabled_)
        {
            if(k != N_ - 1)
                obj = obj + nmpc_params_.Q1 * pow(st_next(0) - P_(5*k+3), 2) + nmpc_params_.Q2 * pow(st_next(1) - P_(5*k+4), 2) + nmpc_params_.Q3 * pow(st_next(2) - P_(5*k+5), 2);
            else
                obj = obj + nmpc_params_.terminal_weight * (pow(st_next(0) - P_(5*k+3), 2) + pow(st_next(1) - P_(5*k+4), 2));

            obj = obj + nmpc_params_.R1 * pow(con(0) - P_(5*k+6), 2) + nmpc_params_.R2 * pow(con(1) - P_(5*k+7), 2);
        }
        else
            obj = obj + nmpc_params_.Q1 * pow(st(0) - P_(5*k+3), 2) + nmpc_params_.Q2 * pow(st(1) - P_(5*k+4), 2) + nmpc_params_.Q3 * pow(st(2) - P_(5*k+5), 2);

        // obj = obj + nmpc_params_.Q1 * pow(st(0) - P_(5*k+3), 2) + nmpc_params_.Q2 * pow(st(1) - P_(5*k+4), 2) + nmpc_params_.Q3 * pow(st(2) - P_(5*k+5), 2)
        //       + nmpc_params_.R1 * pow(con(0) - P_(5*k+6), 2);

        SXDict F_next_d = state_func_(SXDict{{"x", st}, {"u", con}});

        SX st_next_euler = st + time_int_ * F_next_d["robot_dynamics"];

        g = vertcat(g, st_next - st_next_euler);
    }
    
    bool rbf_on = false;
    int ref_num = nx + N_ * (nx + nu);
    for(int k = 1; k < N_+1; k++)
    {
        SX st = X_(Slice(k*nx, (k+1)*nx));

        SX val1 = P_(ref_num) * st(0) + P_(ref_num+1) * st(1) + P_(ref_num+2);
        SX relu1 = if_else(val1 < 0, 0, val1);
        SX val2 = P_(ref_num+3) * st(0) + P_(ref_num+4) * st(1) + P_(ref_num+5);
        SX relu2 = if_else(val2 < 0, 0, val2);
        SX val3 = P_(ref_num+6) * st(0) + P_(ref_num+7) * st(1) + P_(ref_num+8);
        SX relu3 = if_else(val3 < 0, 0, val3);
        SX val4 = P_(ref_num+9) * st(0) + P_(ref_num+10) * st(1) + P_(ref_num+11);
        SX relu4 = if_else(val4 < 0, 0, val4);
        SX val5 = P_(ref_num+12) * st(0) + P_(ref_num+13) * st(1) + P_(ref_num+14);
        SX relu5 = if_else(val5 < 0, 0, val5);

        SX circ_dist = pow(P_(ref_num+15) - st(0), 2) + pow(P_(ref_num+16) - st(1), 2);
        SX circ;
        if(rbf_on)
        {
            circ = exp(-1.0 / P_(ref_num+17) * circ_dist);
        }
        else
        {
            SX circ_val = -(circ_dist - pow(P_(ref_num+17), 2.0));
            circ = if_else(circ_val < 0., 0., circ_val);
        }

        SX keyhole_const = P_(ref_num+18) * relu1 + P_(ref_num+19) * relu2 + P_(ref_num+20) * relu3 + P_(ref_num+21) * relu1 * relu2
                           + P_(ref_num+22) * relu1 * relu2 * relu3 + P_(ref_num+23) * relu1 * relu4 * relu5 + P_(ref_num+24) * relu2 * relu4 * relu5
                           + P_(ref_num+25) * circ * relu1 * relu4 + P_(ref_num+26) * circ * relu2 * relu4 
                           + P_(ref_num+27) * circ * relu1 + P_(ref_num+28) * circ * relu2 + P_(ref_num+29) * circ * relu3
                           + P_(ref_num+30) * circ * relu1 * relu2 + P_(ref_num+31) * circ * relu1 * relu2 * relu3
                           + P_(ref_num+32) * circ + P_(ref_num+33);

        g = vertcat(g, keyhole_const);
    }

    // Acc constraints
    for(int k = 0; k < N_-1; k++)
    {
        SX c_con = U_(Slice(k*nu, (k+1)*nu));
        SX n_con = U_(Slice((k+1)*nu, (k+2)*nu));

        SX u_diff = (n_con - c_con) / time_int_;

        g = vertcat(g, u_diff);
    }

    SX X_reshape = reshape(X_, nx*(N_+1), 1);
    SX U_reshape = reshape(U_, nu*N_, 1);
    SX OPT_variables = vertcat(X_reshape, U_reshape);

    SXDict nlp_prob;
    nlp_prob["f"] = obj;
    nlp_prob["x"] = OPT_variables;
    nlp_prob["g"] = g;
    nlp_prob["p"] = P_;

    const_args_["lbg"] = DM::zeros(nx*(N_+1)+N_+nu*(N_-1), 1); // WARNING: may have problem
    const_args_["ubg"] = DM::zeros(nx*(N_+1)+N_+nu*(N_-1), 1);

    const_args_["lbx"] = DM::zeros(nx*(N_+1)+nu*N_, 1);
    const_args_["ubx"] = DM::zeros(nx*(N_+1)+nu*N_, 1);

    for(size_t i = 0; i < N_; i++)
    {
        const_args_["ubg"](nx*(N_+1)+i) = dinf;
    }

    for(size_t i = 0; i < N_-1; i++)
    {
        const_args_["lbg"](nx*(N_+1)+N_+i*nu) = dyn_params_.v_a_min;
        const_args_["lbg"](nx*(N_+1)+N_+i*nu+1) = dyn_params_.w_a_min;
        const_args_["ubg"](nx*(N_+1)+N_+i*nu) = dyn_params_.v_a_max;
        const_args_["ubg"](nx*(N_+1)+N_+i*nu+1) = dyn_params_.w_a_max;
    }

    for(size_t i = 0; i < N_+1; i++)
    {
        const_args_["lbx"]((i*nx)) = dyn_params_.x_min;
        const_args_["ubx"]((i*nx)) = dyn_params_.x_max;
        const_args_["lbx"]((i*nx+1)) = dyn_params_.y_min;
        const_args_["ubx"]((i*nx+1)) = dyn_params_.y_max;
        const_args_["lbx"]((i*nx+2)) = dyn_params_.theta_min;
        const_args_["ubx"]((i*nx+2)) = dyn_params_.theta_max;
    }

    for(size_t i = 0; i < N_; i++)
    {
        const_args_["lbx"]((nx*(N_+1)+i*nu)) = dyn_params_.v_min;
        const_args_["ubx"]((nx*(N_+1)+i*nu)) = dyn_params_.v_max;
        const_args_["lbx"]((nx*(N_+1)+i*nu+1)) = dyn_params_.w_min;
        const_args_["ubx"]((nx*(N_+1)+i*nu+1)) = dyn_params_.w_max;
    }

    const_args_["p"] = DM::zeros(nx+N_*(nx+nu)+34);

    prev_u_ = DM::zeros(N_, nu);
    prev_opt_u_ = DM::zeros(nu);

    if (solver_type == "ipopt")
    {
        Dict opts;
        opts["print_time"] = 0;
        opts["verbose"] = 0;
        opts["expand"] = 1;
        Dict ipopt_opts;
        ipopt_opts["max_iter"] = 2000;
        ipopt_opts["print_level"] = 0;
        ipopt_opts["print_timing_statistics"] = "no";
        ipopt_opts["linear_solver"] = ipopt_linear_solver;
        ipopt_opts["acceptable_tol"] = 1e-8;
        ipopt_opts["acceptable_obj_change_tol"] = 1e-6;
        ipopt_opts["max_cpu_time"] = 0.85 * time_int_;
        opts["ipopt"] = ipopt_opts;

        opti_func_ = nlpsol("solver", "ipopt", nlp_prob, opts);

        ROS_INFO_STREAM("Constructed NMPC with " << solver_type << " " << ipopt_linear_solver);
    }
    else
    {
        ROS_ERROR_STREAM("Unknown solver type '" << solver_type << "' specified.");
    }

    ROS_INFO_STREAM("Optimal solver created.");
}

// bool PGHCController::generateInitialStateTrajectory(const Eigen::VectorXd& x0, const Eigen::VectorXd& xf,
//                                                 const std::vector<geometry_msgs::PoseStamped>& initial_plan, bool backward)
// {
//     if (initial_plan.size() < 2 || !_dynamics) return false;

//     TimeSeriesSE2::Ptr ts = std::make_shared<TimeSeriesSE2>();

//     int n_init = (int)initial_plan.size();
//     int n_ref  = _grid->getInitialN();
//     if (n_ref < 2)
//     {
//         ROS_ERROR("PGHCController::generateInitialStateTrajectory(): grid not properly initialized");
//         return false;
//     }
//     ts->add(0.0, x0);

//     double dt_ref = _grid->getInitialDt();
//     double tf_ref = (double)(n_ref - 1) * dt_ref;

//     Eigen::VectorXd x(_dynamics->getStateDimension());

//     // we initialize by assuming equally distributed poses
//     double dt_init = tf_ref / double(n_init - 1);

//     double t = dt_init;
//     for (int i = 1; i < n_init - 1; ++i)
//     {
//         // get yaw from the orientation of the distance vector between pose_{i+1} and pose_{i}
//         double yaw;
//         if (_initial_plan_estimate_orientation)
//         {
//             double dx = initial_plan[i + 1].pose.position.x - initial_plan[i].pose.position.x;
//             double dy = initial_plan[i + 1].pose.position.y - initial_plan[i].pose.position.y;
//             yaw       = std::atan2(dy, dx);
//             if (backward) normalize_theta(yaw + M_PI);
//         }
//         else
//         {
//             yaw = tf2::getYaw(initial_plan[i].pose.orientation);
//         }
//         PoseSE2 intermediate_pose(initial_plan[i].pose.position.x, initial_plan[i].pose.position.y, yaw);
//         _dynamics->getSteadyStateFromPoseSE2(intermediate_pose, x);
//         ts->add(t, x);
//         t += dt_init;
//     }

//     ts->add(tf_ref, xf);

//     _x_seq_init.setTrajectory(ts, corbo::TimeSeries::Interpolation::Linear);
//     return true;
// }

// bool PGHCController::isPoseTrajectoryFeasible(base_local_planner::CostmapModel* costmap_model, const std::vector<geometry_msgs::Point>& footprint_spec,
//                                           double inscribed_radius, double circumscribed_radius, double min_resolution_collision_check_angular,
//                                           int look_ahead_idx)
// {
//     if (!_grid)
//     {
//         ROS_ERROR("Controller must be configured before invoking step().");
//         return false;
//     }
//     if (_grid->getN() < 2) return false;

//     // we currently require a full discretization grid as we want to have fast access to
//     // individual states without requiring any new simulation.
//     // Alternatively, other grids could be used in combination with method getStateAndControlTimeSeries()
//     const FullDiscretizationGridBaseSE2* fd_grid = dynamic_cast<const FullDiscretizationGridBaseSE2*>(_grid.get());
//     if (!fd_grid)
//     {
//         ROS_ERROR("isPoseTrajectoriyFeasible is currently only implemented for fd grids");
//         return true;
//     }

//     if (look_ahead_idx < 0 || look_ahead_idx >= _grid->getN()) look_ahead_idx = _grid->getN() - 1;

//     for (int i = 0; i <= look_ahead_idx; ++i)
//     {
//         if (costmap_model->footprintCost(fd_grid->getState(i)[0], fd_grid->getState(i)[1], fd_grid->getState(i)[2], footprint_spec, inscribed_radius,
//                                          circumscribed_radius) == -1)
//         {
//             return false;
//         }
//         // Checks if the distance between two poses is higher than the robot radius or the orientation diff is bigger than the specified threshold
//         // and interpolates in that case.
//         // (if obstacles are pushing two consecutive poses away, the center between two consecutive poses might coincide with the obstacle ;-)!
//         if (i < look_ahead_idx)
//         {
//             double delta_rot           = normalize_theta(fd_grid->getState(i + 1)[2] - fd_grid->getState(i)[2]);
//             Eigen::Vector2d delta_dist = fd_grid->getState(i + 1).head(2) - fd_grid->getState(i).head(2);
//             if (std::abs(delta_rot) > min_resolution_collision_check_angular || delta_dist.norm() > inscribed_radius)
//             {
//                 int n_additional_samples = std::max(std::ceil(std::abs(delta_rot) / min_resolution_collision_check_angular),
//                                                     std::ceil(delta_dist.norm() / inscribed_radius)) -
//                                            1;

//                 PoseSE2 intermediate_pose(fd_grid->getState(i)[0], fd_grid->getState(i)[1], fd_grid->getState(i)[2]);
//                 for (int step = 0; step < n_additional_samples; ++step)
//                 {
//                     intermediate_pose.position() = intermediate_pose.position() + delta_dist / (n_additional_samples + 1.0);
//                     intermediate_pose.theta()    = g2o::normalize_theta(intermediate_pose.theta() + delta_rot / (n_additional_samples + 1.0));
//                     if (costmap_model->footprintCost(intermediate_pose.x(), intermediate_pose.y(), intermediate_pose.theta(), footprint_spec,
//                                                      inscribed_radius, circumscribed_radius) == -1)
//                     {
//                         return false;
//                     }
//                 }
//             }
//         }
//     }
//     return true;
// }

// MX PGHCController::getState(const geometry_msgs::PoseStamped& input_pose)
// {
//     Eigen::Quaterniond q(input_pose.pose.orientation.x, input_pose.pose.orientation.y, input_pose.pose.orientation.z, input_pose.pose.orientation.w);
//     auto euler = q.toRotationMatrix().eulerAngles(0, 1, 2);
//     double yaw = euler[2];

//     double x = input_pose.pose.position.x;
//     double y = input_pose.pose.position.y;

//     return MX(std::vector<double>{x, y, yaw});
// }

}  // namespace mpc_local_planner
