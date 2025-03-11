#ifndef PG_MPC_UTILS_H_
#define PG_MPC_UTILS_H_

#include <geometry_msgs/PoseStamped.h>
#include <pips_trajectory_msgs/trajectory_point.h>
#include <pips_trajectory_msgs/trajectory_points.h>

#include <tf2/LinearMath/Quaternion.h>

namespace pg_mpc_local_planner {
    constexpr double dinf = std::numeric_limits<double>::infinity();
    
    struct printFlags
    {
        bool print_debug_info = false;
        bool print_timing = false;
    };

    struct dynamicsParams
    {
        double v_min = 0.;
        double v_max = 0.5;
        double w_min = -M_PI / 2;
        double w_max = M_PI / 2;
        double v_a_max = 0.5;
        double v_a_min = -v_a_max;
        double w_a_max = 0.5;
        double w_a_min = -w_a_max;

        // double x_min = -1000;
        // double x_max = 1000;
        // double y_min = -1000;
        // double y_max = 1000;
        // double theta_min = -1000;
        // double theta_max = 1000;
        double x_min = -dinf;
        double x_max = dinf;
        double y_min = -dinf;
        double y_max = dinf;
        double theta_min = -dinf;
        double theta_max = dinf;
    };

    struct nmpcParams
    {
        int N = 8;
        double T = 0.5;

        double Q1 = 1;
        double Q2 = 1;
        double Q3 = 0.5;
        double R1 = 0.5;
        double R2 = 0.05;
        double terminal_weight = 1;

        double u_lin_ref = 0.3;
    };
    
    inline geometry_msgs::PoseStamped trajPoint2PoseStamped(pips_trajectory_msgs::trajectory_point pt)
    {
        geometry_msgs::PoseStamped pose;
        pose.pose.position.x = pt.x;
        pose.pose.position.y = pt.y;

        tf2::Quaternion myQuaternion;
        myQuaternion.setRPY(0, 0, pt.theta);
        myQuaternion.normalize();
        pose.pose.orientation.x = myQuaternion.getX();
        pose.pose.orientation.y = myQuaternion.getY();
        pose.pose.orientation.z = myQuaternion.getZ();
        pose.pose.orientation.w = myQuaternion.getW();

        return pose;
    }

    inline double quat2Yaw(geometry_msgs::Quaternion quat)
    {
        Eigen::Quaterniond q(quat.w, quat.x, quat.y, quat.z);
        auto euler = q.toRotationMatrix().eulerAngles(0, 1, 2);
        return euler[2];
    }

    inline geometry_msgs::Quaternion euler2Quat(double euler)
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
}

#endif