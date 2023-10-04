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

#ifndef PG_MPC_HARD_CSTR_LOCAL_PLANNER_ROS_H_
#define PG_MPC_HARD_CSTR_LOCAL_PLANNER_ROS_H_

#include <ros/ros.h>

// base local planner base class and utilities
#include <base_local_planner/costmap_model.h>
#include <base_local_planner/goal_functions.h>
#include <base_local_planner/odometry_helper_ros.h>
#include <mbf_costmap_core/costmap_controller.h>
#include <nav_core/base_local_planner.h>

// mpc_local_planner related classes
#include <potential_gap_mpc/controller_hc.h>
#include <mpc_local_planner/utils/publisher.h>
#include <potential_gap_mpc/print_utils.h>

// teb_local_planner related classes
#include <teb_local_planner/obstacles.h>
#include <teb_local_planner/pose_se2.h>
#include <teb_local_planner/robot_footprint_model.h>

// message types
#include <costmap_converter/ObstacleMsg.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

// transforms
#include <tf2/utils.h>
#include <tf2_ros/buffer.h>

// costmap
#include <costmap_2d/costmap_2d_ros.h>
#include <costmap_converter/costmap_converter_interface.h>

// dynamic reconfigure
// #include <dynamic_reconfigure/server.h>
// #include <mpc_local_planner/MpcLocalPlannerReconfigureConfig.h>

#include <boost/shared_ptr.hpp>

#include <memory>
#include <mutex>

#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>

// Potential Gap Generation
#include <potential_gap/planner.h>
#include <dynamic_reconfigure/server.h>
#include <potential_gap/pgConfig.h>
#include <potential_gap/utils.h>

// #include <turtlebot_trajectory_testing/obstacle_avoidance_controller.h>
// #include <turtlebot_trajectory_testing/turtlebot_trajectory_tester.h>
// #include <pips_trajectory_testing/pips_trajectory_tester.h>
#include <turtlebot_trajectory_testing/ni_config_utility.h>
#include <turtlebot_trajectory_functions/path.h>
#include <trajectory_generator_ros_interface.h>
#include <pips_trajectory_msgs/trajectory_point.h>
#include <pips_trajectory_msgs/trajectory_points.h>

#include <tf2_pips/tf2_trajectory.h>

#include <cubic_spline_smoother/cubic_spline_interpolator.h>
#include <potential_gap_mpc/OptimalStats.h>

namespace pg_mpc_local_planner {

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

typedef TurtlebotGenAndTest::trajectory_ptr trajectory_ptr;
typedef TurtlebotGenAndTest::traj_func_type traj_func_type;
typedef TurtlebotGenAndTest::traj_func_ptr traj_func_ptr;
typedef TurtlebotGenAndTest::trajectory_points trajectory_points;
typedef TurtlebotGenAndTest::TrajBridge TrajBridge;
typedef std::shared_ptr<TurtlebotGenAndTest> GenAndTest_ptr;

/**
 * @class PGMpcHcLocalPlannerROS
 * @brief Implements both nav_core::BaseLocalPlanner and mbf_costmap_core::CostmapController abstract
 * interfaces, so the teb_local_planner plugin can be used both in move_base and move_base_flex (MBF).
 * @todo Escape behavior, more efficient obstacle handling
 */
class PGMpcHcLocalPlannerROS : public nav_core::BaseLocalPlanner, public mbf_costmap_core::CostmapController
{
   using PoseSE2                = teb_local_planner::PoseSE2;
   using RobotFootprintModelPtr = teb_local_planner::RobotFootprintModelPtr;
   using Point2dContainer       = teb_local_planner::Point2dContainer;
   using ObstContainer          = teb_local_planner::ObstContainer;
   using ViaPointContainer      = std::vector<PoseSE2>;

   using ObstaclePtr      = teb_local_planner::ObstaclePtr;
   using PointObstacle    = teb_local_planner::PointObstacle;
   using CircularObstacle = teb_local_planner::CircularObstacle;
   using LineObstacle     = teb_local_planner::LineObstacle;
   using PolygonObstacle  = teb_local_planner::PolygonObstacle;

public:
   /**
    * @brief Default constructor of the plugin
    */
   PGMpcHcLocalPlannerROS();
   /**
    * @brief  Destructor of the plugin
    */
   ~PGMpcHcLocalPlannerROS();

   /**
    * @brief Initializes the teb plugin
    * @param name The name of the instance
    * @param tf Pointer to a tf buffer
    * @param costmap_ros Cost map representing occupied and free space
    */
   //  void initialize(std::string name, tf2_ros::Buffer* tf, costmap_2d::Costmap2DROS* costmap_ros);
   void initialize(std::string name, tf2_ros::Buffer* tf, costmap_2d::Costmap2DROS* costmap_ros);

   /**
    * @brief Set the plan that the teb local planner is following
    * @param orig_global_plan The plan to pass to the local planner
    * @return True if the plan was updated successfully, false otherwise
    */
   bool setPlan(const std::vector<geometry_msgs::PoseStamped>& orig_global_plan);

   /**
    * @brief Given the current position, orientation, and velocity of the robot, compute velocity commands to send to the base
    * @param cmd_vel Will be filled with the velocity command to be passed to the robot base
    * @return True if a valid trajectory was found, false otherwise
    */
   bool computeVelocityCommands(geometry_msgs::Twist& cmd_vel);

   /**
    * @brief Given the current position, orientation, and velocity of the robot, compute velocity commands to send to the base.
    * @remark Extended version for MBF API
    * @param pose the current pose of the robot.
    * @param velocity the current velocity of the robot.
    * @param cmd_vel Will be filled with the velocity command to be passed to the robot base.
    * @param message Optional more detailed outcome as a string
    * @return Result code as described on ExePath action result:
    *         SUCCESS         = 0
    *         1..9 are reserved as plugin specific non-error results
    *         FAILURE         = 100   Unspecified failure, only used for old, non-mfb_core based plugins
    *         CANCELED        = 101
    *         NO_VALID_CMD    = 102
    *         PAT_EXCEEDED    = 103
    *         COLLISION       = 104
    *         OSCILLATION     = 105
    *         ROBOT_STUCK     = 106
    *         MISSED_GOAL     = 107
    *         MISSED_PATH     = 108
    *         BLOCKED_PATH    = 109
    *         INVALID_PATH    = 110
    *         TF_ERROR        = 111
    *         NOT_INITIALIZED = 112
    *         INVALID_PLUGIN  = 113
    *         INTERNAL_ERROR  = 114
    *         121..149 are reserved as plugin specific errors
    */
   uint32_t computeVelocityCommands(const geometry_msgs::PoseStamped& pose, const geometry_msgs::TwistStamped& velocity,
                                    geometry_msgs::TwistStamped& cmd_vel, std::string& message);

   /**
    * @brief  Check if the goal pose has been achieved
    *
    * The actual check is performed in computeVelocityCommands().
    * Only the status flag is checked here.
    * @return True if achieved, false otherwise
    */
   bool isGoalReached();

   /**
    * @brief Dummy version to satisfy MBF API
    */
   bool isGoalReached(double xy_tolerance, double yaw_tolerance) { return isGoalReached(); };

   void reset();

   /**
    * @brief Requests the planner to cancel, e.g. if it takes too much time
    * @remark New on MBF API
    * @return True if a cancel has been successfully requested, false if not implemented.
    */
   bool cancel() { return false; };

   /** @name Public utility functions/methods */
   //@{

   /**
    * @brief  Transform a tf::Pose type into a Eigen::Vector2d containing the translational and angular velocities.
    *
    * Translational velocities (x- and y-coordinates) are combined into a single translational velocity (first component).
    * @param tf_vel tf::Pose message containing a 1D or 2D translational velocity (x,y) and an angular velocity (yaw-angle)
    * @return Translational and angular velocity combined into an Eigen::Vector2d
    */
   static Eigen::Vector2d tfPoseToEigenVector2dTransRot(const tf::Pose& tf_vel);

   /**
    * @brief Get the current robot footprint/contour model
    * @param nh const reference to the local ros::NodeHandle
    * @param costmap_ros pointer to an intialized instance of costmap_2d::Costmap2dROS
    * @return Robot footprint model used for optimization
    */
   static RobotFootprintModelPtr getRobotFootprintFromParamServer(const ros::NodeHandle& nh, costmap_2d::Costmap2DROS* costmap_ros = nullptr);

   /**
    * @brief Get the current robot footprint/contour model
    * @param costmap_ros reference to an intialized instance of costmap_2d::Costmap2dROS
    * @return Robot footprint model used for optimization
    */
   static RobotFootprintModelPtr getRobotFootprintFromCostmap2d(costmap_2d::Costmap2DROS& costmap_ros);

   /**
    * @brief Set the footprint from the given XmlRpcValue.
    * @remarks This method is copied from costmap_2d/footprint.h, since it is not declared public in all ros distros
    * @remarks It is modified in order to return a container of Eigen::Vector2d instead of geometry_msgs::Point
    * @param footprint_xmlrpc should be an array of arrays, where the top-level array should have 3 or more elements, and the
    * sub-arrays should all have exactly 2 elements (x and y coordinates).
    * @param full_param_name this is the full name of the rosparam from which the footprint_xmlrpc value came.
    * It is used only for reporting errors.
    * @return container of vertices describing the polygon
    */
   static Point2dContainer makeFootprintFromXMLRPC(XmlRpc::XmlRpcValue& footprint_xmlrpc, const std::string& full_param_name);

   /**
    * @brief Get a number from the given XmlRpcValue.
    * @remarks This method is copied from costmap_2d/footprint.h, since it is not declared public in all ros distros
    * @remarks It is modified in order to return a container of Eigen::Vector2d instead of geometry_msgs::Point
    * @param value double value type
    * @param full_param_name this is the full name of the rosparam from which the footprint_xmlrpc value came.
    * It is used only for reporting errors.
    * @returns double value
    */
   static double getNumberFromXMLRPC(XmlRpc::XmlRpcValue& value, const std::string& full_param_name);

   //@}

protected:
   /**
    * @brief Update internal obstacle vector based on occupied costmap cells
    * @remarks All occupied cells will be added as point obstacles.
    * @remarks All previous obstacles are cleared.
    * @sa updateObstacleContainerWithCostmapConverter
    * @todo Include temporal coherence among obstacle msgs (id vector)
    * @todo Include properties for dynamic obstacles (e.g. using constant velocity model)
    */
   // void updateObstacleContainerWithCostmap();

   /**
    * @brief Update internal obstacle vector based on polygons provided by a costmap_converter plugin
    * @remarks Requires a loaded costmap_converter plugin.
    * @remarks All previous obstacles are cleared.
    * @sa updateObstacleContainerWithCostmap
    */
   // void updateObstacleContainerWithCostmapConverter();

   /**
    * @brief Update internal obstacle vector based on custom messages received via subscriber
    * @remarks All previous obstacles are NOT cleared. Call this method after other update methods.
    * @sa updateObstacleContainerWithCostmap, updateObstacleContainerWithCostmapConverter
    */
   // void updateObstacleContainerWithCustomObstacles();

   /**
    * @brief Update internal via-point container based on the current reference plan
    * @remarks All previous via-points will be cleared.
    * @param transformed_plan (local) portion of the global plan (which is already transformed to the planning frame)
    * @param min_separation minimum separation between two consecutive via-points
    */
   // void updateViaPointsContainer(const std::vector<geometry_msgs::PoseStamped>& transformed_plan, double min_separation);

   /**
    * @brief Callback for the dynamic_reconfigure node.
    *
    * This callback allows to modify parameters dynamically at runtime without restarting the node
    * @param config Reference to the dynamic reconfigure config
    * @param level Dynamic reconfigure level
    */
   // void reconfigureCB(MpcLocalPlannerReconfigureConfig& config, uint32_t level);

   /**
    * @brief Callback for custom obstacles that are not obtained from the costmap
    * @param obst_msg pointer to the message containing a list of polygon shaped obstacles
    */
   // void customObstacleCB(const costmap_converter::ObstacleArrayMsg::ConstPtr& obst_msg);

   /**
    * @brief Callback for custom via-points
    * @param via_points_msg pointer to the message containing a list of via-points
    */
   // void customViaPointsCB(const nav_msgs::Path::ConstPtr& via_points_msg);

   /**
    * @brief Prune global plan such that already passed poses are cut off
    *
    * The pose of the robot is transformed into the frame of the global plan by taking the most recent tf transform.
    * If no valid transformation can be found, the method returns \c false.
    * The global plan is pruned until the distance to the robot is at least \c dist_behind_robot.
    * If no pose within the specified treshold \c dist_behind_robot can be found,
    * nothing will be pruned and the method returns \c false.
    * @remarks Do not choose \c dist_behind_robot too small (not smaller the cellsize of the map), otherwise nothing will be pruned.
    * @param tf A reference to a tf buffer
    * @param global_pose The global pose of the robot
    * @param[in,out] global_plan The plan to be transformed
    * @param dist_behind_robot Distance behind the robot that should be kept [meters]
    * @return \c true if the plan is pruned, \c false in case of a transform exception or if no pose cannot be found inside the threshold
    */
   bool pruneGlobalPlan(const tf2_ros::Buffer& tf, const geometry_msgs::PoseStamped& global_pose,
                         std::vector<geometry_msgs::PoseStamped>& global_plan, double dist_behind_robot = 1);

   bool pruneTransformedPlan(const tf2_ros::Buffer& tf, const geometry_msgs::PoseStamped& robot_pose,
                        std::vector<geometry_msgs::PoseStamped>& transformed_plan, double dist_behind_robot = 1);

   bool cropTransformedPlan(const tf2_ros::Buffer& tf, const geometry_msgs::PoseStamped& robot_pose,
                        std::vector<geometry_msgs::PoseStamped>& transformed_plan, double dist_behind_robot = 1);

   /**
    * @brief  Transforms the global plan of the robot from the planner frame to the local frame (modified).
    *
    * The method replaces transformGlobalPlan as defined in base_local_planner/goal_functions.h
    * such that the index of the current goal pose is returned as well as
    * the transformation between the global plan and the planning frame.
    * @param tf A reference to a tf buffer
    * @param global_plan The plan to be transformed
    * @param global_pose The global pose of the robot
    * @param costmap A reference to the costmap being used so the window size for transforming can be computed
    * @param global_frame The frame to transform the plan to
    * @param max_plan_length Specify maximum length (cumulative Euclidean distances) of the transformed plan [if <=0: disabled; the length is also
    * bounded by the local costmap size!]
    * @param[out] transformed_plan Populated with the transformed plan
    * @param[out] current_goal_idx Index of the current (local) goal pose in the global plan
    * @param[out] tf_plan_to_global Transformation between the global plan and the global planning frame
    * @return \c true if the global plan is transformed, \c false otherwise
    */
   //  bool transformGlobalPlan(const tf2_ros::Buffer& tf, const std::vector<geometry_msgs::PoseStamped>& global_plan,
   //                           const geometry_msgs::PoseStamped& global_pose, const costmap_2d::Costmap2D& costmap, const std::string& global_frame,
   //                           double max_plan_length, std::vector<geometry_msgs::PoseStamped>& transformed_plan, int* current_goal_idx = NULL,
   //                           geometry_msgs::TransformStamped* tf_plan_to_global = NULL) const;
   bool transformGlobalPlan(const tf2_ros::Buffer& tf, const std::vector<geometry_msgs::PoseStamped>& global_plan,
                           const geometry_msgs::PoseStamped& global_pose, const costmap_2d::Costmap2D& costmap, const std::string& global_frame,
                           double max_plan_length, std::vector<geometry_msgs::PoseStamped>& transformed_plan, int* current_goal_idx = NULL,
                           geometry_msgs::TransformStamped* tf_plan_to_global = NULL) const;

   /**
    * @brief Estimate the orientation of a pose from the global_plan that is treated as a local goal for the local planner.
    *
    * If the current (local) goal point is not the final one (global)
    * substitute the goal orientation by the angle of the direction vector between
    * the local goal and the subsequent pose of the global plan.
    * This is often helpful, if the global planner does not consider orientations. \n
    * A moving average filter is utilized to smooth the orientation.
    * @param global_plan The global plan
    * @param local_goal Current local goal
    * @param current_goal_idx Index of the current (local) goal pose in the global plan
    * @param[out] tf_plan_to_global Transformation between the global plan and the global planning frame
    * @param moving_average_length number of future poses of the global plan to be taken into account
    * @return orientation (yaw-angle) estimate
    */

   pips_trajectory_msgs::trajectory_points parameterizeT(std::vector<geometry_msgs::PoseStamped>& raw_path, double start_v, double desired_vel, double desired_acc);

   // double estimateLocalGoalOrientation(const std::vector<geometry_msgs::PoseStamped>& global_plan, const geometry_msgs::PoseStamped& local_goal,
   //                                     int current_goal_idx, const geometry_msgs::TransformStamped& tf_plan_to_global,
   //                                     int moving_average_length = 3) const;

   /**
    * @brief Validate current parameter values of the footprint for optimization, obstacle distance and the costmap footprint
    *
    * This method prints warnings if validation fails.
    * @remarks Currently, we only validate the inscribed radius of the footprints
    * @param opt_inscribed_radius Inscribed radius of the RobotFootprintModel for optimization
    * @param costmap_inscribed_radius Inscribed radius of the footprint model used for the costmap
    * @param min_obst_dist desired distance to obstacles
    */
   // void validateFootprints(double opt_inscribed_radius, double costmap_inscribed_radius, double min_obst_dist);

   std::vector<geometry_msgs::PoseStamped> transformLocalPlan(geometry_msgs::PoseArray& orig_pose_array, std::string trans_to_frame, std_msgs::Header header);
   std::vector<geometry_msgs::PoseStamped> transformLocalPlan(std::vector<geometry_msgs::PoseStamped>& orig_pose_array, std::string trans_to_frame, std_msgs::Header header);  

   pips_trajectory_msgs::trajectory_points pathToTrajectory(const std::vector<geometry_msgs::PoseStamped>& local_path, geometry_msgs::PoseStamped robot_pose, geometry_msgs::Twist robot_vel);
   pips_trajectory_msgs::trajectory_points pathToTrajectory(const nav_msgs::Path& local_path, geometry_msgs::PoseStamped robot_pose, geometry_msgs::Twist robot_vel);

   ros::Duration findNearTime(const pips_trajectory_msgs::trajectory_points& traj, geometry_msgs::PoseStamped robot_pose, ros::Duration prev_tdiff);

   void pubKeyholeLevelset(const std::shared_ptr<keyhole::Keyhole>& keyhole, std::string frame_id);

   nav_msgs::Path xSeqToPathMsg(const DM& x_seq, std_msgs::Header header);

private:
   // Definition of member variables

   // external objects (store weak pointers)
   costmap_2d::Costmap2DROS* _costmap_ros;  //!< Pointer to the costmap ros wrapper, received from the navigation stack
   costmap_2d::Costmap2D* _costmap;         //!< Pointer to the 2d costmap (obtained from the costmap ros wrapper)
   tf2_ros::Buffer* _tf;                    //!< pointer to tf buffer

   // potential gap
   potential_gap::Planner potential_gap_;
   std::string name_;
   ros::NodeHandle pnh_;
   bool pf_local_frame_enable_;
   ros::Subscriber laser_sub_, feasi_laser_sub_, pose_sub_;

   bool print_debug_info_, print_timing_;

   bool enable_vis_ = true;
   ros::Publisher static_inf_gap_pub_, keyhole_levelset_pub_;
   bool tracking_crop_ = false;
   double v_des_, a_des_;
   ros::Time plan_start_time_;
   int plan_it_;
   ros::Duration prev_tdiff_;

   boost::shared_ptr<path_smoother::CubicSplineInterpolator> traj_csi_;
   pips_trajectory_msgs::trajectory_points timed_plan_;

   bool ni_enabled_;
   boost::shared_ptr<turtlebot_trajectory_testing::NIConfigUtility> ni_util_;
   GenAndTest_ptr traj_tester_;

   bool use_pose_controller_, has_feedforward_;

   ros::Publisher mpc_traj_pub_;
   ros::Publisher mpc_optimal_stats_pub_;

   bool prev_success_;
      
   boost::shared_ptr<dynamic_reconfigure::Server<potential_gap::pgConfig> > dynamic_recfg_server_;
   dynamic_reconfigure::Server<potential_gap::pgConfig>::CallbackType f_;

   // internal objects
   PGHCController _controller;
   ObstContainer _obstacles;  //!< Obstacle vector that should be considered during local trajectory optimization
   Publisher _publisher;
   std::shared_ptr<base_local_planner::CostmapModel> _costmap_model;

   corbo::TimeSeries::Ptr _x_seq = std::make_shared<corbo::TimeSeries>();
   corbo::TimeSeries::Ptr _u_seq = std::make_shared<corbo::TimeSeries>();

   std::vector<geometry_msgs::PoseStamped> _global_plan;  //!< Store the current global plan

   base_local_planner::OdometryHelperRos _odom_helper;  //!< Provides an interface to receive the current velocity from the robot

   pluginlib::ClassLoader<costmap_converter::BaseCostmapToPolygons> _costmap_converter_loader;  //!< Load costmap converter plugins at runtime
   boost::shared_ptr<costmap_converter::BaseCostmapToPolygons> _costmap_converter;              //!< Store the current costmap_converter

   // std::shared_ptr<dynamic_reconfigure::Server<MpcLocalPlannerReconfigureConfig>>
   //    dynamic_recfg_;                                        //!< Dynamic reconfigure server to allow config modifications at runtime
   ros::Subscriber _custom_obst_sub;                          //!< Subscriber for custom obstacles received via a ObstacleMsg.
   std::mutex _custom_obst_mutex;                             //!< Mutex that locks the obstacle array (multi-threaded)
   costmap_converter::ObstacleArrayMsg _custom_obstacle_msg;  //!< Copy of the most recent obstacle message

   ViaPointContainer _via_points;
   ros::Subscriber _via_points_sub;         //!< Subscriber for custom via-points received via a Path msg.
   bool _custom_via_points_active = false;  //!< Keep track whether valid via-points have been received from via_points_sub_
   std::mutex _via_point_mutex;             //!< Mutex that locks the via_points container (multi-threaded)

   PoseSE2 _robot_pose;                   //!< Store current robot pose
   PoseSE2 _robot_goal;                   //!< Store current robot goal
   geometry_msgs::Twist _robot_vel;       //!< Store current robot translational and angular velocity (vx, vy, omega)
   bool _goal_reached = false;            //!< store whether the goal is reached or not
   ros::Time _time_last_infeasible_plan;  //!< Store at which time stamp the last infeasible plan was detected
   int _no_infeasible_plans = 0;          //!< Store how many times in a row the planner failed to find a feasible plan.
   geometry_msgs::Twist _last_cmd;        //!< Store the last control command generated in computeVelocityCommands()
   ros::Time _time_last_cmd;

   float _total_time, _peak_time;
   uint64_t _loop_num;

   RobotFootprintModelPtr _robot_model;

   std::vector<geometry_msgs::Point> _footprint_spec;  //!< Store the footprint of the robot
   double _robot_inscribed_radius;                     //!< The radius of the inscribed circle of the robot (collision possible)
   double _robot_circumscribed_radius;                 //!< The radius of the circumscribed circle of the robot

   std::string _global_frame;      //!< The frame in which the controller will run
   std::string _robot_base_frame;  //!< Used as the base frame id of the robot

   // flags
   bool _initialized;  //!< Keeps track about the correct initialization of this class

   struct Parameters
   {
      double xy_goal_tolerance                      = 0.2;
      double yaw_goal_tolerance                     = 0.1;
      bool global_plan_overwrite_orientation        = true;
      double global_plan_prune_distance             = 1.0;
      double max_global_plan_lookahead_dist         = 1.5;
      bool is_footprint_dynamic                     = false;
      bool include_costmap_obstacles                = true;
      double costmap_obstacles_behind_robot_dist    = 1.5;
      double global_plan_viapoint_sep               = -1;
      double collision_check_min_resolution_angular = M_PI;
      int collision_check_no_poses                  = -1;
      std::string odom_topic                        = "odom";
      double controller_frequency                   = 10;

   } _params;

   struct CostmapConverterPlugin
   {
      std::string costmap_converter_plugin;
      double costmap_converter_rate      = 5;
      bool costmap_converter_spin_thread = true;

   } _costmap_conv_params;

public:
   EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

};  // end namespace mpc_local_planner

#endif  // PG_MPC_HARD_CSTR_LOCAL_PLANNER_ROS_H_
