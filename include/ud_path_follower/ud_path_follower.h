/** 
 * @file path_follower_node.h
 * 
 * Discription goes here.
 * 
 * Local ROS Parameters:
 *  - map_frame: 
 *  - base_frame: 
 *  - dynamics_mode: str: one of "unicycle" or "holonomic".  
 *      Default is "unicycle"
 * 
 * TODO: Create header file and organize private/public attributes.
 * 
 * Subscribes:
 *  - 
 *  -
 * 
 * Publishes:
 * 
 *
 */
#ifndef UD_PATH_FOLLOWER_H_
#define UD_PATH_FOLLOWER_H_

#include <vector>
#include <cmath>

#include "ros/ros.h"
#include "std_msgs/Float64.h"
#include "std_msgs/Bool.h"
#include "geometry_msgs/TwistStamped.h"
#include "geographic_visualization_msgs/GeoVizItem.h"
#include "project11/utils.h"
#include "project11/tf2_utils.h"
#include "project11/pid.h"
#include "nav_msgs/Odometry.h"

namespace p11 = project11;

namespace ud_path_follower
{

class PathFollower
{
public:
  
  PathFollower();
  ~PathFollower();

  //Baxevani
  //void Prev_cmd_vel_Callback(const geometry_msgs::TwistStamped::ConstPtr& message);
  void Save_Data_Callback(const geometry_msgs::TwistStamped::ConstPtr& message);
  //void Prev_sensor_data_Callback(const nav_msgs::Odometry::ConstPtr& message);

protected:
  void initialize(ros::NodeHandle& nh, ros::NodeHandle& nh_private, const tf2_ros::Buffer *tf);
  bool generateCommands(geometry_msgs::Twist &cmd_vel);
  void setGoal(const std::vector< geometry_msgs::PoseStamped > & 	plan, double speed=0.0);
  double crossTrackError() const;
  double progress() const;
  bool goalReached() const;
  double distanceRemaining() const;
  void updateDisplay();
  void sendDisplay(bool dim=false);

  std::string m_base_frame;
  geographic_visualization_msgs::GeoVizItem vis_display_;

  const tf2_ros::Buffer *m_tf_buffer = nullptr;
private:

  // Dynamics mode
  enum DynamicsMode { unicycle, holonomic, UD_path_follower, UD_OCEANS23_follower };
  DynamicsMode m_dynamics_mode;

  /**
   * @brief Converts string (from ROS parameter) to mode enumeration.
   */
  PathFollower::DynamicsMode str2dynamicsmode(std::string str);


  /** @brief Holds the path bearing [rad, ENU] and distance [m] values. **/
  struct AzimuthDistance
  {
    p11::AngleRadians azimuth;
    double distance;
  };

  std::vector<geometry_msgs::PoseStamped> m_goal_path;

  std::vector<AzimuthDistance> m_segment_azimuth_distances;


  float m_kp_yaw;
  float m_kp_surge;
  float m_kp_sway;
  bool m_turn_in_place;
  float m_turn_in_place_threshold;
  
  int m_current_segment_index;

  double m_goal_speed;

  p11::AngleRadians m_crab_angle;

  double m_prev_lin_vel; //Baxevani
  double m_prev_lin_vel_y; //Baxevani
  double  m_prev_ang_vel; //Baxevani
  double m_current_y; // Baxevani

  double sensor_velx; //Baxevani
  double sensor_vely; //Baxevani
  double sensor_velw; //Baxevani
  
  //double prev_sensor_velx;
  //double prev_sensor_vely;
  //double prev_sensor_velw;
    
  double m_total_distance; // total distance of complete path in meters.
  double m_cumulative_distance; // distance of completed segments in meters.
  double m_current_segment_progress; // distance of completed progress for current segment in meters.
  double m_cross_track_error; // cross-track distance in meters. 

  p11::PID m_pid;
  
  // display
  ros::Publisher display_pub_;

  ros::Publisher prev_sensor_vel;  //Baxevani
  ros::Subscriber prev_cmd;  //Baxevani
  ros::Subscriber prev_y; //Baxevani
  ros::Subscriber simu_data; //Baxevani
  ros::Subscriber prev_sensor_data; //Baxevani

};

} // namespace ud_path_follower

#endif
