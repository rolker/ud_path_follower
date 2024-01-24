/** 
 * @file ud_path_follower_node.cpp
 * 
 *  Path follower designed by K.Baxevani (kleiobax@udel.edu),  H.G. Tanner (btanner@udel.edu),
 *  G. E. Otto (gotto@udel.edu), and A. Trembanis (art@udel.edu)
 *  Theoretical analysis submitted to OCEANS2023
 *  ("Optimal ASV Path-Following for Improved Marine Survey Data Quality")
 *
 *  The code is based on CCOM's Project11 path follower 
 *
 * Local ROS Parameters:
 *  - map_frame: 
 *  - base_frame: 
 *  - dynamics_mode: str: UD_OCEANS23_follower.  
 *  - update_rate: float: update rate in Hz.
 *      Default is 10.0
 * 
 * 
 * 
 * Subscribes:
 *  - 
 *  -
 * 
 * Publishes:
 * 
 *
 */

#include "ud_path_follower/ud_path_follower.h"
#include <ros/console.h>
#include <math.h>
#include <nav_msgs/Odometry.h>
#include <fstream>
#include <ctime>
#include <vector>
#include <string>
#include <iostream>
#include <stdio.h>
#include<numeric>


namespace p11 = project11;

namespace ud_path_follower
{

double prev_sensor_velx=2.0;
double prev_sensor_vely=2.0;
double prev_sensor_velw = 0.0;
double prev_lin_vel = 0.5;
double prev_ang_vel = 0.0;
int count = 0;


PathFollower::PathFollower() :
        m_goal_speed(0.0),
        m_crab_angle(0.0),
        m_current_segment_index(0),
        m_total_distance(0.0),
        m_cumulative_distance(0.0),
        m_current_segment_progress(0.0),
        m_dynamics_mode(UD_OCEANS23_follower),
        m_prev_lin_vel(2.0), //Baxevani
        m_prev_ang_vel(0.0), //Baxevani
        m_current_y(0.0) //Baxevani

{
}

PathFollower::~PathFollower() {
}


void PathFollower::initialize(ros::NodeHandle &nh, ros::NodeHandle &nh_private, std::shared_ptr<tf2_ros::Buffer> tf) {
    tf_buffer_ = tf;
    // Initiate node and get parameters.
    nh_private.param<std::string>("base_frame", this->m_base_frame, "base_link");


    std::string dyn_mode_str;
    nh_private.param<std::string>("dynamics_mode", dyn_mode_str ,
                                "UD_OCEANS23_follower");                        //Baxevani


    if (this->m_dynamics_mode == PathFollower::DynamicsMode::UD_OCEANS23_follower) //Baxevani
    {
        ros::Rate rate(100);
        this->prev_sensor_vel = nh.advertise<nav_msgs::Odometry>("/prev_sensor_velocities", 1);

    }
    this->m_dynamics_mode = this->str2dynamicsmode(dyn_mode_str);

    display_pub_ = nh.advertise<geographic_visualization_msgs::GeoVizItem>
            ("project11/display", 5);
    vis_display_.id = "ud_path_follower";

}


PathFollower::DynamicsMode PathFollower::str2dynamicsmode(std::string str) {
    if (str == "UD_OCEANS23_follower")
        return PathFollower::DynamicsMode::UD_OCEANS23_follower; //Baxevani
    else
        ROS_FATAL_STREAM("ud_path_follower_node: dynamics_mode <" << str <<
                                                               "> is not recognized.  Shutting down");
    ros::shutdown();
    // This is just to avoid a compiler warning - should never get here.
    return PathFollower::DynamicsMode::UD_OCEANS23_follower;
}

void PathFollower::setGoal(const std::vector <geometry_msgs::PoseStamped> &plan, double speed) {
    m_goal_path = plan;
    m_goal_speed = speed;
    this->m_current_segment_index = 0;
    this->m_total_distance = 0.0;
    this->m_cumulative_distance = 0.0;
    this->m_current_segment_progress = 0.0;
    this->m_segment_azimuth_distances.clear();
    for (int i = 0; i + 1 < this->m_goal_path.size(); i++) {
        AzimuthDistance ad;
        double dx = this->m_goal_path[i + 1].pose.position.x -
                    this->m_goal_path[i].pose.position.x;
        double dy = this->m_goal_path[i + 1].pose.position.y -
                    this->m_goal_path[i].pose.position.y;
        ad.azimuth = atan2(dy, dx);
        ad.distance = sqrt(dx * dx + dy * dy);
        this->m_total_distance += ad.distance;
        this->m_segment_azimuth_distances.push_back(ad);
    }
}

bool PathFollower::generateCommands(geometry_msgs::Twist &cmd_vel) {
    if (!m_goal_path.empty()) {
        geometry_msgs::TransformStamped base_to_map;
        try {
            base_to_map = this->tf_buffer_->lookupTransform(
                    m_goal_path[0].header.frame_id, this->m_base_frame, ros::Time(0));
        }
        catch (tf2::TransformException &ex) {
            ROS_WARN_STREAM("PathFollower::generateCommands: " << ex.what());
        }


        ros::TimerEvent event;
        ros::NodeHandle nh;

        double vehicle_distance;

        p11::AngleRadians error_azimuth;
        double sin_error_azimuth;
        double cos_error_azimuth;
        double progress;

        bool found_current_segment = false;
        // For readability, define current segment azimuth and distance vars.
        p11::AngleRadians curr_seg_azi(0.0);
        double curr_seg_dist = 0.0;

        ros::Time now = ros::Time::now();

        while (!found_current_segment) {
            // For hover, dx/dy is the distance:
            // From: start point of the path (where the vehicle started)
            // To: vehicle positon
            double dx =
                    this->m_goal_path[this->m_current_segment_index].pose.position.x -
                    base_to_map.transform.translation.x;
            double dy =
                    this->m_goal_path[this->m_current_segment_index].pose.position.y -
                    base_to_map.transform.translation.y;
            vehicle_distance = sqrt(dx * dx + dy * dy);
            ROS_DEBUG_NAMED("ud_path_follower_node",
                            "path.x: %.1f, veh.x: %.1f, path.y: %.1f, veh.y: %.1f, "
                            "dx: %.1f m , dy: %.1f, vehicle_distance: %.1f m",
                            this->m_goal_path[this->m_current_segment_index].
                                    pose.position.x,
                            base_to_map.transform.translation.x,
                            this->m_goal_path[this->m_current_segment_index].
                                    pose.position.y,
                            base_to_map.transform.translation.y,
                            dx, dy, vehicle_distance);
            // Angle from path to vehicle
            // For hover, this is the angle (ENU) from the start point of the path
            // to the vehicle.
            p11::AngleRadians azimuth = atan2(-dy, -dx);

            // For readability, define current segment azimuth and distance vars.
            curr_seg_azi =
                    this->m_segment_azimuth_distances[this->m_current_segment_index].
                            azimuth;
            curr_seg_dist =
                    this->m_segment_azimuth_distances[this->m_current_segment_index].
                            distance;

            error_azimuth = azimuth - curr_seg_azi;


            sin_error_azimuth = sin(error_azimuth);
            cos_error_azimuth = cos(error_azimuth);

            // Distance traveled along the line.
            progress = vehicle_distance * cos_error_azimuth;
            ROS_DEBUG_NAMED("ud_path_follower_node",
                            "azimuth: %.1f deg, segment_azimuth: %.1f deg, "
                            "error_azimuth: %.1f deg, "
                            "segment_distance: %.1f m, progress %.2f ",
                            (double) azimuth * 180.0 / M_PI,
                            (double) curr_seg_azi * 180.0 / M_PI,
                            (double) error_azimuth * 180.0 / M_PI,
                            curr_seg_dist, progress);

            
            // Have we completed this segment?
            bool segment_complete;
            // if (m_goal_speed == 0.0)
            //     segment_complete = m_goal_path[m_current_segment_index + 1].header.stamp < now; // time based
            // else
                segment_complete = progress >= curr_seg_dist; // distance based

            if (segment_complete) {
                m_cumulative_distance += curr_seg_dist;
                m_current_segment_index += 1;
                if (this->m_current_segment_index >= this->m_goal_path.size() - 1) {
                    m_current_segment_progress = 0;
                    return false;
                }
            } else {
                m_current_segment_progress = progress;
                found_current_segment = true;
            }
        }

        // Distance away from line, m
        m_cross_track_error = vehicle_distance * sin_error_azimuth;


        p11::AngleRadians heading = tf2::getYaw(base_to_map.transform.rotation);

        // Baxevani --
        nav_msgs::Odometry ts_prev;
        //geometry_msgs::TwistStamped ;
        ts_prev.header.frame_id = this->m_base_frame;
        ts_prev.header.stamp = event.current_real;

        std::ofstream cross_track_data;

        cross_track_data.open("cross_track_data.txt", std::ios_base::app);//std::ios_base::app
        cross_track_data << m_cross_track_error << "  " << time(0) << "\n";      

        if (this->m_dynamics_mode == PathFollower::DynamicsMode::UD_OCEANS23_follower) {
            ros::NodeHandle nh;

            simu_data = nh.subscribe("/echo/mavros/global_position/raw/gps_vel", 10, &PathFollower::Save_Data_Callback, this);

            p11::AngleRadians target_heading = curr_seg_azi;

            //p11::AngleRadians theta = -p11::AngleRadiansZeroCentered(target_heading - heading).value();
            p11::AngleRadians theta = p11::AngleRadiansZeroCentered(target_heading - heading).value();


            this->m_current_y = m_cross_track_error;

//			  double r, v_d, dt;
//            nh.getParam("ricatti_param", r); //Ricatti parameter (r>0)
//            nh.getParam("prop_gain_x", px_gain); // Proportional gain on the surge direction
//            nh.getParam("prop_gain_w", pw_gain); // Proportional gain on the yaw direction

            
            double r = 1.8752285;//0.01285; //Ricatti parameter (r>0)
            double px_gain = 100.78;
            double pw_gain = 20.055;

            double v_d = 2.0; // Desired linear velocity of the vehicle
            double dt = 0.1; // TimerCallback runs every 0.01 secs
            double delta = 0.1; //Distance of the (sonar) sensor from the center of mass/position of imu
            

            double acc_x = (this->sensor_velx - prev_sensor_velx) / dt; //Acceleration on the surge axis
            double acc_y = (this->sensor_vely - prev_sensor_vely) / dt; //Acceleration on the sway axis
            double acc_w = (this->sensor_velw - prev_sensor_velw) / dt; //Acceleration on the sway axis
            

            double u = -(this->m_current_y + delta * sin(theta)) / sqrt(r) -
                       (this->sensor_velx * sin(-theta) + (sensor_vely + delta * this->sensor_velw) * cos(-theta)) * sqrt(2.0) /
                       pow(r, (1.0 / 4.0)); // Optimal feedback law for the system

            double a = 1 / r * (v_d - this->sensor_velx) * cos(theta) - u * sin(theta) +
                       delta * pow(this->sensor_velw, 2.0) +
                       (pow(sin(heading), 2.0) - pow(cos(heading), 2.0)) * this->sensor_vely *
                       this->sensor_velw;                 // Linear acceleration on the x axis (axis of motion)

            double alpha = 1 / (r * delta) * (v_d - this->sensor_velx) * sin(theta) + u * cos(theta) / delta -
                           this->sensor_velx * this->sensor_velw / delta
                           - 1 / delta * acc_y + 2 * cos(heading) * sin(heading) * this->sensor_vely * this->sensor_velw /
                                                 delta; // Angular acceleration around z axis

            cmd_vel.linear.x = 1*(prev_sensor_velx + acc_x*dt + px_gain*(a-acc_x)*pow(dt,2.0));
            cmd_vel.linear.y = 0.0;
            cmd_vel.angular.z = 1*(prev_sensor_velw + acc_w*dt + pw_gain*(alpha-acc_w)*pow(dt,2.0));

            ROS_WARN_STREAM( "Linear" << cmd_vel.linear.x << '\n');
            ROS_WARN_STREAM("Angular" << cmd_vel.angular.z << '\n');

            prev_sensor_velx = this->sensor_velx;
            prev_sensor_vely = this->sensor_vely;
            prev_sensor_velw = this->sensor_velw;

            return true;

        } else {
            ROS_FATAL_NAMED("path_follower_node",
                            "Unrecognized DynamicsMode!");
        }
    }
    return false;
}


void PathFollower::Save_Data_Callback(const geometry_msgs::TwistStamped::ConstPtr &message) //Baxevani
{
    this->sensor_velx = message->twist.linear.x;
    this->sensor_vely = message->twist.linear.y;
    this->sensor_velw = message->twist.angular.z;

    //Baxevani
    std::ofstream vel_x;
    std::ofstream vel_y;
    std::ofstream vel_w;


    vel_x.open("velx_data.txt", std::ios_base::app);//std::ios_base::app
    vel_x << sensor_velx << "  " << time(0) << "\n";
    vel_y.open("vely_data.txt", std::ios_base::app);//std::ios_base::app
    vel_y << sensor_vely << "  " << time(0) << "\n";
    vel_w.open("velw_data.txt", std::ios_base::app);//std::ios_base::app
    vel_w << sensor_velw << "  " << time(0) << "\n";

}


// For simulation uncomment this
// ------------------------------
/*
void PathFollower::Save_Data_Callback(const nav_msgs::Odometry::ConstPtr &message) //Baxevani
{
    this->sensor_velx = message->twist.twist.linear.x;
    this->sensor_vely = message->twist.twist.linear.y;
    this->sensor_velw = message->twist.twist.angular.z;

    //Baxevani
    std::ofstream vel_x;
    std::ofstream vel_y;
    std::ofstream vel_w;


    vel_x.open("velx_data.txt", std::ios_base::app);//std::ios_base::app
    vel_x << sensor_velx << "  " << time(0) << "\n";
    vel_y.open("vely_data.txt", std::ios_base::app);//std::ios_base::app
    vel_y << sensor_vely << "  " << time(0) << "\n";
    vel_w.open("velw_data.txt", std::ios_base::app);//std::ios_base::app
    vel_w << sensor_velw << "  " << time(0) << "\n";

}

void PathFollower::Prev_ypos_Callback(const nav_msgs::Odometry::ConstPtr &message) //Baxevani
{
    this->m_current_y = message->pose.pose.position.y;

}
*/

double PathFollower::progress() const {
    if (m_total_distance == 0)
        return 0.0;
    return (this->m_cumulative_distance + m_current_segment_progress) / this->m_total_distance;
}

double PathFollower::crossTrackError() const {
    return m_cross_track_error;
}

bool PathFollower::goalReached() const {
    return !m_goal_path.empty() && m_current_segment_index >= m_goal_path.size() - 1;
}

double PathFollower::distanceRemaining() const {
    if (m_total_distance > 0.0)
        return m_total_distance - m_cumulative_distance - m_current_segment_progress;
    return 0.0;
}

void PathFollower::updateDisplay() {
    vis_display_.lines.clear();
    if (!m_goal_path.empty() and tf_buffer_) {
        try {
            geometry_msgs::TransformStamped map_to_earth = tf_buffer_->lookupTransform("earth",
                                                                                        m_goal_path.front().header.frame_id,
                                                                                        ros::Time(0));

            geographic_visualization_msgs::GeoVizPointList gvpl;

            for (const auto &map_point: m_goal_path) {
                geometry_msgs::Point ecef_point_msg;
                tf2::doTransform(map_point.pose.position, ecef_point_msg, map_to_earth);
                p11::ECEF ecef_point;
                p11::fromMsg(ecef_point_msg, ecef_point);
                p11::LatLongDegrees ll_point = ecef_point;
                geographic_msgs::GeoPoint gp;
                p11::toMsg(ll_point, gp);
                gvpl.points.push_back(gp);
            }
            vis_display_.lines.push_back(gvpl);
        }
        catch (tf2::TransformException &ex) {
            ROS_WARN_STREAM("Unable to find transform to generate display: " << ex.what());
        }

    }
    sendDisplay();
}

void PathFollower::sendDisplay(bool dim) {
    if (!this->vis_display_.lines.empty()) {
        double intensity = 1.0;
        if (dim)
            intensity = 0.5;

        geographic_visualization_msgs::GeoVizPointList past_segments;
        past_segments.size = 5.0;
        past_segments.color.r = 0.5 * intensity;
        past_segments.color.g = 0.5 * intensity;
        past_segments.color.b = 0.5 * intensity;
        past_segments.color.a = 1.0 * intensity;
        geographic_visualization_msgs::GeoVizPointList current_segments;
        current_segments.size = 7.0;
        current_segments.color.r = 0.0 * intensity;
        current_segments.color.g = 1.0 * intensity;
        current_segments.color.b = 0.0 * intensity;
        current_segments.color.a = 1.0 * intensity;
        geographic_visualization_msgs::GeoVizPointList future_segments;
        future_segments.size = 5.0;
        future_segments.color.r = 0.0 * intensity;
        future_segments.color.g = 0.0 * intensity;
        future_segments.color.b = 1.0 * intensity;
        future_segments.color.a = 1.0 * intensity;


        auto &plist = this->vis_display_.lines.front();
        for (int i = 0; i < plist.points.size() && i <= m_current_segment_index; i++)
            past_segments.points.push_back(plist.points[i]);
        for (int i = m_current_segment_index; i < plist.points.size() && i <= m_current_segment_index + 1; i++)
            current_segments.points.push_back(plist.points[i]);
        for (int i = m_current_segment_index + 1; i < plist.points.size(); i++)
            future_segments.points.push_back(plist.points[i]);

        geographic_visualization_msgs::GeoVizItem display;
        display.id = vis_display_.id;
        display.lines.push_back(past_segments);
        display.lines.push_back(current_segments);
        display.lines.push_back(future_segments);
        display_pub_.publish(display);
    } else
        this->display_pub_.publish(this->vis_display_);
}

} // namespace ud_path_follower
