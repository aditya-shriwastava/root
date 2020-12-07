// BSD 3-Clause License
//
// Copyright (c) 2020, Aditya Shriwastava
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// 1. Redistributions of source code must retain the above copyright notice, this
//    list of conditions and the following disclaimer.
//
// 2. Redistributions in binary form must reproduce the above copyright notice,
//    this list of conditions and the following disclaimer in the documentation
//    and/or other materials provided with the distribution.
//
// 3. Neither the name of the copyright holder nor the names of its
//    contributors may be used to endorse or promote products derived from
//    this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
// FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
// DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
// SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
// CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
// OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE

//! @file gt_pub.h
//! @author Aditya Shriwastava

#pragma once

#include <string>
#include <vector>
#include <cmath>

#include <ros/ros.h>

#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

#include <gazebo_msgs/LinkStates.h>

#include <tf2/LinearMath/Transform.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Vector3.h>

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>

namespace ground_truth{
  //! @brief Class for publishing ground truth information from gazebo
  //! and other debugging information.
  //!
  //! GtPub: Ground truth Publisher.
  class GtPub{
  public:
    //! @brief Constructor
    //!
    //! ## Algorithm
    //! 1. Load all the parameter from parameter server
    //! 2. Initialize Subscribers for:
    //!   1. gazebo/link_states
    //!   2. All odom and pose topics as indicated by
    //!   source_topic param
    //! 3. Initialize Publishers for:
    //!   1. gt_path and othe paths as configured by the param
    GtPub();
  private:
    ros::NodeHandle _nh;

    tf2_ros::Buffer _tf_buffer;
    tf2_ros::TransformListener _tf_listener;
    tf2_ros::TransformBroadcaster _tf_bdcstr;
    tf2_ros::StaticTransformBroadcaster _static_tf_bdcstr;

    ros::Subscriber _gazebo_link_states_sub;

    //! @brief ground truth path taken by the base_footprint of the robot
    nav_msgs::Path _gt_path;
    ros::Publisher _gt_path_pub;

    std::vector<ros::Subscriber> _source_sub;

    //! @brief Vector of paths taken by the robot as indicated by the
    //! corresponding source topics.
    std::vector<nav_msgs::Path> _source_path;
    std::vector<ros::Publisher> _source_path_pub;

    //! @brief Holds param ~robot
    std::string _robot;
    //! @brief Holds param ~base_footprint
    std::string _base_footprint;
    //! @brief Holds param ~tf_tree_root
    std::string _tf_tree_root;
    //! @brief Holds param ~layout
    std::string _layout;

    //! @brief Transorm of starting pose of the robot base_footprint
    //! in layout frame
    //!
    //! This is holding the first transform received from gazebo
    //! link_state topic
    tf2::Transform _gt_start_layout_tf;

    //! @brief Holds param ~source_topic
    std::vector<std::string> _source_topic;
    //! @brief Holds param ~source_type
    std::vector<std::string> _source_type;
    //! @brief Holds param ~frame_name
    std::vector<std::string> _frame_name;
    //! @brief Holds param ~publish_tf
    std::vector<bool> _publish_tf;
    //! @brief Holds param ~publish_path
    std::vector<bool> _publish_path;
    //! @brief Holds param ~publish_gt_path
    bool _publish_gt_path;

    //! @brief Holds param ~path_resolution
    double _path_resolution;

    //! @brief Load all the initialization parameters from parameter server.
    //!
    //! ## Parameter List
    //! * ~robot
    //!   - (string) Name of the robot as specified in robot URDF file.
    //! * ~base_footprint
    //!   - (string) Base Footprint of the robot.
    //! * ~tf_tree_root
    //!   - (string) Root of the tf_tree befor tf added by GtPub.
    //!   - Typically "odom" or "map".
    //! * ~layout
    //!   - (string)Frame attached to the layout/map
    //! * ~source_topic
    //!   - (vector<string>) Array of source topic (publishing robot pose information) to subscribe
    //! * ~source_type
    //!   - (vector<string>) Message type of the source topic in the order of source_topic
    //!   - Supported message types:
    //!     1. nav_msgs/Odometry
    //!     2. geometry_msgs/PoseWithCovarianceStamped
    //! * ~frame_name
    //!   - (vector<string>) Array of frame names associated with the source in the order of source_topic
    //! * ~publish_tf
    //!   - (vector<bool>) Array of bool indicating whether to publish tf or not corrosponding to frame at that index
    //! * ~publish_path
    //!   - (vector<bool>) Array of bool indicating whether to publish path or not corrosponding to frame at that index
    //! * ~publish_gt_path
    //!   - (bool) Whether to publish ground truth path or not
    //! * ~path_resolution
    //!   - (double) Minimum distance by which pose will be separated in the path
    void LoadParameters();

    //! @brief Odom callback
    //!
    //! ## Algorithm
    //! 1. Get the tf2::Transform form the odometry msg.
    //! 2. If publish tf is set to true, then publish the tf.
    //!   - ("gt_start") -> (<frame_name>)
    //! 3. If publish path is set to true, then:
    //!   1. Add pose to the path.
    //!   2. Publish path.
    //!
    //! @param[in] index This is the index of _source_topic to
    //! indicate from which topic this msg has came. This is
    //! important as multiple topic can have came callback function.
    //!
    //! @param[in] odom Odometry msg received.
    void OdomCb
    (const nav_msgs::Odometry::ConstPtr& odom, int index);

    //! @brief Pose callback
    //!
    //! ## Algorithm
    //! 1. Get the tf2::Transform form the pose msg.
    //! 2. If publish tf is set to true, then publish the tf.
    //!   - ("gt_start") -> (<frame_name>)
    //! 3. If publish path is set to true, then:
    //!   1. Add pose to the path.
    //!   2. Publish path.
    //!
    //! @param[in] index This is the index of _source_topic to
    //! indicate from which topic this msg has came. This is
    //! important as multiple topic can have came callback function.
    //!
    //! @param[in] pose Pose msg received.
    void PoseCb(const
    geometry_msgs::PoseWithCovarianceStamped::ConstPtr&
    pose, int index);

    //! @brief LinkStates callback from gazebo
    //!
    //! ## Algorithm
    //! 1. Get base_footprint tf wrt layout frame from link_states.
    //! 2. (For first iteration) Mark this tf as tf of gt_start wrt
    //! layout and publish static tf corresponding to this.
    //! 3. if _publish_gt_path is true then append ground truth robot
    //! pose in gt_path and publish it. (Take path_resolution into
    //! consideration while doing this)
    //!   - This path will be in gt_start frame.
    //! 4. Find tf of tf_tree_root wrt gt_start from tf tree and
    //! publish it
    //!
    //! @param[in] link_states holds the state of all the links
    //! being simulated in gazebo
    void GazeboLinkStatesCb
    (const gazebo_msgs::LinkStates::ConstPtr& link_states);

    //! @brief Extracts tf of base_footprint wrt layout from link_states
    //!
    //! @param[in] link_states holds the state of all the links
    //! being simulated in gazebo
    //! @param[out] tf tf of base_footprint wrt layout
    bool GetBaseFootprintLayoutTf
    (const gazebo_msgs::LinkStates::ConstPtr& link_states,
    tf2::Transform* tf);

    //! @brief Gets the requested tf from tf tree
    //!
    //! @param[in] parent parent of the transform requested
    //! @param[in] child child of the transform requested
    tf2::Transform GetTf
    (std::string parent, std::string child);

    //! @brief Publishes static tf as requested
    //!
    //! @param[in] parent parent of the static tf
    //! @param[in] child child of the static tf
    //! @param[in] tf static transform to publish
    void PublishStaticTf(std::string parent,
    std::string child, tf2::Transform* tf);

    //! @brief Publishes tf as requested
    //!
    //! @param[in] parent parent of the tf
    //! @param[in] child child of the tf
    //! @param[in] sequence_id in the tf msg
    //! @param[in] tf static transform to publish
    void PublishTf
    (std::string parent, std::string child,
    int sequence_id, tf2::Transform* tf);

    //! @brief Helper function to append pose to path
    //!
    //! It appends pose and resets the header of the msg as well
    //!
    //! @param[out] path path to be appended
    //! @param[in] tf transform to append
    //! @param[in] frame_id frame associated with the path
    //! @param[in] path_resolution minimum distance between
    //! poses in the path
    //! @return bool indicating whether pose was appended or not
    bool AddPosePath(nav_msgs::Path* path,
    tf2::Transform* tf, std::string frame_id,
    double path_resolution);
  };
}
