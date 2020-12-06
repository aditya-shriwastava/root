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
    GtPub();
  private:
    ros::NodeHandle _nh;

    tf2_ros::Buffer _tf_buffer;
    tf2_ros::TransformListener _tf_listener;
    tf2_ros::TransformBroadcaster _tf_bdcstr;
    tf2_ros::StaticTransformBroadcaster _static_tf_bdcstr;

    ros::Subscriber _gazebo_link_states_sub;
    nav_msgs::Path _gt_path;
    ros::Publisher _gt_path_pub;

    std::vector<ros::Subscriber> _source_sub;
    std::vector<nav_msgs::Path> _source_path;
    std::vector<ros::Publisher> _source_path_pub;

    std::string _robot,
                _base_footprint,
                _tf_tree_root,
                _layout;

    tf2::Transform _gt_start_layout_tf;

    std::vector<std::string> _source_topic;
    std::vector<std::string> _source_type;
    std::vector<std::string> _frame_name;
    std::vector<bool> _publish_tf;
    std::vector<bool> _publish_path;
    bool _publish_gt_path;
    double _path_resolution;

    //! @brief Load all the initialization parameters from parameter server.
    //!
    //! ## Parameter List
    //! * ~robot
    //!   - (string) Name of the robot as specified in robot URDF file.
    //! * ~base_footprint
    //!   - (string) Base Footprint of the robot.
    //! * ~tf_tree_root
    //!   - Root of the tf_tree befor tf added by GtPub.
    //!   - Typically "odom" or "map".
    //! * ~layout
    //! * ~source_topic
    //! * ~source_type
    //! * ~frame_name
    //! * ~publish_tf
    //! * ~publish_path
    //! * ~publish_gt_path
    //! * ~path_resolution
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

    void GazeboLinkStatesCb
    (const gazebo_msgs::LinkStates::ConstPtr& link_states);

    bool GetBaseFootprintLayoutTf
    (const gazebo_msgs::LinkStates::ConstPtr& link_states,
    tf2::Transform* tf);

    tf2::Transform GetTf
    (std::string parent, std::string child);

    void PublishStaticTf(std::string parent,
    std::string child, tf2::Transform* tf);

    void PublishTf
    (std::string parent, std::string child,
    int sequence_id, tf2::Transform* tf);

    bool AddPosePath(nav_msgs::Path* path,
    tf2::Transform* tf, std::string frame_id,
    double path_resolution);
  };
}
