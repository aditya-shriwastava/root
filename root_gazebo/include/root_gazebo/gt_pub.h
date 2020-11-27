/**BSD 3-Clause License

Copyright (c) 2020, Aditya Shriwastava
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this
   list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice,
   this list of conditions and the following disclaimer in the documentation
   and/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its
   contributors may be used to endorse or promote products derived from
   this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.**/

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
  /** Info
   * GtPub -> Ground truth Publisher
   * gt_layout ----> gt_start ----> tf_tree_root ----> base_footprint
   *                  \   \
   *                  \   \>est_odom2
   *                  \>est_odom1
  **/
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

    void LoadParameters();

    void OdomCb
    (const nav_msgs::Odometry::ConstPtr& odom, int index);

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
