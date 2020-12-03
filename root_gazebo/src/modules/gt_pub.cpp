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
#include "root_gazebo/gt_pub.h"

namespace ground_truth{

  GtPub::GtPub()
  : _tf_listener(_tf_buffer){

    this->_gazebo_link_states_sub =
    this->_nh.subscribe( "gazebo/link_states",
      10, &GtPub::GazeboLinkStatesCb, this);

    this->LoadParameters();

    if(this->_publish_gt_path){
      this->_gt_path_pub = this->_nh.advertise
      <nav_msgs::Path>("gt_path", 10);
    }

    if(!this->_source_topic.empty()){
      for(int i=0; i<this->_source_topic.size(); i++){
        if(this->_source_type.at(i) == "nav_msgs/Odometry"){

          this->_source_sub.emplace_back(
            this->_nh.subscribe<nav_msgs::Odometry>
            (this->_source_topic.at(i), 10,
            boost::bind(&GtPub::OdomCb, this, _1, i))
          );

        }else if(this->_source_type.at(i) ==
        "geometry_msgs/PoseWithCovarianceStamped"){

          this->_source_sub.emplace_back(
            this->_nh.subscribe<geometry_msgs::PoseWithCovarianceStamped>
            (this->_source_topic.at(i), 10,
            boost::bind(&GtPub::PoseCb, this, _1, i))
          );

        }else{
          ROS_FATAL("Invalid source type!!");
        }

        this->_source_path.emplace_back(nav_msgs::Path());

        if(this->_publish_path.at(i)){
          this->_source_path_pub.emplace_back
          (this->_nh.advertise<nav_msgs::Path>
          (this->_frame_name.at(i) + "_path", 10));
        }else{
          this->_source_path_pub.emplace_back
          (ros::Publisher());
        }

      }
    }

  }

  void GtPub::LoadParameters(){
    ros::NodeHandle nh_private("~");

    if(!nh_private.getParam("robot",
    this->_robot)){
      ROS_FATAL("~/robot does not exist");
    }

    if(!nh_private.getParam("base_footprint",
    this->_base_footprint)){
      ROS_FATAL("~/base_footprint does not exist");
    }

    if(!nh_private.getParam("tf_tree_root",
    this->_tf_tree_root)){
      ROS_FATAL("~/tf_tree_root does not exist");
    }

    if(!nh_private.getParam("layout",
    this->_layout)){
      ROS_FATAL("~/layout does not exist");
    }

    if(!nh_private.getParam("source_topic",
    this->_source_topic)){
      ROS_FATAL("~/source_topic does not exist");
    }

    if(!nh_private.getParam("source_type",
    this->_source_type)){
      ROS_FATAL("~/source_type does not exist");
    }

    if(!nh_private.getParam("frame_name",
    this->_frame_name)){
      ROS_FATAL("~/frame_name does not exist");
    }

    if(!nh_private.getParam("publish_tf",
    this->_publish_tf)){
      ROS_FATAL("~/publish_tf does not exist");
    }

    if(!nh_private.getParam("publish_path",
    this->_publish_path)){
      ROS_FATAL("~/publish_path does not exist");
    }

    if(!nh_private.getParam("publish_gt_path",
    this->_publish_gt_path)){
      ROS_FATAL("~/publish_gt_path does not exist");
    }

    if(!nh_private.getParam("path_resolution",
    this->_path_resolution)){
      ROS_FATAL("~/path_resolution does not exist");
    }
  }

  void GtPub::OdomCb
  (const nav_msgs::Odometry::ConstPtr& odom, int index){
    tf2::Transform tf;
    tf2::fromMsg(odom->pose.pose, tf);

    if(this->_publish_tf.at(index)){
      static int sequence_id = 0;
      sequence_id++;
      this->PublishTf
      ("gt_start", this->_frame_name.at(index), sequence_id, &tf);
    }

    if(this->_publish_path.at(index)){
      if( this->AddPosePath(&this->_source_path.at(index),
          &tf, "gt_start",
          this->_path_resolution) ){
        this->_source_path_pub.at(index)
        .publish(this->_source_path.at(index));
      }
    }
  }

  void GtPub::PoseCb(const
  geometry_msgs::PoseWithCovarianceStamped::ConstPtr&
  pose, int index){
    tf2::Transform tf;
    tf2::fromMsg(pose->pose.pose, tf);

    if(this->_publish_tf.at(index)){
      static int sequence_id = 0;
      sequence_id++;
      this->PublishTf
      ("gt_start", this->_frame_name.at(index), sequence_id, &tf);
    }

    if(this->_publish_path.at(index)){
      if( this->AddPosePath(&this->_source_path.at(index),
          &tf, "gt_start",
          this->_path_resolution) ){
        this->_source_path_pub.at(index)
        .publish(this->_source_path.at(index));
      }
    }
  }

  void GtPub::GazeboLinkStatesCb
  (const gazebo_msgs::LinkStates::ConstPtr& link_states){

    bool base_footprint_found = false;
    tf2::Transform base_footprint_layout_tf;

    while(!base_footprint_found){
      base_footprint_found =
      this->GetBaseFootprintLayoutTf(link_states, &base_footprint_layout_tf);
      if(!base_footprint_found){
        ROS_WARN("base_footprint not found!!");
        ros::Duration(0.1).sleep();
      }
    }

    static bool first = true;
    if(first){
      this->_gt_start_layout_tf = base_footprint_layout_tf;
      this->PublishStaticTf(this->_layout,
        "gt_start",
        &this->_gt_start_layout_tf);
      first = false;
    }

    if(this->_publish_gt_path){
      if( this->AddPosePath(&_gt_path,
          &base_footprint_layout_tf, this->_layout,
          this->_path_resolution) ){
        this->_gt_path_pub.publish(this->_gt_path);
      }
    }

    tf2::Transform base_footprint_tf_tree_root_tf =
    this->GetTf(this->_tf_tree_root, this->_base_footprint);

    tf2::Transform tf_tree_root_gt_start_tf =
    (this->_gt_start_layout_tf.inverse()) *
    (base_footprint_layout_tf) *
    (base_footprint_tf_tree_root_tf.inverse());

    static int sequence_id = 0;
    sequence_id++;
    this->PublishTf("gt_start", this->_tf_tree_root,
    sequence_id, &tf_tree_root_gt_start_tf);
  }

  bool GtPub::GetBaseFootprintLayoutTf
  (const gazebo_msgs::LinkStates::ConstPtr& link_states,
  tf2::Transform* tf){
    int base_footprint_index;
    bool base_footprint_found = false;
    for(int i=0; i<link_states->name.size(); i++){
      if(link_states->name.at(i) == this->_robot +
                                    "::" +
                                    this->_base_footprint){
        base_footprint_index = i;
        base_footprint_found = true;
        break;
      }
    }

    if(!base_footprint_found){
      return false;
    }

    tf->setOrigin(tf2::Vector3(
    link_states->pose.at(base_footprint_index).position.x,
    link_states->pose.at(base_footprint_index).position.y,
    link_states->pose.at(base_footprint_index).position.z));

    tf->setRotation(tf2::Quaternion(
    link_states->pose.at(base_footprint_index).orientation.x,
    link_states->pose.at(base_footprint_index).orientation.y,
    link_states->pose.at(base_footprint_index).orientation.z,
    link_states->pose.at(base_footprint_index).orientation.w));

    return true;
  }

  void GtPub::PublishStaticTf(std::string parent,
  std::string child, tf2::Transform* tf){
    geometry_msgs::TransformStamped tf_msg;
    tf_msg.transform = tf2::toMsg(*tf);
    tf_msg.header.seq = 1;
    tf_msg.header.stamp = ros::Time::now();
    tf_msg.header.frame_id = parent;
    tf_msg.child_frame_id = child;

    this->_static_tf_bdcstr.sendTransform(tf_msg);
  }

  tf2::Transform GtPub::GetTf
  (std::string parent, std::string child){
    bool success = false;
    tf2::Transform tf;
    geometry_msgs::TransformStamped tf_msg;

    while(!success){
      try{
        tf_msg =
        this->_tf_buffer.lookupTransform(
          parent,
          child,
          ros::Time(0)
        );
        success = true;
      }catch (tf2::TransformException &ex) {
        ROS_WARN("%s",ex.what());
        ros::Duration(0.1).sleep();
        success = false;
      }
    }
    tf2::fromMsg(
      tf_msg.transform,
      tf
    );
    return tf;
  }

  void GtPub::PublishTf(std::string parent,
  std::string child, int sequence_id,
  tf2::Transform* tf){
    geometry_msgs::TransformStamped tf_msg;

    tf_msg.header.seq = sequence_id;
    tf_msg.header.stamp = ros::Time::now();
    tf_msg.header.frame_id = parent;

    tf_msg.child_frame_id = child;

    tf_msg.transform =
    tf2::toMsg(*tf);

    this->_tf_bdcstr.sendTransform(tf_msg);
  }

  bool GtPub::AddPosePath(nav_msgs::Path* path,
  tf2::Transform* tf, std::string frame_id,
  double path_resolution){

    if(path->poses.size() != 0){
      double distance = std::sqrt(
        std::pow(tf->getOrigin().x() -
        path->poses.front().pose.position.x, 2) +
        std::pow(tf->getOrigin().y() -
        path->poses.front().pose.position.y, 2) +
        std::pow(tf->getOrigin().z() -
        path->poses.front().pose.position.z, 2)
      );

      if(distance < path_resolution){
        return false;
      }
    }

    path->header.seq = 0;
    path->header.stamp = ros::Time::now();
    path->header.frame_id = frame_id;

    geometry_msgs::PoseStamped pose;
    pose.header.seq = 0;
    pose.header.stamp = ros::Time::now();
    pose.header.frame_id = frame_id;
    tf2::toMsg(*tf, pose.pose);

    path->poses.emplace_back(pose);
    return true;
  }

}
