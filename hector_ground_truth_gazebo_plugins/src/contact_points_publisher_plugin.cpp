// Copyright (c) 2020 Stefan Fabian. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for full license information.

#include "hector_ground_truth_gazebo_plugins/contact_points_publisher_plugin.h"

#include <hector_ground_truth_gazebo_plugins_msgs/ContactPointsStamped.h>
#include <ros/ros.h>

using namespace hector_ground_truth_gazebo_plugins_msgs;

namespace hector_ground_truth_gazebo_plugins
{
void ContactPointsPublisherPlugin::Load( gazebo::physics::WorldPtr _world, sdf::ElementPtr _sdf )
{
  world_ = _world;
  contact_manager_ = _world->Physics()->GetContactManager();
  if ( !ros::isInitialized())
  {
    ros::init( ros::M_string{}, "gazebo", ros::init_options::AnonymousName | ros::init_options::NoSigintHandler );
  }
  nh_ = std::make_shared<ros::NodeHandle>();
  nh_->setCallbackQueue( &queue_ );
  spinner_ = std::make_shared<ros::AsyncSpinner>( 1, &queue_ );
  spinner_->start();
  double rate = _sdf->Get<double>( "rate", 10.0 ).first;
  timer_ = nh_->createTimer( ros::Rate( rate ), &ContactPointsPublisherPlugin::publishContactPoints, this );
  pub_ = nh_->advertise<ContactPointsStamped>( "ground_truth/contact_points", 100 );
  frame_ = _sdf->Get<std::string>( "frame", "world" ).first;
  ROS_INFO_NAMED( "hector_contact_points_publisher_plugin",
                  "ContactPointsPublisherPlugin loaded. World frame: %s", frame_.c_str());

  model_name_ = _sdf->Get<std::string>( "robot", "" ).first;
  update_connection_ = gazebo::event::Events::ConnectWorldUpdateEnd(
    std::bind( &ContactPointsPublisherPlugin::onWorldUpdate, this ));
}

void ContactPointsPublisherPlugin::onWorldUpdate()
{
  if ( pub_.getNumSubscribers() == 0 ) return; // No need to collect data if no one is listening
  if ( model_ == nullptr ) loadModel();
  ROS_INFO_ONCE( "WORLD UPDATE MODEL LOADED" );
  // After the world was updated we collect the new contact points
  const ignition::math::Pose3d &pose = model_->WorldPose();
  size_t i = 0;
  const ignition::math::Pose3d &transform = pose.Inverse();
  for ( const auto &contact : contact_manager_->GetContacts())
  {
    // According to the docs the vector may actually contain more contacts which are invalid
    if ( i == contact_manager_->GetContactCount()) break;
    ++i;
    if ( contact->collision1->GetSurface()->collideWithoutContact ||
         contact->collision2->GetSurface()->collideWithoutContact )
      continue;
    if ( !contact->collision1->GetLink()->GetEnabled() || !contact->collision2->GetLink()->GetEnabled()) continue;
    // We're only interested in robot world contacts and the robot is (probably) not static
    if ( contact->collision1->IsStatic() && contact->collision2->IsStatic()) continue;
    bool is_robot1 = false;
    gazebo::physics::ModelPtr model = contact->collision1->GetModel();
    while ( model != nullptr && !(is_robot1 = (model == model_)) && model != model->GetParentModel())
      model = model->GetParentModel();
    bool is_robot2 = false;
    model = contact->collision2->GetModel();
    while ( model != nullptr && !(is_robot2 = (model == model_)) && model != model->GetParentModel())
      model = model->GetParentModel();
    // If both collision participants are either from the robot or both are not from the robot, we ignore them
    if ( is_robot1 == is_robot2 ) continue;
    for ( int k = 0; k < contact->count; ++k )
    {
      // Transform contact points into model frame, so they can be aggregated
      contact_points_.push_back( transform.CoordPositionAdd(contact->positions[k]) );
    }
  }
}

void ContactPointsPublisherPlugin::publishContactPoints( const ros::TimerEvent & )
{
  if ( pub_.getNumSubscribers() == 0 ) return; // No need to publish data if no one is listening
  if ( model_ == nullptr ) return; // Model hasn't been initialized yet, nothing to publish
  ContactPointsStamped msg;
  msg.header.stamp = ros::Time::now();
  msg.header.frame_id = frame_;

  const ignition::math::Pose3d &pose = model_->WorldPose();
  msg.robot_pose.position.x = pose.Pos().X();
  msg.robot_pose.position.y = pose.Pos().Y();
  msg.robot_pose.position.z = pose.Pos().Z();
  msg.robot_pose.orientation.w = pose.Rot().W();
  msg.robot_pose.orientation.x = pose.Rot().X();
  msg.robot_pose.orientation.y = pose.Rot().Y();
  msg.robot_pose.orientation.z = pose.Rot().Z();

  for ( const auto &cp : contact_points_ )
  {
    geometry_msgs::Point point;
    point.x = cp.X();
    point.y = cp.Y();
    point.z = cp.Z();
    msg.contact_points.push_back( point );
  }
  contact_points_.clear();
  pub_.publish( msg );
}

void ContactPointsPublisherPlugin::loadModel()
{
  if ( !model_name_.empty())
  {
    model_ = world_->ModelByName( model_name_ );
    if ( model_ != nullptr )
    {
      ROS_INFO_NAMED( "hector_contact_points_publisher_plugin",
                      "Model '%s' loaded.", model_name_.c_str());
      return;
    }
    ROS_DEBUG_NAMED( "hector_contact_points_publisher_plugin",
                     "Waiting for model '%s' to be loaded.", model_name_.c_str());
    return;
  }
  // Find first model that is valid and not static
  unsigned int index = 0;
  while ( index < world_->ModelCount() &&
          ((model_ = world_->ModelByIndex( index )) == nullptr || model_->IsStatic()))
    ++index;
  if ( index == world_->ModelCount())
  {
    ROS_WARN_NAMED( "hector_contact_points_publisher_plugin",
                    "No robot models found. Waiting for robot model!" );
    model_ = nullptr;
    return;
  }
  ROS_INFO_NAMED( "hector_contact_points_publisher_plugin",
                  "Loaded first available non-static model (of %u models): '%s'.",
                  world_->ModelCount(), model_->GetName().c_str());
}
}

GZ_REGISTER_WORLD_PLUGIN( hector_ground_truth_gazebo_plugins::ContactPointsPublisherPlugin )
