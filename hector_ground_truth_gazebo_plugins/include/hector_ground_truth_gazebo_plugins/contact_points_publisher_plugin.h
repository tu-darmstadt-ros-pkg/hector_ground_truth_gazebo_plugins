// Copyright (c) 2020 Stefan Fabian. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for full license information.

#ifndef HECTOR_GROUND_TRUTH_GAZEBO_PLUGINS_CONTACT_POINTS_PUBLISHER_PLUGIN_H
#define HECTOR_GROUND_TRUTH_GAZEBO_PLUGINS_CONTACT_POINTS_PUBLISHER_PLUGIN_H

#include <gazebo/common/Plugin.hh>
#include <gazebo/physics/physics.hh>
#include <ros/callback_queue.h>
#include <ros/node_handle.h>

namespace hector_ground_truth_gazebo_plugins
{

class ContactPointsPublisherPlugin : public gazebo::WorldPlugin
{
public:
  void Load( gazebo::physics::WorldPtr _world, sdf::ElementPtr _sdf ) override;

private:
  void onWorldUpdate();

  void publishContactPoints( const ros::TimerEvent & );

  void loadModel();

  gazebo::physics::WorldPtr world_;
  gazebo::physics::ContactManager *contact_manager_;
  gazebo::physics::ModelPtr model_;
  gazebo::event::ConnectionPtr update_connection_;
  std::shared_ptr<ros::NodeHandle> nh_;
  ros::CallbackQueue queue_;
  std::shared_ptr<ros::AsyncSpinner> spinner_;
  ros::Timer timer_;
  ros::Publisher pub_;
  std::vector<ignition::math::Vector3d> contact_points_;
  std::string frame_;
  std::string model_name_;
};
}

#endif //HECTOR_GROUND_TRUTH_GAZEBO_PLUGINS_CONTACT_POINTS_PUBLISHER_PLUGIN_H
