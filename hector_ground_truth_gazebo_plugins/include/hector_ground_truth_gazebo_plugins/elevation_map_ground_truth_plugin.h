// Copyright (c) 2019 Stefan Fabian. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for full license information.

#ifndef HECTOR_GROUND_TRUTH_GAZEBO_PLUGINS_ELEVATION_MAP_GROUND_TRUTH_PLUGIN_H
#define HECTOR_GROUND_TRUTH_GAZEBO_PLUGINS_ELEVATION_MAP_GROUND_TRUTH_PLUGIN_H

#include <hector_ground_truth_gazebo_plugins_msgs/GenerateGroundTruth.h>

#include <gazebo/common/Plugin.hh>
#include <ros/callback_queue.h>
#include <ros/node_handle.h>
#include <ros/service_server.h>

namespace hector_ground_truth_gazebo_plugins
{

class ElevationMapGroundTruthPlugin : public gazebo::WorldPlugin
{
public:
  void Load( gazebo::physics::WorldPtr world, sdf::ElementPtr sdf ) override;

private:
  bool
  GenerateGroundTruthCallback( hector_ground_truth_gazebo_plugins_msgs::GenerateGroundTruthRequest &req,
                               hector_ground_truth_gazebo_plugins_msgs::GenerateGroundTruthResponse &resp );

  std::shared_ptr<ros::NodeHandle> nh_;
  ros::CallbackQueue queue_;
  std::shared_ptr<ros::AsyncSpinner> spinner_;
  ros::ServiceServer service_server_;
  ros::Publisher publisher_;
  gazebo::physics::WorldPtr world_;
};
}

#endif //HECTOR_GROUND_TRUTH_GAZEBO_PLUGINS_ELEVATION_MAP_GROUND_TRUTH_PLUGIN_H
