// Copyright (c) 2019 Stefan Fabian. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for full license information.

#include "hector_ground_truth_gazebo_plugins/elevation_map_ground_truth_plugin.h"

#include <grid_map_ros/GridMapRosConverter.hpp>

#include <gazebo/physics/PhysicsEngine.hh>
#include <gazebo/physics/RayShape.hh>
#include <gazebo/physics/World.hh>

using namespace gazebo::physics;
using namespace ignition;
using namespace hector_ground_truth_gazebo_plugins_msgs;

namespace hector_ground_truth_gazebo_plugins
{
void ElevationMapGroundTruthPlugin::Load( gazebo::physics::WorldPtr world, sdf::ElementPtr sdf )
{
  world_ = world;
  if ( !ros::isInitialized())
  {
    ros::init( ros::M_string{}, "gazebo", ros::init_options::AnonymousName | ros::init_options::NoSigintHandler );
  }
  nh_ = std::make_shared<ros::NodeHandle>();
  nh_->setCallbackQueue( &queue_ );
  spinner_ = std::make_shared<ros::AsyncSpinner>( 1, &queue_ );
  spinner_->start();
  service_server_ = nh_->advertiseService( "ground_truth/generate_elevation_map",
                                           &ElevationMapGroundTruthPlugin::GenerateGroundTruthCallback, this );
  publisher_ = nh_->advertise<grid_map_msgs::GridMap>( "ground_truth/elevation_map", 10, true );
}

bool ElevationMapGroundTruthPlugin::GenerateGroundTruthCallback( GenerateGroundTruthRequest &req,
                                                                 GenerateGroundTruthResponse &resp )
{
  long cells_x = req.cells_x;
  double cells_x_2 = cells_x / 2.0;
  long cells_y = req.cells_y;
  double cells_y_2 = cells_y / 2.0;
  long samples = req.samples;
  if ( samples <= 0 )
  {
    ROS_INFO( "Samples needs to be greater than 0. Set to 1, was: %li", samples );
    samples = 1;
  }
  double sample_dist = 1.0 / (samples + 1);
  grid_map::GridMap result( { "elevation" } );
  result.setGeometry( { cells_x * req.resolution, cells_y * req.resolution }, req.resolution );

  PhysicsEnginePtr engine = world_->Physics();
  engine->InitForThread();
  auto ray = boost::dynamic_pointer_cast<RayShape>( engine->CreateShape( "ray", nullptr ));

  math::Quaterniond q( req.origin.orientation.w, req.origin.orientation.x, req.origin.orientation.y,
                       req.origin.orientation.z );
  std::string entity_name;
  double resolution = req.resolution;
  math::Vector3d origin( req.origin.position.x, req.origin.position.y, req.origin.position.z );
  for ( Eigen::Index x = 0; x < cells_x; ++x )
  {
    for ( Eigen::Index y = 0; y < cells_y; ++y )
    {
      double min_dist = std::numeric_limits<double>::quiet_NaN();
      // Make samples^2 samples per cell
      for ( long xi = 0; xi < samples; ++xi )
      {
        for ( long yi = 0; yi < samples; ++yi )
        {
          math::Vector3d start((x - cells_x_2 + 0.5 + (xi - (samples - 1) / 2.0) * sample_dist) * resolution,
                               (y - cells_y_2 + 0.5 + (yi - (samples - 1) / 2.0) * sample_dist) * resolution,
                               req.max_z_height );
          math::Vector3d end( start.X(), start.Y(), start.Z() - req.truncation_distance );
          ray->SetPoints( origin + q * start, origin + q * end );
          double dist = 0;
          entity_name.clear();
          ray->GetIntersection( dist, entity_name );
          if ( entity_name.empty() || min_dist < dist ) continue; // NaN safe comparison
          min_dist = dist;
        }
      }

      double elevation = req.max_z_height - min_dist;
      if ( std::isfinite( elevation ))
      {
        // Matrix needs to be flipped as apparently grid map coordinate system is opposite to the gazebo coordinate system
        result["elevation"]( cells_x - 1 - x, cells_y - 1 - y ) = elevation;
      }
    }
  }

  result.setFrameId( "world" );
  grid_map::GridMapRosConverter::toMessage( result, resp.map );
  if ( req.keep_origin ) resp.map.info.pose.position = req.origin.position;
  publisher_.publish( resp.map );
  return true;
}
}

GZ_REGISTER_WORLD_PLUGIN( hector_ground_truth_gazebo_plugins::ElevationMapGroundTruthPlugin )
