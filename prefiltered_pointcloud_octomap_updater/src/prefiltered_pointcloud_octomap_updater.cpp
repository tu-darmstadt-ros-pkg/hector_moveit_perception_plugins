// MIT License
//
// Copyright (c) 2023 Technische Universität Darmstadt ROS Packages
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.
// based on:
/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2011, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/* Author: Jon Binney, Ioan Sucan, Simon Giegerich */

#include <cmath>
#include <moveit/prefiltered_pointcloud_octomap_updater/prefiltered_pointcloud_octomap_updater.h>
#include <moveit/occupancy_map_monitor/occupancy_map_monitor.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/LinearMath/Vector3.h>
#include <tf2/LinearMath/Transform.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <XmlRpcException.h>

#include <memory>

namespace occupancy_map_monitor
{
static const std::string LOGNAME = "occupancy_map_monitor";

PrefilteredPointCloudOctomapUpdater::PrefilteredPointCloudOctomapUpdater()
  : OccupancyMapUpdater( "PrefilteredPointCloudOctomapUpdater" )
    , private_nh_( "~" )
    , max_range_( std::numeric_limits<double>::infinity())
    , point_subsample_( 1 )
    , max_update_rate_( 0 )
    , point_cloud_subscriber_( nullptr )
    , point_cloud_filter_( nullptr )
{
}

PrefilteredPointCloudOctomapUpdater::~PrefilteredPointCloudOctomapUpdater()
{
  stopHelper();
}

bool PrefilteredPointCloudOctomapUpdater::setParams( XmlRpc::XmlRpcValue &params )
{
  try
  {
    if ( !params.hasMember( "point_cloud_topic" ))
      return false;
    point_cloud_topic_ = static_cast<const std::string &>(params["point_cloud_topic"]);

    readXmlParam( params, "max_range", &max_range_ );
    readXmlParam( params, "point_subsample", &point_subsample_ );
    if ( params.hasMember( "max_update_rate" ))
      readXmlParam( params, "max_update_rate", &max_update_rate_ );
    if ( params.hasMember( "ns" ))
      ns_ = static_cast<const std::string &>(params["ns"]);
  }
  catch ( XmlRpc::XmlRpcException &ex )
  {
    ROS_ERROR_STREAM_NAMED( LOGNAME, "XmlRpc Exception: " << ex.getMessage());
    return false;
  }

  return true;
}

bool PrefilteredPointCloudOctomapUpdater::initialize()
{
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>();
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>( *tf_buffer_, root_nh_ );
  shape_mask_ = std::make_unique<point_containment_filter::ShapeMask>();
  shape_mask_->setTransformCallback(
    [ this ]( ShapeHandle shape, Eigen::Isometry3d &tf ) { return getShapeTransform( shape, tf ); } );

  std::string prefix;
  if ( !ns_.empty())
    prefix = ns_ + "/";
  return true;
}

void PrefilteredPointCloudOctomapUpdater::start()
{
  if ( point_cloud_subscriber_ )
    return;
  /* subscribe to point cloud topic using tf filter*/
  point_cloud_subscriber_ = new message_filters::Subscriber<sensor_msgs::PointCloud2>( root_nh_, point_cloud_topic_,
                                                                                       5 );
  if ( tf_listener_ && tf_buffer_ && !monitor_->getMapFrame().empty())
  {
    point_cloud_filter_ = new tf2_ros::MessageFilter<sensor_msgs::PointCloud2>( *point_cloud_subscriber_, *tf_buffer_,
                                                                                monitor_->getMapFrame(), 5, root_nh_ );
    point_cloud_filter_->registerCallback(
      [ this ]( const sensor_msgs::PointCloud2::ConstPtr &cloud ) { cloudMsgCallback( cloud ); } );
    ROS_INFO_NAMED( LOGNAME, "Listening to '%s' using message filter with target frame '%s'",
                    point_cloud_topic_.c_str(),
                    point_cloud_filter_->getTargetFramesString().c_str());
  }
  else
  {
    point_cloud_subscriber_->registerCallback(
      [ this ]( const sensor_msgs::PointCloud2::ConstPtr &cloud ) { cloudMsgCallback( cloud ); } );
    ROS_INFO_NAMED( LOGNAME, "Listening to '%s'", point_cloud_topic_.c_str());
  }
}

void PrefilteredPointCloudOctomapUpdater::stopHelper()
{
  delete point_cloud_filter_;
  delete point_cloud_subscriber_;
}

void PrefilteredPointCloudOctomapUpdater::stop()
{
  stopHelper();
  point_cloud_filter_ = nullptr;
  point_cloud_subscriber_ = nullptr;
}

ShapeHandle PrefilteredPointCloudOctomapUpdater::excludeShape( const shapes::ShapeConstPtr &shape )
{
  ShapeHandle h = 0;
  if ( shape_mask_ )
    h = shape_mask_->addShape( shape, 0.0, 0.0 );
  else
    ROS_ERROR_NAMED( LOGNAME, "Shape filter not yet initialized!" );
  return h;
}

void PrefilteredPointCloudOctomapUpdater::forgetShape( ShapeHandle handle )
{
}

bool PrefilteredPointCloudOctomapUpdater::getShapeTransform( ShapeHandle h, Eigen::Isometry3d &transform ) const
{
  auto it = transform_cache_.find( h );
  if ( it != transform_cache_.end())
    transform = it->second;
  return it != transform_cache_.end();
}

void PrefilteredPointCloudOctomapUpdater::updateMask( const sensor_msgs::PointCloud2 & /*cloud*/,
                                                      const Eigen::Vector3d & /*sensor_origin*/,
                                                      std::vector<int> & /*mask*/)
{
}

void PrefilteredPointCloudOctomapUpdater::cloudMsgCallback( const sensor_msgs::PointCloud2::ConstPtr &cloud_msg )
{
  ROS_DEBUG_NAMED( LOGNAME, "Received a new point cloud message" );
  ros::WallTime start = ros::WallTime::now();

  if ( max_update_rate_ > 0 )
  {
    // ensure we are not updating the octomap representation too often
    if ( ros::Time::now() - last_update_time_ <= ros::Duration( 1.0 / max_update_rate_ ))
      return;
    last_update_time_ = ros::Time::now();
  }

  if ( monitor_->getMapFrame().empty())
    monitor_->setMapFrame( cloud_msg->header.frame_id );

  /* get transform for cloud into map frame */
  tf2::Stamped<tf2::Transform> map_h_sensor;
  if ( monitor_->getMapFrame() == cloud_msg->header.frame_id )
    map_h_sensor.setIdentity();
  else
  {
    if ( tf_buffer_ )
    {
      try
      {
        tf2::fromMsg( tf_buffer_->lookupTransform( monitor_->getMapFrame(), cloud_msg->header.frame_id,
                                                   cloud_msg->header.stamp ), map_h_sensor );
      }
      catch ( tf2::TransformException &ex )
      {
        ROS_ERROR_STREAM_NAMED( LOGNAME, "Transform error of sensor data: " << ex.what() << "; quitting callback" );
        return;
      }
    }
    else
      return;
  }

  /* compute sensor origin in map frame */
  const tf2::Vector3 &sensor_origin_tf = map_h_sensor.getOrigin();
  octomap::point3d sensor_origin( sensor_origin_tf.getX(), sensor_origin_tf.getY(), sensor_origin_tf.getZ());
  Eigen::Vector3d sensor_origin_eigen( sensor_origin_tf.getX(), sensor_origin_tf.getY(), sensor_origin_tf.getZ());

  if ( !updateTransformCache( cloud_msg->header.frame_id, cloud_msg->header.stamp ))
    return;

  octomap::KeySet free_cells, occupied_cells;

  // We only use these iterators if we are creating a filtered_cloud for
  // publishing. We cannot default construct these, so we use unique_ptr's
  // to defer construction
  std::unique_ptr<sensor_msgs::PointCloud2Iterator<float>> iter_filtered_x;
  std::unique_ptr<sensor_msgs::PointCloud2Iterator<float>> iter_filtered_y;
  std::unique_ptr<sensor_msgs::PointCloud2Iterator<float>> iter_filtered_z;

  tree_->lockRead();

  try
  {
    /* do ray tracing to find which cells this point cloud indicates should be free, and which it indicates
     * should be occupied */
    for ( unsigned int row = 0; row < cloud_msg->height; row += point_subsample_ )
    {
      unsigned int row_c = row * cloud_msg->width;
      sensor_msgs::PointCloud2ConstIterator<float> pt_iter( *cloud_msg, "x" );
      // set iterator to point at start of the current row
      pt_iter += row_c;

      for ( unsigned int col = 0; col < cloud_msg->width; col += point_subsample_, pt_iter += point_subsample_ )
      {
        /* check for NaN */
        if ( !std::isnan( pt_iter[0] ) && !std::isnan( pt_iter[1] ) && !std::isnan( pt_iter[2] ))
        {
          tf2::Vector3 point_tf = map_h_sensor * tf2::Vector3( pt_iter[0], pt_iter[1], pt_iter[2] );
          occupied_cells.insert( tree_->coordToKey( point_tf.getX(), point_tf.getY(), point_tf.getZ()));
        }
      }
    }

    /* compute the free cells along each ray that ends at an occupied cell */
    for ( const octomap::OcTreeKey &occupied_cell: occupied_cells )
      if ( tree_->computeRayKeys( sensor_origin, tree_->keyToCoord( occupied_cell ), key_ray_ ))
        free_cells.insert( key_ray_.begin(), key_ray_.end());
  }
  catch ( ... )
  {
    tree_->unlockRead();
    return;
  }

  tree_->unlockRead();

  /* occupied cells are not free */
  for ( const octomap::OcTreeKey &occupied_cell: occupied_cells )
    free_cells.erase( occupied_cell );

  tree_->lockWrite();

  try
  {
    /* mark free cells only if not seen occupied in this cloud */
    for ( const octomap::OcTreeKey &free_cell: free_cells )
      tree_->updateNode( free_cell, false );

    /* now mark all occupied cells */
    for ( const octomap::OcTreeKey &occupied_cell: occupied_cells )
      tree_->updateNode( occupied_cell, true );
  }
  catch ( ... )
  {
    ROS_ERROR_NAMED( LOGNAME, "Internal error while updating octree" );
  }
  tree_->unlockWrite();
  ROS_DEBUG_NAMED( LOGNAME, "Processed point cloud in %lf ms", (ros::WallTime::now() - start).toSec() * 1000.0 );
  tree_->triggerUpdateCallback();
}
}  // namespace occupancy_map_monitor
