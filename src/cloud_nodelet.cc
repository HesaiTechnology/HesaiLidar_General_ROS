/*
 *  Copyright (C) 2012 Austin Robot Technology, Jack O'Quin
 *  Copyright (c) 2017 Hesai Photonics Technology, Yang Sheng
 *  Copyright (c) 2020 Hesai Photonics Technology, Lingwen Fang
 *  License: Modified BSD Software License Agreement
 *
 *  $Id$
 */

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include "pandarGeneral_sdk/pandarGeneral_sdk.h"
#include <fstream>
#include <pluginlib/class_list_macros.h>
#include <nodelet/nodelet.h>
#include "main.cc"


namespace pandar_pointcloud
{
  class CloudNodelet: public nodelet::Nodelet
  {
  public:

    CloudNodelet() {}
    ~CloudNodelet() {}

  private:

    virtual void onInit();
    boost::shared_ptr<HesaiLidarClient> HesaiLidarClient_;
  };

  /** @brief Nodelet initialization. */
  void CloudNodelet::onInit()
  {
    HesaiLidarClient_.reset(new HesaiLidarClient(getNodeHandle(), getPrivateNodeHandle()));
  }

} // namespace pandar_pointcloud


// Register this plugin with pluginlib.  Names must match nodelet_pandar.xml.
//
// parameters: class type, base class type
PLUGINLIB_EXPORT_CLASS(pandar_pointcloud::CloudNodelet, nodelet::Nodelet);