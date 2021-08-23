/******************************************************************************
 * Copyright 2019 The Hesai Technology Authors. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/

#ifndef INCLUDE_PANDARGENERAL_H_
#define INCLUDE_PANDARGENERAL_H_

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pthread.h>
#include <semaphore.h>

#include <string>

#include <boost/function.hpp>

#include "pandarGeneral/point_types.h"
#include "hesai_lidar/PandarScan.h"

class PandarGeneral_Internal;

class PandarGeneral {
 public:
  /**
   * @brief Constructor
   * @param device_ip         The ip of the device
   *        lidar_port        The port number of lidar data
   *        gps_port          The port number of gps data
   *        pcl_callback      The callback of PCL data structure
   *        gps_callback      The callback of GPS structure
   *        start_angle       The start angle of every point cloud ,
   *                          should be <real angle> * 100.
   */
  PandarGeneral(std::string device_ip, uint16_t lidar_port, uint16_t gps_port,
            boost::function<void(boost::shared_ptr<PPointCloud>, double, hesai_lidar::PandarScanPtr)>
                pcl_callback,
            boost::function<void(double)> gps_callback, uint16_t start_angle,
            int tz, int pcl_type, std::string lidar_type, std::string frame_id, std::string timestampType, // the default timestamp type is LiDAR time
            std::string lidar_correction_file, std::string multicast_ip, bool coordinate_correction_flag,
            std::string target_frame, std::string fixed_frame); 

  /**
   * @brief Constructor
   * @param pcap_path         The path of pcap file
   *        pcl_callback      The callback of PCL data structure
   *        start_angle       The start angle of every point cloud,
   *                          should be <real angle> * 100.
   *        tz                The timezone
   *        frame_id          The frame id of pcd
   */
  PandarGeneral(std::string pcap_path, \
      boost::function<void(boost::shared_ptr<PPointCloud>, double, hesai_lidar::PandarScanPtr)> pcl_callback, \
      uint16_t start_angle, int tz, int pcl_type, std::string lidar_type, std::string frame_id, std::string timestampType,  // the default timestamp type is LiDAR time
      std::string lidar_correction_file, bool coordinate_correction_flag,
      std::string target_frame, std::string fixed_frame); 

  /**
   * @brief deconstructor
   */
  ~PandarGeneral();

  /**
   * @brief load the lidar correction file
   * @param contents The correction contents of lidar correction
   */
  int LoadCorrectionFile(std::string contents);

  /**
   * @brief Reset Lidar's start angle.
   * @param angle The start angle
   */
  void ResetStartAngle(uint16_t start_angle);

  /**
   * @brief Run SDK.
   */
  void Start();

  /**
   * @brief Stop SDK.
   */
  void Stop();

  void PushScanPacket(hesai_lidar::PandarScanPtr scan);

  bool GetCorrectionFileFlag();
  void SetCorrectionFileFlag(bool flag);

 private:
  PandarGeneral_Internal *internal_;
};

#endif  // INCLUDE_PANDARGENERAL_H_
