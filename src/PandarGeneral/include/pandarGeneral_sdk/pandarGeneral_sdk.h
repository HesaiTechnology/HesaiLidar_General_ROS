/******************************************************************************
 * Copyright 2018 The Hesai Technology Authors. All Rights Reserved.
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

#ifndef INCLUDE_PANDAR40P_SDK_PANDAR40P_SDK_H_
#define INCLUDE_PANDAR40P_SDK_PANDAR40P_SDK_H_

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pthread.h>
#include <semaphore.h>

#include <string>
#include <vector>

#include <boost/function.hpp>

#include "pandarGeneral/pandarGeneral.h"
#include "pandarGeneral/point_types.h"

class PandarGeneralSDK_Internal;

class PandarGeneralSDK {
 public:
  /**
   * @brief Constructor
   * @param device_ip  				The ip of the device
   *        lidar_port 				The port number of lidar data
   *        gps_port   				The port number of gps data
   *        pcl_callback      The callback of PCL data structure
   *        gps_callback      The callback of GPS structure
   *        start_angle       The start angle of every point cloud
   *                          should be <real angle> * 100.
   */
  PandarGeneralSDK(std::string device_ip, const uint16_t lidar_port,
          const uint16_t gps_port,
          boost::function<void(boost::shared_ptr<PPointCloud>, double)>
              pcl_callback,
          boost::function<void(double)> gps_callback, uint16_t start_angle,
          int tz = 0,
          std::string frame_id = std::string("hesai40"));
  /**
   * @brief deconstructor
   */
  ~PandarGeneralSDK();

  /**
   * @brief load the lidar correction file.
   * @param contents The correction contents of lidar correction
   */
  int LoadLidarCorrectionFile(std::string contents);

  /**
   * @brief Reset Lidar's start angle.
   * @param angle The start angle
   */
  void ResetLidarStartAngle(uint16_t start_angle);

  /**
   * @brief Get Lidar's Calibration.
   * @return The correction contents of lidar correction
   */
  std::string GetLidarCalibration();

  /**
   * @brief Run SDK.
   */
  int Start();

  /**
   * @brief Stop SDK.
   */
  void Stop();

 private:
  PandarGeneralSDK_Internal *internal_;
};

#endif  // INCLUDE_PANDAR40P_SDK_PANDAR40P_SDK_H_
