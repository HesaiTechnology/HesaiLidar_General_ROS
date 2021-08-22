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

#include "pandarGeneral/pandarGeneral.h"
#include "src/pandarGeneral_internal.h"
#include "log.h"
/**
 * @brief Constructor
 * @param device_ip         The ip of the device
 *        lidar_port        The port number of lidar data
 *        gps_port          The port number of gps data
 *        pcl_callback      The callback of PCL data structure
 *        gps_callback      The callback of GPS structure
 *        start_angle       The start angle of every point cloud
 */
PandarGeneral::PandarGeneral(
    std::string device_ip, uint16_t lidar_port, uint16_t gps_port,
    boost::function<void(boost::shared_ptr<PPointCloud>, double, hesai_lidar::PandarScanPtr)> pcl_callback,
    boost::function<void(double)> gps_callback, uint16_t start_angle, int tz,
    int pcl_type, std::string lidar_type, std::string frame_id, std::string timestampType,
    std::string lidar_correction_file, std::string multicast_ip, bool coordinate_correction_flag,
    std::string target_frame, std::string fixed_frame) {
      // LOG_FUNC();
  internal_ =
      new PandarGeneral_Internal(device_ip, lidar_port, gps_port, pcl_callback,
                             gps_callback, start_angle, tz, pcl_type, lidar_type, frame_id, timestampType, lidar_correction_file, 
                             multicast_ip, coordinate_correction_flag, target_frame, fixed_frame);
}

/**
 * @brief Constructor
 * @param pcap_path         The path of pcap file
 *        pcl_callback      The callback of PCL data structure
 *        start_angle       The start angle of every point cloud
 *        tz                The timezone
 *        frame_id          The frame id of point cloud
 */
PandarGeneral::PandarGeneral(
    std::string pcap_path, \
    boost::function<void(boost::shared_ptr<PPointCloud>, double, hesai_lidar::PandarScanPtr)> pcl_callback,\
    uint16_t start_angle, int tz, int pcl_type, std::string lidar_type, std::string frame_id, std::string timestampType, 
    std::string lidar_correction_file, bool coordinate_correction_flag, std::string target_frame, std::string fixed_frame) {
  internal_ = new PandarGeneral_Internal(pcap_path, pcl_callback, start_angle, \
      tz, pcl_type, lidar_type, frame_id, timestampType, lidar_correction_file, \
      coordinate_correction_flag, target_frame, fixed_frame);
}

/**
 * @brief deconstructor
 */
PandarGeneral::~PandarGeneral() { delete internal_; }

/**
 * @brief load the lidar correction file
 * @param contents The correction contents of lidar correction
 */
int PandarGeneral::LoadCorrectionFile(std::string file) {
  return internal_->LoadCorrectionFile(file);
}

/**
 * @brief Reset Lidar's start angle.
 * @param angle The start angle
 */
void PandarGeneral::ResetStartAngle(uint16_t start_angle) {
  internal_->ResetStartAngle(start_angle);
}

/**
 * @brief Run SDK.
 */
void PandarGeneral::Start() { internal_->Start(); }

/**
 * @brief Stop SDK.
 */
void PandarGeneral::Stop() { internal_->Stop(); }

void PandarGeneral::PushScanPacket(hesai_lidar::PandarScanPtr scan) {
  if (internal_) {
    internal_->PushScanPacket(scan);
  }
}

bool PandarGeneral::GetCorrectionFileFlag(){
  if (internal_) {
    return internal_->GetCorrectionFileFlag();
  }
}

void PandarGeneral::SetCorrectionFileFlag(bool flag){
  if (internal_) {
    internal_->SetCorrectionFileFlag(flag);
  }
}
