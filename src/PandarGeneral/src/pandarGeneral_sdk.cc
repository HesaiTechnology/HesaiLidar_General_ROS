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

#include "pandarGeneral_sdk/pandarGeneral_sdk.h"
#include "src/tcp_command_client.h"
#include "yaml-cpp/yaml.h"

#define PANDARGENERALSDK_TCP_COMMAND_PORT (9347)

/*
class PandarGeneralSDK_Internal {
 public:
  PandarGeneralSDK_Internal(
      std::string device_ip, const uint16_t lidar_port, const uint16_t gps_port,
      boost::function<void(boost::shared_ptr<PPointCloud>, double)>
          pcl_callback,
      boost::function<void(double)> gps_callback, uint16_t start_angle,
      int tz, std::string frame_id);
  ~PandarGeneralSDK_Internal();
  int LoadLidarCorrectionFile(std::string correction_content);
  void ResetLidarStartAngle(uint16_t start_angle);
  std::string GetLidarCalibration();
  void GetCalibrationFromDevice();
  int Start();
  void Stop();

 private:
  PandarGeneral *pandarGeneral_;
  void *tcp_command_client_;
  boost::thread *get_calibration_thr_;
  bool enable_get_calibration_thr_;
  bool got_lidar_calibration_;
  std::string correction_content_;
};
*/
PandarGeneralSDK::PandarGeneralSDK(
    std::string device_ip, const uint16_t lidar_port, const uint16_t gps_port,
    boost::function<void(boost::shared_ptr<PPointCloud>, double)>
        pcl_callback,
    boost::function<void(double)> gps_callback, uint16_t start_angle,
    int tz, int pcl_type, std::string frame_id) {
  pandarGeneral_ = NULL;

  pandarGeneral_ = new PandarGeneral(device_ip, lidar_port,
            gps_port, pcl_callback, gps_callback, start_angle, tz, pcl_type, frame_id);

  tcp_command_client_ =
      TcpCommandClientNew(device_ip.c_str(), PANDARGENERALSDK_TCP_COMMAND_PORT);
  if (!tcp_command_client_) {
    std::cout << "Init TCP Command Client Failed" << std::endl;
  }
  get_calibration_thr_ = NULL;
  enable_get_calibration_thr_ = false;
  got_lidar_calibration_ = false;
}

PandarGeneralSDK::~PandarGeneralSDK() {
  Stop();
  if (pandarGeneral_) {
    delete pandarGeneral_;
  }
}

/**
 * @brief load the correction file
 * @param file The path of correction file
 */
int PandarGeneralSDK::LoadLidarCorrectionFile(
    std::string correction_content) {
  return pandarGeneral_->LoadCorrectionFile(correction_content);
}

/**
 * @brief load the correction file
 * @param angle The start angle
 */
void PandarGeneralSDK::ResetLidarStartAngle(uint16_t start_angle) {
  if (!pandarGeneral_) return;
  pandarGeneral_->ResetStartAngle(start_angle);
}

std::string PandarGeneralSDK::GetLidarCalibration() {
  return correction_content_;
}

int PandarGeneralSDK::Start() {
  Stop();

  if (pandarGeneral_) {
    pandarGeneral_->Start();
  }

  enable_get_calibration_thr_ = true;
  get_calibration_thr_ = new boost::thread(
      boost::bind(&PandarGeneralSDK::GetCalibrationFromDevice, this));
}

void PandarGeneralSDK::Stop() {
  if (pandarGeneral_) pandarGeneral_->Stop();

  enable_get_calibration_thr_ = false;
  if (get_calibration_thr_) {
    get_calibration_thr_->join();
  }
}

void PandarGeneralSDK::GetCalibrationFromDevice() {
  if (!tcp_command_client_) {
    return;
  }

  int32_t ret = 0;

  while (enable_get_calibration_thr_ && !got_lidar_calibration_) {
    if (!got_lidar_calibration_) {
      // get lidar calibration.
      char *buffer = NULL;
      uint32_t len = 0;

      ret = TcpCommandGetLidarCalibration(tcp_command_client_, &buffer, &len);
      if (ret == 0 && buffer) {
        // success;
        got_lidar_calibration_ = true;
        correction_content_ = std::string(buffer);
        if (pandarGeneral_) {
          ret = pandarGeneral_->LoadCorrectionFile(correction_content_);
          if (ret != 0) {
            std::cout << "Parse Lidar Correction Error" << std::endl;
            got_lidar_calibration_ = false;
          } else {
            std::cout << "Parse Lidar Correction Success!!!" << std::endl;
          }
        }
        free(buffer);
      }
    }

    sleep(1);
  }
}