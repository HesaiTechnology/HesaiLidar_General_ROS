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

#include "pandarGeneral_sdk/pandarGeneral_sdk.h"
#include "src/tcp_command_client.h"
#include "yaml-cpp/yaml.h"

#define PANDARGENERALSDK_TCP_COMMAND_PORT (9347)

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
  apollo::drivers::hesai::PandarGeneral *pandarGeneral_;
  void *tcp_command_client_;
  boost::thread *get_calibration_thr_;
  bool enable_get_calibration_thr_;
  bool got_lidar_calibration_;
  std::string correction_content_;
};

PandarGeneralSDK_Internal::PandarGeneralSDK_Internal(
    std::string device_ip, const uint16_t lidar_port, const uint16_t gps_port,
    boost::function<void(boost::shared_ptr<PPointCloud>, double)>
        pcl_callback,
    boost::function<void(double)> gps_callback, uint16_t start_angle,
    int tz, std::string frame_id) {
  pandarGeneral_ = NULL;

  pandarGeneral_ = new apollo::drivers::hesai::PandarGeneral(device_ip, lidar_port,
            gps_port, pcl_callback, gps_callback, start_angle, tz, frame_id);

  tcp_command_client_ =
      TcpCommandClientNew(device_ip.c_str(), PANDARGENERALSDK_TCP_COMMAND_PORT);
  if (!tcp_command_client_) {
    std::cout << "Init TCP Command Client Failed" << std::endl;
  }
  get_calibration_thr_ = NULL;
  enable_get_calibration_thr_ = false;
  got_lidar_calibration_ = false;
}

PandarGeneralSDK_Internal::~PandarGeneralSDK_Internal() {
  Stop();
  if (pandarGeneral_) {
    delete pandarGeneral_;
  }
}

/**
 * @brief load the correction file
 * @param file The path of correction file
 */
int PandarGeneralSDK_Internal::LoadLidarCorrectionFile(
    std::string correction_content) {
  return pandarGeneral_->LoadCorrectionFile(correction_content);
}

/**
 * @brief load the correction file
 * @param angle The start angle
 */
void PandarGeneralSDK_Internal::ResetLidarStartAngle(uint16_t start_angle) {
  if (!pandarGeneral_) return;
  pandarGeneral_->ResetStartAngle(start_angle);
}

std::string PandarGeneralSDK_Internal::GetLidarCalibration() {
  return correction_content_;
}

int PandarGeneralSDK_Internal::Start() {
  Stop();

  if (pandarGeneral_) {
    pandarGeneral_->Start();
  }

  enable_get_calibration_thr_ = true;
  get_calibration_thr_ = new boost::thread(
      boost::bind(&PandarGeneralSDK_Internal::GetCalibrationFromDevice, this));
}

void PandarGeneralSDK_Internal::Stop() {
  if (pandarGeneral_) pandarGeneral_->Stop();

  enable_get_calibration_thr_ = false;
  if (get_calibration_thr_) {
    get_calibration_thr_->join();
  }
}

void PandarGeneralSDK_Internal::GetCalibrationFromDevice() {
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

/*****************************************************************************************
PandarGeneralSDK Part
*****************************************************************************************/
/**
 * @brief Constructor
 * @param device_ip         The ip of the device
 *        lidar_port        The port number of lidar data
 *        gps_port          The port number of gps data
 *        pcl_callback      The callback of PCL data structure
 *        gps_callback      The callback of GPS structure
 *        start_angle       The start angle of every point cloud
 */
PandarGeneralSDK::PandarGeneralSDK(
    std::string device_ip, const uint16_t lidar_port, const uint16_t gps_port,
    boost::function<void(boost::shared_ptr<PPointCloud>, double)> pcl_callback,
    boost::function<void(double)> gps_callback, uint16_t start_angle,
    int tz, std::string frame_id) {
  internal_ = new PandarGeneralSDK_Internal(
      device_ip, lidar_port, gps_port, pcl_callback, gps_callback, start_angle,
      tz, frame_id);
}

/**
 * @brief deconstructor
 */
PandarGeneralSDK::~PandarGeneralSDK() { delete internal_; }

/**
 * @brief load the lidar correction file
 * @param contents The correction contents of lidar correction
 */
int PandarGeneralSDK::LoadLidarCorrectionFile(std::string contents) {
  internal_->LoadLidarCorrectionFile(contents);
}

/**
 * @brief Reset Lidar's start angle.
 * @param angle The start angle
 */
void PandarGeneralSDK::ResetLidarStartAngle(uint16_t start_angle) {
  internal_->ResetLidarStartAngle(start_angle);
}

/**
 * @brief Get Lidar's Calibration.
 * @return The correction contents of lidar correction
 */
std::string PandarGeneralSDK::GetLidarCalibration() {
  return internal_->GetLidarCalibration();
}

/**
 * @brief Run SDK.
 */
int PandarGeneralSDK::Start() { internal_->Start(); }

/**
 * @brief Stop SDK.
 */
void PandarGeneralSDK::Stop() { internal_->Stop(); }
