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

void gpsCallback(int timestamp) {
  printf("gps: %d", timestamp);
}

void lidarCallback(boost::shared_ptr<PPointCloud> cld, double timestamp) {
  printf("Frame timestamp: %lf,\n", timestamp);
  printf("point_size: %ld,\n",cld->points.size());
}

int main(int argc, char** argv) {
  // PandarGeneralSDK pandarGeneral(std::string("192.168.1.201"), 2368, 10110, \
  //     lidarCallback, gpsCallback, 0, 0, 1, std::string("Pandar40P"));

  PandarGeneralSDK pandarGeneral(std::string("/path/to/pcapFile"), \
  lidarCallback, 0, 0, 1, std::string("PandarXT-16"));
  std::string filePath = "/path/to/correctionFile";
  std::ifstream fin(filePath);
  int length = 0;
  std::string strlidarCalibration;
  fin.seekg(0, std::ios::end);
  length = fin.tellg();
  fin.seekg(0, std::ios::beg);
  char *buffer = new char[length];
  fin.read(buffer, length);
  fin.close();
  strlidarCalibration = buffer;
  pandarGeneral.LoadLidarCorrectionFile(strlidarCalibration);

  pandarGeneral.Start();

  while (true) {
    sleep(100);
  }

  return 0;
}
