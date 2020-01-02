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

#include <sstream>

#include "src/input.h"
#include "src/pandarGeneral_internal.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

double degreeToRadian(double degree) { return degree * M_PI / 180; }

// elevation angle of each line for HS Line 40 Lidar, Line 1 - Line 40
static const float pandar40p_elev_angle_map[] = {
    15.0f, 11.0f, 8.0f, 5.0f, 3.0f, 2.0f, 1.67f, 1.33f, \
    1.0f, 0.67f, 0.33f, 0.0f, -0.33f, -0.67f, -1.0f, -1.33f, \
    -1.66f, -2.0f, -2.33f, -2.67f, -3.0f, -3.33f, -3.67f, -4.0f, \
    -4.33f, -4.67f, -5.0f, -5.33f, -5.67f, -6.0f, -7.0f, -8.0f, \
    -9.0f, -10.0f, -11.0f, -12.0f, -13.0f, -14.0f, -19.0f, -25.0f
};

static const float pandarGeneral_elev_angle_map[] = {
    14.882f, 11.032f, 8.059f, 5.057f, 3.04f, 2.028f, 1.86f, 1.688f, \
    1.522f, 1.351f, 1.184f, 1.013f, 0.846f, 0.675f, 0.508f, 0.337f, \
    0.169f, 0.0f, -0.169f, -0.337f, -0.508f, -0.675f, -0.845f, -1.013f, \
    -1.184f, -1.351f, -1.522f, -1.688f, -1.86f, -2.028f, -2.198f, -2.365f, \
    -2.536f, -2.7f, -2.873f, -3.04f, -3.21f, -3.375f, -3.548f, -3.712f, \
    -3.884f, -4.05f, -4.221f, -4.385f, -4.558f, -4.72f, -4.892f, -5.057f, \
    -5.229f, -5.391f, -5.565f, -5.726f, -5.898f, -6.061f, -7.063f, -8.059f, \
    -9.06f, -9.885f, -11.032f, -12.006f, -12.974f, -13.93f, -18.889f, -24.897f
};

// Line 40 Lidar azimuth Horizatal offset ,  Line 1 - Line 40
static const float pandar40p_horizatal_azimuth_offset_map[] = {
    -1.042f, -1.042f, -1.042f, -1.042f, -1.042f, -1.042f, 3.125f, -5.208f, \
    -1.042f, 3.125f, -5.208f, -1.042f, 3.125f, -5.208f, -1.042f, 3.125f, \
    -5.208f, -1.042f, 3.125f, -5.208f, -1.042f, 3.125f, -5.208f, -1.042f, \
    3.125f, -5.208f, -1.042f, 3.125f, -5.208f, -1.042f, -1.042f, -1.042f, \
    -1.042f, -1.042f, -1.042f, -1.042f, -1.042f, -1.042f, -1.042f, -1.042f
};

static const float pandarGeneral_horizatal_azimuth_offset_map[] = {
    -1.042f, -1.042f, -1.042f, -1.042f,  -1.042f, -1.042f, 1.042f, 3.125f, \
    5.208f, -5.208f, -3.125f, -1.042f, 1.042f, 3.125f, 5.208f, -5.208f, \
    -3.125f, -1.042f, 1.042f, 3.125f, 5.208f, -5.208f, -3.125f, -1.042f, \
    1.042f, 3.125f, 5.208f, -5.208f, -3.125f, -1.042f, 1.042f, 3.125f, \
    5.208f, -5.208f, -3.125f, -1.042f, 1.042f, 3.125f, 5.208f, -5.208f, \
    -3.125f, -1.042f, 1.042f, 3.125f, 5.208f, -5.208f, -3.125f, -1.042f, \
    1.042f, 3.125f, 5.208f, -5.208f, -3.125f, -1.042f, -1.042f, -1.042f, \
    -1.042f, -1.042f, -1.042f, -1.042f, -1.042f, -1.042f, -1.042f, -1.042f
};

static const float pandar20_elev_angle_map[] = {
  3.058 ,2.046 ,1.706 ,1.031 ,0.355 ,0.018 ,-0.319 ,-0.995 ,
  -1.67 ,-2.347 ,-3.022 ,-4.032 ,-5.039 ,-6.043 ,-7.045 ,-8.041 ,
  -10.027 ,-11.988 ,-13.912 ,-18.871
};

static const float pandar20_horizatal_azimuth_offset_map[] = {
  -1.042 ,-1.042 ,3.125 ,-1.042 ,-5.208 ,-1.042 ,3.125 ,-1.042 ,-5.208 ,
  3.125 ,-1.042 ,-1.042 ,-1.042 ,-1.042 ,-1.042 ,-1.042 ,-1.042 ,-1.042,
  -1.042 ,-1.042
};

PandarGeneral_Internal::PandarGeneral_Internal(
    std::string device_ip, uint16_t lidar_port, uint16_t gps_port,
    boost::function<void(boost::shared_ptr<PPointCloud>, double)> pcl_callback,
    boost::function<void(double)> gps_callback, uint16_t start_angle, int tz,
    std::string frame_id) {
  
  
  pthread_mutex_init(&lidar_lock_, NULL);
  sem_init(&lidar_sem_, 0, 0);

  lidar_recv_thr_ = NULL;
  lidar_process_thr_ = NULL;

  enable_lidar_recv_thr_ = false;
  enable_lidar_process_thr_ = false;

  start_angle_ = start_angle;

  for (uint16_t rotIndex = 0; rotIndex < ROTATION_MAX_UNITS; ++rotIndex) {
    float rotation = degreeToRadian(0.01 * static_cast<double>(rotIndex));
    cos_lookup_table_[rotIndex] = cosf(rotation);
    sin_lookup_table_[rotIndex] = sinf(rotation);
  }

  input_.reset(new Input(lidar_port, gps_port));

  pcl_callback_ = pcl_callback;
  gps_callback_ = gps_callback;

  last_azimuth_ = 0;

  //laser40
  // init the block time offset, us
  block40OffsetSingle_[9] = 55.56f * 0.0f + 28.58f;
  block40OffsetSingle_[8] = 55.56f * 1.0f + 28.58f;
  block40OffsetSingle_[7] = 55.56f * 2.0f + 28.58f;
  block40OffsetSingle_[6] = 55.56f * 3.0f + 28.58f;
  block40OffsetSingle_[5] = 55.56f * 4.0f + 28.58f;
  block40OffsetSingle_[4] = 55.56f * 5.0f + 28.58f;
  block40OffsetSingle_[3] = 55.56f * 6.0f + 28.58f;
  block40OffsetSingle_[2] = 55.56f * 7.0f + 28.58f;
  block40OffsetSingle_[1] = 55.56f * 8.0f + 28.58f;
  block40OffsetSingle_[0] = 55.56f * 9.0f + 28.58f;

  block40OffsetDual_[9] = 55.56f * 0.0f + 28.58f;
  block40OffsetDual_[8] = 55.56f * 0.0f + 28.58f;
  block40OffsetDual_[7] = 55.56f * 1.0f + 28.58f;
  block40OffsetDual_[6] = 55.56f * 1.0f + 28.58f;
  block40OffsetDual_[5] = 55.56f * 2.0f + 28.58f;
  block40OffsetDual_[4] = 55.56f * 2.0f + 28.58f;
  block40OffsetDual_[3] = 55.56f * 3.0f + 28.58f;
  block40OffsetDual_[2] = 55.56f * 3.0f + 28.58f;
  block40OffsetDual_[1] = 55.56f * 4.0f + 28.58f;
  block40OffsetDual_[0] = 55.56f * 4.0f + 28.58f;

  // init the laser shot time offset, us
  laser40Offset_[3] = 3.62f;
  laser40Offset_[39] = 3.62f;
  laser40Offset_[35] = 4.92f;
  laser40Offset_[27] = 6.23f;
  laser40Offset_[11] = 8.19f;
  laser40Offset_[15] = 8.19f;
  laser40Offset_[31] = 9.5f;
  laser40Offset_[23] = 11.47f;
  laser40Offset_[28] = 12.77f;
  laser40Offset_[16] = 14.74f;
  laser40Offset_[2] = 16.04f;
  laser40Offset_[38] = 16.04f;
  laser40Offset_[34] = 17.35f;
  laser40Offset_[24] = 18.65f;
  laser40Offset_[8] = 20.62f;
  laser40Offset_[12] = 20.62f;
  laser40Offset_[30] = 21.92f;
  laser40Offset_[20] = 23.89f;
  laser40Offset_[25] = 25.19f;
  laser40Offset_[13] = 27.16f;
  laser40Offset_[1] = 28.47f;
  laser40Offset_[37] = 28.47f;
  laser40Offset_[33] = 29.77f;
  laser40Offset_[5] = 31.74f;
  laser40Offset_[21] = 31.7447f;
  laser40Offset_[9] = 33.71f;
  laser40Offset_[29] = 35.01f;
  laser40Offset_[17] = 36.98f;
  laser40Offset_[22] = 38.95f;
  laser40Offset_[10] = 40.91f;
  laser40Offset_[0] = 42.22f;
  laser40Offset_[36] = 42.22f;
  laser40Offset_[32] = 43.52f;
  laser40Offset_[4] = 45.49f;
  laser40Offset_[18] = 45.49f;
  laser40Offset_[6] = 47.46f;
  laser40Offset_[26] = 48.76f;
  laser40Offset_[14] = 50.73f;
  laser40Offset_[19] = 52.7f;
  laser40Offset_[9] = 54.67f;

  //laser64 init the laser shot time offset, us
  // init the block time offset, us
  block64OffsetSingle_[5] = 55.56f * 0.0f + 42.58f;
  block64OffsetSingle_[4] = 55.56f * 1.0f + 42.58f;
  block64OffsetSingle_[3] = 55.56f * 2.0f + 42.58f;
  block64OffsetSingle_[2] = 55.56f * 3.0f + 42.58f;
  block64OffsetSingle_[1] = 55.56f * 4.0f + 42.58f;
  block64OffsetSingle_[0] = 55.56f * 5.0f + 42.58f;

  block64OffsetDual_[5] = 55.56f * 0.0f + 42.58f;
  block64OffsetDual_[4] = 55.56f * 0.0f + 42.58f;
  block64OffsetDual_[3] = 55.56f * 1.0f + 42.58f;
  block64OffsetDual_[2] = 55.56f * 1.0f + 42.58f;
  block64OffsetDual_[1] = 55.56f * 2.0f + 42.58f;
  block64OffsetDual_[0] = 55.56f * 2.0f + 42.58f;

  laser64Offset_[50] = 1.304f * 0.0f + 1.968f * 0.0f + 3.62f;
  laser64Offset_[60] = 1.304f * 0.0f + 1.968f * 0.0f + 3.62f;
  laser64Offset_[44] = 1.304f * 1.0f + 1.968f * 0.0f + 3.62f;
  laser64Offset_[59] = 1.304f * 1.0f + 1.968f * 0.0f + 3.62f;
  laser64Offset_[38] = 1.304f * 2.0f + 1.968f * 0.0f + 3.62f;
  laser64Offset_[56] = 1.304f * 2.0f + 1.968f * 0.0f + 3.62f;
  laser64Offset_[8]  = 1.304f * 3.0f + 1.968f * 0.0f + 3.62f;
  laser64Offset_[54] = 1.304f * 3.0f + 1.968f * 0.0f + 3.62f;
  laser64Offset_[48] = 1.304f * 4.0f + 1.968f * 0.0f + 3.62f;
  laser64Offset_[62] = 1.304f * 4.0f + 1.968f * 0.0f + 3.62f;
  laser64Offset_[42] = 1.304f * 5.0f + 1.968f * 0.0f + 3.62f;
  laser64Offset_[58] = 1.304f * 5.0f + 1.968f * 0.0f + 3.62f;
  laser64Offset_[6]  = 1.304f * 6.0f + 1.968f * 0.0f + 3.62f;
  laser64Offset_[55] = 1.304f * 6.0f + 1.968f * 0.0f + 3.62f;
  laser64Offset_[52] = 1.304f * 7.0f + 1.968f * 0.0f + 3.62f;
  laser64Offset_[63] = 1.304f * 7.0f + 1.968f * 0.0f + 3.62f;
  laser64Offset_[46] = 1.304f * 8.0f + 1.968f * 0.0f + 3.62f;
  laser64Offset_[61] = 1.304f * 8.0f + 1.968f * 0.0f + 3.62f;
  laser64Offset_[40] = 1.304f * 9.0f + 1.968f * 0.0f + 3.62f;
  laser64Offset_[57] = 1.304f * 9.0f + 1.968f * 0.0f + 3.62f;
  laser64Offset_[5]  = 1.304f * 10.0f + 1.968f * 0.0f + 3.62f;
  laser64Offset_[53] = 1.304f * 10.0f + 1.968f * 0.0f + 3.62f;
  laser64Offset_[4]  = 1.304f * 11.0f + 1.968f * 0.0f + 3.62f;
  laser64Offset_[47] = 1.304f * 11.0f + 1.968f * 0.0f + 3.62f;
  laser64Offset_[3]  = 1.304f * 12.0f + 1.968f * 0.0f + 3.62f;
  laser64Offset_[49] = 1.304f * 12.0f + 1.968f * 0.0f + 3.62f;
  laser64Offset_[2]  = 1.304f * 13.0f + 1.968f * 0.0f + 3.62f;
  laser64Offset_[51] = 1.304f * 13.0f + 1.968f * 0.0f + 3.62f;
  laser64Offset_[1]  = 1.304f * 14.0f + 1.968f * 0.0f + 3.62f;
  laser64Offset_[45] = 1.304f * 14.0f + 1.968f * 0.0f + 3.62f;
  laser64Offset_[0]  = 1.304f * 15.0f + 1.968f * 0.0f + 3.62f;
  laser64Offset_[43] = 1.304f * 15.0f + 1.968f * 0.0f + 3.62f;
  laser64Offset_[23] = 1.304f * 15.0f + 1.968f * 1.0f + 3.62f;
  laser64Offset_[32] = 1.304f * 15.0f + 1.968f * 1.0f + 3.62f;
  laser64Offset_[26] = 1.304f * 15.0f + 1.968f * 2.0f + 3.62f;
  laser64Offset_[41] = 1.304f * 15.0f + 1.968f * 2.0f + 3.62f;
  laser64Offset_[20] = 1.304f * 15.0f + 1.968f * 3.0f + 3.62f;
  laser64Offset_[35] = 1.304f * 15.0f + 1.968f * 3.0f + 3.62f;
  laser64Offset_[14] = 1.304f * 15.0f + 1.968f * 4.0f + 3.62f;
  laser64Offset_[29] = 1.304f * 15.0f + 1.968f * 4.0f + 3.62f;
  laser64Offset_[21] = 1.304f * 15.0f + 1.968f * 5.0f + 3.62f;
  laser64Offset_[36] = 1.304f * 15.0f + 1.968f * 5.0f + 3.62f;
  laser64Offset_[15] = 1.304f * 15.0f + 1.968f * 6.0f + 3.62f;
  laser64Offset_[30] = 1.304f * 15.0f + 1.968f * 6.0f + 3.62f;
  laser64Offset_[9]  = 1.304f * 15.0f + 1.968f * 7.0f + 3.62f;
  laser64Offset_[24] = 1.304f * 15.0f + 1.968f * 7.0f + 3.62f;
  laser64Offset_[18] = 1.304f * 15.0f + 1.968f * 8.0f + 3.62f;
  laser64Offset_[33] = 1.304f * 15.0f + 1.968f * 8.0f + 3.62f;
  laser64Offset_[12] = 1.304f * 15.0f + 1.968f * 9.0f + 3.62f;
  laser64Offset_[27] = 1.304f * 15.0f + 1.968f * 9.0f + 3.62f;
  laser64Offset_[19] = 1.304f * 15.0f + 1.968f * 10.0f + 3.62f;
  laser64Offset_[34] = 1.304f * 15.0f + 1.968f * 10.0f + 3.62f;
  laser64Offset_[13] = 1.304f * 15.0f + 1.968f * 11.0f + 3.62f;
  laser64Offset_[28] = 1.304f * 15.0f + 1.968f * 11.0f + 3.62f;
  laser64Offset_[7]  = 1.304f * 15.0f + 1.968f * 12.0f + 3.62f;
  laser64Offset_[22] = 1.304f * 15.0f + 1.968f * 12.0f + 3.62f;
  laser64Offset_[16] = 1.304f * 15.0f + 1.968f * 13.0f + 3.62f;
  laser64Offset_[31] = 1.304f * 15.0f + 1.968f * 13.0f + 3.62f;
  laser64Offset_[10] = 1.304f * 15.0f + 1.968f * 14.0f + 3.62f;
  laser64Offset_[25] = 1.304f * 15.0f + 1.968f * 14.0f + 3.62f;
  laser64Offset_[17] = 1.304f * 15.0f + 1.968f * 15.0f + 3.62f;
  laser64Offset_[37] = 1.304f * 15.0f + 1.968f * 15.0f + 3.62f;
  laser64Offset_[11] = 1.304f * 15.0f + 1.968f * 16.0f + 3.62f;
  laser64Offset_[39] = 1.304f * 15.0f + 1.968f * 16.0f + 3.62f;

  for (int i = 0; i < HS_LIDAR_L20_BLOCK_NUMBER; i++) {
    block20OffsetSingle_[i] = 28.58f + (HS_LIDAR_L20_BLOCK_NUMBER - 1 - i) * 55.56f;
    block20OffsetDual_[i] = 28.58f + \
        static_cast<int>((HS_LIDAR_L20_BLOCK_NUMBER - 1 - i) / 2) * 55.56f;
  }

  laser20AOffset_[1] = 1.304f * 0.0f + 1.968f * 0.0f + 3.62f;
  laser20AOffset_[19]  = 1.304f * 0.0f + 1.968f * 0.0f + 3.62f;
  laser20AOffset_[16] = 1.304f * 1.0f + 1.968f * 0.0f + 3.62f;
  laser20AOffset_[14] = 1.304f * 3.0f + 1.968f * 1.0f + 3.62f;
  laser20AOffset_[11] = 1.304f * 3.0f + 1.968f * 2.0f + 3.62f;
  laser20AOffset_[0] = 1.304f * 5.0f + 1.968f * 3.0f + 3.62f;
  laser20AOffset_[18]  = 1.304f * 5.0f + 1.968f * 3.0f + 3.62f;
  laser20AOffset_[5] = 1.304f * 7.0f + 1.968f * 4.0f + 3.62f;
  laser20AOffset_[7] = 1.304f * 7.0f + 1.968f * 4.0f + 3.62f;
  laser20AOffset_[10] = 1.304f * 8.0f + 1.968f * 5.0f + 3.62f;
  laser20AOffset_[17] = 1.304f * 10.0f + 1.968f * 6.0f + 3.62f;
  laser20AOffset_[15]  = 1.304f * 11.0f + 1.968f * 6.0f + 3.62f;
  laser20AOffset_[3] = 1.304f * 11.0f + 1.968f * 7.0f + 3.62f;
  laser20AOffset_[13] = 1.304f * 12.0f + 1.968f * 8.0f + 3.62f;
  laser20AOffset_[9] = 1.304f * 12.0f + 1.968f * 9.0f + 3.62f;
  laser20AOffset_[6] = 1.304f * 12.0f + 1.968f * 11.0f + 3.62f;
  laser20AOffset_[2]  = 1.304f * 14.0f + 1.968f * 12.0f + 3.62f;
  laser20AOffset_[4] = 1.304f * 14.0f + 1.968f * 13.0f + 3.62f;
  laser20AOffset_[12] = 1.304f * 15.0f + 1.968f * 13.0f + 3.62f;
  laser20AOffset_[8] = 1.304f * 15.0f + 1.968f * 14.0f + 3.62f;

  laser20BOffset_[17] = 1.304f * 1.0f + 1.968f * 0.0f + 3.62f;
  laser20BOffset_[5]  = 1.304f * 2.0f + 1.968f * 1.0f + 3.62f;
  laser20BOffset_[15] = 1.304f * 3.0f + 1.968f * 1.0f + 3.62f;
  laser20BOffset_[11] = 1.304f * 3.0f + 1.968f * 2.0f + 3.62f;
  laser20BOffset_[8] = 1.304f * 4.0f + 1.968f * 3.0f + 3.62f;
  laser20BOffset_[19] = 1.304f * 5.0f + 1.968f * 3.0f + 3.62f;
  laser20BOffset_[3]  = 1.304f * 7.0f + 1.968f * 4.0f + 3.62f;
  laser20BOffset_[6] = 1.304f * 7.0f + 1.968f * 4.0f + 3.62f;
  laser20BOffset_[14] = 1.304f * 8.0f + 1.968f * 4.0f + 3.62f;
  laser20BOffset_[10] = 1.304f * 8.0f + 1.968f * 5.0f + 3.62f;
  laser20BOffset_[18] = 1.304f * 10.0f + 1.968f * 6.0f + 3.62f;
  laser20BOffset_[16]  = 1.304f * 11.0f + 1.968f * 6.0f + 3.62f;
  laser20BOffset_[1] = 1.304f * 11.0f + 1.968f * 7.0f + 3.62f;
  laser20BOffset_[13] = 1.304f * 12.0f + 1.968f * 8.0f + 3.62f;
  laser20BOffset_[4] = 1.304f * 12.0f + 1.968f * 11.0f + 3.62f;
  laser20BOffset_[0] = 1.304f * 14.0f + 1.968f * 12.0f + 3.62f;
  laser20BOffset_[9]  = 1.304f * 14.0f + 1.968f * 12.0f + 3.62f;
  laser20BOffset_[2] = 1.304f * 14.0f + 1.968f * 13.0f + 3.62f;
  laser20BOffset_[12] = 1.304f * 15.0f + 1.968f * 13.0f + 3.62f;
  laser20BOffset_[7] = 1.304f * 15.0f + 1.968f * 14.0f + 3.62f;


  for (int i = 0; i < LASER_COUNT; i++) {
    // for all the laser offset 
    elev_angle_map_[i] = pandar40p_elev_angle_map[i];
    horizatal_azimuth_offset_map_[i] = \
        pandar40p_horizatal_azimuth_offset_map[i];
  }

  for (int i = 0; i < HS_LIDAR_L64_UNIT_NUM; i++) {
    // for all the laser offset 
    General_elev_angle_map_[i] = pandarGeneral_elev_angle_map[i];
    General_horizatal_azimuth_offset_map_[i] = \
        pandarGeneral_horizatal_azimuth_offset_map[i];
  }

  for (int i = 0; i < HS_LIDAR_L20_UNIT_NUM; i++) {
    Pandar20_elev_angle_map_[i] = pandar20_elev_angle_map[i];
    Pandar20_horizatal_azimuth_offset_map_[i] = pandar20_horizatal_azimuth_offset_map[i];
  }

  frame_id_ = frame_id;
  tz_second_ = tz * 3600;
}

PandarGeneral_Internal::~PandarGeneral_Internal() {
  Stop();
  sem_destroy(&lidar_sem_);
  pthread_mutex_destroy(&lidar_lock_);
}

/**
 * @brief load the correction file
 * @param file The path of correction file
 */
int PandarGeneral_Internal::LoadCorrectionFile(std::string correction_content) {
  std::istringstream ifs(correction_content);

  std::string line;
  if (std::getline(ifs, line)) {  // first line "Laser id,Elevation,Azimuth"
    std::cout << "Parse Lidar Correction..." << std::endl;
  }

  double azimuthOffset[HS_LIDAR_L64_UNIT_NUM];
  double elev_angle[HS_LIDAR_L64_UNIT_NUM];

  int lineCounter = 0;
  while (std::getline(ifs, line)) {
    if (lineCounter++ >= HS_LIDAR_L64_UNIT_NUM) break;

    int lineId = 0;
    double elev, azimuth;

    std::stringstream ss(line);
    std::string subline;
    std::getline(ss, subline, ',');
    std::stringstream(subline) >> lineId;
    std::getline(ss, subline, ',');
    std::stringstream(subline) >> elev;
    std::getline(ss, subline, ',');
    std::stringstream(subline) >> azimuth;

    if (lineId != lineCounter) {
      return -1;
    }

    elev_angle[lineId - 1] = elev;
    azimuthOffset[lineId - 1] = azimuth;
  }

  for (int i = 0; i < HS_LIDAR_L64_UNIT_NUM; ++i) {
    /* for all the laser offset */
    General_elev_angle_map_[i] = elev_angle[i];
    General_horizatal_azimuth_offset_map_[i] = azimuthOffset[i];
  }

  return 0;
}

/**
 * @brief load the correction file
 * @param angle The start angle
 */
void PandarGeneral_Internal::ResetStartAngle(uint16_t start_angle) {
  start_angle_ = start_angle;
}

int PandarGeneral_Internal::Start() {
  Stop();
  enable_lidar_recv_thr_ = true;
  enable_lidar_process_thr_ = true;
  lidar_process_thr_ = new boost::thread(
      boost::bind(&PandarGeneral_Internal::ProcessLiarPacket, this));

  lidar_recv_thr_ =
      new boost::thread(boost::bind(&PandarGeneral_Internal::RecvTask, this));
}

void PandarGeneral_Internal::Stop() {
  enable_lidar_recv_thr_ = false;
  enable_lidar_process_thr_ = false;

  if (lidar_process_thr_) {
    lidar_process_thr_->join();
    delete lidar_process_thr_;
    lidar_process_thr_ = NULL;
  }

  if (lidar_recv_thr_) {
    lidar_recv_thr_->join();
    delete lidar_recv_thr_;
    lidar_recv_thr_ = NULL;
  }
  return;
}

void PandarGeneral_Internal::RecvTask() {
  int ret = 0;
  while (enable_lidar_recv_thr_) {
    PandarPacket pkt;
    int rc = input_->getPacket(&pkt);
    if (rc == -1) {
      continue;
    }

    if (pkt.size == GPS_PACKET_SIZE) {
      PandarGPS gpsMsg;
      ret = ParseGPS(&gpsMsg, pkt.data, pkt.size);
      if (ret == 0) {
        ProcessGps(gpsMsg);
      }
      continue;
    }

    PushLiDARData(pkt);
  }
}

void PandarGeneral_Internal::ProcessLiarPacket() {
  double lastTimestamp = 0.0f;
  struct timespec ts;
  int ret = 0;

  boost::shared_ptr<PPointCloud> outMsg(new PPointCloud());

  while (enable_lidar_process_thr_) {
    if (clock_gettime(CLOCK_REALTIME, &ts) == -1) {
      std::cout << "get time error" << std::endl;
    }

    ts.tv_sec += 1;
    if (sem_timedwait(&lidar_sem_, &ts) == -1) {
      continue;
    }

    pthread_mutex_lock(&lidar_lock_);
    PandarPacket packet = lidar_packets_.front();
    lidar_packets_.pop_front();
    pthread_mutex_unlock(&lidar_lock_);

    if (packet.size == PACKET_SIZE) {
      Pandar40PPacket pkt;
      ret = ParseRawData(&pkt, packet.data, packet.size);

      if (ret != 0) {
        continue;
      }

      for (int i = 0; i < BLOCKS_PER_PACKET; ++i) {
        int azimuthGap = 0; /* To do */

        if(last_azimuth_ > pkt.blocks[i].azimuth) {
          azimuthGap = static_cast<int>(pkt.blocks[i].azimuth) + (36000 - static_cast<int>(last_azimuth_));
        } else {
          azimuthGap = static_cast<int>(pkt.blocks[i].azimuth) - static_cast<int>(last_azimuth_);
        }
        
        if (last_azimuth_ != pkt.blocks[i].azimuth && 
                      azimuthGap < 600 /* 6 degree*/) {
          /* for all the blocks */
          if ((last_azimuth_ > pkt.blocks[i].azimuth &&
               start_angle_ <= pkt.blocks[i].azimuth) ||
              (last_azimuth_ < start_angle_ &&
               start_angle_ <= pkt.blocks[i].azimuth)) {
            if (pcl_callback_ && outMsg->points.size() > 0) {
              pcl_callback_(outMsg, outMsg->points[0].timestamp);
              outMsg.reset(new PPointCloud());
            }
          }
        }
        CalcPointXYZIT(&pkt, i, outMsg);
        last_azimuth_ = pkt.blocks[i].azimuth;
      }
    } else if (packet.size == HS_LIDAR_L64_6PACKET_SIZE || \
        packet.size == HS_LIDAR_L64_7PACKET_SIZE || \
        packet.size == HS_LIDAR_L64_6PACKET_WITHOUT_UDPSEQ_SIZE || \
        packet.size == HS_LIDAR_L64_7PACKET_WITHOUT_UDPSEQ_SIZE) {
      HS_LIDAR_L64_Packet pkt;
      ret = ParseL64Data(&pkt, packet.data, packet.size);

      if (ret != 0) {
        continue;
      }

      for (int i = 0; i < pkt.header.chBlockNumber; ++i) {
        int azimuthGap = 0; /* To do */
        if(last_azimuth_ > pkt.blocks[i].azimuth) {
          azimuthGap = static_cast<int>(pkt.blocks[i].azimuth) + (36000 - static_cast<int>(last_azimuth_));
        } else {
          azimuthGap = static_cast<int>(pkt.blocks[i].azimuth) - static_cast<int>(last_azimuth_);
        }
        
        if (last_azimuth_ != pkt.blocks[i].azimuth && 
                      azimuthGap < 600 /* 6 degree*/) {
          /* for all the blocks */
          if ((last_azimuth_ > pkt.blocks[i].azimuth &&
               start_angle_ <= pkt.blocks[i].azimuth) ||
              (last_azimuth_ < start_angle_ &&
               start_angle_ <= pkt.blocks[i].azimuth)) {
            if (pcl_callback_ && outMsg->points.size() > 0) {
              pcl_callback_(outMsg, outMsg->points[0].timestamp);
              outMsg.reset(new PPointCloud());
            }
          }
        } else {
          //printf("last_azimuth_:%d pkt.blocks[i].azimuth:%d  *******azimuthGap:%d\n", last_azimuth_, pkt.blocks[i].azimuth, azimuthGap);
        }

        CalcL64PointXYZIT(&pkt, i, pkt.header.chLaserNumber, outMsg);
        last_azimuth_ = pkt.blocks[i].azimuth;
      }
    } else if (HS_LIDAR_L20_PACKET_SIZE == packet.size) {
      HS_LIDAR_L20_Packet pkt;
      ret = ParseL20Data(&pkt, packet.data, packet.size);

      if (ret != 0) {
        continue;
      }

      for (int i = 0; i < pkt.header.chBlockNumber; ++i) {
        int azimuthGap = 0; /* To do */
        if(last_azimuth_ > pkt.blocks[i].azimuth) {
          azimuthGap = static_cast<int>(pkt.blocks[i].azimuth) + (36000 - static_cast<int>(last_azimuth_));
        } else {
          azimuthGap = static_cast<int>(pkt.blocks[i].azimuth) - static_cast<int>(last_azimuth_);
        }

        if (last_azimuth_ != pkt.blocks[i].azimuth && \
            azimuthGap < 600 /* 6 degree*/) {
          /* for all the blocks */
          if ((last_azimuth_ > pkt.blocks[i].azimuth &&
               start_angle_ <= pkt.blocks[i].azimuth) ||
              (last_azimuth_ < start_angle_ &&
               start_angle_ <= pkt.blocks[i].azimuth)) {
            if (pcl_callback_ && outMsg->points.size() > 0) {
              pcl_callback_(outMsg, outMsg->points[0].timestamp);
              outMsg.reset(new PPointCloud());
            }
          }
        } else {
          //printf("last_azimuth_:%d pkt.blocks[i].azimuth:%d  *******azimuthGap:%d\n", last_azimuth_, pkt.blocks[i].azimuth, azimuthGap);
        }
        CalcL20PointXYZIT(&pkt, i, pkt.header.chLaserNumber, outMsg);
        last_azimuth_ = pkt.blocks[i].azimuth;
      }
    } else {
      continue;
    }

    outMsg->header.frame_id = frame_id_;
    outMsg->height = 1;
  }
}

void PandarGeneral_Internal::PushLiDARData(PandarPacket packet) {
  pthread_mutex_lock(&lidar_lock_);
  lidar_packets_.push_back(packet);
  sem_post(&lidar_sem_);
  pthread_mutex_unlock(&lidar_lock_);
}

void PandarGeneral_Internal::ProcessGps(const PandarGPS &gpsMsg) {
  struct tm t;
  t.tm_sec = gpsMsg.second;
  t.tm_min = gpsMsg.minute;

  t.tm_hour = gpsMsg.hour;
  t.tm_mday = gpsMsg.day;

  // UTC's month start from 1, but mktime only accept month from 0.
  t.tm_mon = gpsMsg.month - 1;
  // UTC's year only include 0 - 99 year , which indicate 2000 to 2099.
  // and mktime's year start from 1900 which is 0. so we need add 100 year.
  t.tm_year = gpsMsg.year + 100;
  t.tm_isdst = 0;

  if (gps_callback_) {
    gps_callback_(static_cast<double>(mktime(&t) + tz_second_));
  }
}

int PandarGeneral_Internal::ParseRawData(Pandar40PPacket *packet,
                                     const uint8_t *buf, const int len) {
  if (len != PACKET_SIZE) {
    std::cout << "packet size mismatch PandarGeneral_Internal " << len << ","
              << PACKET_SIZE << std::endl;
    return -1;
  }

  int index = 0;
  // 10 BLOCKs
  for (int i = 0; i < BLOCKS_PER_PACKET; i++) {
    Pandar40PBlock &block = packet->blocks[i];

    block.sob = (buf[index] & 0xff) | ((buf[index + 1] & 0xff) << 8);
    block.azimuth = (buf[index + 2] & 0xff) | ((buf[index + 3] & 0xff) << 8);
    index += SOB_ANGLE_SIZE;
    // 40x units
    for (int j = 0; j < LASER_COUNT; j++) {
      Pandar40PUnit &unit = block.units[j];
      uint32_t range = (buf[index] & 0xff) | ((buf[index + 1] & 0xff) << 8);

      // distance is M.
      unit.distance =
          (static_cast<double>(range)) * LASER_RETURN_TO_DISTANCE_RATE;
      unit.intensity = (buf[index + 2] & 0xff);

      // TODO(Philip.Pi): Filtering wrong data for LiDAR.
      if ((unit.distance == 0x010101 && unit.intensity == 0x0101) || \
          unit.distance > (200 * 1000 / 2 /* 200m -> 2mm */)) {
        unit.distance = 0;
        unit.intensity = 0;
      }

      index += RAW_MEASURE_SIZE;
    }
  }

  index += RESERVE_SIZE;  // skip reserved bytes

  index += REVOLUTION_SIZE;

  packet->usec = (buf[index] & 0xff) | \
      (buf[index+1] & 0xff) << 8 | ((buf[index+2] & 0xff) << 16) | \
      ((buf[index+3] & 0xff) << 24);
  packet->usec %= 1000000;

  index += TIMESTAMP_SIZE;
  packet->echo = buf[index] & 0xff;

  index += FACTORY_INFO_SIZE + ECHO_SIZE;

  // parse the UTC Time.

  // UTC's year only include 0 - 99 year , which indicate 2000 to 2099.
  // and mktime's year start from 1900 which is 0. so we need add 100 year.
  packet->t.tm_year = (buf[index + 0] & 0xff) + 100;
  // UTC's month start from 1, but mktime only accept month from 0.
  packet->t.tm_mon = (buf[index + 1] & 0xff) - 1;
  packet->t.tm_mday = buf[index + 2] & 0xff;
  packet->t.tm_hour = buf[index + 3] & 0xff;
  packet->t.tm_min = buf[index + 4] & 0xff;
  packet->t.tm_sec = buf[index + 5] & 0xff;
  packet->t.tm_isdst = 0;

  return 0;
}

int PandarGeneral_Internal::ParseL64Data(HS_LIDAR_L64_Packet *packet,
                                const uint8_t *recvbuf, const int len) {
  if (len != HS_LIDAR_L64_6PACKET_SIZE &&
      len != HS_LIDAR_L64_7PACKET_SIZE &&
      len != HS_LIDAR_L64_6PACKET_WITHOUT_UDPSEQ_SIZE &&
      len != HS_LIDAR_L64_7PACKET_WITHOUT_UDPSEQ_SIZE &&
      len != 1270) {
    std::cout << "packet size mismatch PandarGeneral_Internal " << len << "," << \
        len << std::endl;
    return -1;
  }

  int index = 0;
  int block = 0;
  //Parse 8 Bytes Header
  packet->header.sob = (recvbuf[index] & 0xff) << 8| ((recvbuf[index+1] & 0xff));
  packet->header.chLaserNumber = recvbuf[index+2] & 0xff;
  packet->header.chBlockNumber = recvbuf[index+3] & 0xff;
  packet->header.chReturnType = recvbuf[index+4] & 0xff;
  packet->header.chDisUnit = recvbuf[index+5] & 0xff;
  index += HS_LIDAR_L64_HEAD_SIZE;

  if (packet->header.sob != 0xEEFF) {
    printf("Error Start of Packet!\n");
    return -1;
  }

  for(block = 0; block < packet->header.chBlockNumber; block++) {
    packet->blocks[block].azimuth = (recvbuf[index] & 0xff) | \
        ((recvbuf[index + 1] & 0xff) << 8);
    index += HS_LIDAR_L64_BLOCK_HEADER_AZIMUTH;

    int unit;

    for(unit = 0; unit < packet->header.chLaserNumber; unit++) {
      unsigned int unRange = (recvbuf[index]& 0xff) | ((recvbuf[index + 1]& 0xff) << 8);

      packet->blocks[block].units[unit].distance = \
          (static_cast<double>(unRange * packet->header.chDisUnit)) / (double)1000;
      packet->blocks[block].units[unit].intensity = (recvbuf[index+2]& 0xff);
      index += HS_LIDAR_L64_UNIT_SIZE;
    }
  }

  index += HS_LIDAR_L64_RESERVED_SIZE; // skip reserved bytes
  index += HS_LIDAR_L64_ENGINE_VELOCITY;

  packet->timestamp = (recvbuf[index] & 0xff)| (recvbuf[index+1] & 0xff) << 8 | \
      ((recvbuf[index+2] & 0xff) << 16) | ((recvbuf[index+3] & 0xff) << 24);
    // printf("timestamp %u \n", packet->timestamp);
  index += HS_LIDAR_L64_TIMESTAMP_SIZE;

  packet->echo = recvbuf[index]& 0xff;

  index += HS_LIDAR_L64_ECHO_SIZE;
  index += HS_LIDAR_L64_FACTORY_SIZE;
    
  packet->addtime[0] = recvbuf[index]& 0xff;
  packet->addtime[1] = recvbuf[index + 1]& 0xff;
  packet->addtime[2] = recvbuf[index + 2]& 0xff;
  packet->addtime[3] = recvbuf[index + 3]& 0xff;
  packet->addtime[4] = recvbuf[index + 4]& 0xff;
  packet->addtime[5] = recvbuf[index + 5]& 0xff;

  index += HS_LIDAR_TIME_SIZE;

  return 0;
}

int PandarGeneral_Internal::ParseL20Data(HS_LIDAR_L20_Packet *packet, \
    const uint8_t *recvbuf, const int len) {
  if (len != HS_LIDAR_L20_PACKET_SIZE) {
    std::cout << "packet size mismatch Pandar20A/B " << len << "," << \
        len << std::endl;
    return -1;
  }

  int index = 0;
  int block = 0;
  //Parse 8 Bytes Header
  packet->header.sob = (recvbuf[index] & 0xff) << 8| \
      ((recvbuf[index+1] & 0xff));
  packet->header.chLaserNumber = recvbuf[index+2] & 0xff;
  packet->header.chBlockNumber = recvbuf[index+3] & 0xff;
  packet->header.chReturnType = recvbuf[index+4] & 0xff;
  packet->header.chDisUnit = recvbuf[index+5] & 0xff;
  index += HS_LIDAR_L20_HEAD_SIZE;

  if (packet->header.sob != 0xEEFF) {
    printf("Error Start of Packet!\n");
    return -1;
  }

  for(block = 0; block < packet->header.chBlockNumber; block ++) {
    packet->blocks[block].azimuth = (recvbuf[index]& 0xff) | \
        ((recvbuf[index + 1]& 0xff) << 8);
    index += HS_LIDAR_L20_BLOCK_HEADER_AZIMUTH;

    int unit;
    for(unit = 0; unit < packet->header.chLaserNumber; unit++)
    {
      unsigned int unRange = (recvbuf[index]& 0xff) | \
          ((recvbuf[index+1]& 0xff) << 8);

      packet->blocks[block].units[unit].distance = \
          (static_cast<double>(unRange * packet->header.chDisUnit)) / 1000.0;
      packet->blocks[block].units[unit].intensity = (recvbuf[index+2] & 0xff);
      index += HS_LIDAR_L20_UNIT_SIZE;
    }
  }

  index += HS_LIDAR_L20_RESERVED_SIZE; // skip reserved bytes
  index += HS_LIDAR_L20_ENGINE_VELOCITY;

  packet->timestamp = (recvbuf[index] & 0xff) | \
      (recvbuf[index+1] & 0xff) << 8 | ((recvbuf[index+2] & 0xff) << 16) | \
      ((recvbuf[index+3] & 0xff) << 24);
  index += HS_LIDAR_L20_TIMESTAMP_SIZE;

  packet->echo = recvbuf[index] & 0xff;

  index += HS_LIDAR_L20_ECHO_SIZE;
  index += HS_LIDAR_L20_FACTORY_SIZE;
    
  packet->addtime[0] = recvbuf[index]& 0xff;
  packet->addtime[1] = recvbuf[index+1]& 0xff;
  packet->addtime[2] = recvbuf[index+2]& 0xff;
  packet->addtime[3] = recvbuf[index+3]& 0xff;
  packet->addtime[4] = recvbuf[index+4]& 0xff;
  packet->addtime[5] = recvbuf[index+5]& 0xff;

  index += HS_LIDAR_TIME_SIZE;

  return 0;
}
int PandarGeneral_Internal::ParseGPS(PandarGPS *packet, const uint8_t *recvbuf, \
    const int size) {
  if (size != GPS_PACKET_SIZE) {
    return -1;
  }
  int index = 0;
  packet->flag = (recvbuf[index] & 0xff) | ((recvbuf[index + 1] & 0xff) << 8);
  index += GPS_PACKET_FLAG_SIZE;
  packet->year =
      (recvbuf[index] & 0xff - 0x30) + (recvbuf[index + 1] & 0xff - 0x30) * 10;
  index += GPS_PACKET_YEAR_SIZE;
  packet->month =
      (recvbuf[index] & 0xff - 0x30) + (recvbuf[index + 1] & 0xff - 0x30) * 10;
  index += GPS_PACKET_MONTH_SIZE;
  packet->day =
      (recvbuf[index] & 0xff - 0x30) + (recvbuf[index + 1] & 0xff - 0x30) * 10;
  index += GPS_PACKET_DAY_SIZE;
  packet->second =
      (recvbuf[index] & 0xff - 0x30) + (recvbuf[index + 1] & 0xff - 0x30) * 10;
  index += GPS_PACKET_SECOND_SIZE;
  packet->minute =
      (recvbuf[index] & 0xff - 0x30) + (recvbuf[index + 1] & 0xff - 0x30) * 10;
  index += GPS_PACKET_MINUTE_SIZE;
  packet->hour =
      (recvbuf[index] & 0xff - 0x30) + (recvbuf[index + 1] & 0xff - 0x30) * 10;
  index += GPS_PACKET_HOUR_SIZE;
  packet->fineTime =
      (recvbuf[index] & 0xff) | (recvbuf[index + 1] & 0xff) << 8 |
      ((recvbuf[index + 2] & 0xff) << 16) | ((recvbuf[index + 3] & 0xff) << 24);

  return 0;
}

void PandarGeneral_Internal::CalcPointXYZIT(Pandar40PPacket *pkt, int blockid,
                                        boost::shared_ptr<PPointCloud> cld) {
  Pandar40PBlock *block = &pkt->blocks[blockid];

  double unix_second =
      static_cast<double>(mktime(&pkt->t) + tz_second_);

  for (int i = 0; i < LASER_COUNT; ++i) {
    /* for all the units in a block */
    Pandar40PUnit &unit = block->units[i];
    PPoint point;

    /* skip wrong points */
    if (unit.distance <= 0.1 || unit.distance > 200.0) {
      continue;
    }

    double xyDistance = unit.distance * \
        cosf(degreeToRadian(General_elev_angle_map_[i]));

    point.x = static_cast<float>(xyDistance * \
        sinf(degreeToRadian(General_horizatal_azimuth_offset_map_[i] + \
        (static_cast<double>(block->azimuth)) / 100.0)));
    point.y = static_cast<float>(xyDistance * \
        cosf(degreeToRadian(General_horizatal_azimuth_offset_map_[i] + \
        (static_cast<double>(block->azimuth)) / 100.0)));
    point.z = static_cast<float>(unit.distance * \
        sinf(degreeToRadian(General_elev_angle_map_[i])));

    point.intensity = unit.intensity;

    point.timestamp = unix_second + \
        (static_cast<double>(pkt->usec)) / 1000000.0;

    if (pkt->echo == 0x39) {
      // dual return, block 0&1 (2&3 , 4*5 ...)'s timestamp is the same.
      point.timestamp = point.timestamp - \
          (static_cast<double>(block40OffsetDual_[blockid] + \
          laser40Offset_[i]) / 1000000.0f);
    } else {
      point.timestamp = point.timestamp - \
          (static_cast<double>(block40OffsetSingle_[blockid] + \
          laser40Offset_[i]) / 1000000.0f);
    }

    point.ring = i;

    cld->push_back(point);
  }
}

void PandarGeneral_Internal::CalcL64PointXYZIT(HS_LIDAR_L64_Packet *pkt, int blockid, \
    char chLaserNumber, boost::shared_ptr<PPointCloud> cld) {
  HS_LIDAR_L64_Block *block = &pkt->blocks[blockid];

  struct tm tTm;
  // UTC's year only include 0 - 99 year , which indicate 2000 to 2099.
  // and mktime's year start from 1900 which is 0. so we need add 100 year.
  tTm.tm_year = pkt->addtime[0] + 100;
  // UTC's month start from 1, but mktime only accept month from 0.
  tTm.tm_mon = pkt->addtime[1] - 1;
  tTm.tm_mday = pkt->addtime[2];
  tTm.tm_hour = pkt->addtime[3];
  tTm.tm_min = pkt->addtime[4];
  tTm.tm_sec = pkt->addtime[5];
  tTm.tm_isdst = 0;

  double unix_second = \
      static_cast<double>(mktime(&tTm) + tz_second_);

  for (int i = 0; i < chLaserNumber; ++i) {
    /* for all the units in a block */
    HS_LIDAR_L64_Unit &unit = block->units[i];
    PPoint point;

    /* skip wrong points */
    if (unit.distance <= 0.1 || unit.distance > 200.0) {
      continue;
    }

    double xyDistance = \
        unit.distance * cosf(degreeToRadian(General_elev_angle_map_[i]));
    point.x = static_cast<float>(
        xyDistance *
        sinf(degreeToRadian(General_horizatal_azimuth_offset_map_[i] +
                            (static_cast<double>(block->azimuth)) / 100.0)));
    point.y = static_cast<float>(
        xyDistance *
        cosf(degreeToRadian(General_horizatal_azimuth_offset_map_[i] +
                            (static_cast<double>(block->azimuth)) / 100.0)));
    point.z = static_cast<float>(unit.distance *
                                 sinf(degreeToRadian(General_elev_angle_map_[i])));

    point.intensity = unit.intensity;

    point.timestamp = \
        unix_second + (static_cast<double>(pkt->timestamp)) / 1000000.0;

    if (pkt->echo == 0x39) {
      // dual return, block 0&1 (2&3 , 4*5 ...)'s timestamp is the same.
      point.timestamp =
          point.timestamp - (static_cast<double>(block64OffsetDual_[blockid] +
                                                 laser64Offset_[i]) /
                             1000000.0f);
    } else {
      point.timestamp = point.timestamp - \
          (static_cast<double>(block64OffsetSingle_[blockid] + laser64Offset_[i]) / \
          1000000.0f);
    }

    point.ring = i;

    cld->push_back(point);
  }
}

void PandarGeneral_Internal::CalcL20PointXYZIT(HS_LIDAR_L20_Packet *pkt, int blockid, \
    char chLaserNumber, boost::shared_ptr<PPointCloud> cld) {
  HS_LIDAR_L20_Block *block = &pkt->blocks[blockid];

  struct tm tTm;
  // UTC's year only include 0 - 99 year , which indicate 2000 to 2099.
  // and mktime's year start from 1900 which is 0. so we need add 100 year.
  tTm.tm_year = pkt->addtime[0] + 100;
  // UTC's month start from 1, but mktime only accept month from 0.
  tTm.tm_mon = pkt->addtime[1] - 1;
  tTm.tm_mday = pkt->addtime[2];
  tTm.tm_hour = pkt->addtime[3];
  tTm.tm_min = pkt->addtime[4];
  tTm.tm_sec = pkt->addtime[5];
  tTm.tm_isdst = 0;

  double unix_second = \
      static_cast<double>(mktime(&tTm) + tz_second_);

  for (int i = 0; i < chLaserNumber; ++i) {
    /* for all the units in a block */
    HS_LIDAR_L20_Unit &unit = block->units[i];
    PPoint point;

    /* skip wrong points */
    if (unit.distance <= 0.1 || unit.distance > 200.0) {
      continue;
    }

    double xyDistance = unit.distance * cosf(degreeToRadian(Pandar20_elev_angle_map_[i]));

    point.x = static_cast<float>(xyDistance * \
        sinf(degreeToRadian(Pandar20_horizatal_azimuth_offset_map_[i] + \
        (static_cast<double>(block->azimuth)) / 100.0)));
    point.y = static_cast<float>(xyDistance * \
        cosf(degreeToRadian(Pandar20_horizatal_azimuth_offset_map_[i] + \
        (static_cast<double>(block->azimuth)) / 100.0)));
    point.z = static_cast<float>(unit.distance * \
        sinf(degreeToRadian(Pandar20_elev_angle_map_[i])));

    point.intensity = unit.intensity;

    point.timestamp = unix_second + \
        (static_cast<double>(pkt->timestamp)) / 1000000.0;

    if (pkt->echo == 0x39) {
      // dual return, block 0&1 (2&3 , 4*5 ...)'s timestamp is the same.
      if (strcmp(frame_id_.c_str(), "Pandar20A") == 0) {
        point.timestamp = point.timestamp - \
            (static_cast<double>(block20OffsetDual_[blockid] + \
            laser20AOffset_[i]) / 1000000.0f);
      } else if (strcmp(frame_id_.c_str(), "Pandar20B") == 0) {
        point.timestamp = point.timestamp - \
            (static_cast<double>(block20OffsetDual_[blockid] + \
            laser20BOffset_[i]) / 1000000.0f);
      }
    } else {
      if (strcmp(frame_id_.c_str(), "Pandar20A") == 0) {
        point.timestamp = point.timestamp - \
            (static_cast<double>(block20OffsetSingle_[blockid] + \
            laser20AOffset_[i]) / 1000000.0f);
      } else if (strcmp(frame_id_.c_str(), "Pandar20B") == 0) {
        point.timestamp = point.timestamp - \
            (static_cast<double>(block20OffsetSingle_[blockid] + \
            laser20BOffset_[i]) / 1000000.0f);
      }
    }

    point.ring = i;

    cld->push_back(point);
  }
}
