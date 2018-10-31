/******************************************************************************
 * Copyright 2018 The Apollo Authors. All Rights Reserved.
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

namespace apollo {
namespace drivers {
namespace hesai {

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

double degreeToRadian(double degree) { return degree * M_PI / 180; }

// elevation angle of each line for HS Line 40 Lidar, Line 1 - Line 40
static const float pandar40p_elev_angle_map[] = {
    6.96,   5.976,  4.988,   3.996,   2.999,   2.001,  1.667,   1.333,
    1.001,  0.667,  0.333,   0,       -0.334,  -0.667, -1.001,  -1.334,
    -1.667, -2.001, -2.331,  -2.667,  -3,      -3.327, -3.663,  -3.996,
    -4.321, -4.657, -4.986,  -5.311,  -5.647,  -5.974, -6.957,  -7.934,
    -8.908, -9.871, -10.826, -11.772, -12.705, -13.63, -14.543, -15.444};

  static const float pandarGeneral_elev_angle_map[] = {
    14.882,   11.032,   8.059,    5.057,    3.04,     2.028,    1.86,     1.688,
    1.522,    1.351,    1.184,    1.013,    0.846,    0.675,    0.508,    0.337,
    0.169,    0,        -0.169,   -0.337,   -0.508,   -0.675,   -0.845,   -1.013,
    -1.184,   -1.351,   -1.522,   -1.688,   -1.86,    -2.028,   -2.198,   -2.365,
    -2.536,   -2.7,     -2.873,   -3.04,    -3.21,    -3.375,    -3.548,  -3.712,
    -3.884,   -4.05,    -4.221,   -4.385,   -4.558,   -4.72,     -4.892,  -5.057,
    -5.229,   -5.391,   -5.565,   -5.726,   -5.898,   -6.061,    -7.063,  -8.059,
    -9.06,    -9.885,   -11.032,  -12.006,  -12.974,  -13.93,    -18.889, -24.897};

// Line 40 Lidar azimuth Horizatal offset ,  Line 1 - Line 40
static const float pandar40p_horizatal_azimuth_offset_map[] = {
    0.005,  0.006,  0.006,  0.006,  -2.479, -2.479, 2.491,  -4.953,
    -2.479, 2.492,  -4.953, -2.479, 2.492,  -4.953, 0.007,  2.491,
    -4.953, 0.006,  4.961,  -2.479, 0.006,  4.96,   -2.478, 0.006,
    4.958,  -2.478, 2.488,  4.956,  -2.477, 2.487,  2.485,  2.483,
    0.004,  0.004,  0.003,  0.003,  -2.466, -2.463, -2.46,  -2.457};

static const float pandarGeneral_horizatal_azimuth_offset_map[] = {
    -1.042,   -1.042,    -1.042,    -1.042,    -1.042,   -1.042,    1.042,    3.125,
    5.208,    -5.208,    -3.125,    -1.042,    1.042,    3.125,     5.208,    -5.208,
    -3.125,   -1.042,    1.042,     3.125,     5.208,    -5.208,    -3.125,   -1.042,
    1.042,    3.125,     5.208,     -5.208,    -3.125,   -1.042,    1.042,    3.125,
    5.208,    -5.208,    -3.125,    -1.042,    1.042,    3.125,     5.208,    -5.208,
    -3.125,   -1.042,    1.042,     3.125,     5.208,    -5.208,    -3.125,   -1.042,
    1.042,    3.125,     5.208,     -5.208,    -3.125,   -1.042,    -1.042,   -1.042,
    -1.042,   -1.042,    -1.042,    -1.042,    -1.042,   -1.042,    -1.042,   -1.042};

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
  block40Offset_[9] = 55.1f * 0.0 ;
  block40Offset_[8] = 55.1f * 1.0 ;
  block40Offset_[7] = 55.1f * 2.0 ;
  block40Offset_[6] = 55.1f * 3.0 ;
  block40Offset_[5] = 55.1f * 4.0 ;
  block40Offset_[4] = 55.1f * 5.0 ;
  block40Offset_[3] = 55.1f * 6.0 ;
  block40Offset_[2] = 55.1f * 7.0 ;
  block40Offset_[1] = 55.1f * 8.0 ;
  block40Offset_[0] = 55.1f * 9.0 ;

  // init the laser shot time offset, us
  laser40Offset_[3] = 0.93f * 1.0f;
  laser40Offset_[35] = 0.93f * 2.0f;
  laser40Offset_[39] = 0.93f * 3.0f;
  laser40Offset_[23] = 0.93f * 3.0f + 1.6f * 1.0f;
  laser40Offset_[16] = 0.93f * 3.0f + 1.6f * 2.0f;
  laser40Offset_[27] = 0.93f * 4.0f + 1.6f * 2.0f;
  laser40Offset_[11] = 0.93f * 4.0f + 1.6f * 3.0f;
  laser40Offset_[31] = 0.93f * 5.0f + 1.6f * 3.0f;
  laser40Offset_[28] = 0.93f * 6.0f + 1.6f * 3.0f;
  laser40Offset_[15] = 0.93f * 6.0f + 1.6f * 4.0f;
  laser40Offset_[2] = 0.93f * 7.0f + 1.6f * 4.0f;
  laser40Offset_[34] = 0.93f * 8.0f + 1.6f * 4.0f;
  laser40Offset_[38] = 0.93f * 9.0f + 1.6f * 4.0f;
  laser40Offset_[20] = 0.93f * 9.0f + 1.6f * 5.0f;
  laser40Offset_[13] = 0.93f * 9.0f + 1.6f * 6.0f;
  laser40Offset_[24] = 0.93f * 9.0f + 1.6f * 7.0f;
  laser40Offset_[8] = 0.93f * 9.0f + 1.6f * 8.0f;
  laser40Offset_[30] = 0.93f * 10.0f + 1.6f * 8.0f;
  laser40Offset_[25] = 0.93f * 11.0f + 1.6f * 8.0f;
  laser40Offset_[12] = 0.93f * 11.0f + 1.6f * 9.0f;
  laser40Offset_[1] = 0.93f * 12.0f + 1.6f * 9.0f;
  laser40Offset_[33] = 0.93f * 13.0f + 1.6f * 9.0f;
  laser40Offset_[37] = 0.93f * 14.0f + 1.6f * 9.0f;
  laser40Offset_[17] = 0.93f * 14.0f + 1.6f * 10.0f;
  laser40Offset_[10] = 0.93f * 14.0f + 1.6f * 11.0f;
  laser40Offset_[21] = 0.93f * 14.0f + 1.6f * 12.0f;
  laser40Offset_[5] = 0.93f * 14.0f + 1.6f * 13.0f;
  laser40Offset_[29] = 0.93f * 15.0f + 1.6f * 13.0f;
  laser40Offset_[22] = 0.93f * 15.0f + 1.6f * 14.0f;
  laser40Offset_[9] = 0.93f * 15.0f + 1.6f * 15.0f;
  laser40Offset_[0] = 0.93f * 16.0f + 1.6f * 15.0f;
  laser40Offset_[32] = 0.93f * 17.0f + 1.6f * 15.0f;
  laser40Offset_[36] = 0.93f * 18.0f + 1.6f * 15.0f;
  laser40Offset_[14] = 0.93f * 18.0f + 1.6f * 16.0f;
  laser40Offset_[7] = 0.93f * 18.0f + 1.6f * 17.0f;
  laser40Offset_[18] = 0.93f * 18.0f + 1.6f * 18.0f;
  laser40Offset_[4] = 0.93f * 19.0f + 1.6f * 18.0f;
  laser40Offset_[26] = 0.93f * 20.0f + 1.6f * 18.0f;
  laser40Offset_[19] = 0.93f * 20.0f + 1.6f * 19.0f;
  laser40Offset_[6] = 0.93f * 20.0f + 1.6f * 20.0f;

  //laser64 init the laser shot time offset, us
  // init the block time offset, us

  block64Offset_[5] = 55.56f * 0.0;
  block64Offset_[4] = 55.56f * 1.0 ;
  block64Offset_[3] = 55.56f * 2.0 ;
  block64Offset_[2] = 55.56f * 3.0 ;
  block64Offset_[1] = 55.56f * 4.0 ;
  block64Offset_[0] = 55.56f * 5.0 ;

  laser64Offset_ [11]= 112.0f * 0.008f;//6
  laser64Offset_ [39]= 112.0f * 0.008f;//B5
  laser64Offset_ [17]= 358.0f * 0.008f;//7
  laser64Offset_ [37]= 358.0f * 0.008f;//B21 
  laser64Offset_ [10]= 604.0f * 0.008f;//8
  laser64Offset_ [25]= 604.0f * 0.008f;//B19
  laser64Offset_ [16]= 850.0f * 0.008f;//9
  laser64Offset_ [31]= 850.0f * 0.008f;//B20
  laser64Offset_ [22]= 1096.0f * 0.008f;//10
  laser64Offset_ [7]= 1096.0f * 0.008f;//B16
  laser64Offset_ [28]= 1342.0f * 0.008f;//11
  laser64Offset_ [13]= 1342.0f * 0.008f;//B17
  laser64Offset_ [34]= 1588.0f * 0.008f;//12
  laser64Offset_ [19]= 1588.0f * 0.008f;//B18
  laser64Offset_ [12]= 1834.0f * 0.008f;//17
  laser64Offset_ [27]= 1834.0f * 0.008f;//B3
  laser64Offset_ [18]= 2080.0f * 0.008f;//18
  laser64Offset_ [33]= 2080.0f * 0.008f;//B4
  laser64Offset_ [24]= 2326.0f * 0.008f;//19
  laser64Offset_ [9]= 2326.0f * 0.008f;//B0
  laser64Offset_ [30]= 2572.0f * 0.008f;//20
  laser64Offset_ [15]= 2572.0f * 0.008f;//B1
  laser64Offset_ [36]= 2818.0f * 0.008f;//21
  laser64Offset_ [21]= 2818.0f * 0.008f;//B2
  laser64Offset_ [14]= 3064.0f * 0.008f;//25
  laser64Offset_ [29]= 3064.0f * 0.008f;//B9
  laser64Offset_ [20]= 3310.0f * 0.008f;//26
  laser64Offset_ [35]= 3310.0f * 0.008f;//B10
  laser64Offset_ [26]= 3556.0f * 0.008f;//27
  laser64Offset_ [41]= 3556.0f * 0.008f;//B11
  laser64Offset_ [32]= 3802.0f * 0.008f;//28
  laser64Offset_ [23]= 3802.0f * 0.008f;//B8
  laser64Offset_ [0]= 4048.0f * 0.008f;//0
  laser64Offset_ [43]= 4048.0f * 0.008f;//B22
  laser64Offset_ [1]= 4211.0f * 0.008f;//1
  laser64Offset_ [45]= 4211.0f * 0.008f;//B6
  laser64Offset_ [2]= 4374.0f * 0.008f;//2
  laser64Offset_ [51]= 4374.0f * 0.008f;//B7
  laser64Offset_ [3]= 4537.0f * 0.008f;//3
  laser64Offset_ [49]= 4537.0f * 0.008f;//B23
  laser64Offset_ [4]= 4700.0f * 0.008f ;//4
  laser64Offset_ [47]= 4700.0f * 0.008f;//B12
  laser64Offset_ [5]= 4863.0f * 0.008f;//5
  laser64Offset_ [53]= 4863.0f * 0.008f;//B13
  laser64Offset_ [40]= 5026.0f * 0.008f;//13
  laser64Offset_ [57]= 5026.0f * 0.008f;//B25
  laser64Offset_ [46]= 5189.0f * 0.008f;//14
  laser64Offset_ [61]= 5189.0f * 0.008f;//B29
  laser64Offset_ [52]= 5352.0f * 0.008f;//15
  laser64Offset_ [63]= 5352.0f * 0.008f;//B31
  laser64Offset_ [8]= 5515.0f * 0.008f;//16
  laser64Offset_ [55]= 5515.0f * 0.008f;//B15
  laser64Offset_ [42]= 5678.0f * 0.008f;//22
  laser64Offset_ [58]= 5678.0f * 0.008f;//B26
  laser64Offset_ [48]= 5841.0f * 0.008f;//23
  laser64Offset_ [62]= 5841.0f * 0.008f;//B30
  laser64Offset_ [8]= 6004.0f * 0.008f;//24
  laser64Offset_ [54]= 6004.0f * 0.008f;//B14
  laser64Offset_ [38]= 6167.0f * 0.008f;//29
  laser64Offset_ [56]= 6167.0f * 0.008f;//B24
  laser64Offset_ [44]= 6330.0f * 0.008f;//30
  laser64Offset_ [59]= 6330.0f * 0.008f;//B27
  laser64Offset_ [50]= 6493.0f * 0.008f;//31
  laser64Offset_ [60]= 6493.0f * 0.008f;//B28

  for (int i = 0; i < LASER_COUNT; ++i) {
    // for all the laser offset 
    elev_angle_map_[i] = pandar40p_elev_angle_map[i];
    horizatal_azimuth_offset_map_[i] =
        pandar40p_horizatal_azimuth_offset_map[i];
  }

  for (int i = 0; i < HS_LIDAR_L64_UNIT_NUM; ++i) {
    // for all the laser offset 
    General_elev_angle_map_[i] = pandarGeneral_elev_angle_map[i];
    General_horizatal_azimuth_offset_map_[i] =
        pandarGeneral_horizatal_azimuth_offset_map[i];
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
    } else if (packet.size == HS_LIDAR_L64_6PACKET_SIZE ||
      packet.size == HS_LIDAR_L64_7PACKET_SIZE ||
      packet.size == HS_LIDAR_L64_6PACKET_WITHOUT_UDPSEQ_SIZE ||
      packet.size == HS_LIDAR_L64_7PACKET_WITHOUT_UDPSEQ_SIZE ||
      packet.size == 1270) {
      HS_LIDAR_L64_Packet pkt;
      ret = ParseL64Data(&pkt, packet.data, packet.size);
      if (ret != 0) {
        continue;
      }

      //printf("pkt.header.chBlockNumber:%d, chLaserNumber:%d\n", pkt.header.chBlockNumber, pkt.header.chLaserNumber);

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
      if ((unit.distance == 0x010101 && unit.intensity == 0x0101) ||
          unit.distance > (200 * 1000 / 2 /* 200m -> 2mm */)) {
        unit.distance = 0;
        unit.intensity = 0;
      }
      index += RAW_MEASURE_SIZE;
    }
  }

  index += RESERVE_SIZE;  // skip reserved bytes

  index += REVOLUTION_SIZE;

  packet->usec = (buf[index] & 0xff) | (buf[index + 1] & 0xff) << 8 |
                 ((buf[index + 2] & 0xff) << 16) |
                 ((buf[index + 3] & 0xff) << 24);
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
    std::cout << "packet size mismatch PandarGeneral_Internal " << len << ","
              << len << std::endl;
    return -1;
  }

  int index = 0;
  int block = 0;
  //Parse 8 Bytes Header
  packet->header.sob =  (recvbuf[index] & 0xff) << 8| ((recvbuf[index + 1] & 0xff));
  packet->header.chLaserNumber = recvbuf[index + 2] & 0xff;
  packet->header.chBlockNumber = recvbuf[index + 3] & 0xff;
  packet->header.chReturnType = recvbuf[index + 4] & 0xff;
  packet->header.chDisUnit = recvbuf[index + 5] & 0xff;
  index += HS_LIDAR_L64_HEAD_SIZE;

  if (packet->header.sob != 0xEEFF) {
      printf("Error Start of Packet!\n");
      return -1;
  }

  for(block = 0 ; block < packet->header.chBlockNumber ; block ++)
  {
      packet->blocks[block].azimuth = (recvbuf[index]& 0xff) | ((recvbuf[index + 1]& 0xff) << 8);
      index += HS_LIDAR_L64_BLOCK_HEADER_AZIMUTH;

      // 64x or 40x units
      int unit;
      for(unit = 0 ; unit < packet->header.chLaserNumber ; unit++)
      {
          unsigned int unRange = (recvbuf[index]& 0xff) | ((recvbuf[index + 1]& 0xff) << 8);

          packet->blocks[block].units[unit].distance = 
          (static_cast<double>(unRange * packet->header.chDisUnit)) / (double)1000;
          packet->blocks[block].units[unit].reflectivity = (recvbuf[index + 2]& 0xff);
          index += HS_LIDAR_L64_UNIT_SIZE;
      }
  }

  packet->downcavity_temp[0] = (recvbuf[index + 3] & 0xff) ;
  packet->downcavity_temp[1] = (recvbuf[index + 4] & 0xff) ;

  packet->status_id = recvbuf[index + 2]& 0xff;
  packet->status_data = (recvbuf[index]& 0xff) | ((recvbuf[index + 1]& 0xff) << 8);

  packet->error_per_second = (recvbuf[index + 6] & 0xff) | (recvbuf[index + 7]& 0xff) << 8;

  index += HS_LIDAR_L64_RESERVED_SIZE; // skip reserved bytes

  packet->engine_velocity = (recvbuf[index]& 0xff)| (recvbuf[index + 1]& 0xff) << 8;
  index += HS_LIDAR_L64_ENGINE_VELOCITY;
    // printf("speed %d\n", packet->engine_velocity * 6 /11);

  packet->timestamp = (recvbuf[index]& 0xff)| (recvbuf[index + 1]& 0xff) << 8 |
                        ((recvbuf[index + 2 ]& 0xff) << 16) | ((recvbuf[index + 3]& 0xff) << 24);
    // printf("timestamp %u \n", packet->timestamp);
  index += HS_LIDAR_L64_TIMESTAMP_SIZE;

  packet->echo = recvbuf[index]& 0xff;
  packet->factory = recvbuf[index + 1]& 0xff;

  index += HS_LIDAR_L64_ECHO_SIZE;
  index += HS_LIDAR_L64_FACTORY_SIZE;
    
  packet->addtime[0] = recvbuf[index]& 0xff;
  packet->addtime[1] = recvbuf[index + 1]& 0xff;
  packet->addtime[2] = recvbuf[index + 2]& 0xff;
  packet->addtime[3] = recvbuf[index + 3]& 0xff;
  packet->addtime[4] = recvbuf[index + 4]& 0xff;
  packet->addtime[5] = recvbuf[index + 5]& 0xff;

  packet->packersizeflag = 1;
  index += HS_LIDAR_TIME_SIZE;

  packet->udpSequence = (recvbuf[index]& 0xff)| (recvbuf[index + 1]& 0xff) << 8 |
        ((recvbuf[index + 2 ]& 0xff) << 16) | ((recvbuf[index + 3]& 0xff) << 24);

  return 0;
}

int PandarGeneral_Internal::ParseGPS(PandarGPS *packet, const uint8_t *recvbuf,
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
#ifdef DEBUG
  if (packet->year != 18) {
    printf("error gps\n");
    char str[128];
    int fd = open("/var/tmp/error_gps.txt", O_RDWR | O_CREAT, 0666);
    lseek(fd, 0, SEEK_END);
    int i = 0;
    for (i = 0; i < 512; i++) {
      snprintf(str, "%02x ", recvbuf[i], 127);
      write(fd, str, strlen(str));
    }
    write(fd, "\n", 1);
    close(fd);
  }
#endif
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
    if (unit.distance <= 0.5 || unit.distance > 200.0) {
      continue;
    }

    double xyDistance =
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

    point.timestamp =
        unix_second + (static_cast<double>(pkt->usec)) / 1000000.0;

    if (pkt->echo == 0x39) {
      // dual return, block 0&1 (2&3 , 4*5 ...)'s timestamp is the same.
      point.timestamp = \
          point.timestamp - (static_cast<double>(block40Offset_[blockid / 2] + \
          laser40Offset_[i]) / 1000000.0f);
    } else {
      point.timestamp =
          point.timestamp -
          (static_cast<double>(block40Offset_[blockid] + laser40Offset_[i]) /
           1000000.0f);
    }

    point.ring = i;

    cld->push_back(point);
  }
}

void PandarGeneral_Internal::CalcL64PointXYZIT(HS_LIDAR_L64_Packet *pkt, int blockid,
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

  double unix_second =
      static_cast<double>(mktime(&tTm) + tz_second_);

  for (int i = 0; i < chLaserNumber; ++i) {
    /* for all the units in a block */
    HS_LIDAR_L64_Unit &unit = block->units[i];
    PPoint point;

    /* skip wrong points */
    if (unit.distance <= 0.5 || unit.distance > 200.0) {
      continue;
    }

    double xyDistance =
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

    point.intensity = unit.reflectivity;

    point.timestamp =
        unix_second + (static_cast<double>(pkt->timestamp)) / 1000000.0;

    if (pkt->echo == 0x39) {
      // dual return, block 0&1 (2&3 , 4*5 ...)'s timestamp is the same.
      point.timestamp =
          point.timestamp - (static_cast<double>(block64Offset_[blockid / 2] +
                                                 laser64Offset_[i]) /
                             1000000.0f);
    } else {
      point.timestamp =
          point.timestamp -
          (static_cast<double>(block64Offset_[blockid] + laser64Offset_[i]) /
           1000000.0f);
    }

    point.ring = i;

    cld->push_back(point);
  }
}

}  // namespace hesai
}  // namespace drivers
}  // namespace apollo
