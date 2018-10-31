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

#ifndef SRC_PANDARGENERAL_INTERNAL_H_
#define SRC_PANDARGENERAL_INTERNAL_H_

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pthread.h>
#include <semaphore.h>

#include <list>
#include <string>

#include <boost/function.hpp>

#include "pandarGeneral/point_types.h"
#include "src/input.h"

#define SOB_ANGLE_SIZE (4)
#define RAW_MEASURE_SIZE (3)
#define LASER_COUNT (40)
#define BLOCKS_PER_PACKET (10)
#define BLOCK_SIZE (RAW_MEASURE_SIZE * LASER_COUNT + SOB_ANGLE_SIZE)
#define TIMESTAMP_SIZE (4)
#define FACTORY_INFO_SIZE (1)
#define ECHO_SIZE (1)
#define RESERVE_SIZE (8)
#define REVOLUTION_SIZE (2)
#define INFO_SIZE                                                  \
  (TIMESTAMP_SIZE + FACTORY_INFO_SIZE + ECHO_SIZE + RESERVE_SIZE + \
   REVOLUTION_SIZE)
#define UTC_TIME (6)
#define PACKET_SIZE (BLOCK_SIZE * BLOCKS_PER_PACKET + INFO_SIZE + UTC_TIME)
#define LASER_RETURN_TO_DISTANCE_RATE (0.004)


/**
 * Pandar 64
 */
#define HS_LIDAR_TIME_SIZE (6)
// Each Packet have 8 byte 
#define HS_LIDAR_L64_HEAD_SIZE (8)
// Block number 6 or 7
#define HS_LIDAR_L64_BLOCK_NUMBER_6 (6)
#define HS_LIDAR_L64_BLOCK_NUMBER_7 (7)

// each block first 2 byte  is azimuth
#define HS_LIDAR_L64_BLOCK_HEADER_AZIMUTH (2)
// each block have  64 Unit
#define HS_LIDAR_L64_UNIT_NUM (64)
// each Unit have 3 byte :2 bytes(distance) + 1 byte(intensity)
#define HS_LIDAR_L64_UNIT_SIZE (3)
// total block size 194
#define HS_LIDAR_L64_BLOCK_SIZE (HS_LIDAR_L64_UNIT_SIZE * \
  HS_LIDAR_L64_UNIT_NUM + HS_LIDAR_L64_BLOCK_HEADER_AZIMUTH)

// Block tail = timestamp ( 4 bytes ) + factory num (2 bytes)
#define HS_LIDAR_L64_TIMESTAMP_SIZE (4)
#define HS_LIDAR_L64_ECHO_SIZE (1)
#define HS_LIDAR_L64_FACTORY_SIZE (1)
#define HS_LIDAR_L64_RESERVED_SIZE (8)
#define HS_LIDAR_L64_ENGINE_VELOCITY (2)

// packet body size two type
#define HS_LIDAR_L64_6_BLOCK_PACKET_BODY_SIZE (HS_LIDAR_L64_BLOCK_SIZE * \
  HS_LIDAR_L64_BLOCK_NUMBER_6)
#define HS_LIDAR_L64_7_BLOCK_PACKET_BODY_SIZE (HS_LIDAR_L64_BLOCK_SIZE * \
HS_LIDAR_L64_BLOCK_NUMBER_7)

// packet tail size 
#define HS_LIDAR_L64_PACKET_TAIL_SIZE (26)
#define HS_LIDAR_L64_PACKET_TAIL_WITHOUT_UDPSEQ_SIZE (22)

//total packet size two type,length: 1198 and 1392
#define HS_LIDAR_L64_6PACKET_SIZE ( HS_LIDAR_L64_HEAD_SIZE + \
HS_LIDAR_L64_6_BLOCK_PACKET_BODY_SIZE + HS_LIDAR_L64_PACKET_TAIL_SIZE)
#define HS_LIDAR_L64_7PACKET_SIZE ( HS_LIDAR_L64_HEAD_SIZE + \
HS_LIDAR_L64_7_BLOCK_PACKET_BODY_SIZE + HS_LIDAR_L64_PACKET_TAIL_SIZE)

#define HS_LIDAR_L64_6PACKET_WITHOUT_UDPSEQ_SIZE ( HS_LIDAR_L64_HEAD_SIZE + \
HS_LIDAR_L64_6_BLOCK_PACKET_BODY_SIZE + HS_LIDAR_L64_PACKET_TAIL_WITHOUT_UDPSEQ_SIZE)
#define HS_LIDAR_L64_7PACKET_WITHOUT_UDPSEQ_SIZE ( HS_LIDAR_L64_HEAD_SIZE + \
HS_LIDAR_L64_7_BLOCK_PACKET_BODY_SIZE + HS_LIDAR_L64_PACKET_TAIL_WITHOUT_UDPSEQ_SIZE)


#define GPS_PACKET_SIZE (512)
#define GPS_PACKET_FLAG_SIZE (2)
#define GPS_PACKET_YEAR_SIZE (2)
#define GPS_PACKET_MONTH_SIZE (2)
#define GPS_PACKET_DAY_SIZE (2)
#define GPS_PACKET_HOUR_SIZE (2)
#define GPS_PACKET_MINUTE_SIZE (2)
#define GPS_PACKET_SECOND_SIZE (2)
#define GPS_ITEM_NUM (7)

#define HesaiLidarSDK_DEFAULT_LIDAR_RECV_PORT 8080
#define HesaiLidarSDK_DEFAULT_GPS_RECV_PORT 10110

struct Pandar40PUnit_s {
  uint8_t intensity;
  double distance;
};
typedef struct Pandar40PUnit_s Pandar40PUnit;

struct Pandar40PBlock_s {
  uint16_t azimuth;
  uint16_t sob;
  Pandar40PUnit units[LASER_COUNT];
};
typedef struct Pandar40PBlock_s Pandar40PBlock;

struct Pandar40PPacket_s {
  Pandar40PBlock blocks[BLOCKS_PER_PACKET];
  struct tm t;
  uint32_t usec;
  int echo;
};
typedef struct Pandar40PPacket_s Pandar40PPacket;

/************Pandar64*******************************/
typedef struct HS_LIDAR_L64_Header_s{
    unsigned short sob;     // 0xFFEE 2bytes
    char chLaserNumber;     // laser number 1byte
    char chBlockNumber;     //block number 1byte
    char chReturnType;      // return mode 1 byte  when dual return 0-Single Return 
                            // 1-The first block is the 1 st return. 
                            // 2-The first block is the 2 nd return
    char chDisUnit;         // Distance unit, 6mm/5mm/4mm
    public:
    HS_LIDAR_L64_Header_s() {
        sob = 0;
        chLaserNumber = 0;
        chBlockNumber = 0;
        chReturnType = 0;
        chDisUnit = 0;
    }
}HS_LIDAR_L64_Header;

typedef struct HS_LIDAR_L64_Unit_s{
    double distance; // *6mm or *5mm or *4mm , 
                           // real distance =  distance * 2 mm; 
                           // max distance: (2^24 â€?1) * 2mm = 33554.43m
    unsigned short reflectivity; // reflectivity
}HS_LIDAR_L64_Unit;


typedef struct HS_LIDAR_L64_Block_s{
    unsigned short azimuth; // packet angle  ,Azimuth = RealAzimuth * 100
    HS_LIDAR_L64_Unit units[HS_LIDAR_L64_UNIT_NUM];
}HS_LIDAR_L64_Block;

typedef struct HS_LIDAR_L64_Packet_s{
    HS_LIDAR_L64_Header header;
    HS_LIDAR_L64_Block blocks[HS_LIDAR_L64_BLOCK_NUMBER_7];
    unsigned int upcavity_temp;
    unsigned int downcavity_temp[4];
    unsigned char status_id;
    unsigned short status_data;
    unsigned char reserved;
    unsigned short error_per_second;
    unsigned short engine_velocity; // real velocity = <engine_velocity> * 6/11 round/min
    unsigned int timestamp; // ms
    unsigned int echo;
    unsigned char factory;
    unsigned char packersizeflag;
    unsigned char addtime[6];
    unsigned int udpSequence;
}HS_LIDAR_L64_Packet;
/***************Pandar64****************************/

struct PandarGPS_s {
  uint16_t flag;
  uint16_t year;
  uint16_t month;
  uint16_t day;
  uint16_t second;
  uint16_t minute;
  uint16_t hour;
  uint32_t fineTime;
};
typedef struct PandarGPS_s PandarGPS;

#define ROTATION_MAX_UNITS (36001)

namespace apollo {
namespace drivers {
namespace hesai {

class PandarGeneral_Internal {
 public:
  /**
   * @brief Constructor
   * @param device_ip  				The ip of the device
   *        lidar_port 				The port number of lidar data
   *        gps_port   				The port number of gps data
   *        pcl_callback      The callback of PCL data structure
   *        gps_callback      The callback of GPS structure
   *        type       				The device type
   */
  PandarGeneral_Internal(
      std::string device_ip, uint16_t lidar_port, uint16_t gps_port,
      boost::function<void(boost::shared_ptr<PPointCloud>, double)>
          pcl_callback,
      boost::function<void(double)> gps_callback, uint16_t start_angle, int tz,
      std::string frame_id);

  /**
   * @brief load the correction file
   * @param correction The path of correction file
   */
  int LoadCorrectionFile(std::string correction);

  /**
   * @brief load the correction file
   * @param angle The start angle
   */
  void ResetStartAngle(uint16_t start_angle);

  ~PandarGeneral_Internal();

  int Start();
  void Stop();

 private:
  void RecvTask();
  void ProcessGps(const PandarGPS &gpsMsg);
  void ProcessLiarPacket();
  void PushLiDARData(PandarPacket packet);
  int ParseRawData(Pandar40PPacket *packet, const uint8_t *buf, const int len);
  int ParseL64Data(HS_LIDAR_L64_Packet *packet, const uint8_t *recvbuf, const int len);
  int ParseGPS(PandarGPS *packet, const uint8_t *recvbuf, const int size);
  void CalcPointXYZIT(Pandar40PPacket *pkt, int blockid,
                      boost::shared_ptr<PPointCloud> cld);
  void CalcL64PointXYZIT(HS_LIDAR_L64_Packet *pkt, int blockid, char chLaserNumber,
                      boost::shared_ptr<PPointCloud> cld);

  pthread_mutex_t lidar_lock_;
  sem_t lidar_sem_;
  boost::thread *lidar_recv_thr_;
  boost::thread *lidar_process_thr_;
  bool enable_lidar_recv_thr_;
  bool enable_lidar_process_thr_;
  int start_angle_;

  std::list<struct PandarPacket_s> lidar_packets_;

  boost::shared_ptr<Input> input_;
  boost::function<void(boost::shared_ptr<PPointCloud> cld, double timestamp)>
      pcl_callback_;
  boost::function<void(double timestamp)> gps_callback_;

  float sin_lookup_table_[ROTATION_MAX_UNITS];
  float cos_lookup_table_[ROTATION_MAX_UNITS];

  uint16_t last_azimuth_;

  float elev_angle_map_[LASER_COUNT];
  float horizatal_azimuth_offset_map_[LASER_COUNT];

  float General_elev_angle_map_[HS_LIDAR_L64_UNIT_NUM];
  float General_horizatal_azimuth_offset_map_[HS_LIDAR_L64_UNIT_NUM];

  float block64Offset_[HS_LIDAR_L64_UNIT_NUM];
  float laser64Offset_[HS_LIDAR_L64_UNIT_NUM];

  float block40Offset_[LASER_COUNT];
  float laser40Offset_[LASER_COUNT];

  int tz_second_;
  std::string frame_id_;
};

}  // namespace hesai
}  // namespace drivers
}  // namespace apollo

#endif  // SRC_PANDARGENERAL_INTERNAL_H_
