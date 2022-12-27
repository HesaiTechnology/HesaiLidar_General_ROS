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
#include "src/pandarQT.h"
#include "src/pandarXT.h"
#include "src/pcap_reader.h"
#include "hesai_lidar/PandarScan.h"
#include "hesai_lidar/PandarPacket.h"
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <Eigen/Dense>
#define MAX_ITERATOR_DIFF (400)
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
#define INFO_SIZE (TIMESTAMP_SIZE + FACTORY_INFO_SIZE + ECHO_SIZE + \
    RESERVE_SIZE + REVOLUTION_SIZE)
#define UTC_TIME (6)
#define PACKET_SIZE (BLOCK_SIZE * BLOCKS_PER_PACKET + INFO_SIZE + UTC_TIME)
#define LASER_RETURN_TO_DISTANCE_RATE (0.004)
#define SEQ_NUM_SIZE (4)

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

/**
 * Pandar20
 */
#define HS_LIDAR_L20_HEAD_SIZE (8)
#define HS_LIDAR_L20_BLOCK_NUMBER (20)
#define HS_LIDAR_L20_BLOCK_HEADER_AZIMUTH (2)
#define HS_LIDAR_L20_UNIT_NUM (20)
#define HS_LIDAR_L20_UNIT_SIZE (3)
#define HS_LIDAR_L20_BLOCK_SIZE (HS_LIDAR_L20_UNIT_SIZE * \
    HS_LIDAR_L20_UNIT_NUM + HS_LIDAR_L20_BLOCK_HEADER_AZIMUTH)
#define HS_LIDAR_L20_TIMESTAMP_SIZE (4)
#define HS_LIDAR_L20_ECHO_SIZE (1)
#define HS_LIDAR_L20_FACTORY_SIZE (1)
#define HS_LIDAR_L20_RESERVED_SIZE (8)
#define HS_LIDAR_L20_ENGINE_VELOCITY (2)
#define HS_LIDAR_L20_BLOCK_PACKET_BODY_SIZE (HS_LIDAR_L20_BLOCK_SIZE * \
    HS_LIDAR_L20_BLOCK_NUMBER)
#define HS_LIDAR_L20_PACKET_TAIL_SIZE (22)
#define HS_LIDAR_L20_PACKET_SIZE ( HS_LIDAR_L20_HEAD_SIZE + \
    HS_LIDAR_L20_BLOCK_PACKET_BODY_SIZE + HS_LIDAR_L20_PACKET_TAIL_SIZE)


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

#define MAX_LASER_NUM (256)
#define MAX_POINT_CLOUD_NUM (1000000)
#define MAX_POINT_CLOUD_NUM_PER_CHANNEL (10000)
#define MAX_AZIMUTH_DEGREE_NUM (36000)
#define HS_LIDAR_XT_COORDINATE_CORRECTION_H (0.0315)
#define HS_LIDAR_XT_COORDINATE_CORRECTION_B (0.013)
#define HS_LIDAR_XTM_COORDINATE_CORRECTION_H (0.0305)
#define HS_LIDAR_XTM_COORDINATE_CORRECTION_B (0.013)
#define HS_LIDAR_QT_COORDINATE_CORRECTION_ODOG (0.0298)
#define HS_LIDAR_QT_COORDINATE_CORRECTION_ODOT (0.0072)
#define HS_LIDAR_QT_COORDINATE_CORRECTION_F (0.0295)
#define HS_LIDAR_QT_COORDINATE_CORRECTION_I0 (0.0006)
#define HS_LIDAR_QT_COORDINATE_CORRECTION_S0 (0.00017)
#define HS_LIDAR_QT_COORDINATE_CORRECTION_D0 (20)
#define COORDINATE_CORRECTION_CHECK (false)

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
  double timestamp_point;
  float spin_speed;
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
} HS_LIDAR_L64_Header;

typedef struct HS_LIDAR_L64_Unit_s{
    double distance;
    unsigned short intensity;
} HS_LIDAR_L64_Unit;


typedef struct HS_LIDAR_L64_Block_s{
    unsigned short azimuth; // packet angle  ,Azimuth = RealAzimuth * 100
    HS_LIDAR_L64_Unit units[HS_LIDAR_L64_UNIT_NUM];
} HS_LIDAR_L64_Block;

typedef struct HS_LIDAR_L64_Packet_s{
    HS_LIDAR_L64_Header header;
    HS_LIDAR_L64_Block blocks[HS_LIDAR_L64_BLOCK_NUMBER_7];
    unsigned int timestamp; // ms
    unsigned int echo;
    unsigned char addtime[6];
    double timestamp_point;
    float spin_speed;
} HS_LIDAR_L64_Packet;
/***************Pandar64****************************/

/************Pandar20A/B*******************************/
typedef struct HS_LIDAR_L20_Header_s{
    unsigned short sob;     // 0xFFEE 2bytes
    char chLaserNumber;     // laser number 1byte
    char chBlockNumber;     //block number 1byte
    char chReturnType;      // return mode 1 byte  when dual return 0-Single Return 
                            // 1-The first block is the 1 st return. 
                            // 2-The first block is the 2 nd return
    char chDisUnit;         // Distance unit, 6mm/5mm/4mm
    public:
    HS_LIDAR_L20_Header_s() {
        sob = 0;
        chLaserNumber = 0;
        chBlockNumber = 0;
        chReturnType = 0;
        chDisUnit = 0;
    }
} HS_LIDAR_L20_Header;

typedef struct HS_LIDAR_L20_Unit_s{
    double distance;
    unsigned short intensity;
} HS_LIDAR_L20_Unit;


typedef struct HS_LIDAR_L20_Block_s{
    unsigned short azimuth;
    HS_LIDAR_L20_Unit units[HS_LIDAR_L20_UNIT_NUM];
} HS_LIDAR_L20_Block;

typedef struct HS_LIDAR_L20_Packet_s{
    HS_LIDAR_L20_Header header;
    HS_LIDAR_L20_Block blocks[HS_LIDAR_L20_BLOCK_NUMBER];
    unsigned int timestamp; // ms
    unsigned int echo;
    unsigned char addtime[6];
    double timestamp_point;
    float spin_speed;
} HS_LIDAR_L20_Packet;
/************Pandar20A/B*******************************/


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

typedef std::array<PandarPacket, 36000> PktArray;

typedef struct PacketsBuffer_s {
  PktArray m_buffers{};
  PktArray::iterator m_iterPush;
  PktArray::iterator m_iterCalc;
  bool m_startFlag;
  inline PacketsBuffer_s() {
    m_iterPush = m_buffers.begin();
    m_iterCalc = m_buffers.begin();
    m_startFlag = false;
  }
  inline int push_back(PandarPacket pkt) {
    if (!m_startFlag) {
      *m_iterPush = pkt;
      m_startFlag = true;
      return 1;
    } 
    m_iterPush++;

    if(((m_iterPush - m_iterCalc) > MAX_ITERATOR_DIFF) ||
    ((m_iterPush < m_iterCalc) && (m_iterCalc - m_iterPush) < m_buffers.size() - MAX_ITERATOR_DIFF)){

      while((((m_iterPush - m_iterCalc) > MAX_ITERATOR_DIFF) ||
      ((m_iterPush < m_iterCalc) && (m_iterCalc - m_iterPush) < m_buffers.size() - MAX_ITERATOR_DIFF)))
        usleep(1000);
    }

    if (m_iterPush == m_iterCalc) {
      printf("buffer don't have space!,%d\n", m_iterPush - m_buffers.begin());
      return 0;
    }

    if (m_buffers.end() == m_iterPush) {
      m_iterPush = m_buffers.begin();
      *m_iterPush = pkt;
    }
    *m_iterPush = pkt;
    return 1;
    
  }
  inline bool hasEnoughPackets() {
    return ((m_iterPush - m_iterCalc > 0 ) ||
            ((m_iterPush - m_iterCalc + 36000 > 0 ) && (m_buffers.end() - m_iterCalc < 1000) && (m_iterPush - m_buffers.begin() < 1000)));
  }
  inline PktArray::iterator getIterCalc() { return m_iterCalc;}
  inline void moveIterCalc() {
    m_iterCalc++;
    if (m_buffers.end() == m_iterCalc) {
      m_iterCalc = m_buffers.begin();
    }
  }
} PacketsBuffer;

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
      boost::function<void(boost::shared_ptr<PPointCloud>, double, hesai_lidar::PandarScanPtr)>
          pcl_callback, boost::function<void(double)> gps_callback, 
          uint16_t start_angle, int tz, int pcl_type, std::string lidar_type, std::string frame_id, std::string timestampType, // the default timestamp type is LiDAR time
          std::string lidar_correction_file, std::string multicast_ip, bool coordinate_correction_flag,
          std::string target_frame, std::string fixed_frame);

  /**
   * @brief Constructor
   * @param pcap_path         The path of pcap file
   *        pcl_callback      The callback of PCL data structure
   *        start_angle       The start angle of frame
   *        tz                The timezone
   *        pcl_type          Structured Pointcloud
   *        frame_id          The frame id of pcd
   */
  PandarGeneral_Internal(
      std::string pcap_path, \
      boost::function<void(boost::shared_ptr<PPointCloud>, double, hesai_lidar::PandarScanPtr)> \
      pcl_callback, uint16_t start_angle, int tz, int pcl_type, \
      std::string lidar_type, std::string frame_id, std::string timestampType, // the default timestamp type is LiDAR time
      std::string lidar_correction_file, bool coordinate_correction_flag, \
      std::string target_frame, std::string fixed_frame);
  ~PandarGeneral_Internal();

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

  void Start();
  void Stop();
  void PushScanPacket(hesai_lidar::PandarScanPtr scan);
  bool GetCorrectionFileFlag();
  void SetCorrectionFileFlag(bool flag);

 private:
  void Init();
  void RecvTask();
  void ProcessGps(const PandarGPS &gpsMsg);
  void ProcessLiarPacket();
  void PushLiDARData(PandarPacket packet);
  int ParseRawData(Pandar40PPacket *packet, const uint8_t *buf, const int len);
  int ParseL64Data(HS_LIDAR_L64_Packet *packet, const uint8_t *recvbuf, const int len);
  int ParseL20Data(HS_LIDAR_L20_Packet *packet, const uint8_t *recvbuf, const int len);
  int ParseQTData(HS_LIDAR_QT_Packet *packet, const uint8_t *recvbuf, const int len);
  int ParseXTData(HS_LIDAR_XT_Packet *packet, const uint8_t *recvbuf, const int len);

  int ParseGPS(PandarGPS *packet, const uint8_t *recvbuf, const int size);
  void CalcPointXYZIT(Pandar40PPacket *pkt, int blockid,
                      boost::shared_ptr<PPointCloud> cld);
  void CalcL64PointXYZIT(HS_LIDAR_L64_Packet *pkt, int blockid, char chLaserNumber,
                      boost::shared_ptr<PPointCloud> cld);
  void CalcL20PointXYZIT(HS_LIDAR_L20_Packet *pkt, int blockid, char chLaserNumber,
                      boost::shared_ptr<PPointCloud> cld);
  void CalcQTPointXYZIT(HS_LIDAR_QT_Packet *pkt, int blockid, char chLaserNumber,
                      boost::shared_ptr<PPointCloud> cld);
  void CalcXTPointXYZIT(HS_LIDAR_XT_Packet *pkt, int blockid, char chLaserNumber,
                      boost::shared_ptr<PPointCloud> cld);
  void FillPacket(const uint8_t *buf, const int len, double timestamp);

  void EmitBackMessege(char chLaserNumber, boost::shared_ptr<PPointCloud> cld, hesai_lidar::PandarScanPtr scan);
  void SetEnvironmentVariableTZ();
  hesai_lidar::PandarPacket SaveCorrectionFile(int laserNumber);

  bool calculateTransformMatrix(Eigen::Affine3f& matrix, const std::string& target_frame, const std::string& source_frame, const ros::Time& time);
  void manage_tf_buffer();
  bool computeTransformToTarget(const ros::Time &scan_time);
  bool computeTransformToFixed(const ros::Time &packet_time) ;
  void transformPoint(float& x, float& y, float& z);
  float GetFiretimeOffset(float speed, float deltT);  
  pthread_mutex_t lidar_lock_;
  sem_t lidar_sem_;
  boost::thread *lidar_recv_thr_;
  boost::thread *lidar_process_thr_;
  bool enable_lidar_recv_thr_;
  bool enable_lidar_process_thr_;
  int start_angle_;
  std::string m_sTimestampType;
  double m_dPktTimestamp;

  std::list<struct PandarPacket_s> lidar_packets_;

  boost::shared_ptr<Input> input_;
  boost::function<void(boost::shared_ptr<PPointCloud> cld, double timestamp, hesai_lidar::PandarScanPtr scan)>
      pcl_callback_;
  boost::function<void(double timestamp)> gps_callback_;

  float sin_lookup_table_[ROTATION_MAX_UNITS];
  float cos_lookup_table_[ROTATION_MAX_UNITS];

  uint16_t last_azimuth_;
  double last_timestamp_;

  float elev_angle_map_[LASER_COUNT];
  float horizatal_azimuth_offset_map_[LASER_COUNT];

  float General_elev_angle_map_[MAX_LASER_NUM];
  float General_horizatal_azimuth_offset_map_[MAX_LASER_NUM];

  float Pandar20_elev_angle_map_[HS_LIDAR_L20_UNIT_NUM];
  float Pandar20_horizatal_azimuth_offset_map_[HS_LIDAR_L20_UNIT_NUM];

  float PandarQT_elev_angle_map_[HS_LIDAR_QT_UNIT_NUM];
  float PandarQT_horizatal_azimuth_offset_map_[HS_LIDAR_QT_UNIT_NUM];

  float PandarXT_elev_angle_map_[HS_LIDAR_XT_UNIT_NUM];
  float PandarXT_horizatal_azimuth_offset_map_[HS_LIDAR_XT_UNIT_NUM];

  float block64OffsetSingle_[HS_LIDAR_L64_BLOCK_NUMBER_6];
  float block64OffsetDual_[HS_LIDAR_L64_BLOCK_NUMBER_6];
  float laser64Offset_[HS_LIDAR_L64_UNIT_NUM];

  float block40OffsetSingle_[BLOCKS_PER_PACKET];
  float block40OffsetDual_[BLOCKS_PER_PACKET];
  float laser40Offset_[LASER_COUNT];

  float block20OffsetSingle_[HS_LIDAR_L20_BLOCK_NUMBER];
  float block20OffsetDual_[HS_LIDAR_L20_BLOCK_NUMBER];
  float laser20AOffset_[HS_LIDAR_L20_UNIT_NUM];
  float laser20BOffset_[HS_LIDAR_L20_UNIT_NUM];

  float blockQTOffsetSingle_[HS_LIDAR_QT_BLOCK_NUMBER];
  float blockQTOffsetDual_[HS_LIDAR_QT_BLOCK_NUMBER];
  float laserQTOffset_[HS_LIDAR_QT_UNIT_NUM];

  float blockXTOffsetSingle_[HS_LIDAR_XT_BLOCK_NUMBER];
  float blockXTOffsetDual_[HS_LIDAR_XT_BLOCK_NUMBER];
  float blockXTOffsetTriple_[HS_LIDAR_XT_BLOCK_NUMBER];
  float laserXTOffset_[HS_LIDAR_XT_UNIT_NUM];

  int tz_second_;
  std::string m_sFrameId;
  int pcl_type_;
  PcapReader *pcap_reader_;
  bool connect_lidar_;
  std::string m_sLidarType;
  std::vector<float> m_sin_azimuth_map_;
  std::vector<float> m_cos_azimuth_map_;
  std::vector<float> m_sin_elevation_map_;
  std::vector<float> m_cos_elevation_map_;
  std::vector<float> m_sin_azimuth_map_h;
  std::vector<float> m_cos_azimuth_map_h;
  std::vector<float> m_sin_azimuth_map_b;
  std::vector<float> m_cos_azimuth_map_b;
  bool got_lidar_correction_flag;
  std::string correction_file_path_;
  PacketsBuffer m_PacketsBuffer;
  bool m_bCoordinateCorrectionFlag;
  std::shared_ptr<tf2_ros::TransformListener> m_tf_listener;
  std::shared_ptr<tf2_ros::Buffer> m_tf_buffer;
  Eigen::Affine3f m_tf_matrix_to_fixed;
  Eigen::Affine3f m_tf_matrix_to_target;
  std::string m_sSensorFrame;
  std::string m_sFixedFrame;
  std::string m_sTargetFrame;
  uint16_t m_iAzimuthRange;
  int m_iPointCloudIndex;
  std::vector<std::vector<PPoint> > m_vPointCloudList;
  std::vector<PPoint> m_vPointCloud;

};

#endif  // SRC_PANDARGENERAL_INTERNAL_H_
