#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include "pandarGeneral_sdk/pandarGeneral_sdk.h"

#include <livox_ros_driver/CustomMsg.h>
#include <iostream>


#include <fstream>
// #define PRINT_FLAG 

using namespace std;

class HesaiLidarClient
{
public:
  HesaiLidarClient(ros::NodeHandle node, ros::NodeHandle nh)
  {
    lidarPublisher = node.advertise<livox_ros_driver::CustomMsg>("lidar", 10);
    packetPublisher = node.advertise<hesai_lidar::PandarScan>("pandar_packets",10);

    string serverIp;
    int lidarRecvPort;
    int gpsPort;
    double startAngle;
    string lidarCorrectionFile;  // Get local correction when getting from lidar failed
    string lidarType;
    string frameId;
    int pclDataType;
    string pcapFile;
    string dataType;
    string multicastIp;
    bool coordinateCorrectionFlag;
    string targetFrame;
    string fixedFrame;

    nh.getParam("pcap_file", pcapFile);
    nh.getParam("server_ip", serverIp);
    nh.getParam("lidar_recv_port", lidarRecvPort);
    nh.getParam("gps_port", gpsPort);
    nh.getParam("start_angle", startAngle);
    nh.getParam("lidar_correction_file", lidarCorrectionFile);
    nh.getParam("lidar_type", lidarType);
    nh.getParam("frame_id", frameId);
    nh.getParam("pcldata_type", pclDataType);
    nh.getParam("publish_type", m_sPublishType);
    nh.getParam("timestamp_type", m_sTimestampType);
    nh.getParam("data_type", dataType);
    nh.getParam("multicast_ip", multicastIp);
    nh.getParam("coordinate_correction_flag", coordinateCorrectionFlag);
    nh.getParam("target_frame", targetFrame);
    nh.getParam("fixed_frame", fixedFrame);
  
    if(!pcapFile.empty()){
      hsdk = new PandarGeneralSDK(pcapFile, boost::bind(&HesaiLidarClient::lidarCallback, this, _1, _2, _3), \
      static_cast<int>(startAngle * 100 + 0.5), 0, pclDataType, lidarType, frameId, m_sTimestampType, lidarCorrectionFile, \
      coordinateCorrectionFlag, targetFrame, fixedFrame);
      if (hsdk != NULL) {
        std::ifstream fin(lidarCorrectionFile);
        if (fin.is_open()) {
          std::cout << "Open correction file " << lidarCorrectionFile << " succeed" << std::endl;
          int length = 0;
          std::string strlidarCalibration;
          fin.seekg(0, std::ios::end);
          length = fin.tellg();
          fin.seekg(0, std::ios::beg);
          char *buffer = new char[length];
          fin.read(buffer, length);
          fin.close();
          strlidarCalibration = buffer;
          int ret = hsdk->LoadLidarCorrectionFile(strlidarCalibration);
          if (ret != 0) {
            std::cout << "Load correction file from " << lidarCorrectionFile <<" failed" << std::endl;
          } else {
            std::cout << "Load correction file from " << lidarCorrectionFile << " succeed" << std::endl;
          }
        }
        else{
          std::cout << "Open correction file " << lidarCorrectionFile << " failed" << std::endl;
        }
      }
    }
    else if ("rosbag" == dataType){
      hsdk = new PandarGeneralSDK("", boost::bind(&HesaiLidarClient::lidarCallback, this, _1, _2, _3), \
      static_cast<int>(startAngle * 100 + 0.5), 0, pclDataType, lidarType, frameId, m_sTimestampType, \
      lidarCorrectionFile, coordinateCorrectionFlag, targetFrame, fixedFrame);
      if (hsdk != NULL) {
        packetSubscriber = node.subscribe("pandar_packets",10,&HesaiLidarClient::scanCallback, (HesaiLidarClient*)this, ros::TransportHints().tcpNoDelay(true));
      }
    }
    else {
      hsdk = new PandarGeneralSDK(serverIp, lidarRecvPort, gpsPort, \
        boost::bind(&HesaiLidarClient::lidarCallback, this, _1, _2, _3), \
        boost::bind(&HesaiLidarClient::gpsCallback, this, _1), static_cast<int>(startAngle * 100 + 0.5), 0, pclDataType, lidarType, frameId,\
         m_sTimestampType, lidarCorrectionFile, multicastIp, coordinateCorrectionFlag, targetFrame, fixedFrame);
    }
    
    if (hsdk != NULL) {
        hsdk->Start();
        // hsdk->LoadLidarCorrectionFile("...");  // parameter is stream in lidarCorrectionFile
    } else {
        printf("create sdk fail\n");
    }
  }

  void lidarCallback(boost::shared_ptr<PPointCloud> cld, double timestamp, hesai_lidar::PandarScanPtr scan) // the timestamp from first point cloud of cld
  {
   
    if (m_sPublishType == "both" || m_sPublishType == "points")
    {
      pcl_conversions::toPCL(ros::Time(timestamp), cld->header.stamp);
// <<<<<<< HEAD
      // sensor_msgs::PointCloud2 output;
      // pcl::toROSMsg(*cld, output);
      livox_ros_driver::CustomMsg output;

      int input_point_size = cld->width;
      output.points.reserve(input_point_size+1); // [input_point_size]要素(以上)の領域を事前に確保しておく
      // output.rsvd =
      output.err.time_sync_status = 2;
      output.err.pps_status = 1;
      output.err.system_status = 0;
      static int publish_count;
      publish_count++;
      output.header.seq = publish_count;

      static double timebase_double;
      static uint64_t timebase_uint64;
      static double pre_timestamp;
      static double pre_timebase_double;
      static uint64_t pre_timebase_uint64;
      if(publish_count == 0){
        pre_timestamp = timestamp;
        pre_timebase_double = timebase_double;
        pre_timebase_uint64 = timebase_uint64;
      }

      timebase_double = timestamp * 1e9;
      timebase_uint64 = (uint64_t) timebase_double;
      output.timebase = timebase_uint64;

      double d_timestamp = (double)timestamp - (double)pre_timestamp;
      double d_timebase_double = timebase_double - pre_timebase_double;
      uint64_t d_timebase_uint64 = timebase_uint64 - pre_timebase_uint64;
      if(d_timestamp > 1.00){
        ROS_WARN("The time distance between the two points is too wide.");
        std::cout << "pre_timestamp = " << pre_timestamp << ", pre_timebase_double = " << pre_timebase_double << ", pre_timebase_uint64 = " << pre_timebase_uint64 <<"\n";
        std::cout << "timestamp = " << timestamp << ", timebase_double = " << timebase_double << ", timebase_uint64 = " << timebase_uint64 << ", d_timestamp = " << d_timestamp << ", d_timebase_double = " << d_timebase_double << ", d_timebase_uint64 = " << d_timebase_uint64 <<"\n";
      } 
      
      pre_timestamp = timestamp;
      pre_timebase_double = timebase_double;
      pre_timebase_uint64 = timebase_uint64;
      

      for (int i = 0; i < input_point_size; i++)
      {
        livox_ros_driver::CustomPoint sub_output;
      
          // printf("msg.timebase : %d\n",output.timebase);
          double timestamp_double_ns_point_i = cld->points[i].timestamp*1e9;
          // printf("timestamp_double_ns_point_%d: %f\n", i, timestamp_double_ns_point_i);
          uint64_t timestamp_uint64_ns_point_i = (uint64_t) timestamp_double_ns_point_i;
          // printf("timestamp_uint64_ns_point_%d :", i);
          // std::cout << timestamp_uint64_ns_point_i << std::endl;
          uint32_t timestamp_uint32_ns_point_i = static_cast<uint32_t>(timestamp_uint64_ns_point_i);
          // printf("timestamp_uint32_ns_point_%d :", i);
          // std::cout << timestamp_uint32_ns_point_i << std::endl;

          uint32_t timebase_uint32 = (uint32_t)timebase_uint64;
          // printf("\ntimebase_uint32 : %d\n",timebase_uint32);
          uint32_t time_offset_ns_point_i = timestamp_uint32_ns_point_i - timebase_uint32;
          // printf("time_offset_ns_point_%d : ", i);
          // std::cout << time_offset_ns_point_i << std::endl;
          sub_output.offset_time = (uint32_t)(time_offset_ns_point_i);
          // printf("\nmsg.offset_time :");
          // std::cout << sub_output.offset_time << std::endl;
          // printf("\n---------------------------\n");
        //  }
        sub_output.x = cld->points[i].x;
        sub_output.y = cld->points[i].y;
        sub_output.z = cld->points[i].z;
        sub_output.reflectivity = cld->points[i].intensity;
        sub_output.line = cld->points[i].ring; 
        sub_output.tag = 16;
        // if(i<=1){
        // std::cout << "output.timebase"<< output.timebase;
        // std::cout << " , time_offset["<< i << "] = " << sub_output.offset_time <<"\n";
        // }
        output.points.push_back(sub_output);
      }
      std::cout << "width = " << cld->width <<std::endl;
      std::cout << "height = " << cld->height <<std::endl;

      std::cout << "reserved = " << output.points.size() <<std::endl;
      // std::cout << "capacity = " << output.points.capacity() <<std::endl;
          
          // sub_output.offset_time = cld->points[input_point_num-1].timestamp;
      output.point_num = output.points.size(); //★
      // std::cout << " output.points.size() = " << output.points.size() <<std::endl;

      std::cout << "---------------------------" << std::endl;

          
          lidarPublisher.publish(output);
      // printf("timebase:%f ,timestamp: %f, point size: %d, width: %d.\n",output.timebase,timestamp,input_point_size,cld->width);
    }
    if(m_sPublishType == "both" || m_sPublishType == "raw"){
      packetPublisher.publish(scan);
      // printf("raw size: %d.\n", scan->packets.size());
// =======
//       sensor_msgs::PointCloud2 output;
//       pcl::toROSMsg(*cld, output);
//       lidarPublisher.publish(output);
// #ifdef PRINT_FLAG
//         printf("timestamp: %f, point size: %ld.\n",timestamp, cld->points.size());
// #endif        
//     }
//     if(m_sPublishType == "both" || m_sPublishType == "raw"){
//       packetPublisher.publish(scan);
// #ifdef PRINT_FLAG
//         printf("raw size: %d.\n", scan->packets.size());
// #endif
// >>>>>>> Hesai_upstream/master
    }
  }

  void gpsCallback(int timestamp) {
    #ifdef PRINT_FLAG
       printf("gps: %d\n", timestamp);
    #endif      

  }

  void scanCallback(const hesai_lidar::PandarScanPtr scan)
  {
    // printf("pandar_packets topic message received,\n");
    hsdk->PushScanPacket(scan);
  }

private:
  ros::Publisher lidarPublisher;
  ros::Publisher packetPublisher;
  PandarGeneralSDK* hsdk;
  string m_sPublishType;
  string m_sTimestampType;
  ros::Subscriber packetSubscriber;
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "pandar");
  ros::NodeHandle nh("~");
  ros::NodeHandle node;
  HesaiLidarClient pandarClient(node, nh);

  ros::spin();
  return 0;
}
