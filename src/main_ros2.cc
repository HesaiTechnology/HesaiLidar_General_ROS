// #include <ros/ros.h>
#include <rclcpp/rclcpp.hpp>
#include <hesai_lidar/msg/pandar_scan.hpp>
#include <hesai_lidar/msg/pandar_packet.hpp>
#include <image_transport/image_transport.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include "pandarGeneral_sdk/pandarGeneral_sdk.h"
#include <fstream>
#include <memory>
#include <chrono>
#include <string>
#include <functional>
#include "std_msgs/msg/string.hpp"
// #define PRINT_FLAG 

using namespace std;
namespace hesai_lidar
{
class HesaiLidarClient: public rclcpp::Node
{
public:
  HesaiLidarClient():Node("hesai_lidar")
  { 
    this->declare_parameter<std::string>("pcap_file", "");
    this->declare_parameter<std::string>("server_ip", "");
    this->declare_parameter<int>("lidar_recv_port", 2368);
    this->declare_parameter<int>("gps_port", 10110);
    this->declare_parameter<double>("start_angle", 0.0);
    this->declare_parameter<std::string>("lidar_correction_file", "");
    this->declare_parameter<std::string>("lidar_type", "");
    this->declare_parameter<std::string>("frame_id", "");
    this->declare_parameter<int>("pcldata_type", 0);
    this->declare_parameter<std::string>("publish_type", "");
    this->declare_parameter<std::string>("timestamp_type", "");
    this->declare_parameter<std::string>("data_type", "");
    this->declare_parameter<std::string>("multicast_ip", "");
    this->declare_parameter<bool>("coordinate_correction_flag", false);
    this->declare_parameter<std::string>("target_frame", "");
    this->declare_parameter<std::string>("fixed_frame", "");
    rclcpp::QoS qos(rclcpp::KeepLast(7)); 
    auto sensor_qos = rclcpp::QoS(rclcpp::SensorDataQoS());
    lidarPublisher = this->create_publisher<sensor_msgs::msg::PointCloud2>("pandar", sensor_qos);
    packetPublisher = this->create_publisher<hesai_lidar::msg::PandarScan>("pandar_packets", qos);
    this->timer_callback();
  }


private:

  void lidarCallback(boost::shared_ptr<PPointCloud> cld, double timestamp, hesai_lidar::msg::PandarScan::SharedPtr scan) // the timestamp from first point cloud of cld
  {
    if(m_sPublishType == "both" || m_sPublishType == "points"){
      rclcpp::Time now = this->now();
      pcl_conversions::toPCL(now, cld->header.stamp);
      sensor_msgs::msg::PointCloud2 output;
      pcl::toROSMsg(*cld, output);
      lidarPublisher->publish(output);
#ifdef PRINT_FLAG
        std::cout.setf(ios::fixed);
        std::cout << "timestamp: " << std::setprecision(10) << timestamp << ", point size: " << cld->points.size() << std::endl;
#endif        
    }
    if(m_sPublishType == "both" || m_sPublishType == "raw"){
      packetPublisher->publish(*scan);
#ifdef PRINT_FLAG
        std::cout << "raw size: "<< scan->packets.size() << std::endl;
#endif
    }
  }

  void gpsCallback(int timestamp) {
#ifdef PRINT_FLAG
      std::cout << "gps: " << timestamp << std::endl;
#endif      
  }

  void scanCallback(const hesai_lidar::msg::PandarScan::SharedPtr scan)
  {
    // printf("pandar_packets topic message received,\n");
    hsdk->PushScanPacket(scan);
  }

  void timer_callback()
  {
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

    this->get_parameter("pcap_file", pcapFile);
    this->get_parameter("server_ip", serverIp);
    this->get_parameter("lidar_recv_port", lidarRecvPort);
    this->get_parameter("gps_port", gpsPort);
    this->get_parameter("start_angle", startAngle);
    this->get_parameter("lidar_correction_file", lidarCorrectionFile);
    this->get_parameter("lidar_type", lidarType);
    this->get_parameter("frame_id", frameId);
    this->get_parameter("pcldata_type", pclDataType);
    this->get_parameter("publish_type", m_sPublishType);
    this->get_parameter("timestamp_type", m_sTimestampType);
    this->get_parameter("data_type", dataType);
    this->get_parameter("multicast_ip", multicastIp);
    this->get_parameter("coordinate_correction_flag", coordinateCorrectionFlag);
    this->get_parameter("target_frame", targetFrame);
    this->get_parameter("fixed_frame", fixedFrame);
    this->get_parameter("background_b", targetFrame);
  
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
        packetSubscriber = this->create_subscription<hesai_lidar::msg::PandarScan>("pandar_packets", 10, std::bind(&HesaiLidarClient::scanCallback, this, std::placeholders::_1));
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
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr lidarPublisher;
  rclcpp::Publisher<hesai_lidar::msg::PandarScan>::SharedPtr packetPublisher;
  rclcpp::TimerBase::SharedPtr timer_;
  PandarGeneralSDK* hsdk;
  string m_sPublishType;
  string m_sTimestampType;
  rclcpp::Subscription<hesai_lidar::msg::PandarScan>::SharedPtr packetSubscriber;
};
}
int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<hesai_lidar::HesaiLidarClient>());
  rclcpp::shutdown();
  return 0;
}
#include "rclcpp_components/register_node_macro.hpp"

// RCLCPP_COMPONENTS_REGISTER_NODE(HesaiLidarClient)
