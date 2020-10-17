#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include "pandarGeneral_sdk/pandarGeneral_sdk.h"

using namespace std;

class HesaiLidarClient
{
public:
  HesaiLidarClient(ros::NodeHandle node, ros::NodeHandle nh)
  {
    lidarPublisher = node.advertise<sensor_msgs::PointCloud2>("pandar", 10);
    packetPublisher = node.advertise<hesai_lidar::PandarScan>("pandar_packets",10);

    string serverIp;
    int lidarRecvPort;
    int gpsPort;
    double startAngle;
    string lidarCorrectionFile;  // Get local correction when getting from lidar failed
    string lidarType;
    int pclDataType;
    string pcapFile;
    string dataType;

    nh.getParam("pcap_file", pcapFile);
    nh.getParam("server_ip", serverIp);
    nh.getParam("lidar_recv_port", lidarRecvPort);
    nh.getParam("gps_port", gpsPort);
    nh.getParam("start_angle", startAngle);
    nh.getParam("lidar_correction_file", lidarCorrectionFile);
    nh.getParam("lidar_type", lidarType);
    nh.getParam("pcldata_type", pclDataType);
    nh.getParam("publish_type", m_sPublishType);
    nh.getParam("timestamp_type", m_sTimestampType);
    nh.getParam("data_type", dataType);

    if(!pcapFile.empty()){
      hsdk = new PandarGeneralSDK(pcapFile, boost::bind(&HesaiLidarClient::lidarCallback, this, _1, _2, _3), \
      static_cast<int>(startAngle * 100 + 0.5), 0, pclDataType, lidarType, m_sTimestampType);
      if (hsdk != NULL) {
        ifstream fin(lidarCorrectionFile);
        int length = 0;
        std::string strlidarCalibration;
        fin.seekg(0, std::ios::end);
        length = fin.tellg();
        fin.seekg(0, std::ios::beg);
        char *buffer = new char[length];
        fin.read(buffer, length);
        fin.close();
        strlidarCalibration = buffer;
        hsdk->LoadLidarCorrectionFile(strlidarCalibration);
      }
    }
    else if ("rosbag" == dataType){
      hsdk = new PandarGeneralSDK("", boost::bind(&HesaiLidarClient::lidarCallback, this, _1, _2, _3), \
      static_cast<int>(startAngle * 100 + 0.5), 0, pclDataType, lidarType, m_sTimestampType);
      if (hsdk != NULL) {
        ifstream fin(lidarCorrectionFile);
        int length = 0;
        std::string strlidarCalibration;
        fin.seekg(0, std::ios::end);
        length = fin.tellg();
        fin.seekg(0, std::ios::beg);
        char *buffer = new char[length];
        fin.read(buffer, length);
        fin.close();
        strlidarCalibration = buffer;
        hsdk->LoadLidarCorrectionFile(strlidarCalibration);
        packetSubscriber = node.subscribe("pandar_packets",10,&HesaiLidarClient::scanCallback, (HesaiLidarClient*)this, ros::TransportHints().tcpNoDelay(true));
      }
    }
    else {
      hsdk = new PandarGeneralSDK(serverIp, lidarRecvPort, gpsPort, \
        boost::bind(&HesaiLidarClient::lidarCallback, this, _1, _2, _3), \
        NULL, static_cast<int>(startAngle * 100 + 0.5), 0, pclDataType, lidarType, m_sTimestampType);
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
    if(m_sPublishType == "both" || m_sPublishType == "points"){
      pcl_conversions::toPCL(ros::Time(timestamp), cld->header.stamp);
      sensor_msgs::PointCloud2 output;
      pcl::toROSMsg(*cld, output);
      lidarPublisher.publish(output);
      printf("timestamp: %f, point size: %ld.\n",timestamp, cld->points.size());
    }
    if(m_sPublishType == "both" || m_sPublishType == "raw"){
      packetPublisher.publish(scan);
      printf("raw size: %d.\n", scan->packets.size());
    }
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
