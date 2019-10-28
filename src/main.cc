#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
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
    lidarPublisher = node.advertise<sensor_msgs::PointCloud2>("points_raw", 10);

    string serverIp;
    int serverPort;
    int lidarRecvPort;
    int gpsPort;
    double startAngle;
    string lidarCorrectionFile;
    int laserReturnType;
    int laserCount;
    int pclDataType;
    string pcapFile;
    string lidarType;

    nh.getParam("pcap_file", pcapFile);
    nh.getParam("server_ip", serverIp);
    nh.getParam("server_port", serverPort);
    nh.getParam("lidar_recv_port", lidarRecvPort);
    nh.getParam("gps_port", gpsPort);
    nh.getParam("start_angle", startAngle);
    nh.getParam("lidar_correction_file", lidarCorrectionFile);
    nh.getParam("laser_return_type", laserReturnType);
    nh.getParam("laser_count", laserCount);
    nh.getParam("pcldata_type", pclDataType);
    nh.getParam("lidar_type", lidarType);

    if ((strcmp(lidarType.c_str(), "Pandar20A") != 0) && \
        (strcmp(lidarType.c_str(), "Pandar20B") != 0) && \
        (strcmp(lidarType.c_str(), "Pandar40P") != 0)) {
      lidarType = string("Pandar64");
    }

    // pcapFile = "/home/pandora/Desktop/pandar40p.pcap";

    if(!pcapFile.empty())
    {
      return;
    }
    else
    {
      hsdk = new PandarGeneralSDK(serverIp, lidarRecvPort, gpsPort, \
          boost::bind(&HesaiLidarClient::lidarCallback, this, _1, _2), \
          NULL, 0, 0, lidarType);
    }

    hsdk->Start();
  }

  void lidarCallback(boost::shared_ptr<PPointCloud> cld, double timestamp)
  {
    pcl_conversions::toPCL(ros::Time(timestamp), cld->header.stamp);
    cld->header.frame_id = "pandar";
    sensor_msgs::PointCloud2 output;
    pcl::toROSMsg(*cld, output);
    lidarPublisher.publish(output);
  }

private:
  ros::Publisher lidarPublisher;
  image_transport::Publisher imgPublishers[5];
  PandarGeneralSDK* hsdk;
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "pandora_ros");
  ros::NodeHandle nh("~");
  ros::NodeHandle node;
  HesaiLidarClient pandoraClient(node, nh);

  ros::spin();
  return 0;
}
