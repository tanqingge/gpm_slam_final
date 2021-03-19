/*
 subscribe laser data and put it into deque for further process
 */

#ifndef GPM_SLAM_SUBSCRIBER_CLOUD_SUBSCRIBER_HPP_
#define GPM_SLAM_SUBSCRIBER_CLOUD_SUBSCRIBER_HPP_

#include <deque>

#include <ros/ros.h>
#include <laser_geometry/laser_geometry.h>

#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/LaserScan.h>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>

#include "sensor_data/cloud_data.hpp"

namespace gpm_slam {
class CloudSubscriber {
  public:
    CloudSubscriber(ros::NodeHandle& nh, std::string topic_name, size_t buff_size);
    //CloudSubscriber() = default;
    void ParseData(std::deque<CloudData>& deque_cloud_data);

  private:
    void msg_callback(const sensor_msgs::LaserScan::ConstPtr& scan);

  private:
    ros::NodeHandle nh_;
    ros::Subscriber subscriber_;

    std::deque<CloudData> new_cloud_data_;
};
}

#endif