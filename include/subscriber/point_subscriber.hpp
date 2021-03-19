/*
 subscribe laser data and put it into deque for further process
 */

#ifndef GPM_SLAM_SUBSCRIBER_POINT_SUBSCRIBER_HPP_
#define GPM_SLAM_SUBSCRIBER_POINT_SUBSCRIBER_HPP_

#include <deque>

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>


#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <iostream>
#include <vector>
#include <Eigen/Dense>
#include <cmath>


#define MaxRange 10
using PointSet=std::vector <cv::Point2f> ;


namespace gpm_slam
{
class PointSubscriber
{
  public:
    PointSubscriber(ros::NodeHandle& nh, std::string topic_name, size_t buff_size);
    //CloudSubscriber() = default;
    void ParseData(std::deque<PointSet>& deque_cloud_data);

  private:
    void msg_callback(const sensor_msgs::LaserScan::ConstPtr& scan);

  private:
    ros::NodeHandle nh_;
    ros::Subscriber subscriber_;
  public:
    PointSet frame_point_set;
    //std::deque<PointSet> frame_point_data_;
};
}

#endif