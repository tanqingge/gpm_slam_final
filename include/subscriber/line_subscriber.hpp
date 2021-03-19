#ifndef GPM_SLAM_SUBSCRIBER_LINE_SUBSCRIBER_HPP_
#define GPM_SLAM_SUBSCRIBER_LINE_SUBSCRIBER_HPP_

#include "sensor_data/line_data.hpp"

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <cmath>

//define parameter in temporary lines
#define MAXSEGMENTS 100
#define connect_th_ 0.15
#define min_distance_ 0.15
#define count_threhold_ 3
#define length_limit_ 0.1

namespace gpm_slam {
class LineSubscriber {
  public:
    struct SegmentStats{
        float x_start_,y_start_,x_end_,y_end_,xMean_,yMean_;
        int count_; //the horizontal length of the line
        double grad_;//gradient, dy/dx
        double invgrad_; //inv gradient, dx/dy
        float x_,y_,xy_,xx_,yy_; //for gradient stats
    };

  public:
    LineSubscriber(ros::NodeHandle& nh, std::string topic_name,size_t buff_size);
    void ParseData(std::deque<LineData>& line_data_buff);

  public:
    void NewLineDataInit();
    void FitLineWithPoint();
    void WhetherSegmentIsValid(LineData line_data_); 

  private:
    void msg_callback(const sensor_msgs::PointCloud2::ConstPtr& cloud_msg_ptr);
  public:
    ros::NodeHandle nh_;
    ros::Subscriber subscriber_;

    std::deque<LineData> new_line_data_;
    LineData::CLOUD_PTR origincloud_for_line_ptr;
    SegmentStats init_segs_[MAXSEGMENTS];

};
}

#endif