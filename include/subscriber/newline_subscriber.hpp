#ifndef GPM_SLAM_NEWLINE_SUBSCRIBER_SUBSCRIBER_HPP_
#define GPM_SLAM_NEWLINE_SUBSCRIBER_SUBSCRIBER_HPP_

#include <iostream>
#include <fstream>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <cmath>
#include <vector>
#include <float.h>
#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>
#include "subscriber/point_subscriber.hpp"
#include "sensor_data/line_data.hpp"

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <cmath>


#include <cmath>

//define parameter in temporary lines
/*#define PI 3.1415926
#define MAXSEGMENTS 100
#define connect_th_ 0.10
#define min_distance_ 0.10
#define count_threhold_ 5
#define length_limit_ 0.15*/

//beihang
//define parameter in temporary lines
#define PI 3.1415926
#define MAXSEGMENTS 200
#define connect_th_ 0.03
#define min_distance_ 0.03
#define count_threhold_ 8
#define length_limit_ 0.2

namespace gpm_slam {
class NewLineSubscriber {
  public:
    struct SegmentStats{
        float x_start_,y_start_,x_end_,y_end_,xMean_,yMean_;
        int count_; //the horizontal length of the line
        double grad_;//gradient, dy/dx
        double invgrad_; //inv gradient, dx/dy
        float x_,y_,xy_,xx_,yy_; //for gradient stats
    };

  public:
    NewLineSubscriber(const PointSet point_data);
    void ParseData(std::deque<LineData>& line_data_buff);

  public:
    void NewLineDataInit();
    void FitLineWithPoint();
    void WhetherSegmentIsValid(LineData line_data_); 

  private:
    void msg_callback();
  public:

    std::deque<LineData> new_line_data_;
    PointSet originpoint;
    SegmentStats init_segs_[MAXSEGMENTS];

};
}

#endif