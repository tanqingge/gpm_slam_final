/*
 * @Description: 订阅激光点云信息，并解析数据
 * @Author: Ren Qian
 * @Date: 2020-02-05 02:27:30
 */

#include "subscriber/point_subscriber.hpp"
#include <cmath>
#include <iostream>
#include <fstream>

namespace gpm_slam {
PointSubscriber::PointSubscriber(ros::NodeHandle& nh, std::string topic_name, size_t buff_size)
    :nh_(nh) {
    subscriber_ = nh_.subscribe(topic_name, buff_size, &PointSubscriber::msg_callback, this);
}

void PointSubscriber::msg_callback(const sensor_msgs::LaserScan::ConstPtr& scan) {
    //PointSet frame_point_set;
    std::cout<<"The Origin size of scan point is "<<scan->ranges.size()<<std::endl;

    const double& ang_min = scan->angle_min;
    const double& ang_max = scan->angle_max;
    const double& ang_inc = scan->angle_increment;
    //const double& range_min = scan->range_min;
    const double& range_min = 1.5;
    for(int i = 0; i < scan->ranges.size(); i ++)
    {
        double R = scan->ranges.at(i); 
        if(R > MaxRange || R < range_min)
            continue;
        double angle = ang_inc * i + ang_min;
        double cangle = cos(angle);
        double sangle = sin(angle);
        cv::Point2f pt_tp;
        pt_tp.x = R * cangle;
	    pt_tp.y = R * sangle;
        frame_point_set.push_back(pt_tp);
    }
    std::cout<<"Valid Point Size is "<<frame_point_set.size()<<std::endl;
}
    

void PointSubscriber::ParseData(std::deque<PointSet>& point_data_buff) {
    if (frame_point_set.size() > 0) {
        point_data_buff.push_back(frame_point_set);
        frame_point_set.clear();
    }
}
} // namespace data_input