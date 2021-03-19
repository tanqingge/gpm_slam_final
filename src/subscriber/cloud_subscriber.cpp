/*
 * @Description: 订阅激光点云信息，并解析数据
 * @Author: Ren Qian
 * @Date: 2020-02-05 02:27:30
 */

#include "subscriber/cloud_subscriber.hpp"



namespace gpm_slam {
CloudSubscriber::CloudSubscriber(ros::NodeHandle& nh, std::string topic_name, size_t buff_size)
    :nh_(nh) {
    subscriber_ = nh_.subscribe(topic_name, buff_size, &CloudSubscriber::msg_callback, this);
}

void CloudSubscriber::msg_callback(const sensor_msgs::LaserScan::ConstPtr& scan) {
    CloudData cloud_data;
    laser_geometry::LaserProjection projector_;
    sensor_msgs::PointCloud2 cloud_two;
    //start conversion from scan to pcl deque
    cloud_data.time = scan->header.stamp.toSec();
    projector_.projectLaser(*scan, cloud_two);
    pcl::fromROSMsg(cloud_two, *(cloud_data.cloud_ptr));

    new_cloud_data_.push_back(cloud_data);
}

void CloudSubscriber::ParseData(std::deque<CloudData>& cloud_data_buff) {
    if (new_cloud_data_.size() > 0) {
        cloud_data_buff.insert(cloud_data_buff.end(), new_cloud_data_.begin(), new_cloud_data_.end());
        new_cloud_data_.clear();
    }
}
} // namespace data_input