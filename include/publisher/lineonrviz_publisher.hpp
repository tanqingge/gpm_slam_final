#ifndef GPM_SLAM_PUBLISHER_LINEONRVIZ_PUBLISHER_HPP_
#define GPM_SLAM_PUBLISHER_LINEONRVIZ_PUBLISHER_HPP_


#include "sensor_data/line_data.hpp"
#include <ros/ros.h>
#include <Eigen/Dense>
#include <Eigen/Core>

namespace gpm_slam
{
    class LineOnRvizPublisher
    {
    public:
        LineOnRvizPublisher(ros::NodeHandle& nh,
                   std::string topic_name,
                   size_t buff_size,
                   std::string frame_id);
        void Publish(LineData line_data,Eigen::Matrix4f odometry_matrix);
    
    private:
        /* data */
        ros::NodeHandle& nh_;
        ros::Publisher publisher_;
        std::string frame_id_;

        



    };
    
}



#endif