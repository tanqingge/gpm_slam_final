#ifndef GPM_SLAM_PUBLISHER_LINE_PUBLISHER_HPP_
#define GPM_SLAM_PUBLISHER_LINE_PUBLISHER_HPP_

#include "sensor_data/line_data.hpp"
#include <ros/ros.h>
#include <gpm_slam/Line_Segment.h>

namespace gpm_slam {
    class LinePublisher{
        public:
        LinePublisher(ros::NodeHandle& nh,
                   std::string topic_name,
                   size_t buff_size,
                   std::string frame_id);
        void Publish(LineData line_data);
        private:
            ros::NodeHandle nh_;
            ros::Publisher publisher_;
            std::string frame_id_;
            gpm_slam::Line_Segment new_line_msgs_;


    };
}
#endif