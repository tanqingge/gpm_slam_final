#ifndef GPM_SLAM_NEWTF_LISTENER_HPP_
#define GPM_SLAM_NEWTF_LISTENER_HPP_

#include <string>

#include <Eigen/Dense>
#include <ros/ros.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>

namespace gpm_slam {
class NewTFListener {
  public:
    NewTFListener(ros::NodeHandle& nh, std::string base_frame_id, std::string child_frame_id);
    //TFListener() = default;

    bool LookupData(double & x, double & y, double & theta);
  
  private:
    bool GetParam(const tf::StampedTransform& transform);

  private:
    ros::NodeHandle nh_;
    tf::TransformListener listener_;
    std::string base_frame_id_;
    std::string child_frame_id_;
    double pose_x_;
    double pose_y_;
    double pose_theta_;
};
}

#endif