
#include "tf_listener/newtf_listener.hpp"

#include <Eigen/Geometry>

namespace gpm_slam {
NewTFListener::NewTFListener(ros::NodeHandle& nh, std::string base_frame_id, std::string child_frame_id) 
    :nh_(nh), base_frame_id_(base_frame_id), child_frame_id_(child_frame_id) {
        pose_x_=0;
        pose_y_=0;
        pose_theta_=0;
}

bool NewTFListener::LookupData(double & x, double & y, double & theta) {
    try {
        tf::StampedTransform tf_;
        //listener_.waitForTransform((base_frame_id_, child_frame_id_,ros::Time(0), ros::Duration(0.2)));
        listener_.lookupTransform(base_frame_id_, child_frame_id_, ros::Time(0), tf_);
        GetParam(tf_);
        x=pose_x_;
        y=pose_y_;
        theta=pose_theta_;
        return true;
    } catch (tf::TransformException &ex) {
        return false;
    }
}

bool NewTFListener::GetParam(const tf::StampedTransform& transform) {
    pose_x_=transform.getOrigin().getX();
    pose_y_=transform.getOrigin().getY();
    double roll, pitch, yaw;
    tf::Matrix3x3(transform.getRotation()).getEulerYPR(yaw, pitch, roll);
    pose_theta_=yaw;
    /*std::cout<<"pose theta: "<<pose_theta_<<std::endl;
    std::cout<<"pose x: "<<pose_x_<<std::endl;
    std::cout<<"pose y: "<<pose_y_<<std::endl;*/
        /*Eigen::Translation3f tl_btol(transform.getOrigin().getX(), transform.getOrigin().getY(), transform.getOrigin().getZ());
    
    double roll, pitch, yaw;
    tf::Matrix3x3(transform.getRotation()).getEulerYPR(yaw, pitch, roll);
    Eigen::AngleAxisf rot_x_btol(roll, Eigen::Vector3f::UnitX());
    Eigen::AngleAxisf rot_y_btol(pitch, Eigen::Vector3f::UnitY());
    Eigen::AngleAxisf rot_z_btol(yaw, Eigen::Vector3f::UnitZ());

    // 此矩阵为 child_frame_id 到 base_frame_id 的转换矩阵
    transform_matrix = (tl_btol * rot_z_btol * rot_y_btol * rot_x_btol).matrix();*/

    return true;
}
}