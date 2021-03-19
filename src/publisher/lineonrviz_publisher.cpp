#include "publisher/lineonrviz_publisher.hpp"
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/Point.h>
#include <Eigen/Geometry>
#include <cmath>
namespace gpm_slam
{
    LineOnRvizPublisher::LineOnRvizPublisher(ros::NodeHandle& nh,
                               std::string topic_name,
                               size_t buff_size,
                               std::string frame_id):
    nh_(nh),frame_id_(frame_id){
        publisher_ = nh.advertise<visualization_msgs::Marker>("topic_name", buff_size);
        
    }

    void LineOnRvizPublisher::Publish(LineData line_data,Eigen::Matrix4f odometry_matrix)
    {
        static int id_;
        int line_number=line_data.line_ptr->size();
        Eigen::Vector4f line_startpoint;
        Eigen::Vector4f line_endpoint;
        visualization_msgs::Marker points,line_list;
        points.header.frame_id=line_list.header.frame_id=frame_id_;
        points.header.stamp = line_list.header.stamp = ros::Time::now();
        points.ns = line_list.ns = "gpm_slam";
        points.action = line_list.action = visualization_msgs::Marker::ADD;
        /*points.pose.position.x = line_list.pose.position.x = odometry_matrix(0,3);
        points.pose.position.y = line_list.pose.position.y = odometry_matrix(1,3);
        Eigen::Matrix3f Rotation_Matrix_=odometry_matrix.block<3,3>(0,0);
        Eigen::Quaternionf quaternion_(Rotation_Matrix_);
        points.pose.orientation.x = line_list.pose.orientation.x = quaternion_.x();
        points.pose.orientation.y = line_list.pose.orientation.y = quaternion_.y();
        points.pose.orientation.z = line_list.pose.orientation.z = quaternion_.z();
        points.pose.orientation.w = line_list.pose.orientation.w = quaternion_.w();*/
        points.pose.position.x = line_list.pose.position.x = 0;
        points.pose.position.y = line_list.pose.position.y = 0;
        points.pose.orientation.x = line_list.pose.orientation.x = 0;
        points.pose.orientation.y = line_list.pose.orientation.y = 0;
        points.pose.orientation.z = line_list.pose.orientation.z = 0;
        points.pose.orientation.w = line_list.pose.orientation.w = 1;
        
        points.id = 0;
        line_list.id = id_;
        id_++;
        points.type = visualization_msgs::Marker::POINTS;
        line_list.type = visualization_msgs::Marker::LINE_LIST;
        line_list.scale.x = 0.05;
        line_list.color.r = 1.00;
        line_list.color.a = 1.0;
        for (int i=0;i<line_number;i++)
        {
        geometry_msgs::Point p1;
        /*p1.x=GLOBALLINES_[i].x_start_;
        p1.y=GLOBALLINES_[i].y_start_;
        p1.z=0;*/
        line_startpoint<<(*line_data.line_ptr)[i].start_point.x,(*line_data.line_ptr)[i].start_point.y,0,1;
        line_endpoint<<(*line_data.line_ptr)[i].end_point.x,(*line_data.line_ptr)[i].end_point.y,0,1;
        /*line_startpoint = odometry_matrix*line_startpoint;
        line_endpoint = odometry_matrix*line_endpoint;*/
        p1.x=line_startpoint(0);
        p1.y=line_startpoint(1);
        p1.z=0;
        line_list.points.push_back(p1);
        geometry_msgs::Point p2;
        /*p2.x=GLOBALLINES_[i].x_end_;
        p2.y=GLOBALLINES_[i].y_end_;
        p2.z=0;*/
        p2.x=line_endpoint(0);
        p2.y=line_endpoint(1);
        p2.z=0;
        line_list.points.push_back(p2);

    }
    publisher_.publish(line_list);


    };
}

